"""Offline camera identity/clock admission and actual local SUB worker fixtures; no robot access."""
from copy import deepcopy
from io import BytesIO
import time

import numpy as np
from PIL import Image
import pytest
import zmq

from robot_platform.helm.task_capture import CAMERAS, CameraCache, LiveTaskCapture, validate_config
from robot_platform.transport.wire import pack_camera

NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')


def config():
    return dict(schema=1, camera_roles=list(CAMERAS), cameras={n: dict(endpoint=f'tcp://127.0.0.1:{5600+i}',
                camera_id=n, timestamp_clock='local_unix', timestamp_semantics='host_read_complete') for i, n in enumerate(CAMERAS)})


def camera(name, stamp=1000., frame=99, width=8):
    payload = BytesIO()
    Image.fromarray(np.full((6, 8, 3), 80, dtype=np.uint8)).save(payload, format='JPEG')
    return pack_camera(dict(camera_id=name, timestamp=stamp, frame_number=frame,
                       color=dict(format='jpeg', width=width, height=6)), payload.getvalue())


def fill(cache, *, stamp=1000., received=10.01, frame=99):
    for name in CAMERAS:
        assert cache.ingest(name, camera(name, stamp, frame), received_at=received, wall_time=stamp+.01)


def test_camera_counter_can_reset_but_duplicate_acquisition_never_renews_freshness():
    cache = CameraCache(config())
    fill(cache)
    original = cache.frames['left_wrist'][-1]
    assert not cache.ingest('left_wrist', camera('left_wrist'), received_at=10.1, wall_time=1000.1)
    assert cache.frames['left_wrist'][-1] is original
    fill(cache, stamp=1000.05, received=10.06, frame=0)
    chosen = cache.select(10.07)
    assert chosen['left_wrist']['publisher_frame_number'] == 0
    assert chosen['left_wrist']['receive_id'] > original['receive_id']
    assert chosen['head']['sampled_at'] == pytest.approx(10.05)
    assert chosen['head']['image'].dtype == np.uint8
    assert not chosen['head']['image'].flags.writeable
    with pytest.raises(ValueError, match='no fresh'):
        cache.select(10.3)


@pytest.mark.parametrize('kind', ['identity', 'future', 'backwards', 'dimensions'])
def test_camera_invalid_identity_clock_or_header_cannot_reuse_previous_frame(kind):
    cache = CameraCache(config())
    fill(cache)
    parts = camera('wrong' if kind == 'identity' else 'head',
                   1000.1 if kind == 'future' else 999.99 if kind == 'backwards' else 1000.01,
                   width=9 if kind == 'dimensions' else 8)
    assert not cache.ingest('head', parts, received_at=10.02, wall_time=1000.02)
    with pytest.raises(ValueError, match='head:'):
        cache.select(10.03)


def test_camera_cutoff_never_uses_future_received_frame_and_skew_is_bounded():
    cache = CameraCache(config())
    fill(cache)
    with pytest.raises(ValueError, match='no fresh'):
        cache.select(10.005)
    assert cache.ingest('head', camera('head', 1000.04), received_at=10.05, wall_time=1000.05)
    with pytest.raises(ValueError, match='skew'):
        cache.select(10.06)


@pytest.mark.parametrize('change', ['remote', 'clock', 'missing', 'budget'])
def test_live_capture_profile_requires_same_host_clock_and_complete_declared_roster(change):
    value = config()
    if change == 'remote': value['cameras']['head']['endpoint'] = 'tcp://10.42.0.1:5564'
    if change == 'clock': value['cameras']['head']['timestamp_clock'] = 'unspecified'
    if change == 'missing': del value['cameras']['head']
    if change == 'budget': value['max_receiver_age_s'] = .1
    with pytest.raises(ValueError):
        validate_config(value)


def result(now_ms=100):
    row = [now_ms]
    for i in range(14): row += [float(i), .0, .0, i+.2]
    feedback = [dict(sampled_at_ms=now_ms, fault=False) for _ in NAMES]
    return dict(status=dict(now_ms=now_ms, joint_names=list(NAMES)), feedback=feedback), [row]


def test_receiver_pairs_preserve_emitted_commands_and_conservative_clock_interval():
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    capture.names, capture.config = NAMES, validate_config(config())
    response, rows = result()
    response['status']['now_ms'] = 102
    capture.receive(response, rows, started_at=10., received_at=10.004)
    sample = capture._sample
    assert sample['q'] == tuple(float(i) for i in range(14))
    assert sample['qcmd'] == tuple(i+.2 for i in range(14))
    assert sample['source_time'] == pytest.approx(9.998)
    assert sample['clock_uncertainty_s'] == pytest.approx(.004)
    for change in ('order', 'fault', 'stale', 'future', 'dropped'):
        response, rows = result()
        if change == 'order': response['status']['joint_names'].reverse()
        if change == 'fault': response['feedback'][0]['fault'] = True
        if change == 'stale': response['feedback'][0]['sampled_at_ms'] = 1
        if change == 'future': rows[0][0] = 101
        capture.receive(response, rows, dropped=int(change == 'dropped'), started_at=10., received_at=10.004)
        assert capture._sample['error'], change


class Writer:
    def __init__(self, directory, names, metadata, *, hz):
        assert metadata['data_source'] == 'real'
        self.roles = tuple(metadata['camera_roles'])
        self.frames, self.active, self.complete, self.error = [], True, False, None
    def append(self, **frame):
        assert frame['scene_time'] is None
        assert set(frame['images']) == set(self.roles)
        assert len(frame['q']) == len(frame['qcmd']) == 14
        self.frames.append(frame)
    def status(self):
        return dict(active=self.active, complete=self.complete, error=self.error,
                    state='failed' if self.error else 'complete' if self.complete else 'recording', frames=len(self.frames))
    def finish(self):
        self.active, self.complete = False, True
    def abort(self, reason):
        self.active, self.error = False, reason


@pytest.fixture
def socket_capture(tmp_path, request):
    # A simulated host clock makes these scheduler-independent; real local PUB/SUB transport is exercised.
    clock = [10.]
    context = zmq.Context()
    publishers, cfg = {}, config()
    roles = tuple(getattr(request, 'param', CAMERAS))
    cfg['camera_roles'] = list(roles)
    cfg['cameras'] = {name: cfg['cameras'][name] for name in roles}
    for name in roles:
        pub = context.socket(zmq.PUB)
        pub.setsockopt(zmq.LINGER, 0)
        port = pub.bind_to_random_port('tcp://127.0.0.1')
        cfg['cameras'][name]['endpoint'] = f'tcp://127.0.0.1:{port}'
        publishers[name] = pub
    writers = []
    def factory(*a, **k):
        writer = Writer(*a, **k)
        writers.append(writer)
        return writer
    capture = LiveTaskCapture(tmp_path, NAMES, cfg, clock=lambda: clock[0],
                              wall_clock=lambda: clock[0]+990., writer_factory=factory)
    def pump(predicate, *, missing=(), receiver=True, timeout=4.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            clock[0] += .004
            for name, pub in publishers.items():
                if name not in missing:
                    pub.send_multipart(camera(name, clock[0]+990.-.01, frame=0))
            if receiver:
                response, rows = result()
                capture.receive(response, rows, started_at=clock[0]-.001, received_at=clock[0])
            time.sleep(.006)
            if predicate(): return
        pytest.fail(f'Capture fixture timed out: {capture.snapshot()}')
    yield capture, writers, pump
    capture.close()
    for pub in publishers.values(): pub.close()
    context.term()


def test_socket_worker_requires_all_cameras_and_explicit_finish(socket_capture):
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['cameras'].get('left_wrist', {}).get('fresh'), missing=('head',))
    assert not capture.snapshot()['capture_ready']
    capture.command('start_record')
    pump(lambda: (capture.snapshot()['last_command'] or {}).get('state') == 'failed', missing=('head',))
    assert not writers
    pump(lambda: capture.snapshot()['capture_ready'])
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 3)
    assert not writers[0].complete
    capture.command('stop_record')
    pump(lambda: capture.snapshot()['recording'].get('complete'))
    frame = writers[0].frames[0]
    assert all(t <= frame['source_time'] for t in frame['camera_times'].values())
    assert 'publisher_frame_number' in frame['task_state']['cameras']['head']
    assert not capture.snapshot()['failed']


def test_socket_worker_aborts_stalled_receiver_without_completed_episode(socket_capture):
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 1)
    pump(lambda: capture.snapshot()['failed'], receiver=False)
    assert writers[0].error and not writers[0].complete
    assert not capture.snapshot()['available']


def test_close_during_recording_aborts_instead_of_publishing(socket_capture):
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 1)
    capture.close()
    assert writers[0].error and not writers[0].complete


def test_worker_initialization_failure_is_visible(tmp_path):
    def fail():
        raise RuntimeError('fixture unavailable dependency')
    capture = LiveTaskCapture(tmp_path, NAMES, config(), writer_factory=Writer, context_factory=fail)
    capture._thread.join(timeout=1.)
    assert capture.snapshot()['failed']
    assert 'fixture unavailable' in capture.snapshot()['error']
    capture.close()


def test_health_does_not_copy_diagnostics_and_expires_without_worker_progress():
    import threading
    class NoCopy:
        def __deepcopy__(self, memo):
            pytest.fail('Control health copied full UI diagnostics')
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    now = [10.01]
    capture.clock, capture.config = lambda: now[0], validate_config(config())
    capture._lock = threading.Lock()
    capture._sample = {'source_time': 10., 'error': None}
    capture._state = dict(available=True, error=None, failed=False, capture_ready=True,
                          recording={'active': True, 'state': 'recording', 'error': None, 'complete': False},
                          task=NoCopy(), capabilities=NoCopy())
    assert capture.health()['capture_ready']
    now[0] = 10.04
    health = capture.health()
    assert not health['capture_ready']
    assert health['source_age_s'] == pytest.approx(.04)
    assert health['recording']['active']


def test_two_actual_socket_streams_do_not_admit_recording_without_left_wrist(socket_capture):
    from robot_platform.helm.server import camera_preflight
    capture, writers, pump = socket_capture
    pump(lambda: all(capture.snapshot()['cameras'].get(n, {}).get('fresh')
                     for n in ('right_wrist', 'head')), missing=('left_wrist',))
    status = capture.snapshot()
    assert camera_preflight(status['cameras']) == '2/3 cameras fresh; missing or stale: left_wrist'
    assert not status['capture_ready']
    capture.command('start_record')
    pump(lambda: (capture.snapshot()['last_command'] or {}).get('state') == 'failed', missing=('left_wrist',))
    assert not writers


@pytest.mark.parametrize('socket_capture', [('right_wrist', 'head')], indirect=True)
def test_explicit_two_camera_capture_has_no_left_placeholder(socket_capture):
    from robot_platform.helm.server import camera_preflight
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    status = capture.snapshot()
    assert status['camera_roles'] == ['right_wrist', 'head']
    assert camera_preflight(status['cameras'], status['camera_roles']) == '2/2 cameras fresh'
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 2)
    capture.command('stop_record')
    pump(lambda: capture.snapshot()['recording'].get('complete'))
    assert writers[0].roles == ('right_wrist', 'head')
    for frame in writers[0].frames:
        assert set(frame['images']) == set(frame['camera_times']) == {'right_wrist', 'head'}
        assert len(frame['q']) == len(frame['qcmd']) == 14


@pytest.mark.parametrize('roles', [None, ['head'], ['head', 'head'], ['head', 'unknown']])
def test_capture_camera_roster_must_be_explicit_supported_and_distinct(roles):
    value = config()
    value['camera_roles'] = roles
    with pytest.raises(ValueError, match='camera_roles'):
        validate_config(value)
