"""Offline camera identity/clock admission and actual local SUB worker fixtures; no robot access."""
from copy import deepcopy
from io import BytesIO
import json
import time

import numpy as np
from PIL import Image
import pytest
import zmq

from robot_platform.helm.task_capture import CAMERAS, CameraCache, LiveTaskCapture, paired_receiver_sample, recording_images, validate_config
from robot_platform.transport.wire import pack_camera, unpack_camera

NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')


def config():
    return dict(schema=1, camera_roles=list(CAMERAS), cameras={n: dict(endpoint=f'tcp://127.0.0.1:{5600+i}',
                camera_id=n, timestamp_clock='local_unix', timestamp_semantics='host_read_complete') for i, n in enumerate(CAMERAS)})


def camera(name, stamp=1000., frame=99, width=8, rotation=None):
    payload = BytesIO()
    Image.fromarray(np.full((6, 8, 3), 80, dtype=np.uint8)).save(payload, format='JPEG')
    header = dict(camera_id=name, timestamp=stamp, frame_number=frame,
                  color=dict(format='jpeg', width=width, height=6))
    if rotation is not None:
        header['image_rotation_deg'] = rotation
    return pack_camera(header, payload.getvalue())


def fill(cache, *, stamp=1000., received=10.01, frame=99):
    for name in CAMERAS:
        assert cache.ingest(name, camera(name, stamp, frame), received_at=received, wall_time=stamp+.01)


@pytest.mark.parametrize('rotation', [None, 0, 180])
def test_camera_orientation_preserved_in_selected_frame_provenance_and_status(rotation):
    cache = CameraCache(config())
    fill(cache)
    message = unpack_camera(camera('head', stamp=1000.02))
    payload = message['color'].pop('data_bytes')
    if rotation is not None:
        message['image_rotation_deg'] = rotation
    assert cache.ingest('head', pack_camera(message, payload), received_at=10.03, wall_time=1000.03)
    assert cache.select(10.031)['head']['image_rotation_deg'] == rotation
    assert cache.status(10.031)['head']['image_rotation_deg'] == rotation
    # No inference that legacy, unlabelled publishers applied zero correction.
    assert cache.status(10.031)['right_wrist']['image_rotation_deg'] is None


@pytest.mark.parametrize('rotation', [True, 90, 180., '180', {}])
def test_invalid_orientation_metadata_invalidates_cached_frame(rotation):
    cache = CameraCache(config())
    fill(cache)
    message = unpack_camera(camera('head', stamp=1000.02))
    payload = message['color'].pop('data_bytes')
    message['image_rotation_deg'] = rotation
    assert not cache.ingest('head', pack_camera(message, payload), received_at=10.03, wall_time=1000.03)
    with pytest.raises(ValueError, match='image_rotation_deg'):
        cache.select(10.031)


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
    # The earlier head frame remains a valid coherent set; the newer head
    # cannot force selection of an incoherent set while history is available.
    assert cache.select(10.06)['head']['publisher_time'] == 1000.
    cache.frames['head'].popleft()
    with pytest.raises(ValueError, match='skew'):
        cache.select(10.06)


def test_unequal_camera_rates_select_real_history_without_relaxing_limits():
    cfg = config()
    cfg['camera_roles'] = ['right_wrist', 'head']
    cfg['cameras'] = {n: cfg['cameras'][n] for n in cfg['camera_roles']}
    cache = CameraCache(cfg)
    # Observed hardware rates: wrist about 50 Hz, head about 15 Hz.
    chose_older_wrist = False
    for tick in range(120):
        stamp = tick / 300.
        for name, every in [('right_wrist', 6), ('head', 20)]:
            if tick % every == 0:
                assert cache.ingest(name, camera(name, 1000.+stamp, tick),
                                    received_at=10.+stamp+.004, wall_time=1000.+stamp+.004)
        if tick < 20 or tick % 3:
            continue
        cutoff = 10.+stamp+.005
        chosen = cache.select(cutoff)
        times = [frame['sampled_at'] for frame in chosen.values()]
        assert max(times)-min(times) <= .02
        assert all(0 <= cutoff-t <= .2 for t in times)
        assert all(frame['received_at'] <= cutoff for frame in chosen.values())
        assert all(any(frame is original for original in cache.frames[name]) for name, frame in chosen.items())
        chose_older_wrist |= chosen['right_wrist'] is not cache.frames['right_wrist'][-1]
    assert chose_older_wrist
    with pytest.raises(ValueError, match='no fresh'):
        cache.select(11.)


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
    return dict(status=dict(now_ms=now_ms, joint_names=list(NAMES), state='executing'),
                hardware_enabled=True, feedback=feedback), [row]


def test_receiver_pairs_preserve_emitted_commands_and_conservative_clock_interval():
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    capture.names, capture.config = NAMES, validate_config(config())
    response, rows = result()
    response['status']['now_ms'] = 102
    capture.receive(response, rows, started_at=10., received_at=10.004)
    sample = capture._sample
    assert sample['q'] == tuple(float(i) for i in range(14))
    assert sample['qcmd'] == tuple(i+.2 for i in range(14))
    assert sample['source_time'] == pytest.approx(9.997)
    assert sample['clock_uncertainty_s'] == pytest.approx(.005)
    for change in ('order', 'fault', 'stale', 'future', 'dropped'):
        response, rows = result()
        if change == 'order': response['status']['joint_names'].reverse()
        if change == 'fault': response['feedback'][0]['fault'] = True
        if change == 'stale': response['feedback'][0]['sampled_at_ms'] = 1
        if change == 'future': rows[0][0] = 101
        capture.receive(response, rows, dropped=int(change == 'dropped'), started_at=10., received_at=10.004)
        assert capture._sample['error'], change


def test_receiver_fractional_recorder_stamp_preserves_observed_snapshot_and_uncertainty():
    # Saved Jetson loaded-camera failure: status floors milliseconds while the
    # recorder reads the clock later within that same millisecond.
    response, rows = result(2106)
    response['status']['state'], response['hardware_enabled'] = 'disarmed', False
    rows[0][0] = 2106.23714
    for feedback in response['feedback']:
        feedback['sampled_at_ms'] = 2103
    sample = paired_receiver_sample(response, rows, NAMES, validate_config(config()),
                                    started_at=10., received_at=10.004)
    assert sample['error'] is None
    assert sample['receiver_ms'] == 2106.23714
    assert sample['receiver_feedback_ms'] == [2103]*14
    assert sample['receiver_status_ms'] == 2106
    assert sample['receiver_status_upper_ms'] == 2107
    assert sample['source_time'] == pytest.approx(9.996)
    assert sample['rpc_round_trip_s'] == pytest.approx(.004)
    assert sample['status_clock_quantization_s'] == .001
    assert sample['clock_uncertainty_s'] == pytest.approx(.005)
    assert not sample['command_valid']  # Parsing fresh stopped evidence never grants authority.


@pytest.mark.parametrize('now_ms', [-1, 100.5, float('nan'), float('inf'), True, '100'])
def test_receiver_quantization_requires_integral_nonnegative_status_clock(now_ms):
    response, rows = result()
    response['status']['now_ms'] = now_ms
    sample = paired_receiver_sample(response, rows, NAMES, validate_config(config()),
                                    started_at=10., received_at=10.004)
    assert 'finite nonnegative integer' in sample['error']


@pytest.mark.parametrize('row_ms,feedback_ms,accepted', [
    (100.999999, 100, True),  # Strictly inside status [100, 101).
    (101, 100, False), (101.000001, 100, False),  # A later recorder bucket is unproven.
    (71, 100, True), (70.999999, 100, False),  # Conservative row age <=30 ms.
    (100, 71, True), (100, 70.999999, False),  # Same bound for every feedback stamp.
    (100, 100.000001, False),  # Feedback must not be later than integer status.
    (-.001, 100, False), (100, -.001, False),
    (True, 100, False), (100, True, False),
])
def test_receiver_quantization_half_open_bucket_and_strict_age_bounds(row_ms, feedback_ms, accepted):
    response, rows = result(100.)  # Integral numeric representation is supported.
    rows[0][0] = row_ms
    response['feedback'][0]['sampled_at_ms'] = feedback_ms
    sample = paired_receiver_sample(response, rows, NAMES, validate_config(config()),
                                    started_at=10., received_at=10.004)
    assert (sample['error'] is None) is accepted
    if accepted:
        assert sample['source_time'] == pytest.approx(10.-(101-min(row_ms, feedback_ms))/1000)


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
    def wait_closed(self, *, timeout):
        assert not self.active and timeout >= 0


@pytest.fixture
def socket_capture(tmp_path, request):
    # A simulated host clock makes these scheduler-independent; real local PUB/SUB transport is exercised.
    clock = [10.]
    context = zmq.Context()
    publishers, cfg = {}, config()
    options = getattr(request, 'param', CAMERAS)
    roles = tuple(options.get('roles', CAMERAS) if isinstance(options, dict) else options)
    if isinstance(options, dict) and 'recording_image_size' in options:
        cfg['recording_image_size'] = options['recording_image_size']
    open_s = options.get('open_s', 0.) if isinstance(options, dict) else 0.
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
        clock[0] += open_s  # Opening the episode file takes this long.
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
                    pub.send_multipart(camera(name, clock[0]+990.-.01, frame=0,
                                              rotation=180 if name == 'head' else None))
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
    assert frame['task_state']['cameras']['head']['image_rotation_deg'] == 180
    assert frame['task_state']['cameras']['right_wrist']['image_rotation_deg'] is None
    assert not capture.snapshot()['failed']


@pytest.mark.parametrize('socket_capture', [{'open_s': .05}], indirect=True)
def test_a_slow_episode_open_records_from_a_sample_taken_after_it(socket_capture):
    # Opening the episode file on the Jetson's SD card outlasted the receiver sample read before it, and the first
    # frame failed as expired with nothing written (live-20261020T004826Z, 2026-10-19). The first frame now pairs a
    # sample taken after the open; every frame still meets the receiver age bound.
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 3)
    assert not capture.snapshot()['failed'] and writers[0].error is None
    capture.command('stop_record')
    pump(lambda: capture.snapshot()['recording'].get('complete'))
    for frame in writers[0].frames:
        admitted = frame['task_state']['capture_admission_at']
        assert 0 <= admitted - frame['source_time'] <= capture.config['max_receiver_age_s']


@pytest.mark.parametrize('socket_capture', [{'open_s': .05}], indirect=True)
def test_a_receiver_stalled_after_a_slow_open_still_fails_the_episode(socket_capture):
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    capture.command('start_record')
    pump(lambda: capture.snapshot()['failed'], receiver=False)
    assert writers and not writers[0].frames and not writers[0].complete
    assert 'expired' in (writers[0].error or '')


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


@pytest.mark.parametrize('size', [[0, 240], [320, -1], [True, 240], [320., 240], [1281, 960],
                                  [1280, 961], [320], [320, 240, 3], '320x240', (320, 240), {}])
def test_recording_image_size_rejects_invalid_or_unbounded_profiles(size):
    with pytest.raises(ValueError, match='recording_image_size'):
        validate_config(dict(config(), recording_image_size=size))


def test_default_capture_keeps_source_resolution_and_profile_keeps_timing_bounds():
    from pathlib import Path
    assert validate_config(config())['recording_image_size'] is None
    assert validate_config(dict(config(), recording_image_size=None))['recording_image_size'] is None
    profile = validate_config(json.loads((Path(__file__).parents[2]/'config/devices/minerva-task-capture.json').read_text()))
    assert profile['recording_image_size'] == [320, 240]
    assert (profile['hz'], profile['max_receiver_age_s'], profile['max_camera_age_s'],
            profile['max_camera_skew_s']) == (20., .03, .2, .02)
    cache = CameraCache(config())
    fill(cache)
    frames = cache.select(10.02)
    images, provenance = recording_images(frames, None)
    for role in CAMERAS:
        assert images[role] is frames[role]['image']
        assert provenance[role]['image_transform']['operation'] == 'identity'
        assert provenance[role]['image_transform']['source_shape'] == [6, 8, 3]
        assert provenance[role]['image_transform']['stored_shape'] == [6, 8, 3]


@pytest.mark.parametrize('size', [[16, 12], [4, 4]])
def test_recording_resize_refuses_upsampling_or_aspect_distortion(size):
    cache = CameraCache(config())
    fill(cache)
    with pytest.raises(ValueError, match='downsample without changing aspect ratio'):
        recording_images(cache.select(10.02), size)


def test_resized_rgb_hdf5_preserves_pairing_source_pixels_and_declared_geometry(tmp_path):
    import h5py
    import PIL
    from robot_platform.data.task_capture import TaskEpisodeWriter
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    capture.config = validate_config(dict(config(), recording_image_size=[320, 240]))
    clock = [10.01]
    capture.clock, capture._abort_reason = lambda: clock[0], None
    # A textured nonconstant RGB input makes an incorrect resize observable.
    source = np.random.default_rng(174).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    original = source.copy()
    source.setflags(write=False)
    frames = {role: dict(image=source, publisher_time=1000., sampled_at=9.99,
                         received_at=9.995, receive_id=i, image_rotation_deg=180 if role == 'head' else None,
                         timestamp_semantics='host_read_complete', exposure_time_known=False)
              for i, role in enumerate(CAMERAS)}
    sample = dict(q=list(range(14)), qcmd=[i+.25 for i in range(14)], source_time=10.,
                  received_at=10.002, error=None, command_valid=True, receiver_ms=125.5)
    writer = TaskEpisodeWriter(tmp_path, NAMES, dict(data_source='real', capture_profile=capture.config))
    try:
        capture._append_frame(writer, sample, frames)
        clock[0] = 10.06
        later_frames = {role: dict(frame, publisher_time=1000.05, sampled_at=10.04, received_at=10.045)
                        for role, frame in frames.items()}
        capture._append_frame(writer, dict(sample, source_time=10.05), later_frames)
        path = writer.finish()
    finally:
        if not writer.status()['complete']:
            writer.abort('test teardown')
            writer.wait_closed(timeout=2.)
    # Independent integer arithmetic, not the production Pillow operation.
    summed = original.astype(np.uint16).reshape(240, 2, 320, 2, 3).sum(axis=(1, 3), dtype=np.uint16)
    expected = ((summed+2)//4).astype(np.uint8)
    np.testing.assert_array_equal(source, original)
    assert not source.flags.writeable
    with h5py.File(path) as f:
        assert f.attrs['complete'] and not f.attrs['training_promoted']
        assert not list(tmp_path.glob('episode_*.hdf5'))
        np.testing.assert_array_equal(f['observations/qpos'][0], sample['q'])
        np.testing.assert_array_equal(f['observations/qcmd'][0], sample['qcmd'])
        assert f['timestamps/telem'][0] == 10.
        assert f['action_valid'][:].tolist() == [True, False]
        state = json.loads(f['task_state'][0])
        assert state['capture']['receiver_ms'] == 125.5
        assert state['capture_admission_at'] == 10.01
        for role in CAMERAS:
            dataset = f['observations/images/'+role]
            assert dataset.dtype == np.uint8 and dataset.shape == (2, 240, 320, 3)
            assert dataset.chunks == (1, 240, 320, 3) and dataset.compression == 'gzip'
            np.testing.assert_array_equal(dataset[0], expected)
            assert f['timestamps/camera_'+role][0] == 9.99
            provenance = state['cameras'][role]
            assert {k: v for k, v in provenance.items() if k != 'image_transform'} == {
                k: v for k, v in frames[role].items() if k != 'image'}
            assert provenance['image_transform'] == dict(operation='integer_reduce', source_shape=[480, 640, 3],
                stored_shape=[240, 320, 3], algorithm='Pillow.Image.reduce', pillow_version=PIL.__version__,
                color_order='RGB', stored_image_intrinsics=None, factor=[2, 2])


@pytest.mark.parametrize('expired', ['receiver', 'camera'])
def test_resize_time_cannot_renew_original_sample_or_camera_freshness(monkeypatch, expired):
    from robot_platform.helm import task_capture
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    capture.config = validate_config(dict(config(), recording_image_size=[4, 3]))
    now = [10.008 if expired == 'receiver' else 10.]
    capture.clock, capture._abort_reason = lambda: now[0], None
    cache = CameraCache(config())
    fill(cache, stamp=1000., received=9.815 if expired == 'camera' else 9.995)
    frames = cache.select(10.)
    sample = dict(q=[0.]*14, qcmd=[.1]*14, source_time=10., error=None, command_valid=True)
    writer = Writer(None, NAMES, dict(data_source='real', camera_roles=CAMERAS), hz=20.)
    original_transform = task_capture.recording_images
    def slow_transform(*args):
        transformed = original_transform(*args)
        now[0] += .024 if expired == 'receiver' else .011
        # A newer paired sample exists, but must never replace the selected q/qcmd.
        capture._sample = dict(sample, source_time=now[0], qcmd=[9.]*14)
        return transformed
    monkeypatch.setattr(task_capture, 'recording_images', slow_transform)
    message = 'Receiver sample expired' if expired == 'receiver' else 'camera frame expired'
    with pytest.raises(ValueError, match=message):
        capture._append_frame(writer, sample, frames)
    assert not writer.frames
    assert sample['source_time'] == 10. and sample['qcmd'] == [.1]*14


def test_recorded_frames_leave_no_image_work_between_pairing_and_admission(monkeypatch):
    # Live 2026-10-19, every recording failed within seconds: 'Receiver sample expired before capture'. Reducing
    # the recorded 640x480 triplet ran between pairing the receiver sample and admitting the frame, and on the
    # Jetson's shared camera CPUs it took up to 16 ms of the 30 ms bound.
    capture = LiveTaskCapture.__new__(LiveTaskCapture)
    capture.config = validate_config(dict(config(), recording_image_size=[4, 3]))
    now = [10.008]
    capture.clock, capture._abort_reason = lambda: now[0], None
    cache = CameraCache(capture.config)
    fill(cache, stamp=1000., received=9.995)
    frames = cache.select(10.)
    sample = dict(q=[0.]*14, qcmd=[.1]*14, source_time=10., error=None, command_valid=True)
    writer = Writer(None, NAMES, dict(data_source='real', camera_roles=CAMERAS), hz=20.)
    def contended(original):
        def slow(self, *args, **kwargs):
            now[0] += .024
            return original(self, *args, **kwargs)
        return slow
    monkeypatch.setattr(Image.Image, 'reduce', contended(Image.Image.reduce))
    monkeypatch.setattr(Image.Image, 'resize', contended(Image.Image.resize))
    capture._append_frame(writer, sample, frames)
    state = writer.frames[0]['task_state']
    assert state['capture_admission_at'] == 10.008
    assert all(image.shape == (3, 4, 3) for image in writer.frames[0]['images'].values())
    assert {c['image_transform']['operation'] for c in state['cameras'].values()} == {'jpeg_dct_scale'}


def jpeg_camera(name, image, *, stamp=1000.):
    payload = BytesIO()
    Image.fromarray(image).save(payload, format='JPEG', quality=95)
    header = dict(camera_id=name, timestamp=stamp, frame_number=99,
                  color=dict(format='jpeg', width=image.shape[1], height=image.shape[0]))
    return pack_camera(header, payload.getvalue())


def test_jpeg_twice_the_recording_size_decodes_straight_to_it():
    import PIL
    y, x = np.mgrid[0:48, 0:64]
    source = np.stack([x*4, y*5, (x+y)*2], axis=-1).astype(np.uint8)
    cache = CameraCache(dict(config(), recording_image_size=[32, 24]))
    for name in CAMERAS:
        assert cache.ingest(name, jpeg_camera(name, source), received_at=10.01, wall_time=1000.01)
    frames = cache.select(10.02)
    images, provenance = recording_images(frames, [32, 24])
    # The decoder's 2x scaling stays a faithful 2x reduction of the same JPEG decoded in full.
    with Image.open(BytesIO(unpack_camera(jpeg_camera('head', source))['color']['data_bytes'])) as image:
        reference = np.asarray(image.convert('RGB').reduce(2)).astype(int)
    for role in CAMERAS:
        assert images[role] is frames[role]['image'] and images[role].shape == (24, 32, 3)
        assert np.abs(images[role].astype(int)-reference).mean() < 2
        assert 'decoded_from' not in provenance[role]
        assert provenance[role]['image_transform'] == dict(operation='jpeg_dct_scale', source_shape=[48, 64, 3],
            stored_shape=[24, 32, 3], algorithm='Pillow.Image.draft', pillow_version=PIL.__version__,
            color_order='RGB', stored_image_intrinsics=None, factor=[2, 2])
    with pytest.raises(ValueError, match='decoded for a different recording size'):
        recording_images(frames, [16, 12])


def test_jpeg_of_another_size_decodes_in_full_and_is_reduced_when_recorded():
    source = np.arange(9*12*3, dtype=np.uint8).reshape(9, 12, 3)
    cache = CameraCache(dict(config(), recording_image_size=[4, 3]))
    for name in CAMERAS:
        assert cache.ingest(name, jpeg_camera(name, source), received_at=10.01, wall_time=1000.01)
    frames = cache.select(10.02)
    assert all(frame['image'].shape == (9, 12, 3) and 'decoded_from' not in frame for frame in frames.values())
    images, provenance = recording_images(frames, [4, 3])
    assert all(image.shape == (3, 4, 3) for image in images.values())
    assert provenance['head']['image_transform']['operation'] == 'resize'
    assert provenance['head']['image_transform']['source_shape'] == [9, 12, 3]


@pytest.mark.parametrize('socket_capture', [{'recording_image_size': [4, 3]}], indirect=True)
def test_socket_worker_resizes_only_selected_recorded_triplets(socket_capture, monkeypatch):
    from robot_platform.helm import task_capture
    capture, writers, pump = socket_capture
    calls = []
    original_transform = task_capture.recording_images
    def observed_transform(frames, size):
        calls.append(tuple(frame['receive_id'] for frame in frames.values()))
        return original_transform(frames, size)
    monkeypatch.setattr(task_capture, 'recording_images', observed_transform)
    pump(lambda: capture.snapshot()['capture_ready'])
    assert not calls  # Ingest/preflight/preview does not resize incoming images.
    capture.command('start_record')
    pump(lambda: writers and len(writers[0].frames) >= 3)
    capture.command('stop_record')
    pump(lambda: capture.snapshot()['recording'].get('complete'))
    assert len(calls) == len(writers[0].frames) and len(calls) == len(set(calls))
    for frame in writers[0].frames:
        assert all(image.shape == (3, 4, 3) for image in frame['images'].values())
        assert all(c['image_transform']['source_shape'] == [6, 8, 3]
                   for c in frame['task_state']['cameras'].values())


def test_authority_loss_during_resize_aborts_even_if_new_valid_sample_arrives(socket_capture, monkeypatch):
    from robot_platform.helm import task_capture
    capture, writers, pump = socket_capture
    pump(lambda: capture.snapshot()['capture_ready'])
    original_transform = task_capture.recording_images
    def lose_authority(*args):
        transformed = original_transform(*args)
        old = capture._sample
        capture.receive_sample(dict(old, command_valid=False))
        capture.receive_sample(old)
        return transformed
    monkeypatch.setattr(task_capture, 'recording_images', lose_authority)
    capture.command('start_record')
    pump(lambda: capture.snapshot()['failed'])
    assert writers and not writers[0].frames and not writers[0].complete
    assert 'command authority' in writers[0].error
    assert 'command authority' in capture.snapshot()['error']


def test_recording_transform_supports_legacy_pillow_without_resampling_enum(monkeypatch):
    import PIL
    from types import SimpleNamespace
    cache = CameraCache(config())
    fill(cache)
    frames = cache.select(10.02)
    for frame in frames.values():
        frame['image'] = np.arange(8*12*3, dtype=np.uint8).reshape(8, 12, 3)
    expected = np.asarray(Image.fromarray(frames['head']['image']).resize((3, 2), Image.Resampling.BOX))
    # Jetson currently has Pillow9.0.1: constants live directly on Image. Keep
    # the installed module's internal globals untouched while testing that API.
    legacy_image = SimpleNamespace(fromarray=Image.fromarray, BOX=Image.Resampling.BOX)
    monkeypatch.setattr(PIL, 'Image', legacy_image)
    images, provenance = recording_images(frames, [3, 2])
    np.testing.assert_array_equal(images['head'], expected)
    assert provenance['head']['image_transform']['algorithm'] == 'Pillow.BOX'
    assert 'factor' not in provenance['head']['image_transform']



def test_integer_reduction_has_explicit_rounding_distinct_from_box():
    # Each 2x2 block has channel sum1: integer reduction rounds1/4 to0,
    # whereas the separable BOX operation rounds twice and produces1.
    plane = np.tile(np.array([[0, 0], [0, 1]], dtype=np.uint8), (2, 2))
    source = np.repeat(plane[:, :, None], 3, axis=2)
    frames = {'head': dict(image=source, sampled_at=10.)}
    images, provenance = recording_images(frames, [2, 2])
    assert not images['head'].any()
    assert np.asarray(Image.fromarray(source).resize((2, 2), Image.Resampling.BOX)).min() == 1
    assert provenance['head']['image_transform']['operation'] == 'integer_reduce'
    assert provenance['head']['image_transform']['algorithm'] == 'Pillow.Image.reduce'
    assert provenance['head']['image_transform']['factor'] == [2, 2]
    np.testing.assert_array_equal(source[:, :, 0], plane)


@pytest.mark.parametrize('shape,target', [((9, 15, 3), [5, 3]), ((8, 12, 3), [3, 2])])
def test_other_downsample_ratios_keep_box_without_integer_round_up(shape, target, monkeypatch):
    source = np.arange(np.prod(shape), dtype=np.uint8).reshape(shape)
    expected = np.asarray(Image.fromarray(source).resize(tuple(target), Image.Resampling.BOX))
    def forbidden_reduce(*args, **kwargs):
        pytest.fail('Non-2x shape used integer reduce/round-up')
    monkeypatch.setattr(Image.Image, 'reduce', forbidden_reduce)
    images, provenance = recording_images({'head': {'image': source}}, target)
    np.testing.assert_array_equal(images['head'], expected)
    assert images['head'].shape == (target[1], target[0], 3)
    assert provenance['head']['image_transform']['operation'] == 'resize'
    assert provenance['head']['image_transform']['algorithm'] == 'Pillow.BOX'
    assert 'factor' not in provenance['head']['image_transform']
