import asyncio
import copy
import json
import math
from pathlib import Path
import struct
import time

import pytest

from robot_platform.control.joint_calibration import (
    checked_positions, check_reference, direction_result, edit_preview,
    live_positions, load_plan, run_wizard, seed_results,
    save_json,
    write_guide,
    SshCapture,
    preview_reconnect_allowed, run_live,
    preview_transmission_row, validate_preview_transmissions,
)
from robot_platform.control.joint_capture import capture_roster, stopped_exchange, live_scan


def reference():
    return {'id': 'reference', 'asset_id': 'fixture', 'model_revision': 'abc', 'motors': [
        {'name': n, 'bus': 'canTest', 'id': i + 4, 'uid': '0102030405060708',
         'firmware': '0.2.3.34', 'raw_reference_rad': 0.}
        for i, n in enumerate(('shoulder', 'elbow'))]}


def snapshot(q=0., other=0.):
    ref = reference()
    return {'success': True, 'asset_id': ref['asset_id'], 'enable_requested': False,
            'parameter_writes_requested': False, 'after': {'canTest': {'flags': ['NOARP']}},
            'motors': [{**m, 'position_rad': angle, 'samples_rad': [angle] * 6, 'reset_confirmed': True}
                       for m, angle in zip(ref['motors'], (q, other))]}


STEP = {'motor': 'shoulder', 'joint': 'hinge', 'label': 'Shoulder',
        'instruction': 'Match the positive viewer movement.', 'demonstration_rad': .2}


@pytest.mark.parametrize('q,sign', [(.3, 1), (-.3, -1)])
def test_direction_does_not_infer_ratio_from_demonstration(q, sign):
    row = direction_result(STEP, snapshot(), snapshot(q), reference())
    assert row['sign'] == sign
    assert row['ratio'] is None and not row['scale_verified'] and not row['runtime_approved']


@pytest.mark.parametrize('q,other', [(.01, 0), (.3, .2), (2 * math.pi, 0)])
def test_tiny_motion_other_joint_motion_and_turn_change_are_rejected(q, other):
    with pytest.raises(ValueError):
        direction_result(STEP, snapshot(), snapshot(q, other), reference())


@pytest.mark.parametrize('failure', ['missing', 'duplicate', 'uid', 'firmware', 'reset', 'nan', 'moving', 'can_up', 'enabled', 'write'])
def test_bad_stopped_captures_are_rejected(failure):
    data = snapshot()
    if failure == 'missing': data['motors'].pop()
    if failure == 'duplicate': data['motors'][1] = copy.deepcopy(data['motors'][0])
    if failure == 'uid': data['motors'][0]['uid'] = 'wrong'
    if failure == 'firmware': data['motors'][0]['firmware'] = 'wrong'
    if failure == 'reset': data['motors'][0]['reset_confirmed'] = False
    if failure == 'nan': data['motors'][0]['position_rad'] = math.nan
    if failure == 'moving': data['motors'][0]['samples_rad'][0] = .1
    if failure == 'can_up': data['after']['canTest']['flags'].append('UP')
    if failure == 'enabled': data['enable_requested'] = True
    if failure == 'write': data['parameter_writes_requested'] = True
    with pytest.raises(ValueError): checked_positions(data, reference())


def test_reference_check_never_wraps_motor_angle():
    with pytest.raises(ValueError, match='shoulder'):
        check_reference({'shoulder': 2 * math.pi, 'elbow': 0}, reference())


def test_wizard_requires_return_before_saving_and_resumes_completed_steps():
    state = {'results': {}}
    replies = iter([snapshot(), snapshot(-.3), snapshot(.15), snapshot()])
    files, prompts, saves = [], [], []
    def capture(label):
        files.append(label)
        return next(replies), str(len(files))
    run_wizard([STEP], reference(), state, capture,
               lambda: saves.append(copy.deepcopy(state)), lambda *_: None,
               ask=lambda p: prompts.append(p) or '', say=lambda _: None)
    assert len(prompts) == 4  # Misaligned return rejected, then captured again.
    assert saves[0]['results']['hinge']['return_capture'] == '4'
    assert state['results']['hinge']['sign'] == -1 and state['directions_complete']
    run_wizard([STEP], reference(), state, lambda _: (snapshot(), 'fresh'),
               lambda: None, lambda *_: None, ask=lambda _: '', say=lambda _: None)


def test_abort_does_not_accept_a_direction_without_return():
    state = {'results': {}}
    inputs = iter(['', '', 'q'])
    readings = iter([snapshot(), snapshot(.3)])
    with pytest.raises(KeyboardInterrupt):
        run_wizard([STEP], reference(), state, lambda _: (next(readings), 'capture'),
                   lambda: None, lambda *_: None, ask=lambda _: next(inputs), say=lambda _: None)
    assert not state['results']


def test_live_preview_requires_identity_and_bounded_scan():
    data = snapshot(.3)
    data.update(kind='positions', scan_seconds=.2, roundtrip_seconds=.25)
    assert live_positions(data, reference())['shoulder'] == .3
    data['scan_seconds'] = .6
    with pytest.raises(ValueError): live_positions(data, reference())
    data['scan_seconds'] = .2
    data['motors'][0]['uid'] = 'wrong'
    with pytest.raises(ValueError): live_positions(data, reference())


def test_preview_edits_preserve_raw_zero_on_flip_and_do_not_approve_runtime():
    mapping = {'hinge': {'sign': 1, 'offset_rad': -.1, 'direction_verified': True}}
    edit_preview('flip hinge', mapping, {'shoulder': .4}, [STEP])
    assert mapping['hinge']['sign'] == -1 and mapping['hinge']['offset_rad'] == .1
    assert not mapping['hinge']['direction_verified']
    edit_preview('align hinge 0', mapping, {'shoulder': .4}, [STEP])
    assert mapping['hinge']['offset_rad'] == .4
    edit_preview('accept hinge', mapping, {'shoulder': .4}, [STEP])
    assert mapping['hinge']['direction_verified'] and not mapping['hinge']['runtime_approved']
    with pytest.raises(ValueError): edit_preview('align hinge nan', mapping, {}, [STEP])


class Bus:
    def __init__(self):
        self.events = []
        self.missing = False
    def __call__(self, bus, frame, kind, parameter):
        motor = frame[0] & 255
        self.events.append((motor, kind, parameter))
        if self.missing and motor == 5 and kind == 'identity': raise TimeoutError('missing')
        if kind == 'identity': return (motor << 8) | 0xfe, bytes(range(1, 9))
        if kind == 'read':
            return (17 << 24) | (motor << 8) | 0xaa, struct.pack('<H', parameter) + bytes(6)
        data = bytes([0, 196, 86, 0, 2, 3, 34, 0]) if kind == 'version' else bytes(8)
        return (2 << 24) | (motor << 8) | 0xaa, data


def test_capture_only_stops_and_reads_even_on_failure():
    bus = Bus()
    rows = capture_roster(reference()['motors'], bus, sleep=lambda _: None)
    assert len(rows) == 2 and all(len(r['samples_rad']) == 6 for r in rows)
    assert {e[1] for e in bus.events} <= {'identity', 'stop', 'version', 'read'}
    bus.missing = True
    with pytest.raises(TimeoutError): capture_roster(reference()['motors'], bus, sleep=lambda _: None)
    assert bus.events[-2:] == [(4, 'stop', None), (5, 'stop', None)]
    for kind in ('enable', 'zero', 'save', 'control', 'write'):
        with pytest.raises(ValueError): stopped_exchange(bus)('canTest', (0, bytes(8)), kind, None)


def test_live_scan_never_returns_partial_motor_roster():
    class Reader:
        def request(self, motor, kind, parameter=None):
            if motor['name'] == 'elbow' and kind == 'read':
                raise TimeoutError('dropped reply')
            return 0.
    with pytest.raises(TimeoutError, match='elbow'):
        live_scan(Reader(), reference()['motors'])


def test_minerva_plan_and_seed_bind_the_completed_signs():
    root = Path(__file__).resolve().parents[2]
    plan, _, _, ref, _, _, _ = load_plan(root / 'config/devices/minerva-joint-calibration.json')
    seeded = seed_results(root / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json', plan['steps'], ref)
    assert len(plan['steps']) == 12
    assert {n: r['sign'] for n, r in seeded.items()} == {'left_arm_j1': 1, 'right_arm_j1': -1}


def test_live_guide_marks_stale_and_does_not_clamp_measured_angles(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.viewer.server import create_app
    urdf = tmp_path / 'test.urdf'
    urdf.write_text('<robot name="fixture"><link name="base"/><link name="arm"/>'
        '<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>')
    profile = ModelProfile.from_urdf(urdf)
    model = UrdfModel(profile)
    guide = tmp_path / 'guide.json'
    data = {'model': profile.id, 'urdf_sha256': model.sha256, 'live': True,
            'positions': {'hinge': 1.5}, 'updated_at': time.time()-2, 'scan_seconds': .2, 'source_age_seconds': .25}
    guide.write_text(json.dumps(data))
    async def run():
        async with TestClient(TestServer(create_app([profile], guide=guide))) as client:
            response = await client.get('/api/calibration-guide')
            assert response.status == 200
            observed = await response.json()
            assert observed['stale'] and observed['positions']['hinge'] == 1.5
            data['urdf_sha256'] = 'changed'; guide.write_text(json.dumps(data))
            assert (await client.get('/api/calibration-guide')).status == 422
    asyncio.run(run())


def test_atomic_save_retries_a_short_windows_reader_lock(tmp_path, monkeypatch):
    from robot_platform.control import joint_calibration as module
    original = module.os.replace
    calls = []
    def replace(source, target):
        calls.append(1)
        if len(calls) < 3:
            raise PermissionError('reader holds file')
        original(source, target)
    monkeypatch.setattr(module.os, 'replace', replace)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    output = tmp_path / 'frame.json'
    save_json(output, {'complete': True})
    assert len(calls) == 3 and json.loads(output.read_text()) == {'complete': True}


def test_append_feed_uses_last_complete_frame_even_during_write(tmp_path):
    from robot_platform.viewer.server import read_guide
    path = tmp_path / 'guide.jsonl'
    write_guide(path, {'sequence': 1})
    write_guide(path, {'sequence': 2})
    with path.open('ab') as output:
        output.write(b'{"sequence":3')
    assert read_guide(path) == {'sequence': 2}


def retry_report():
    return {'success': False, 'error_code': 'incomplete_scans', 'error': 'Three incomplete scans',
            'asset_id': reference()['asset_id'], 'identity_verified': True,
            'enable_requested': False, 'parameter_writes_requested': False,
            'after': {'canTest': {'flags': ['NOARP']}}}


def test_preview_retry_requires_confirmed_disabled_identity_and_complete_cleanup():
    report = retry_report()
    assert preview_reconnect_allowed(report, reference())
    for change in ({'after': {}}, {'after': {'canTest': {'flags': ['UP']}}},
                   {'after': {'canTest': {}}}, {'cleanup_errors': ['failed']},
                   {'error_code': 'wrong_identity'}, {'identity_verified': False},
                   {'enable_requested': True}, {'parameter_writes_requested': True},
                   {'asset_id': 'different_robot'}, {'success': True}):
        assert not preview_reconnect_allowed({**report, **change}, reference())


def test_live_failure_publishes_stopped_pose_and_bounds_reconnection_attempts(tmp_path):
    from types import SimpleNamespace
    from robot_platform.viewer.server import read_guide
    class Client:
        calls = 0
        def stream(self, receive, stop):
            self.stop = stop
            self.calls += 1
            return retry_report()
    client = Client()
    state = {'results': {}}
    ref = reference()
    for motor in ref['motors']: motor['raw_reference_rad'] = 0.
    model = SimpleNamespace(profile=SimpleNamespace(id='fixture'), sha256='revision')
    guide = tmp_path / 'guide.jsonl'
    def ask(_):
        # Leave the terminal waiting while the reader fails in the background.
        deadline = time.perf_counter() + 5
        while not hasattr(client, 'stop') or not client.stop.is_set():
            assert time.perf_counter() < deadline
            time.sleep(.01)
        assert read_guide(guide)['feed_state'] == 'stopped'
        return 'q'
    with pytest.raises(RuntimeError, match='Three incomplete scans'):
        run_live(client, [STEP], ref, state, tmp_path, lambda: None, model, {'hinge': 0}, guide, ask=ask)
    assert client.calls == 4
    assert read_guide(guide)['reconnections'] == 3
    assert len(list(tmp_path.glob('live-cleanup-*.json'))) == 4


def expiry_report():
    return {**retry_report(), 'success': True, 'error': None, 'error_code': None,
            'completion_reason': 'duration_limit'}


def test_healthy_expiry_renews_but_operator_quit_stops_the_new_reader(tmp_path):
    from types import SimpleNamespace
    from robot_platform.viewer.server import read_guide
    import threading
    ready = threading.Event()
    class Client:
        calls = 0
        def stream(self, receive, stop):
            self.calls += 1
            if self.calls == 1:
                return expiry_report()
            ready.set()
            assert stop.wait(5), 'Operator stop did not reach renewed reader'
            return {**expiry_report(), 'completion_reason': 'client_stop'}
    client = Client()
    guide = tmp_path / 'guide.jsonl'
    def ask(_):
        assert ready.wait(5), 'Healthy reader expiry did not renew'
        return 'q'
    model = SimpleNamespace(profile=SimpleNamespace(id='fixture'), sha256='revision')
    run_live(client, [STEP], reference(), {'results': {}}, tmp_path, lambda: None,
             model, {'hinge': 0}, guide, ask=ask)
    assert client.calls == 2
    assert read_guide(guide)['feed_state'] == 'stopped'
    assert read_guide(guide)['reconnections'] == 0
    assert len(list(tmp_path.glob('live-cleanup-*.json'))) == 2


def test_healthy_expiries_do_not_reset_fault_retry_budget(tmp_path):
    from types import SimpleNamespace
    from robot_platform.viewer.server import read_guide
    class Client:
        calls = 0
        def stream(self, receive, stop):
            self.stop = stop
            self.calls += 1
            return retry_report() if self.calls % 2 else expiry_report()
    client = Client()
    guide = tmp_path / 'guide.jsonl'
    def ask(_):
        deadline = time.perf_counter() + 5
        while not hasattr(client, 'stop') or not client.stop.is_set():
            assert time.perf_counter() < deadline
            time.sleep(.01)
        return 'q'
    model = SimpleNamespace(profile=SimpleNamespace(id='fixture'), sha256='revision')
    with pytest.raises(RuntimeError, match='Three incomplete scans'):
        run_live(client, [STEP], reference(), {'results': {}}, tmp_path, lambda: None,
                 model, {'hinge': 0}, guide, ask=ask)
    assert client.calls == 7
    assert read_guide(guide)['reconnections'] == 3
    assert read_guide(guide)['feed_state'] == 'stopped'


@pytest.mark.parametrize('change', [
    {'after': {}}, {'after': {'canTest': {'flags': ['UP']}}},
    {'identity_verified': False}, {'enable_requested': True},
])
def test_expiry_never_renews_unverified_cleanup(tmp_path, change):
    from types import SimpleNamespace
    class Client:
        calls = 0
        def stream(self, receive, stop):
            self.calls += 1
            self.stop = stop
            return {**expiry_report(), **change}
    client = Client()
    def ask(_):
        deadline = time.perf_counter() + 5
        while not hasattr(client, 'stop') or not client.stop.is_set():
            assert time.perf_counter() < deadline
            time.sleep(.01)
        return 'q'
    model = SimpleNamespace(profile=SimpleNamespace(id='fixture'), sha256='revision')
    with pytest.raises(RuntimeError, match='cleanup not confirmed'):
        run_live(client, [STEP], reference(), {'results': {}}, tmp_path, lambda: None,
                 model, {'hinge': 0}, tmp_path / 'guide.jsonl', ask=ask)
    assert client.calls == 1


@pytest.mark.parametrize('expired', [True, False])
def test_worker_distinguishes_healthy_expiry_from_client_stop(tmp_path, monkeypatch, capsys, expired):
    import io
    from types import SimpleNamespace
    from robot_platform.control import joint_capture as module
    incoming = io.BytesIO((json.dumps({'device': {}, 'reference': {}, 'request_id': 'fixture'}) + '\nstop\n').encode())
    monkeypatch.setattr(module.sys, 'argv', ['joint_capture.py', '--live'])
    monkeypatch.setattr(module.sys, 'stdin', SimpleNamespace(buffer=SimpleNamespace(raw=incoming)))
    monkeypatch.setattr(module, '__file__', str(tmp_path / 'joint_capture.py'))
    monkeypatch.setattr(module.signal, 'SIGHUP', 1, raising=False)
    monkeypatch.setattr(module.signal, 'signal', lambda *_: None)
    monkeypatch.setattr(module.select, 'select', lambda *_: ([incoming], [], []))
    clocks = iter([0., 901. if expired else 0.])
    monkeypatch.setattr(module.time, 'perf_counter', lambda: next(clocks))
    def capture(device, reference, *, live):
        result = {'success': True, 'events': []}
        live(None, [], result)
        return result
    monkeypatch.setattr(module, 'run_capture', capture)
    module.main()
    result = json.loads(capsys.readouterr().out)
    assert result['success'], result.get('error')
    assert result['completion_reason'] == ('duration_limit' if expired else 'client_stop')
    assert result['frames'] == 0


def test_viewer_stopped_status_is_stale_even_with_recent_timestamp(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.viewer.server import create_app
    from robot_platform.models import ModelProfile, UrdfModel
    urdf = tmp_path / 'test.urdf'
    urdf.write_text('<robot name="fixture"><link name="base"/><link name="arm"/>'
        '<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>')
    profile = ModelProfile.from_urdf(urdf); model = UrdfModel(profile)
    guide = tmp_path / 'guide.jsonl'
    write_guide(guide, {'model': profile.id, 'urdf_sha256': model.sha256, 'live': True,
                      'feed_state': 'stopped', 'positions': {'hinge': 0},
                      'updated_at': time.time(), 'source_age_seconds': 0.})
    async def run():
        async with TestClient(TestServer(create_app([profile], guide=guide))) as client:
            response = await client.get('/api/calibration-guide')
            assert response.status == 200
            assert response.headers['Cache-Control'] == 'no-store'
            assert (await response.json())['stale']
    asyncio.run(run())


@pytest.mark.parametrize('wrong_nonce', [False, True])
def test_live_transport_rejects_old_requests_and_delayed_frames(monkeypatch, wrong_nonce):
    from robot_platform.control import joint_calibration as module
    class Channel:
        def __init__(self): self.writes = []; self.calls = 0
        def write(self, text): self.writes.append(text)
        def flush(self): pass
        def readline(self):
            token = json.loads(self.writes[0])['request_id']
            self.calls += 1
            if self.calls == 2:
                return json.dumps({'kind': 'finished', 'request_id': token, 'success': True})
            nonce = self.writes[-1].strip().split()[1]
            return json.dumps({'kind': 'positions', 'request_id': token,
                               'request_nonce': 'old' if wrong_nonce else nonce, 'sequence': 1})
    class Client:
        def exec_command(self, *args, **kwargs): return channel, channel, channel
    class Stop:
        def is_set(self): return False
    channel = Channel()
    client = object.__new__(SshCapture)
    client.client, client.device, client.reference, client.stage = Client(), {}, {}, '/fixture'
    clocks = iter([0., 1., 2.])
    monkeypatch.setattr(module.time, 'perf_counter', lambda: next(clocks))
    accepted = []
    if wrong_nonce:
        with pytest.raises(RuntimeError, match='fresh request'):
            client.stream(accepted.append, Stop())
    else:
        assert client.stream(accepted.append, Stop())['success']
    assert not accepted


@pytest.fixture
def linear_plan(tmp_path):
    """Two unrelated linear axes, one mimic follower, and one rotary step."""
    from robot_platform.models import ModelProfile, UrdfModel
    urdf = tmp_path / 'fixture.urdf'
    urdf.write_text('<robot name="fixture"><link name="base"/><link name="arm"/>'
        '<link name="carriage"/><link name="follower"/><link name="other"/>'
        '<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
        '<joint name="slide" type="prismatic"><parent link="arm"/><child link="carriage"/>'
        '<axis xyz="1 0 0"/><limit lower="0" upper=".04" effort="1" velocity="1"/></joint>'
        '<joint name="follower_slide" type="prismatic"><parent link="arm"/><child link="follower"/>'
        '<axis xyz="1 0 0"/><limit lower="-.04" upper="0" effort="1" velocity="1"/>'
        '<mimic joint="slide" multiplier="-1" offset="0"/></joint>'
        '<joint name="other_slide" type="prismatic"><parent link="arm"/><child link="other"/>'
        '<axis xyz="0 1 0"/><limit lower="-.04" upper="0" effort="1" velocity="1"/></joint></robot>')
    model_path = tmp_path / 'model.json'
    save_json(model_path, {'schema_version': 1, 'id': 'fixture', 'title': 'Fixture', 'urdf': urdf.name})
    model = UrdfModel(ModelProfile.load(model_path))
    ref = reference()
    ref.update(model_revision=model.sha256, power_cycle_verified=True)
    ref['motors'].extend([{**ref['motors'][0], 'name': 'linear_a', 'id': 6, 'raw_reference_rad': 2.},
                          {**ref['motors'][0], 'name': 'linear_b', 'id': 7, 'raw_reference_rad': -1.}])
    save_json(tmp_path / 'reference.json', ref)
    save_json(tmp_path / 'device.json', {'asset_id': ref['asset_id']})
    transmissions = [
        {'motor': motor, 'joint': joint, 'motor_reference_rad': zero, 'joint_reference_m': 0.,
         'metres_per_motor_rad': scale, 'direction_verified': verified, 'scale_verified': False,
         'note': 'Unverified displacement estimate for display.', 'source': 'Fixture geometry',
         'runtime_approved': False}
        for motor, joint, zero, scale, verified in
        [('linear_a', 'slide', 2., .01, True), ('linear_b', 'other_slide', -1., -.01, False)]]
    plan = {'schema': 1, 'device': 'device.json', 'reference': 'reference.json', 'model': 'model.json',
            'reference_positions': {'hinge': 0.}, 'steps': [STEP], 'preview_transmissions': transmissions}
    path = tmp_path / 'plan.json'
    save_json(path, plan)
    return path, plan, ref, model


def test_linear_preview_uses_recorded_reference_and_signed_scale(linear_plan):
    _, plan, ref, model = linear_plan
    entries = validate_preview_transmissions(plan['preview_transmissions'], [STEP], ref, model)
    positive, negative = [preview_transmission_row(entry, entry['motor_reference_rad'] + 2.) for entry in entries]
    assert positive['cad_value'] == pytest.approx(.02)
    assert negative['cad_value'] == pytest.approx(-.02)
    assert positive['sign'] == 1 and negative['sign'] == -1
    assert positive['offset_m'] == pytest.approx(-.02)
    assert negative['offset_m'] == pytest.approx(-.01)
    for row in (positive, negative):
        assert row['cad_unit'] == 'm' and 'cad_rad' not in row
        assert row['display_only'] and not row['runtime_approved'] and not row['scale_verified']
    assert positive['direction_verified'] and not negative['direction_verified']
    positions = {'hinge': 0., positive['joint']: positive['cad_value'], negative['joint']: negative['cad_value']}
    assert 'follower_slide' not in positions
    assert model.positions(positions)['follower_slide'] == pytest.approx(-.02)
    entries[0]['joint_reference_m'] = .003
    assert preview_transmission_row(entries[0], 2.)['cad_value'] == .003


def test_linear_reference_is_profile_bound_and_unmapped_axes_keep_defaults(linear_plan):
    path, plan, _, model = linear_plan
    _, _, _, _, _, pose, bindings = load_plan(path)
    assert pose == {'hinge': 0., 'slide': 0., 'other_slide': 0.}
    assert model.positions(pose)['follower_slide'] == 0.
    plan['preview_transmissions'][0]['metres_per_motor_rad'] = .012
    save_json(path, plan)
    assert load_plan(path)[-1]['plan'] != bindings['plan']
    # Each mapping is optional; an unmapped axis retains the model default.
    plan['preview_transmissions'].pop()
    save_json(path, plan)
    assert load_plan(path)[5]['other_slide'] == model.by_name['other_slide']['default']
    del plan['preview_transmissions']
    save_json(path, plan)
    assert load_plan(path)[5]['slide'] == model.by_name['slide']['default']


@pytest.mark.parametrize('key', [
    'motor', 'joint', 'motor_reference_rad', 'joint_reference_m', 'metres_per_motor_rad',
    'direction_verified', 'scale_verified', 'runtime_approved', 'note', 'source',
])
def test_linear_preview_rejects_missing_fields(linear_plan, key):
    path, plan, _, _ = linear_plan
    del plan['preview_transmissions'][0][key]
    save_json(path, plan)
    with pytest.raises(ValueError):
        load_plan(path)


@pytest.mark.parametrize('change', [
    {'motor': 'unknown'}, {'motor': 'shoulder'}, {'joint': 'unknown'}, {'joint': 'hinge'},
    {'joint': 'follower_slide'}, {'motor_reference_rad': 2.01}, {'metres_per_motor_rad': 0},
    {'direction_verified': 1}, {'scale_verified': True}, {'runtime_approved': True},
    {'scale_verified': 0}, {'runtime_approved': 0}, {'note': ''}, {'source': 3},
])
def test_linear_preview_rejects_wrong_axes_reference_or_approval(linear_plan, change):
    path, plan, _, _ = linear_plan
    plan['preview_transmissions'][0].update(change)
    save_json(path, plan)
    with pytest.raises(ValueError):
        load_plan(path)


@pytest.mark.parametrize('key', ['motor_reference_rad', 'joint_reference_m', 'metres_per_motor_rad'])
@pytest.mark.parametrize('value', [True, math.nan, math.inf, '0.01'])
def test_linear_preview_rejects_non_numeric_values(linear_plan, key, value):
    path, plan, _, _ = linear_plan
    plan['preview_transmissions'][0][key] = value
    path.write_text(json.dumps(plan))
    with pytest.raises(ValueError):
        load_plan(path)


@pytest.mark.parametrize('key', ['motor', 'joint'])
def test_linear_preview_rejects_duplicate_bindings(linear_plan, key):
    path, plan, _, _ = linear_plan
    plan['preview_transmissions'][1][key] = plan['preview_transmissions'][0][key]
    save_json(path, plan)
    with pytest.raises(ValueError, match='Duplicate'):
        load_plan(path)


def test_linear_preview_rejects_conflicting_reference_pose(linear_plan):
    path, plan, _, _ = linear_plan
    plan['reference_positions']['slide'] = .02
    save_json(path, plan)
    with pytest.raises(ValueError, match='conflicts'):
        load_plan(path)


def test_linear_preview_reference_preserves_urdf_range_validation(linear_plan):
    path, plan, _, _ = linear_plan
    plan['preview_transmissions'][0]['joint_reference_m'] = .05
    save_json(path, plan)
    with pytest.raises(ValueError, match='URDF range'):
        load_plan(path)


@pytest.mark.parametrize('value', [None, {}, [None]])
def test_linear_preview_rejects_malformed_entries(linear_plan, value):
    path, plan, _, _ = linear_plan
    plan['preview_transmissions'] = value
    save_json(path, plan)
    with pytest.raises(ValueError):
        load_plan(path)


def test_live_linear_preview_preserves_rotary_mapping_and_only_drives_independent_joints(linear_plan, tmp_path):
    import threading
    from robot_platform.viewer.server import read_guide
    path, _, _, _ = linear_plan
    plan, _, _, ref, model, pose, _ = load_plan(path)
    ready = threading.Event()
    frame = {'kind': 'positions', 'asset_id': ref['asset_id'], 'enable_requested': False,
             'parameter_writes_requested': False, 'scan_seconds': .2, 'roundtrip_seconds': .25,
             'sequence': 1, 'request_id': 'fixture',
             'motors': [{**motor, 'position_rad': raw, 'reset_confirmed': True}
                        for motor, raw in zip(ref['motors'], (.3, 0., 7., 1.))]}
    class Client:
        def stream(self, receive, stop):
            receive(frame)
            ready.set()
            assert stop.wait(5), 'Operator stop did not reach preview reader'
            return {**expiry_report(), 'completion_reason': 'client_stop'}
    guide = tmp_path / 'guide.jsonl'
    state = {'results': {'hinge': {'sign': -1}}}
    def ask(_):
        assert ready.wait(5), 'Preview did not publish'
        observed = read_guide(guide)
        assert observed['positions']['hinge'] == -.3
        assert observed['positions']['slide'] == pytest.approx(.05)  # No clipping at .04.
        assert observed['positions']['other_slide'] == pytest.approx(-.02)
        assert 'follower_slide' not in observed['positions']
        rows = {row['joint']: row for row in observed['rows']}
        assert rows['hinge']['cad_rad'] == rows['hinge']['cad_value'] == -.3
        assert rows['hinge']['cad_unit'] == 'rad'
        assert rows['slide']['raw_rad'] == 7. and rows['slide']['cad_unit'] == 'm'
        assert rows['other_slide']['direction_verified'] is False
        assert 'unverified' in observed['message'] and 'Grippers are not mapped' not in observed['message']
        return 'q'
    run_live(Client(), plan['steps'], ref, state, tmp_path, lambda: None, model, pose, guide,
             transmissions=plan['preview_transmissions'], ask=ask)
    assert set(state['preview_mapping']) == {'hinge'}
    assert not state['preview_mapping']['hinge']['runtime_approved']
