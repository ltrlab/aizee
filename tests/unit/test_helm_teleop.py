"""Helm teleop plumbing without native builds: profile checks, advisory verdicts,
route edits, the loop's messaging and the web endpoints."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from robot_platform.control.scheduling import prefer_responsive
from robot_platform.helm.server import create_app
from robot_platform.helm.teleop import AdvisoryCompiler, TeleopCore, TeleopSession, load_profile

ROOT = Path(__file__).resolve().parents[2]
TELEOP = ROOT / 'config/devices/minerva-teleop.json'


def write_profile(tmp_path, **changes):
    profile = json.loads(TELEOP.read_text(encoding='utf-8'))
    receiver = changes.pop('receiver', {})
    profile.update(changes)
    profile['receiver'].update(receiver)
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile), encoding='utf-8')
    return path


def test_profile_must_be_virtual_and_unapproved(tmp_path):
    assert load_profile(TELEOP)['receiver']['collision_mode'] == 'advisory'
    for change in ({'kind': 'other'}, {'backend': 'physical'}, {'runtime_approved': True}):
        with pytest.raises(ValueError, match='virtual, unapproved'):
            load_profile(write_profile(tmp_path, **change))
    with pytest.raises(ValueError, match='enforced, advisory or disabled'):
        load_profile(write_profile(tmp_path, receiver={'collision_mode': 'off'}))


class Proximity:
    def __init__(self, pairs):
        self.pairs, self.calls = pairs, []

    def proximity(self, cad, *, threshold, scope):
        self.calls.append((cad, threshold, scope))
        return {'near_pairs': self.pairs}


def advisory(pairs, **options):
    config = {'collision': {'world_revision': 'world', 'scope': 'inter_branch'},
              'joints': [{'name': 'arm_j1', 'lower': -1., 'upper': 1.},
                         {'name': 'gripper', 'lower': -.1, 'upper': 2.}]}
    joint_map = {'shoulder': {'actuator': 'arm_j1', 'scale': -1., 'offset': .1},
                 'slide': {'actuator': 'gripper', 'scale': .01, 'offset': 0.}}
    return AdvisoryCompiler(Proximity(pairs), config, joint_map, compute_budget_s=.02, **options)


def test_advisory_checks_the_target_pose_and_never_blocks_the_virtual_receiver():
    compiler = advisory([])
    feedback = [{'position': .3}, {'position': 1.}]
    envelope = compiler.compile({'offer': {'id': 7}}, feedback)
    cad, threshold, scope = compiler.backend.calls[-1]
    assert cad == pytest.approx({'shoulder': -.2, 'slide': .01}) and threshold == .05 and scope == 'inter_branch'
    assert envelope == {'offer': 7, 'world_revision': 'world', 'scope': 'inter_branch',
                        'lower': [-1., -.1], 'upper': [1., 2.]}
    assert compiler.last_verdict['reason'] == 'target_clear' and compiler.last_verdict['clear']
    compiler.compile({'offer': {'id': 8}}, feedback, [{'position': .5}, {'position': 1.5}])
    assert compiler.backend.calls[-1][0] == pytest.approx({'shoulder': -.4, 'slide': .015})
    # One live arm: the rest of the model stays parked; a closed gripper is clamped into the model.
    parked = advisory([], fixed={'elbow': .7}, cad_limits={'slide': (0., .02)})
    parked.compile({'offer': {'id': 11}}, [{'position': .3}, {'position': -.05}])
    assert parked.backend.calls[-1][0] == pytest.approx({'elbow': .7, 'shoulder': -.2, 'slide': 0.})

    near = advisory([{'distance_m': .004, 'shapes': ['a', 'b']}, {'distance_m': .03, 'shapes': ['c', 'd']}])
    assert near.compile({'offer': {'id': 9}}, feedback)['offer'] == 9
    assert near.last_verdict == {'advisory': True, 'clear': False, 'reason': 'target_contact',
                                 'distance_m': .004, 'shapes': ['a', 'b']}
    assert near.blocked == 1
    near.backend.pairs = [{'distance_m': .03, 'shapes': ['c', 'd']}]
    near.compile({'offer': {'id': 10}}, feedback)
    assert near.last_verdict['reason'] == 'target_near' and near.last_verdict['clear'] and near.blocked == 1


def core(tmp_path, asset='minerva-production-01'):
    return TeleopCore({'teleop_profile': str(TELEOP), 'routes_path': str(tmp_path / 'teleop' / 'routes.json'),
                       'asset_id': asset})


def test_route_edits_persist_per_profile_and_asset(tmp_path):
    first = core(tmp_path)
    assert first.routes()['left_arm_j2']['sign'] == -1
    first.route('flip', 'left_arm_j2')
    first.route('accept', 'left_arm_j2')
    first.route('flip', 'left_gripper')
    with pytest.raises(ValueError, match='flip or accept'):
        first.route('flip', 'left_arm_j7')
    with pytest.raises(ValueError, match='flip or accept'):
        first.route('rotate', 'left_arm_j2')
    saved = json.loads((tmp_path / 'teleop' / 'routes.json').read_text(encoding='utf-8'))
    assert saved['runtime_approved'] is False and saved['profile'] == 'minerva-production-01-teleop'
    assert [entry['action'] for entry in saved['routes']['left_arm_j2']['history']] == ['flip', 'accept']
    routes = core(tmp_path).routes()
    assert routes['left_arm_j2']['sign'] == 1 and routes['left_arm_j2']['verified'] is True
    assert routes['left_gripper']['sign'] == -1 and routes['left_gripper']['verified'] is False
    assert routes['right_arm_j2']['sign'] == -1 and 'verified' not in routes['right_arm_j2']
    with pytest.raises(ValueError, match='another teleop profile'):
        core(tmp_path, asset='other-robot')


def test_profiles_with_the_same_routes_share_their_edits_in_one_folder(tmp_path):
    """The position-only and velocity feed-forward tuning profiles run from one folder: a sign verified under one
    holds under the other. Other routes, or a file saved before the routes' digest, stay the saving profile's."""
    from robot_platform.helm.teleop import load_route_edits, routes_sha256, save_route_edits
    path = tmp_path / 'teleop' / 'routes.json'
    profile = json.loads(TELEOP.read_text(encoding='utf-8'))
    twin = dict(profile, id='twin-teleop')
    other = dict(profile, id='other-teleop', routes=dict(profile['routes'], left_arm_j2=dict(
        profile['routes']['left_arm_j2'], channel=5)))
    edits = {'left_arm_j2': {'sign': 1, 'verified': True, 'history': [{'action': 'flip'}]}}
    save_route_edits(path, profile, 'minerva-production-01', edits)
    assert json.loads(path.read_text(encoding='utf-8'))['routes_sha256'] == routes_sha256(twin)
    assert load_route_edits(path, twin, 'minerva-production-01') == edits
    save_route_edits(path, twin, 'minerva-production-01', {**edits, 'left_arm_j3': {'sign': -1, 'verified': True}})
    assert set(load_route_edits(path, profile, 'minerva-production-01')) == {'left_arm_j2', 'left_arm_j3'}
    with pytest.raises(ValueError, match='another teleop profile'):
        load_route_edits(path, other, 'minerva-production-01')
    legacy = json.loads(path.read_text(encoding='utf-8'))
    del legacy['routes_sha256']
    path.write_text(json.dumps(legacy), encoding='utf-8')
    assert set(load_route_edits(path, twin, 'minerva-production-01')) == {'left_arm_j2', 'left_arm_j3'}  # Its own.
    with pytest.raises(ValueError, match='another teleop profile'):
        load_route_edits(path, profile, 'minerva-production-01')


def test_trigger_edits_from_the_released_opens_mapping_are_dropped(tmp_path):
    path = tmp_path / 'teleop' / 'routes.json'
    path.parent.mkdir()
    path.write_text(json.dumps({'schema': 1, 'profile': 'minerva-production-01-teleop',
                                'asset_id': 'minerva-production-01', 'runtime_approved': False, 'routes': {
                                    'left_arm_j3': {'sign': -1, 'verified': True},
                                    'left_gripper': {'squeeze_sign': -1, 'verified': True}}}))
    edited = core(tmp_path)
    routes = edited.routes()
    assert routes['left_arm_j3'] == {'leader': 'left', 'channel': 3, 'sign': -1, 'verified': True}
    assert routes['left_gripper'] == {'leader': 'left', 'channel': 7, 'sign': 1}
    edited.route('accept', 'left_arm_j3')
    assert set(json.loads(path.read_text())['routes']) == {'left_arm_j3'}


class FakeCore:
    period_s = .01
    clock = staticmethod(time.perf_counter)

    def __init__(self, failure=None):
        self.failure, self.phase, self.fault = failure, 'starting', None
        self.keys, self.edits, self.closed = [], [], False

    def build(self):
        if self.failure:
            raise self.failure
        self.phase = 'running'

    def tick(self, key, started):
        if key:
            self.keys.append(key)

    def route(self, action, motor):
        if motor != 'left_arm_j1':
            raise ValueError('Use flip or accept with a routed motor')
        self.edits.append((action, motor))

    def set_zero(self):
        return {'left': {'channel_1': 1.5}}

    def view(self):
        return {'phase': self.phase, 'fault': self.fault, 'keys': list(self.keys)}

    def guide(self):
        return {'sequence': len(self.keys)} if self.phase == 'running' else None

    def close(self):
        self.closed = True
        if self.phase != 'fault':
            self.phase = 'stopped'

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason

    landing = False  # No arm to lower before closing.

    def land(self):
        return self.landing


def helm_stub(limits=True):
    return SimpleNamespace(profile_path=ROOT / 'config/devices/minerva-joint-calibration.json', bindings={},
                           mapping={}, saved_limits={'joints': {}} if limits else None,
                           asset_id='minerva-production-01')


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.01)


def test_session_relays_keys_and_route_edits_and_stops_cleanly(tmp_path):
    fake, stored = FakeCore(), []
    leaders = SimpleNamespace(saved=None, missing=lambda routes: [], store_reference=stored.append)
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, leaders=leaders, core_factory=lambda bundle: fake)
    with pytest.raises(ValueError, match='Start a teleop session'):
        session.key('I')
    session.start()
    try:
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        with pytest.raises(ValueError, match='already running'):
            session.start()
        session.key('I')
        session.key('E')
        with pytest.raises(ValueError, match='Use I, E, T, H or X'):
            session.key('Q')
        wait_until(lambda: session.snapshot()['keys'] == ['I', 'E'])
        assert session.guide() == {'sequence': 2}
        session.route('flip', 'left_arm_j1')
        assert fake.edits == [('flip', 'left_arm_j1')]
        assert session.set_zero() == {'left': {'channel_1': 1.5}}
        assert stored == [{'left': {'channel_1': 1.5}}]  # Saved through the Leaders workspace.
        with pytest.raises(ValueError, match='routed motor'):
            session.route('flip', 'nope')
        fake.fault = 'Guarded controller rejected request: wrong_owner'
        wait_until(lambda: session.snapshot()['fault'])
        with pytest.raises(ValueError, match='stop the session and start it again'):
            session.key('T')
        session.key('H')
        wait_until(lambda: fake.keys == ['I', 'E', 'H'])
    finally:
        session.stop()
    assert fake.closed and not session.active
    assert session.snapshot()['phase'] == 'stopped' and session.guide() is None
    with pytest.raises(ValueError, match='Start a teleop session'):
        session.key('I')


def test_status_names_the_guarded_profiles_interpolated_law(tmp_path):
    """The status row's Control cell for a profile without Motor tuning: its own law, next and while running."""
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, core_factory=lambda bundle: FakeCore())
    snapshot = session.snapshot()
    assert (snapshot['control_law'], snapshot['control_law_next']) == ('interpolated-feedforward', True)
    session.start()
    try:
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        snapshot = session.snapshot()
        assert (snapshot['control_law'], snapshot['control_law_next']) == ('interpolated-feedforward', False)
    finally:
        session.stop()


def test_session_reports_a_failed_start_and_needs_saved_limits(tmp_path):
    with pytest.raises(ValueError, match='Save joint limits'):
        TeleopSession(helm_stub(limits=False), TELEOP, runs=tmp_path).start()
    unswept = SimpleNamespace(saved=None, missing=lambda routes: ['left gripper trigger'])
    with pytest.raises(ValueError, match='Sweep and save the leader ranges .* missing: left gripper trigger'):
        TeleopSession(helm_stub(), TELEOP, runs=tmp_path, leaders=unswept).start()
    fake = FakeCore(RuntimeError('Could not open the leader arms'))
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, core_factory=lambda bundle: fake)
    session.start()
    wait_until(lambda: not session.active)
    wait_until(lambda: session.snapshot()['phase'] == 'fault')
    assert session.snapshot()['fault'] == 'Could not open the leader arms' and fake.closed
    session.stop()


class FakeTeleop:
    def __init__(self, pose):
        self.pose, self.active, self.calls = pose, False, []

    def start(self, *, confirm=False):
        self.calls.append('start')
        self.active = True

    def stop(self):
        self.calls.append('stop')
        self.active = False

    def key(self, key):
        self.calls.append(key)

    def route(self, action, motor):
        self.calls.append((action, motor))

    def set_zero(self):
        self.calls.append('zero')

    def snapshot(self):
        return {'phase': 'running' if self.active else 'stopped'}

    def guide(self):
        return {**self.pose(), 'badge': 'VIRTUAL FOLLOWER'} if self.active else None


class FakeLink:
    running = False

    def connect(self):
        self.running = True

    def disconnect(self):
        self.running = False


def test_web_endpoints_drive_teleop_and_take_the_motors_from_the_reader(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.session import HelmSession
    session = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=tmp_path / 'limits.json')
    teleop, link = FakeTeleop(session.guide), FakeLink()

    async def scenario(client):
        headers = {'X-Helm-Token': 'secret'}
        assert (await (await client.get('/api/calibration-guide')).json()).get('badge') != 'VIRTUAL FOLLOWER'
        assert (await client.post('/api/helm/teleop/start', json={})).status == 403
        state = await (await client.post('/api/helm/teleop/start', json={}, headers=headers)).json()
        assert state['teleop'] == {'phase': 'running'}
        assert (await (await client.get('/api/calibration-guide')).json())['badge'] == 'VIRTUAL FOLLOWER'
        refused = await client.post('/api/helm/connect', json={}, headers=headers)
        assert refused.status == 422 and not link.running
        assert (await client.post('/api/helm/teleop/key', json={'key': 'T'}, headers=headers)).status == 200
        routed = await client.post('/api/helm/teleop/route', json={'action': 'flip', 'motor': 'left_arm_j1'},
                                   headers=headers)
        assert routed.status == 200
        assert (await client.post('/api/helm/teleop/zero', json={}, headers=headers)).status == 200
        assert (await client.post('/api/helm/teleop/key', json={'key': 'T', 'x': 1}, headers=headers)).status == 422
        await client.post('/api/helm/teleop/stop', json={}, headers=headers)
        await client.post('/api/helm/connect', json={}, headers=headers)
        assert link.running
        # Teleop takes the motors from the reader rather than asking for Disconnect first.
        state = await (await client.post('/api/helm/teleop/start', json={}, headers=headers)).json()
        assert not link.running and teleop.calls == ['start', 'T', ('flip', 'left_arm_j1'), 'zero', 'stop', 'start']

    async def run():
        async with TestClient(TestServer(create_app(session, link, teleop=teleop, token='secret'))) as client:
            await scenario(client)
    try:
        asyncio.run(run())
    finally:
        session.close()


@pytest.mark.skipif(sys.platform != 'win32', reason='Windows scheduling hints')
def test_scheduling_hints_apply_to_another_process():
    import ctypes
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        assert prefer_responsive(child.pid) == ['no_power_throttling', 'high_priority']
        kernel32 = ctypes.WinDLL('kernel32')
        kernel32.OpenProcess.restype = ctypes.c_void_p
        handle = kernel32.OpenProcess(0x1000, False, child.pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        try:
            assert kernel32.GetPriorityClass(ctypes.c_void_p(handle)) == 0x80
        finally:
            kernel32.CloseHandle(ctypes.c_void_p(handle))
    finally:
        child.kill()
        child.wait()


@pytest.mark.skipif(sys.platform != 'win32', reason='Windows scheduling hints')
def test_scheduling_hints_applied_to_itself_also_get_a_1ms_timer():
    # Run in a child: the timer resolution lasts for the life of the process that asks.
    code = ('from robot_platform.control.scheduling import prefer_responsive; '
            'print(",".join(prefer_responsive()))')
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60,
                            env={**os.environ, 'PYTHONPATH': str(Path(__file__).resolve().parents[2] / 'src')})
    assert result.stdout.strip().split(',') == ['no_power_throttling', 'high_priority', 'timer_1ms'], result.stderr


def test_scheduling_hints_are_a_no_op_elsewhere(monkeypatch):
    monkeypatch.setattr(sys, 'platform', 'linux')
    assert prefer_responsive() == []


def test_a_silent_helm_ends_the_loop_after_lowering_the_arm():
    """helm.edge: a dropped tether sends no end of stream, so the robot's loop ends a session that hears nothing
    for `silence_s`, as if Helm had closed it; an engaged arm is lowered first, and the reason is logged."""
    from robot_platform.helm.edge import Inbox
    from robot_platform.helm.teleop import run_loop

    class Lowering(FakeCore):
        def __init__(self):
            super().__init__()
            self.lowering, self.gone = 0, []

        def land(self):
            self.lowering = 3  # Cycles to lower the arm.
            return True

        @property
        def landing(self):
            return self.lowering > 0

        def tick(self, key, started):
            super().tick(key, started)
            self.lowering = max(0, self.lowering - 1)

        def helm_gone(self, reason):
            self.gone.append(reason)

    core, commands, sent = Lowering(), Inbox(), []
    commands.put(['beat'])
    started = time.monotonic()
    run_loop(commands, SimpleNamespace(send=sent.append, close=lambda: None), core, silence_s=.2)
    assert .2 <= time.monotonic() - started < 2.
    assert core.gone == ['Helm went silent for 0.2 s'] and core.lowering == 0 and core.closed
