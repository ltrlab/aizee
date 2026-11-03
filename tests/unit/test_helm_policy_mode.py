"""Policy mode through Helm: the session carries the policy (and, to the robot, the capture profile's cameras instead
of a capture process), the robot side takes the source as Helm sent it, and the route starts it."""
import asyncio
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm import edge
from robot_platform.helm.edge import Inbox, serve
from robot_platform.helm.server import create_app
from robot_platform.helm.teleop import TeleopSession

ROOT = Path(__file__).resolve().parents[2]
TELEOP = ROOT / 'config/devices/minerva-teleop.json'
CAPTURE = ROOT / 'config/devices/minerva-task-capture.json'


class Core:
    period_s = .01
    clock = staticmethod(time.perf_counter)
    made = []

    def __init__(self, bundle, **options):
        self.bundle, self.options, self.phase, self.fault, self.scheduling, self.closed = bundle, options, 'starting', None, {}, False
        Core.made.append(self)

    def build(self):
        self.phase = 'running'

    def tick(self, key, started):
        pass

    def view(self):
        return {'phase': self.phase, 'fault': self.fault, 'message': '', 'joints': []}

    def guide(self):
        return None

    def land(self):
        return False

    def helm_gone(self, reason):
        pass

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.01)


def helm_stub():
    return SimpleNamespace(profile_path=ROOT / 'config/devices/minerva-joint-calibration.json', bindings={},
                           mapping={}, saved_limits={'joints': {}}, asset_id='minerva-production-01')


def test_the_robot_side_takes_a_policy_source_as_helm_sent_it(tmp_path, monkeypatch):
    asked = []

    def prepare(start, root, *, leaders=True):
        asked.append(leaders)
        return dict(start['bundle']), None
    monkeypatch.setattr(edge, 'prepare', prepare)
    sent, commands = [], Inbox()
    commands.end('Helm closed the channel')
    cameras = json.loads(CAPTURE.read_text(encoding='utf-8'))
    start = {'bundle': {'source': {'kind': 'policy', 'policy': 'hold_still'}, 'policy_cameras': cameras},
             'key': '00' * 32, 'bind': '127.0.0.1'}
    assert serve(start, commands, SimpleNamespace(send=sent.append, close=lambda: None), core_factory=Core,
                 ups_state=tmp_path / 'no-ups.json') == 0
    core = Core.made[-1]
    assert asked == [False] and sent[0] == ['link', 0]  # No leaders, as for a replay or a sweep.
    assert core.bundle['source'] == {'kind': 'policy', 'policy': 'hold_still'}  # No episode root added.
    assert core.bundle['policy_cameras'] == cameras and 'open_leaders' not in core.options


def test_the_robot_side_links_the_leaders_for_a_policy_session_with_takeover(tmp_path, monkeypatch):
    asked = []
    link = SimpleNamespace(address=('127.0.0.1', 4242), wait_for_pc=lambda timeout: None)

    def prepare(start, root, *, leaders=True):
        asked.append(leaders)
        return dict(start['bundle']), link
    monkeypatch.setattr(edge, 'prepare', prepare)
    sent, commands = [], Inbox()
    commands.end('Helm closed the channel')
    source = {'kind': 'policy', 'policy': 'hold_still', 'takeover': True}
    start = {'bundle': {'source': source}, 'key': '00' * 32, 'bind': '127.0.0.1'}
    assert serve(start, commands, SimpleNamespace(send=sent.append, close=lambda: None), core_factory=Core,
                 ups_state=tmp_path / 'no-ups.json') == 0
    core = Core.made[-1]
    assert asked == [True] and sent[0] == ['link', 4242]  # The leaders, to take over from the policy.
    assert core.bundle['source'] == source and core.options['open_leaders'](None) is link


def test_a_policy_session_carries_the_policy_and_on_the_robot_the_cameras_not_a_capture(tmp_path):
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, core_factory=lambda bundle: Core(bundle),
                            capture_profile=CAPTURE)
    session.task_capture_config = {'hz': 15}  # Task set up for recording: a policy session still records nothing.
    with pytest.raises(ValueError, match='replays an episode, runs a policy or follows the headset: one of them'):
        session.start(replay={'episode': 's/e'}, policy={'policy': 'hold_still'})
    session.start(policy={'policy': 'hold_still'})
    try:
        starting = session.snapshot()
        # The stand-in loop's first view has no policy entry and may already have replaced the starting one.
        assert starting.get('policy') in ({'spec': 'hold_still', 'policy': None, 'loading': True}, None) or starting['phase'] == 'running'
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        bundle = Core.made[-1].bundle
        assert bundle['source'] == {'kind': 'policy', 'policy': 'hold_still'}
        assert 'task_capture' not in bundle and 'policy_cameras' not in bundle  # On this PC: no robot cameras.
    finally:
        session.stop()

    sent = []

    class Robot:  # Stands in for the EdgeWorker: keeps what Helm would send the robot.
        def __init__(self, bundle, leader_profile, **options):
            sent.append((bundle, options))
            self.views, self.commands = Inbox(), SimpleNamespace(send=lambda message: None, close=lambda: None)

        def start(self):
            self.views.end('stand-in')

        def is_alive(self):
            return False

        def join(self, timeout=None):
            pass
    robot = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, target='jetson', edge_factory=Robot, capture_profile=CAPTURE)
    robot.task_capture_config = {'hz': 15}
    robot.start(policy={'policy': 'lerobot:/home/ltr/minerva/policies/act-001'})
    bundle, options = sent[-1]
    assert bundle['source'] == {'kind': 'policy', 'policy': 'lerobot:/home/ltr/minerva/policies/act-001'}
    assert 'task_capture' not in bundle and bundle['policy_cameras']['camera_roles'] == ['left_wrist', 'right_wrist', 'head']
    assert options['name'] == 'The policy'
    robot.stop()


def test_a_policy_session_with_takeover_needs_the_leaders_and_records_on_both_live_arms(tmp_path):
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, core_factory=lambda bundle: Core(bundle),
                            leaders=SimpleNamespace(saved=None, missing=lambda routes: ['J1']))
    session.task_capture_config = {'hz': 15}
    session.receiver = {'mode': 'live', 'arm': 'both'}
    with pytest.raises(ValueError, match='missing: J1'):  # The leaders must be ready to take over.
        session.start(confirm=True, policy={'policy': 'hold_still', 'takeover': True})
    with pytest.raises(ValueError, match='names its policy'):
        session.start(confirm=True, policy={'policy': 'hold_still', 'takeover': 'yes'})
    session.leaders = SimpleNamespace(saved=None, missing=lambda routes: [])
    session.start(confirm=True, policy={'policy': 'hold_still', 'takeover': True})
    try:
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        bundle = Core.made[-1].bundle
        assert bundle['source'] == {'kind': 'policy', 'policy': 'hold_still', 'takeover': True}
        assert bundle['task_capture'] == {'hz': 15}  # Both live arms record a rollout, labelled by the capture.
    finally:
        session.stop()
    session.receiver = {'mode': 'live', 'arm': 'left'}  # One arm records nothing.
    session.start(confirm=True, policy={'policy': 'hold_still', 'takeover': True})
    try:
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        assert 'task_capture' not in Core.made[-1].bundle
    finally:
        session.stop()


def test_a_live_policy_session_runs_only_a_trained_checkpoint_or_hold_still(tmp_path):
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, core_factory=lambda bundle: Core(bundle))
    session.receiver = {'mode': 'live', 'arm': 'both'}
    with pytest.raises(ValueError, match='for the simulation'):
        session.start(confirm=True, policy={'policy': 'penetrating'})


class FakeTeleop:
    def __init__(self):
        self.active, self.starts = False, []

    def start(self, **options):
        self.starts.append(options)
        self.active = True

    def stop(self):
        self.active = False

    def snapshot(self):
        return {'phase': 'running' if self.active else 'stopped'}

    def guide(self):
        return None


def test_the_route_starts_a_policy_session(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.session import HelmSession
    helm = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=tmp_path / 'limits.json')
    teleop = FakeTeleop()

    async def run():
        app = create_app(helm, SimpleNamespace(running=False, disconnect=lambda: None), teleop=teleop, token='secret')
        async with TestClient(TestServer(app)) as client:
            headers = {'X-Helm-Token': 'secret'}
            both = await client.post('/api/helm/teleop/start', headers=headers,
                                     json={'replay': {'episode': 's/e'}, 'policy': {'policy': 'hold_still'}})
            assert both.status != 200 and 'one of them' in (await both.json())['error']
            started = await client.post('/api/helm/teleop/start', headers=headers,
                                        json={'confirm': True, 'policy': {'policy': 'hold_still'}})
            assert started.status == 200
            assert teleop.starts == [{'confirm': True, 'policy': {'policy': 'hold_still'}}]
    try:
        asyncio.run(run())
    finally:
        helm.close()
