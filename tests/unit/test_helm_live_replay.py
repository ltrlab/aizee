"""Live replay through Helm: the robot side binds no leader link and reads its own recordings, the session builds a
replay bundle (never recorded, no leaders asked for), and the web route checks the episode before it starts."""
import asyncio
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
EPISODE = 'live-20261022T200000Z/task_episode_' + 'a' * 32


class Core:
    """Records what the loop was built with; a replay core is given no leaders."""
    period_s = .01
    clock = staticmethod(time.perf_counter)
    made = []

    def __init__(self, bundle, **options):
        self.bundle, self.options, self.phase, self.fault = bundle, options, 'starting', None
        self.scheduling, self.gone, self.closed = {}, [], False
        Core.made.append(self)

    def build(self):
        self.phase = 'running'

    def tick(self, key, started):
        pass

    def view(self):
        return {'phase': self.phase, 'fault': self.fault, 'message': '', 'joints': [],
                'replay': {'episode': (self.bundle.get('source') or {}).get('episode'), 'time_s': 0.}}

    def guide(self):
        return None

    def land(self):
        return False

    def helm_gone(self, reason):
        self.gone.append(reason)

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.01)


def test_a_replay_on_the_robot_binds_no_leader_link_and_reads_the_robots_own_recordings(tmp_path, monkeypatch):
    asked = []

    def prepare(start, root, *, leaders=True):
        asked.append(leaders)
        return dict(start['bundle']), None
    monkeypatch.setattr(edge, 'prepare', prepare)
    sent, commands = [], Inbox()
    commands.end('Helm closed the channel')
    views = SimpleNamespace(send=sent.append, close=lambda: None)
    start = {'bundle': {'source': {'kind': 'episode', 'episode': EPISODE, 'speed': .5, 'root': 'C:/on/the/pc'}},
             'key': '00' * 32, 'bind': '127.0.0.1'}
    assert serve(start, commands, views, core_factory=Core, ups_state=tmp_path / 'no-ups.json') == 0
    core = Core.made[-1]
    assert asked == [False] and sent[0] == ['link', 0]  # No leaders: port 0, as for a servo sweep.
    assert 'open_leaders' not in core.options
    assert core.bundle['source'] == {'kind': 'episode', 'episode': EPISODE, 'speed': .5, 'root': None}
    assert core.closed and core.gone == ['Helm closed the channel']


def helm_stub():
    return SimpleNamespace(profile_path=ROOT / 'config/devices/minerva-joint-calibration.json', bindings={},
                           mapping={}, saved_limits={'joints': {}}, asset_id='minerva-production-01')


def test_a_replay_session_carries_the_episode_never_records_and_asks_for_no_leaders(tmp_path):
    unswept = SimpleNamespace(saved=None, missing=lambda routes: ['left gripper trigger'])
    episodes = tmp_path / 'evidence'
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, leaders=unswept, episodes=episodes,
                            core_factory=lambda bundle: Core(bundle))
    with pytest.raises(ValueError, match='Sweep and save the leader ranges'):
        session.start()  # Teleop still needs the leaders,
    session.task_capture_config = {'hz': 15}  # and once the task is set up for recording, both arms live.
    with pytest.raises(ValueError, match='Configured task capture requires Live both arms'):
        session.start()
    with pytest.raises(ValueError, match='Replay runs at'):
        session.start(replay={'episode': EPISODE, 'speed': 2})
    session.start(replay={'episode': EPISODE, 'speed': .25})
    try:
        starting = session.snapshot()
        assert starting['replay']['episode'] == EPISODE or starting['phase'] == 'running'
        wait_until(lambda: session.snapshot()['phase'] == 'running')
        bundle = Core.made[-1].bundle
        assert bundle['source'] == {'kind': 'episode', 'episode': EPISODE, 'speed': .25, 'root': str(episodes.resolve())}
        assert 'task_capture' not in bundle
        assert session.snapshot()['replay']['episode'] == EPISODE
    finally:
        session.stop()


def test_a_replay_with_the_loop_on_this_pc_needs_a_local_copy_of_the_episodes(tmp_path):
    session = TeleopSession(helm_stub(), TELEOP, runs=tmp_path, target='pc')
    with pytest.raises(ValueError, match='local copy .*--episodes'):
        session.start(replay={'episode': EPISODE})
    with pytest.raises(ValueError, match='An episode is named'):
        session.start(replay={'episode': 'no-session'})


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


class FakeLink:
    running = False

    def disconnect(self):
        pass


def test_the_web_route_starts_a_replay_only_of_a_complete_episode_it_can_read(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.episodes import EpisodeError
    from robot_platform.helm.session import HelmSession
    helm = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=tmp_path / 'limits.json')
    teleop = FakeTeleop()
    summaries = {EPISODE: {'complete': True}, EPISODE[:-1] + 'b': {'complete': False}}

    def episode(name):
        if name not in summaries:
            raise EpisodeError(f'No such episode: {name}')
        return {'summary': summaries[name], 'series': {}}
    library = SimpleNamespace(episode=episode, close=lambda: None)

    async def scenario(client):
        headers = {'X-Helm-Token': 'secret'}

        async def start(payload):
            response = await client.post('/api/helm/teleop/start', json=payload, headers=headers)
            return response.status, await response.json()
        status, body = await start({'replay': {'episode': EPISODE[:-1] + 'b', 'speed': .5}})
        assert status != 200 and 'Only a complete episode' in body['error']
        status, body = await start({'replay': {'episode': EPISODE[:-1] + 'c'}})
        assert status != 200 and 'could not be read: No such episode' in body['error']
        status, body = await start({'replay': {'episode': EPISODE, 'speed': 3}})
        assert status != 200 and 'Replay runs at' in body['error']
        assert teleop.starts == []
        status, _ = await start({'confirm': True, 'replay': {'episode': EPISODE, 'speed': .5}})
        assert status == 200
        assert teleop.starts == [{'confirm': True, 'replay': {'episode': EPISODE, 'speed': .5}}]
        teleop.stop()
        status, _ = await start({})
        assert status == 200 and teleop.starts[-1] == {'confirm': False}  # Teleop as before: no replay passed.

    async def run():
        async with TestClient(TestServer(create_app(helm, FakeLink(), teleop=teleop, token='secret',
                                                    episodes=library))) as client:
            await scenario(client)
    try:
        asyncio.run(run())
    finally:
        helm.close()


def test_without_a_replay_library_the_route_says_so(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.session import HelmSession
    helm = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=tmp_path / 'limits.json')
    teleop = FakeTeleop()

    async def run():
        async with TestClient(TestServer(create_app(helm, FakeLink(), teleop=teleop, token='secret'))) as client:
            response = await client.post('/api/helm/teleop/start', json={'replay': {'episode': EPISODE}},
                                         headers={'X-Helm-Token': 'secret'})
            assert response.status != 200 and 'Replay is not configured' in (await response.json())['error']
            assert teleop.starts == []
    try:
        asyncio.run(run())
    finally:
        helm.close()
