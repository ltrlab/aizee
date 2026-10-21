"""Replay's episode reader, library and API on episodes written by the real TaskEpisodeWriter; no robot access."""
import asyncio
from io import BytesIO
from pathlib import Path
import time

import numpy as np
from PIL import Image
import pytest

from robot_platform.data.task_capture import TaskEpisodeWriter
from robot_platform.helm.episode_worker import Episodes, resolve_episode
from robot_platform.helm.episodes import EpisodeBusy, EpisodeError, EpisodeLibrary, _series, local_transport
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')
ROLES = ('left_wrist', 'right_wrist', 'head')
LEVEL = {'left_wrist': 40, 'right_wrist': 120, 'head': 200}  # Each camera's flat grey, plus the frame index.


def record(directory, frames, *, times=None, finish=True):
    """An episode with flat images (camera level + frame index) and qpos[i][j] = i/100 + j."""
    # Appended faster than real time: a queue deep enough for the whole episode.
    writer = TaskEpisodeWriter(directory, NAMES, {'data_source': 'real', 'camera_roles': list(ROLES)}, hz=20.,
                               max_pending=64)
    times = times or [10. + .05 * i for i in range(frames)]
    for i, t in enumerate(times):
        writer.append(q=[i / 100 + j for j in range(14)], qcmd=[i / 100 + j + .001 for j in range(14)],
                      source_time=t, scene_time=None,
                      images={role: np.full((24, 32, 3), LEVEL[role] + i, dtype=np.uint8) for role in ROLES},
                      camera_times={role: t - .03 for role in ROLES}, task_state={'frame': i}, authority='human')
    if finish:
        return Path(writer.finish())
    # Interrupted like a failed live recording, once its frames are on disk: the partial file stays readable.
    deadline = time.monotonic() + 10.
    while writer.status()['written_frames'] < frames and time.monotonic() < deadline:
        time.sleep(.01)
    writer.abort('test: recording interrupted')
    writer.wait_closed(timeout=5.)
    return writer.partial_path


@pytest.fixture(scope='module')
def root(tmp_path_factory):
    root = tmp_path_factory.mktemp('evidence')
    older = root / 'live-20261020T235949Z' / 'episodes'
    newer = root / 'live-20261021T004507Z' / 'episodes'
    record(older, 6)
    record(newer, 12, times=[10., 10.05, 10.16, 10.2, 10.31, 10.4, 10.45, 10.6, 10.65, 10.7, 10.8, 10.85])
    record(newer, 5, finish=False)
    (root / 'tune-20261020T000000Z').mkdir()
    (newer / 'notes.txt').write_text('not an episode')
    (newer / 'task_episode_nothex.hdf5').write_bytes(b'not hdf5')
    return root


def episode_ids(root):
    return [entry['id'] for entry in Episodes(root).list()]


def jpeg_level(data):
    return float(np.asarray(Image.open(BytesIO(data)).convert('RGB')).mean())


def test_list_finds_only_episodes_newest_first_with_their_facts(root):
    listed = Episodes(root).list()
    assert len(listed) == 3
    complete = [entry for entry in listed if entry['complete']]
    partial = [entry for entry in listed if entry['partial']]
    assert len(complete) == 2 and len(partial) == 1 and not partial[0]['complete']
    assert listed == sorted(listed, key=lambda entry: entry['modified'], reverse=True)
    newer = next(entry for entry in complete if entry['session'] == 'live-20261021T004507Z')
    assert newer['frames'] == 12 and newer['duration'] == pytest.approx(.85)
    assert newer['roles'] == list(ROLES) and newer['error'] is None
    assert partial[0]['frames'] == 5 and partial[0]['name'].endswith('.partial')


def test_summary_series_and_frames_come_from_the_recorded_file(root):
    episodes = Episodes(root)
    episode = next(entry['id'] for entry in episodes.list() if entry['frames'] == 12)
    summary = episodes.summary(episode)
    assert summary['frames'] == 12 and summary['complete'] is True and summary['joints'] == list(NAMES)
    assert summary['duration'] == pytest.approx(.85) and summary['roles'] == list(ROLES)
    assert summary['image_shapes'] == {role: [24, 32, 3] for role in ROLES}
    series = _series(episodes.series(episode))
    assert series['t'][:4] == pytest.approx([0., .05, .16, .2], abs=1e-4)
    assert series['qpos'][3][5] == pytest.approx(5.03, abs=1e-4)
    assert series['qcmd'][3][5] == pytest.approx(5.031, abs=1e-4)
    assert series['action_valid'] == [True] * 11 + [False]
    frame = episodes.frame(episode, 7)
    assert frame['index'] == 7 and frame['t'] == pytest.approx(.6)
    for role in ROLES:
        assert jpeg_level(frame['images'][role]) == pytest.approx(LEVEL[role] + 7, abs=2)
    with pytest.raises(ValueError, match='outside'):
        episodes.frame(episode, 12)
    episodes.close()


@pytest.mark.parametrize('name', ['../x/task_episode_' + '0' * 32, 'live-20261021T004507Z/../../etc',
                                  'live-20261021T004507Z/task_episode_' + '0' * 31, 'live-20261021T004507Z',
                                  'live-20261021T004507Z/notes', 'tune-20261020T000000Z/task_episode_' + '0' * 32,
                                  None, 7])
def test_only_episode_names_below_the_root_resolve(root, name):
    with pytest.raises(ValueError):
        resolve_episode(root.resolve(), name)


class InProcess:
    """A worker connection without a process: counts requests, can be told to fail."""
    def __init__(self, root, log, fail=None):
        self.episodes, self.log, self.fail = Episodes(root), log, fail

    def reply(self, timeout):
        return {'hello': 1}

    def call(self, op, timeout, **fields):
        self.log.append((op, fields.get('index')))
        if self.fail and self.fail(op):
            raise EpisodeError('worker went away')
        if op == 'list':
            return self.episodes.list()
        if op == 'frame':
            return self.episodes.frame(fields['episode'], fields['index'])
        return getattr(self.episodes, op)(fields['episode'])

    def close(self):
        self.episodes.close()


def test_library_fetches_every_camera_of_a_frame_once_and_caches_episodes(root):
    log = []
    library = EpisodeLibrary(lambda: InProcess(root, log))
    try:
        episode = next(entry['id'] for entry in library.listing()['episodes'] if entry['frames'] == 12)
        data = library.episode(episode)
        assert data['summary']['frames'] == 12 and len(data['series']['qpos']) == 12
        assert library.episode(episode) is data
        images = {role: library.frame(episode, 3, role) for role in ROLES}
        assert all(jpeg_level(images[role]) == pytest.approx(LEVEL[role] + 3, abs=2) for role in ROLES)
        assert [op for op in log if op[0] == 'frame'] == [('frame', 3)]
        assert [op for op in log if op[0] in ('summary', 'series')] == [('summary', None), ('series', None)]
        assert library.complete(episode)
        with pytest.raises(KeyError):
            library.frame(episode, 3, 'chest')
    finally:
        library.close()


def test_a_busy_robot_gets_no_requests_but_cached_episodes_keep_playing(root):
    log, busy = [], [None]
    library = EpisodeLibrary(lambda: InProcess(root, log), busy=lambda: busy[0])
    try:
        listed = library.listing()['episodes']
        episode = next(entry['id'] for entry in listed if entry['frames'] == 12)
        other = next(entry['id'] for entry in listed if entry['frames'] == 6)
        library.episode(episode)
        library.frame(episode, 0, 'head')
        requests = len(log)
        busy[0] = 'Replay waits while a recording is running'
        assert library.listing() == {'episodes': listed, 'busy': busy[0], 'error': None}
        assert library.episode(episode)['summary']['frames'] == 12
        assert library.frame(episode, 0, 'left_wrist')
        with pytest.raises(EpisodeBusy, match='recording'):
            library.episode(other)
        with pytest.raises(EpisodeBusy):
            library.frame(episode, 1, 'head')
        assert len(log) == requests
    finally:
        library.close()


def test_a_failed_worker_is_dropped_and_the_next_request_starts_another(root):
    log, opened = [], []
    fail = [True]

    def transport():
        connection = InProcess(root, log, fail=lambda op: fail[0] and op == 'list')
        opened.append(connection)
        return connection
    library = EpisodeLibrary(transport)
    try:
        assert library.listing() == {'episodes': [], 'busy': None, 'error': 'worker went away'}
        fail[0] = False
        assert len(library.listing()['episodes']) == 3
        assert len(opened) == 2
    finally:
        library.close()


def test_the_worker_process_answers_over_its_framed_pipe(root):
    library = EpisodeLibrary(local_transport(root))
    try:
        listed = library.listing()
        assert listed['error'] is None and len(listed['episodes']) == 3
        episode = next(entry['id'] for entry in listed['episodes'] if entry['frames'] == 12)
        assert library.episode(episode)['series']['t'][-1] == pytest.approx(.85, abs=1e-4)
        assert jpeg_level(library.frame(episode, 11, 'head')) == pytest.approx(LEVEL['head'] + 11, abs=2)
        with pytest.raises(EpisodeError, match='No such episode'):
            library.episode('live-20261021T004507Z/task_episode_' + 'a' * 32)
        # A refused request leaves the worker running for the next one.
        assert jpeg_level(library.frame(episode, 10, 'left_wrist')) == pytest.approx(LEVEL['left_wrist'] + 10, abs=2)
    finally:
        library.close()


class Teleop:
    active = False

    def __init__(self):
        self.recording = {'active': False, 'state': 'complete'}

    def snapshot(self):
        return {'task': {'recording': self.recording}}

    def stop(self):
        pass


class Link:
    running = False

    def disconnect(self):
        pass


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    yield session
    session.close()


def test_the_api_lists_serves_and_caches_episodes_and_waits_for_a_recording(root, helm):
    from aiohttp.test_utils import TestClient, TestServer
    teleop = Teleop()
    library = EpisodeLibrary(lambda: InProcess(root, []))

    async def scenario():
        async with TestClient(TestServer(create_app(helm, Link(), teleop=teleop, token='secret',
                                                    episodes=library))) as client:
            listed = await (await client.get('/api/helm/episodes')).json()
            complete = next(entry for entry in listed['episodes'] if entry['frames'] == 12)
            partial = next(entry for entry in listed['episodes'] if entry['partial'])
            session, name = complete['id'].split('/')
            data = await (await client.get(f'/api/helm/episodes/{session}/{name}')).json()
            assert data['summary']['frames'] == 12 and len(data['series']['t']) == 12
            frame = await client.get(f'/api/helm/episodes/{session}/{name}/frame/4/right_wrist')
            assert frame.status == 200 and frame.content_type == 'image/jpeg'
            assert 'immutable' in frame.headers['Cache-Control']
            assert jpeg_level(await frame.read()) == pytest.approx(LEVEL['right_wrist'] + 4, abs=2)
            p_session, p_name = partial['id'].split('/')
            await client.get(f'/api/helm/episodes/{p_session}/{p_name}')
            partial_frame = await client.get(f'/api/helm/episodes/{p_session}/{p_name}/frame/0/head')
            assert partial_frame.status == 200 and partial_frame.headers['Cache-Control'] == 'no-store'
            bad = await client.get(f'/api/helm/episodes/{session}/{name}/frame/x/head')
            assert bad.status == 422 and 'whole number' in (await bad.json())['error']
            missing = await client.get(f'/api/helm/episodes/{session}/{name}/frame/4/chest')
            assert missing.status == 404
            teleop.recording = {'active': True, 'state': 'recording'}
            await asyncio.sleep(.3)  # Past the busy check's cache.
            waiting = await client.get(f'/api/helm/episodes/{session}/{name}/frame/5/head')
            assert waiting.status == 409 and (await waiting.json())['busy'] is True
            cached = await client.get(f'/api/helm/episodes/{session}/{name}/frame/4/head')
            assert cached.status == 200
            busy_list = await (await client.get('/api/helm/episodes')).json()
            assert busy_list['busy'] == 'Replay waits while a recording is running'
            assert busy_list['episodes'] == listed['episodes']
    try:
        asyncio.run(scenario())
    finally:
        library.close()


def test_without_a_library_replay_says_it_is_not_configured(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def scenario():
        async with TestClient(TestServer(create_app(helm, Link(), token='secret'))) as client:
            response = await client.get('/api/helm/episodes')
            assert response.status == 422 and 'not configured' in (await response.json())['error']
    asyncio.run(scenario())


def test_the_worker_source_needs_no_installed_package():
    """Helm sends the worker's source over SSH to a robot that may run another release: no package imports."""
    source = (ROOT / 'src/robot_platform/helm/episode_worker.py').read_text(encoding='utf-8')
    assert 'robot_platform' not in source.split('"""', 2)[2]
