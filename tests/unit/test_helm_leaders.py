"""Helm's Leaders workspace with scripted leader boards: sweep, confirm, save, reload."""
import asyncio
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm.leaders import LeaderWorkspace
from robot_platform.helm.server import create_app
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'


class Sim:
    def __init__(self):
        self.now = 1000.

    def __call__(self):
        return self.now


class Boards:
    """Both leaders; each poll is 20 ms of simulated time."""
    def __init__(self, profile, clock):
        self.clock, self.sequence, self.closed, self.skip = clock, 0, False, 0
        self.positions = {name: [1.] * len(item['joints']) for name, item in profile['leaders'].items()}

    def poll(self):
        if self.skip:
            self.skip -= 1
            raise LeaderSampleSkipped('Leader pair age or skew exceeded the input profile')
        self.sequence += 1
        self.clock.now += .02
        return {name: LeaderSample(tuple(values), (0,) * len(values), self.sequence, self.clock.now,
                                   self.clock.now, (0, 0, 1, 1), 1, 2) for name, values in self.positions.items()}

    def close(self):
        self.closed = True


def teleop_profile(tmp_path):
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'),
                   leader_ranges=str(tmp_path / 'leader-ranges.json'))
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    return path


@pytest.fixture
def workspace(tmp_path):
    clock = Sim()
    boards = {}

    def open_leaders(profile):
        boards['open'] = Boards(profile, clock)
        return boards['open']
    leaders = LeaderWorkspace(teleop_profile(tmp_path), labels={'left_arm_j1': 'Left shoulder swivel (J1)'},
                              evidence=tmp_path / 'evidence', open_leaders=open_leaders, clock=clock)
    leaders.poll_s = .001
    leaders.boards, leaders.sim = boards, clock
    yield leaders
    leaders.disconnect()


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.005)


def move(leaders, leader, index, value, step=.05):
    """Hand speed: one small step per reading (a 2 rad jump in 20 ms would be discarded)."""
    boards = leaders.boards['open']
    start = boards.positions[leader][index]
    count = max(1, math.ceil(abs(value - start) / step))
    for i in range(1, count + 1):
        boards.positions[leader][index] = start + (value - start) * i / count
        seen = boards.sequence
        wait_until(lambda: boards.sequence >= seen + 2)


def hold(leaders, leader, index, value, seconds=.4):
    leaders.boards['open'].positions[leader][index] = value
    until = leaders.sim.now + seconds
    wait_until(lambda: leaders.sim.now >= until)


def row(leaders, key):
    return next(r for r in leaders.snapshot()['rows'] if r['key'] == key)


def test_sweep_confirm_and_save_a_leader_range(workspace, tmp_path):
    leaders = workspace
    state = leaders.snapshot()
    assert state['state'] == 'disconnected' and state['total_channels'] == 14 and state['saved_channels'] == 0
    first = row(leaders, 'left.channel_1')
    assert first['drives'] == 'left_arm_j1' and first['drives_label'] == 'Left shoulder swivel (J1)'
    with pytest.raises(ValueError, match='while the leaders are connected'):
        leaders.act('confirm', 'left.channel_1')
    with pytest.raises(ValueError, match='connected and reading'):
        leaders.capture_reference()
    leaders.connect()
    wait_until(lambda: leaders.fresh())
    with pytest.raises(ValueError, match='already connected'):
        leaders.connect()
    move(leaders, 'left', 0, .2)
    hold(leaders, 'left', 0, .2)
    move(leaders, 'left', 0, 2.2)
    hold(leaders, 'left', 0, 2.2)
    leaders.boards['open'].skip = 3  # Late pairs are skipped, not fatal.
    hold(leaders, 'left', 6, 1.5, .1)
    assert row(leaders, 'left.channel_1')['span_rad'] == pytest.approx(2.)
    leaders.act('confirm', 'left.channel_1')
    with pytest.raises(ValueError, match='both ends'):
        leaders.act('confirm', 'left.channel_5')  # Never moved.
    assert leaders.snapshot()['pending'] == 1
    result = leaders.save()
    assert result['channels'] == 1 and Path(result['path']) == tmp_path / 'leader-ranges.json'
    state = leaders.snapshot()
    assert state['pending'] == 0 and state['saved_channels'] == 1 and state['skipped'] >= 3
    assert row(leaders, 'left.channel_1')['saved']['span_rad'] == pytest.approx(2.)
    assert len(list((tmp_path / 'evidence').glob('saved-*-leader-ranges.json'))) == 1
    routes = json.loads((DEVICES / 'minerva-teleop.json').read_text())['routes']
    missing = leaders.missing(routes)
    assert len(missing) == 14 and 'left gripper trigger' in missing and 'left shoulder swivel' not in missing
    assert 'the leader zero' in missing

    move(leaders, 'left', 0, 1.)  # The zero is every channel's reading now, saved at once.
    leaders.capture_reference()
    state = leaders.snapshot()
    assert state['pending'] == 0 and state['reference']['captured_at']
    assert row(leaders, 'left.channel_1')['reference'] == pytest.approx(1.)
    assert 'the leader zero' not in leaders.missing(routes)
    saved = json.loads((tmp_path / 'leader-ranges.json').read_text())
    assert saved['reference']['leaders']['left']['channel_1'] == pytest.approx(1.)
    assert saved['leaders']['left']['channel_1']['span_rad'] == pytest.approx(2.)  # Ranges kept.
    zero = {name: {f'channel_{i}': .5 for i in range(1, 8)} for name in ('left', 'right')}
    leaders.store_reference(zero)  # As a running teleop session saves it.
    assert json.loads((tmp_path / 'leader-ranges.json').read_text())['reference']['leaders'] == zero
    leaders.disconnect()
    assert leaders.boards['open'].closed and leaders.snapshot()['state'] == 'disconnected'

    reloaded = LeaderWorkspace(teleop_profile(tmp_path))
    assert reloaded.saved['leaders']['left']['channel_1']['span_rad'] == pytest.approx(2.)
    assert reloaded.snapshot()['saved_channels'] == 1
    assert reloaded.snapshot()['reference']['captured_at'] == reloaded.saved['reference']['captured_at']


def test_a_board_that_will_not_open_is_reported(tmp_path):
    def refuse(_profile):
        raise ValueError('Leader left is missing or ambiguous')
    leaders = LeaderWorkspace(teleop_profile(tmp_path), open_leaders=refuse)
    leaders.connect()
    wait_until(lambda: not leaders.running)
    state = leaders.snapshot()
    assert state['state'] == 'fault' and 'Close the leader viewer' in state['message']


class Session:
    active = False


def test_web_endpoints_drive_the_leaders_and_exclude_teleop(workspace, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.session import HelmSession
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=tmp_path / 'limits.json')
    teleop = SimpleNamespace(active=False, snapshot=lambda: {'phase': 'stopped'}, guide=lambda: None,
                             start=lambda: None, stop=lambda: None)
    link = SimpleNamespace(running=False, disconnect=lambda: None)
    leaders = workspace

    async def scenario(client):
        headers = {'X-Helm-Token': 'secret'}
        state = await (await client.get('/api/helm/state')).json()
        assert state['leaders']['state'] == 'disconnected'
        assert (await client.post('/api/helm/leaders/connect', json={})).status == 403
        await client.post('/api/helm/leaders/connect', json={}, headers=headers)
        wait_until(leaders.fresh)
        refused = await client.post('/api/helm/teleop/start', json={}, headers=headers)
        assert refused.status == 422 and 'Disconnect the leaders' in (await refused.json())['error']
        rejected = await client.post('/api/helm/leaders/range', json={'action': 'confirm', 'key': 'left.channel_1'},
                                     headers=headers)
        assert rejected.status == 422 and 'both ends' in (await rejected.json())['error']
        unknown = await client.post('/api/helm/leaders/range', json={'action': 'reset', 'key': 'left.channel_9'},
                                    headers=headers)
        assert unknown.status == 422
        state = await (await client.post('/api/helm/leaders/reference', json={}, headers=headers)).json()
        assert state['leaders']['reference']['captured_at'] and state['leaders']['pending'] == 0
        state = await (await client.post('/api/helm/leaders/disconnect', json={}, headers=headers)).json()
        assert state['leaders']['state'] == 'disconnected'
        teleop.active = True
        refused = await client.post('/api/helm/leaders/connect', json={}, headers=headers)
        assert refused.status == 422 and 'Stop the teleop session' in (await refused.json())['error']

    async def run():
        app = create_app(helm, link, teleop=teleop, leaders=leaders, token='secret')
        async with TestClient(TestServer(app)) as client:
            await scenario(client)
    try:
        asyncio.run(run())
    finally:
        helm.close()
