"""Helm session, web API and live link, without hardware."""
import asyncio
import json
from pathlib import Path
import threading

import pytest

from robot_platform.helm.link import LiveLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession, acquire_lock

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'


class Clock:
    def __init__(self):
        self.now = 1000.

    def __call__(self):
        return self.now


@pytest.fixture
def helm(tmp_path):
    clock = Clock()
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', seed=SEED, clock=clock)
    session.test_clock = clock
    yield session
    session.close()


def frame(session, sequence, positions=None, stream='stream-1'):
    positions = positions or {}
    return {'kind': 'positions', 'request_id': stream, 'sequence': sequence,
            'asset_id': session.asset_id, 'scan_seconds': .28, 'roundtrip_seconds': .3,
            'enable_requested': False, 'parameter_writes_requested': False,
            'motors': [{**{k: m[k] for k in ('name', 'bus', 'id', 'uid', 'firmware')},
                        'position_rad': positions.get(m['name'], 0.), 'reset_confirmed': True}
                       for m in session.reference['motors']]}


def sweep(session, motor, values, start=1):
    for sequence, q in enumerate(values, start):
        session.on_frame(frame(session, sequence, {motor: q}))
    return start + len(values)


class FakeLink:
    def __init__(self):
        self.running = False
        self.calls = []

    def connect(self):
        self.calls.append('connect')
        self.running = True

    def disconnect(self):
        self.calls.append('disconnect')
        self.running = False


def run_client(session, link, scenario):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(session, link, token='secret'))) as client:
            return await scenario(client)
    return asyncio.run(run())


def test_session_starts_disconnected_with_every_motor_and_seeded_directions(helm):
    state = helm.snapshot()
    assert state['feed']['state'] == 'disconnected' and not state['feed']['fresh']
    assert len(state['limits']['rows']) == len(helm.reference['motors']) == 14
    rows = {row['name']: row for row in state['limits']['rows']}
    assert rows['left_arm_j1']['direction'] == dict(joint='left_arm_j1', sign=1, verified=True)
    assert rows['right_arm_j1']['direction']['sign'] == -1
    assert rows['left_gripper']['direction']['editable'] is False
    guide = helm.guide()
    assert guide['live'] is False and 'NOT CONNECTED' in guide['badge']


def test_live_frames_drive_the_model_and_record_limits(helm):
    sweep(helm, 'left_arm_j2', [0., -.6, -.6, .9, .9])
    guide = helm.guide()
    assert guide['live'] and guide['positions']['left_arm_j2'] == pytest.approx(.9)
    row = next(r for r in helm.snapshot()['limits']['rows'] if r['name'] == 'left_arm_j2')
    assert (row['min_rad'], row['max_rad']) == (pytest.approx(-.6), pytest.approx(.9))
    assert (helm.evidence / 'frames.jsonl').read_text().count('\n') == 5


def test_confirm_and_save_writes_a_persistent_bound_calibration(helm, tmp_path):
    helm.link_state('reading', 'live')
    sweep(helm, 'left_arm_j2', [0., -.6, -.6, .9, .9])
    helm.limits('confirm', 'left_arm_j2')
    result = helm.save_limits()
    saved = json.loads((tmp_path / 'limits.json').read_text())
    assert result['joints'] == 1 and saved['joints']['left_arm_j2']['max_rad'] == pytest.approx(.9)
    assert saved['reference'] == helm.reference['id'] and saved['reference_sha256'] == helm.bindings['reference']
    assert saved['joints']['left_arm_j2']['uid'] == next(m['uid'] for m in helm.reference['motors'] if m['name'] == 'left_arm_j2')
    assert helm.snapshot()['limits']['pending'] == 0
    helm.close()
    reopened = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    try:
        state = reopened.snapshot()['limits']
        assert state['saved_joints'] == 1 and state['saved_error'] is None
    finally:
        reopened.close()


def test_confirm_and_direction_edits_need_fresh_live_readings(helm):
    sweep(helm, 'left_arm_j2', [0., -.6, -.6, .9, .9])
    helm.test_clock.now += 5
    with pytest.raises(ValueError, match='connected'):
        helm.limits('confirm', 'left_arm_j2')
    with pytest.raises(ValueError, match='fresh'):
        helm.direction('flip', 'left_arm_j2')
    helm.test_clock.now -= 5
    helm.direction('flip', 'left_arm_j2')
    assert helm.mapping['left_arm_j2']['sign'] == -1 and not helm.mapping['left_arm_j2']['direction_verified']
    persisted = json.loads(helm.state_path.read_text())
    assert persisted['preview_edits'][-1]['command'] == 'flip left_arm_j2'
    assert persisted['preview_edits'][-1]['source'] == 'helm'
    with pytest.raises(ValueError):
        helm.direction('flip', 'left_gripper')


def test_lock_is_exclusive_but_a_dead_owner_is_taken_over(tmp_path):
    path = tmp_path / 'session.lock'
    path.write_text('424242')
    with pytest.raises(RuntimeError, match='PID 424242'):
        acquire_lock(path, alive=lambda pid: True)
    handle, stale = acquire_lock(path, alive=lambda pid: False)
    handle.close()
    assert stale == 424242 and path.read_text().isdigit()
    path.write_text('not-a-pid')
    with pytest.raises(RuntimeError, match='no process id'):
        acquire_lock(path)


def test_second_helm_on_the_same_session_is_refused(helm, tmp_path):
    with pytest.raises(RuntimeError, match='Another calibration process'):
        HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')


def test_web_api_requires_the_token_and_drives_the_same_session(helm):
    link = FakeLink()

    async def scenario(client):
        page = await client.get('/')
        assert page.status == 200 and 'Helm' in await page.text()
        assert (await client.get('/viewer')).status == 200
        state = await (await client.get('/api/helm/state')).json()
        assert state['action_token'] == 'secret' and state['link'] == {'running': False}
        forbidden = await client.post('/api/helm/connect', json={})
        assert forbidden.status == 403 and link.calls == []
        headers = {'X-Helm-Token': 'secret'}
        state = await (await client.post('/api/helm/connect', json={}, headers=headers)).json()
        assert state['link']['running'] and link.calls == ['connect']
        # The first guide request loads the viewer's model; the freshness check below
        # allows only 0.45 s between the last frame and the request.
        await client.get('/api/calibration-guide')
        helm.link_state('reading', 'live')
        sweep(helm, 'right_arm_j4', [0., -.4, -.4, .7, .7])
        guide = await (await client.get('/api/calibration-guide')).json()
        assert guide['live'] and not guide['stale']
        rejected = await client.post('/api/helm/limits', json={'action': 'confirm', 'motor': 'right_arm_j5'}, headers=headers)
        assert rejected.status == 422 and 'Swept range is only 0.000' in (await rejected.json())['error']
        state = await (await client.post('/api/helm/limits', json={'action': 'confirm', 'motor': 'right_arm_j4'},
                                         headers=headers)).json()
        assert state['limits']['pending'] == 1
        saved = await client.post('/api/helm/limits/save', json={}, headers=headers)
        assert saved.status == 200 and (await saved.json())['limits']['saved_joints'] == 1
        state = await (await client.post('/api/helm/disconnect', json={}, headers=headers)).json()
        assert not state['link']['running'] and link.calls == ['connect', 'disconnect']

    run_client(helm, link, scenario)


def test_the_connection_is_the_tether_until_wifi_is_chosen_and_is_locked_while_connected(helm):
    link = FakeLink()
    wifi = helm.device['address']

    async def scenario(client):
        headers = {'X-Helm-Token': 'secret'}
        state = await (await client.get('/api/helm/state')).json()
        assert state['robot']['connection'] == 'tether' and state['robot']['address'] == '10.42.0.1'
        assert state['robot']['connections'] == [{'key': 'tether', 'address': '10.42.0.1'},
                                                 {'key': 'wifi', 'address': wifi}]
        state = await (await client.post('/api/helm/connection', json={'connection': 'wifi'}, headers=headers)).json()
        assert state['robot']['connection'] == 'wifi' and state['robot']['address'] == wifi and helm.address == wifi
        bad = await client.post('/api/helm/connection', json={'connection': 'bluetooth'}, headers=headers)
        assert bad.status == 422 and 'tether or Wi-Fi' in (await bad.json())['error']
        await client.post('/api/helm/connect', json={}, headers=headers)
        locked = await client.post('/api/helm/connection', json={'connection': 'tether'}, headers=headers)
        assert locked.status == 422 and 'Disconnect' in (await locked.json())['error'] and helm.connection == 'wifi'
        await client.post('/api/helm/disconnect', json={}, headers=headers)
        state = await (await client.post('/api/helm/connection', json={'connection': 'tether'},
                                         headers=headers)).json()
        assert state['robot']['address'] == '10.42.0.1'

    run_client(helm, link, scenario)


class FakeClient:
    """Stopped-reader transport stub following SshCapture.stream semantics.

    Each step is a report, or (frames delivered first, report). A failed
    preflight delivers no frames; a mid-stream dropout delivers some first.
    """
    def __init__(self, session, reports, frames=2):
        self.session, self.reports, self.frames = session, list(reports), frames
        self.closed = False
        self.streams = 0

    def stream(self, receive, stop):
        self.streams += 1
        stream_id = f'stream-{self.streams}'
        step = self.reports.pop(0)
        frames, report = step if isinstance(step, tuple) else (self.frames, step)
        for sequence in range(1, frames + 1):
            receive(frame(self.session, sequence, stream=stream_id))
        if report == 'wait':
            stop.wait(5)
            report = cleanup(self.session, 'client_stop')
        if isinstance(report, Exception):
            raise report
        return report

    def close(self):
        self.closed = True


def cleanup(session, reason=None, **change):
    buses = {m['bus'] for m in session.reference['motors']}
    report = {'success': True, 'asset_id': session.asset_id, 'identity_verified': True,
              'enable_requested': False, 'parameter_writes_requested': False,
              'after': {bus: {'flags': ['NOARP']} for bus in buses}, 'completion_reason': reason}
    report.update(change)
    return report


def wait_for(link):
    link._thread.join(5)
    assert not link.running


def test_link_delivers_frames_and_disconnect_confirms_cleanup(helm):
    client = FakeClient(helm, ['wait'])
    link = LiveLink(helm, lambda: client)
    link.connect()
    for _ in range(200):
        if helm.frame and helm.frame['sequence'] == 2:
            break
        threading.Event().wait(.01)
    assert helm.feed['state'] == 'reading' and helm.frame['sequence'] == 2
    link.disconnect()
    assert helm.feed['state'] == 'disconnected' and client.closed
    assert 'CAN links are down' in helm.feed['message']


def dropout(helm, *names, code='motors_unresponsive'):
    return cleanup(helm, success=False, identity_verified=None, error_code=code, unresponsive=list(names),
                   error='Not answering: ' + ', '.join(names))


def wait_until(condition, timeout=5.):
    for _ in range(int(timeout / .01)):
        if condition():
            return True
        threading.Event().wait(.01)
    return False


def test_link_waits_for_a_dropped_motor_and_resumes_when_it_answers(helm):
    # Mid-stream dropout, motor still off at the first retries, then back after a power cycle.
    client = FakeClient(helm, [(3, dropout(helm, 'left_arm_j5', code='incomplete_scans')),
                               (0, dropout(helm, 'left_arm_j5', 'left_arm_j6', 'left_gripper')),
                               (0, dropout(helm, 'left_arm_j5')), (2, 'wait')])
    states = []
    original = helm.link_state
    helm.link_state = lambda state, message, **kw: (states.append((state, message)), original(state, message, **kw))
    link = LiveLink(helm, lambda: client, retry_delays=(.01, .02))
    link.connect()
    assert wait_until(lambda: client.streams == 4 and helm.frame and helm.frame['request_id'] == 'stream-4')
    assert helm.feed['state'] == 'reading' and link.running
    waiting = [message for state, message in states if state == 'waiting' and message.startswith('Not answering')]
    assert len(waiting) == 3 and 'left_arm_j6, left_gripper' in waiting[1]
    assert 'Sweep progress is kept' in waiting[0]
    link.disconnect()
    assert helm.feed['state'] == 'disconnected' and link.reconnections == 3


def test_disconnect_while_waiting_stops_retrying(helm):
    client = FakeClient(helm, [(0, dropout(helm, 'left_arm_j5'))] * 50)
    link = LiveLink(helm, lambda: client, retry_delays=(30.,))
    link.connect()
    assert wait_until(lambda: helm.feed['state'] == 'waiting')
    link.disconnect()
    assert helm.feed['state'] == 'disconnected' and client.streams == 1 and client.closed


def test_link_renews_a_healthy_expiry(helm):
    client = FakeClient(helm, [cleanup(helm, 'duration_limit'), 'wait'])
    link = LiveLink(helm, lambda: client)
    link.connect()
    assert wait_until(lambda: client.streams == 2 and helm.frame['request_id'] == 'stream-2')
    link.disconnect()
    assert link.reconnections == 0


@pytest.mark.parametrize('report', ['uid', 'cleanup', 'transport', 'unknown'])
def test_link_stops_instead_of_retrying_anything_but_a_clean_dropout(helm, report):
    step = {'uid': cleanup(helm, success=False, error='Motor UID mismatch: left_arm_j5'),
            'cleanup': dropout(helm, 'left_arm_j5') | {'after': {'canL': {'flags': ['UP']}, 'canR': {'flags': []}}},
            'transport': OSError('SSH channel closed'),
            'unknown': {'success': False, 'error': 'Wrong host machine identity', 'enable_requested': False}}[report]
    client = FakeClient(helm, [(0, step)])
    link = LiveLink(helm, lambda: client, retry_delays=(.01,))
    link.connect()
    wait_for(link)
    assert client.streams == 1 and helm.feed['state'] == 'fault' and client.closed
    if report == 'transport':
        assert 'Cleanup was not confirmed' in helm.feed['message']
