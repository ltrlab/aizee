"""The remote headset's WebRTC link (helm.webrtc_rate, webrtc_worker, webrtc_link, the server's route): its rate rule, its
framing, what Helm accepts from the channel. The real peer needs aiortc, which only the robot's worker has: see
test_helm_webrtc_peer.py."""
import asyncio
import io
import json
from pathlib import Path
import sys
import time

import pytest

from robot_platform.helm.access import Access, save_password
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.webrtc_link import WebRTCLink
from robot_platform.helm.xr import MAX_RATE_HZ
from robot_platform.helm.webrtc_rate import (MAXIMUM_BPS, MINIMUM_BPS, RESEND_AGAIN_S, RESEND_BURST, RESEND_MINIMUM_PER_S,
                                             RateControl, ResendBudget)
from robot_platform.helm.webrtc_worker import (MAX_HEADER, MAX_PAYLOAD, LinkLog, candidates, page_report, read_messages,
                                               write_message)

ROOT = Path(__file__).resolve().parents[2]
JPEG = b'\xff\xd8 a frame \xff\xd9'
# Answers an offer, echoes a frame's sequence as a pose, and says when a status comes: the real worker's framing, without
# aiortc.
FAKE_WORKER = """
import json, sys
from robot_platform.helm.webrtc_worker import read_messages
for header, payload in read_messages(sys.stdin.buffer):
    kind = header['kind']
    if kind == 'offer':
        answer = {'kind': 'error', 'id': header['id'], 'error': 'refused'} if header['sdp'] == 'refuse' else {
            'kind': 'answer', 'id': header['id'], 'sdp': 'answer to ' + header['sdp'], 'type': 'answer'}
        print(json.dumps(answer), flush=True)
        print(json.dumps({'kind': 'state', 'state': 'connected'}), flush=True)
    elif kind == 'frame':
        print(json.dumps({'kind': 'pose', 'text': json.dumps({'frame': header['sequence'], 'bytes': len(payload)})}), flush=True)
    elif kind == 'status':
        print(json.dumps({'kind': 'pose', 'text': header['text']}), flush=True)
"""


def test_the_rate_follows_the_loss_rule():
    rate = RateControl(start=1_000_000)
    rate.loss(.2)  # Above 10 %: down by half the loss.
    assert rate.target == 900_000
    rate.loss(.05)  # Between: as it is.
    assert rate.target == 900_000
    rate.loss(.0)  # Below 2 %, and near the rate that lost too much: up slowly.
    assert rate.target == pytest.approx(904_500, abs=1)


def test_the_rate_climbs_freely_until_it_has_lost_and_again_once_the_link_has_changed():
    rate = RateControl(start=500_000)
    rate.loss(.0)
    assert rate.target == 525_000
    rate.loss(.5)  # Cut to 75 % of 525 000; the ceiling is 525 000.
    assert rate.ceiling == 525_000 and rate.target == 393_750
    rate.loss(.0)  # Under 85 % of the ceiling: still freely.
    assert rate.target == 413_437
    for _ in range(4):
        rate.loss(.0)
    assert rate.allowed > .85 * rate.ceiling
    held = rate.allowed
    rate.loss(.0)
    assert rate.allowed == pytest.approx(held * 1.005)
    for _ in range(60):
        rate.loss(.0)
    assert rate.ceiling is None  # A minute without loss: the link has changed.
    held = rate.allowed
    rate.loss(.0)
    assert rate.allowed == pytest.approx(held * 1.05)


def test_the_rate_stays_within_its_bounds_and_the_receivers_estimate():
    rate = RateControl()
    for _ in range(100):
        rate.loss(1.)
    assert rate.target == MINIMUM_BPS
    for _ in range(200):
        rate.loss(0.)
    assert rate.target == MAXIMUM_BPS
    rate.estimate(300_000)  # The lower of the two.
    assert rate.target == 300_000
    rate.estimate(50_000)  # A picture stays possible.
    assert rate.target == MINIMUM_BPS
    with pytest.raises(ValueError):
        rate.loss(1.5)
    with pytest.raises(ValueError):
        RateControl(start=1, minimum=2, maximum=3)


def test_resends_stay_within_a_share_of_the_rate_however_many_are_asked_for():
    """A 300 kbit/s cap: a browser asked for 600 packets a second, so nothing new got through."""
    now = [0.]
    rate = RateControl(start=120_000)
    budget = ResendBudget(rate, lambda: now[0])
    for second in range(1, 11):
        asked = 0
        while now[0] < second:
            now[0] = round(now[0] + 1 / 600, 6)
            asked += 1
            budget.allow(asked + 1000 * second)
    # At the lowest rates its least, and a first short burst.
    assert budget.resent <= RESEND_BURST + 10 * RESEND_MINIMUM_PER_S + 1
    assert budget.resent >= 10 * RESEND_MINIMUM_PER_S - 1
    assert budget.refused > 5000
    now[0] = 100.
    fast = ResendBudget(RateControl(start=1_500_000), lambda: now[0])
    sent = 0
    for packet in range(1000):  # At the top: half the rate's packets, 78 a second.
        now[0] += .002
        sent += fast.allow(packet)
    assert 78 * 2 * .9 < sent < 78 * 2 * 1.1 + RESEND_BURST


def test_a_packet_is_resent_again_only_after_about_a_round_trip():
    now = [0.]
    budget = ResendBudget(RateControl(start=1_000_000), lambda: now[0])
    assert budget.allow(7) is True
    now[0] += RESEND_AGAIN_S / 2
    assert budget.allow(7) is False
    assert budget.allow(8) is True  # Another packet is not held back by it.
    now[0] += RESEND_AGAIN_S
    assert budget.allow(7) is True


def test_framing_round_trips_and_is_bounded():
    stream = io.BytesIO()
    write_message(stream, {'kind': 'frame', 'sequence': 7}, JPEG)
    write_message(stream, {'kind': 'close'})
    stream.seek(0)
    assert list(read_messages(stream)) == [({'kind': 'frame', 'sequence': 7}, JPEG), ({'kind': 'close'}, b'')]
    with pytest.raises(ValueError):
        write_message(io.BytesIO(), {'kind': 'frame'}, b'x' * (MAX_PAYLOAD + 1))
    with pytest.raises(ValueError):
        write_message(io.BytesIO(), {'kind': 'x' * MAX_HEADER})
    with pytest.raises(ValueError):
        list(read_messages(io.BytesIO((MAX_HEADER + 1).to_bytes(4, 'big') + bytes(4))))
    cut = io.BytesIO()
    write_message(cut, {'kind': 'frame'}, JPEG)
    assert list(read_messages(io.BytesIO(cut.getvalue()[:-1]))) == []  # Helm went mid-message.


class Video:
    def __init__(self):
        self.viewers, self.sequence, self.closed = set(), 0, False

    def subscribe(self, depth=False):
        assert depth is False  # This link never asks for the head's depth.
        token = object()
        self.viewers.add(token)
        return token

    def unsubscribe(self, token):
        self.viewers.discard(token)

    def close(self):
        self.closed = True

    def snapshot(self):
        self.sequence += 1
        return ({'type': 'status', 'cameras': {}},
                {'head': ({'role': 'head', 'sequence': self.sequence, 'age_s': .02, 'source_timestamp': time.time()}, JPEG),
                 'left_wrist': ({'role': 'left_wrist', 'sequence': self.sequence, 'age_s': .02,
                                 'source_timestamp': time.time()}, b'wrist')})


def fake_link(video, on_pose):
    link = WebRTCLink(video, on_pose, command=(sys.executable, '-c', FAKE_WORKER))
    link.available = lambda: True
    return link


async def until(condition, timeout=8.):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'Timed out'
        await asyncio.sleep(.01)


def test_the_link_log_keeps_an_sdps_candidates_and_the_pages_reports_apart_from_poses(tmp_path):
    assert candidates('v=0\r\na=candidate:1 1 udp 1 198.51.100.7 5000 typ host\r\nm=video 9\r\n') == [
        'a=candidate:1 1 udp 1 198.51.100.7 5000 typ host']
    assert page_report('{"type":"pose","seq":1}') is None
    assert page_report('{"type":"link","shown":3}') == {'kind': 'page', 'type': 'link', 'shown': 3}
    assert page_report('{"type":"link",broken')['unreadable'].startswith('{"type":"link"')  # Still not a pose.
    LinkLog()({'kind': 'state'})  # No folder: nothing, and no complaint.
    log = LinkLog(tmp_path / 'evidence', clock=lambda: 1000.)
    log({'kind': 'state', 'state': 'connected'})
    log({'kind': 'stats', 'frames_in': 3})
    deadline = time.monotonic() + 5.
    while time.monotonic() < deadline and not (log.path.is_file() and log.path.read_text().count('\n') == 2):
        time.sleep(.01)
    assert log.path.name == 'remote-link-19700101T001640Z.jsonl'
    assert [json.loads(line) for line in log.path.read_text().splitlines()] == [
        {'at': 1000., 'kind': 'state', 'state': 'connected'}, {'at': 1000., 'kind': 'stats', 'frames_in': 3}]


def test_link_sends_only_the_head_camera_and_passes_poses_back():
    async def run():
        video, poses = Video(), []
        link = fake_link(video, lambda text, owner: poses.append((json.loads(text), owner)))
        answer = await link.offer('my offer', 'offer', 'a login')
        assert answer == {'sdp': 'answer to my offer', 'type': 'answer'}
        assert len(video.viewers) == 1
        await until(lambda: len(poses) >= 2)
        assert all(pose == ({'frame': pose[0]['frame'], 'bytes': len(JPEG)}, 'a login') for pose in poses)
        await until(lambda: link.state == 'connected')
        link.status({'type': 'status', 'tracking': True})
        await until(lambda: any(pose[0].get('tracking') for pose in poses))
        await link.close()
        assert not video.viewers  # The relay is let go, and with no viewer stops.
        assert link.process is None or link.process.returncode is not None
    asyncio.run(run())


def test_a_refused_offer_is_an_error_and_a_second_offer_keeps_one_viewer():
    async def run():
        video = Video()
        link = fake_link(video, lambda text, owner: None)
        with pytest.raises(ValueError, match='refused'):
            await link.offer('refuse', 'offer', None)
        assert not video.viewers
        await link.offer('one', 'offer', None)
        await link.offer('two', 'offer', None)
        assert len(video.viewers) == 1
        await link.close()
    asyncio.run(run())


def test_no_webrtc_on_this_helm_says_so_and_starts_nothing():
    async def run():
        link = WebRTCLink(Video(), lambda text, owner: None)
        link.available = lambda: False
        with pytest.raises(ValueError, match='no WebRTC'):
            await link.offer('sdp', 'offer', None)
        assert link.process is None
        with pytest.raises(ValueError, match='not configured'):
            await WebRTCLink(None, lambda text, owner: None).offer('sdp', 'offer', None)
    asyncio.run(run())


def test_on_minerva_the_worker_alone_gets_its_dependencies(tmp_path, monkeypatch):
    """Their cryptography must not reach Helm's paramiko: the folder goes on the worker's path, not on Helm's."""
    from robot_platform.helm import webrtc_link
    link = WebRTCLink(Video(), lambda text, owner: None, command=('python',))
    monkeypatch.setenv(webrtc_link.DEPS, str(tmp_path))
    assert link.available() is False  # Not finished installing.
    (tmp_path / '.complete').write_text('done')
    assert link.available() is True
    seen = {}

    async def exec_stub(*_command, **options):
        seen.update(options['env'])
        raise ConnectionError('stop here')
    monkeypatch.setattr(webrtc_link.asyncio, 'create_subprocess_exec', exec_stub)
    with pytest.raises(ConnectionError):
        asyncio.run(link._start_worker())
    assert seen['PYTHONPATH'].split(webrtc_link.os.pathsep) == [str(tmp_path), str(webrtc_link.SOURCE)]
    assert str(tmp_path) not in sys.path


@pytest.fixture
def helm(tmp_path):
    return HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=tmp_path / 'limits', seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')


def test_offer_route_needs_the_token_and_a_real_offer(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        app = create_app(helm, SimulationLink(), token='secret')  # No camera relay.
        async with TestClient(TestServer(app)) as client:
            offer = {'sdp': 'v=0', 'type': 'offer'}
            assert (await client.post('/api/helm/xr/remote', json=offer)).status == 403
            headers = {'X-Helm-Token': 'secret'}
            for bad in ({'sdp': 'v=0', 'type': 'answer'}, {'type': 'offer'}, {'sdp': 'x' * (128 * 1024 + 1), 'type': 'offer'}):
                assert (await client.post('/api/helm/xr/remote', json=bad, headers=headers)).status == 422
            response = await client.post('/api/helm/xr/remote', json=offer, headers=headers)
            assert response.status == 422
            assert 'not configured' in (await response.json())['error']
    asyncio.run(run())


def test_offer_needs_a_login_not_control_and_not_while_another_headset_steers(helm, tmp_path):
    from aiohttp import ClientSession
    from aiohttp.test_utils import TestClient, TestServer
    save_password(tmp_path / 'access.json', 'minerva-helm')
    access = Access(tmp_path / 'access.json')
    quest = access.login('minerva-helm', device='Quest headset', address='198.51.100.60')
    tablet = access.login('minerva-helm', device='Tablet', address='198.51.100.61')
    headers, offer = {'X-Helm-Token': 'secret'}, {'sdp': 'v=0', 'type': 'offer'}

    async def run():
        app = create_app(helm, SimulationLink(), token='secret', access=access)
        async with TestClient(TestServer(app)) as client:
            url = client.make_url('/api/helm/xr/remote')
            assert (await client.post('/api/helm/xr/remote', json=offer, headers=headers)).status == 401
            async with ClientSession(cookies={'helm_session': quest}) as on_quest,                     ClientSession(cookies={'helm_session': tablet}) as on_tablet:
                # Past the control gate, to the offer's own refusal: this Helm has no camera relay.
                response = await on_quest.post(url, json=offer, headers=headers)
                assert response.status == 422 and 'not configured' in (await response.json())['error']
                socket = await on_quest.ws_connect(client.make_url('/api/helm/xr?token=secret'))
                await socket.send_json({'type': 'start'})
                await socket.receive_json(timeout=2)
                response = await on_tablet.post(url, json=offer, headers=headers)
                assert response.status == 422 and 'Quest headset is steering' in (await response.json())['error']
                await socket.close()
    asyncio.run(run())


def test_channel_poses_steer_like_the_socket_for_the_login_that_opened_the_link(helm, monkeypatch):
    from aiohttp.test_utils import TestClient, TestServer
    made = []

    class Link:
        def __init__(self, video, on_pose):
            self.on_pose, self.statuses = on_pose, []
            made.append(self)

        def status(self, reply):
            self.statuses.append(reply)

        async def close(self):
            pass

    monkeypatch.setattr('robot_platform.helm.server.WebRTCLink', Link)

    class Teleop:
        def __init__(self):
            self.heads = []

        def head_pose(self, yaw, pitch):
            self.heads.append((yaw, pitch))
            return True

        def snapshot(self):
            return {'head': {'state': 'FOLLOW'}}

        def stop(self):
            pass

    async def run():
        teleop = Teleop()
        app = create_app(helm, SimulationLink(), teleop=teleop, token='secret')
        async with TestClient(TestServer(app)) as client:
            def pose(seq, **fields):
                return json.dumps({'type': 'pose', 'seq': seq, 'q': [0, 0, 0, 1], **fields})
            link = made[0]
            link.on_pose(pose(1), None)  # No socket yet: nobody steers.
            assert teleop.heads == []
            socket = await client.ws_connect('/api/helm/xr?token=secret')
            await socket.send_json({'type': 'start'})
            await socket.receive_json()
            link.on_pose(pose(1), 'another login')  # Not the login steering.
            assert teleop.heads == []
            link.on_pose(pose(1), None)
            assert teleop.heads == [(0., 0.)]
            assert link.statuses[-1]['head'] == 'FOLLOW'  # The first pose says what the head does.
            for bad in ('not json', '[1]', pose(2, q=[1, 2]), pose(3, hands={'left': {'p': 1}})):  # All dropped quietly.
                link.on_pose(bad, None)
            assert teleop.heads == [(0., 0.)]
            # Wait on the clock that limits the rate: Windows ticks it every 15.6 ms, so a short sleep can read as none.
            accepted = time.monotonic()
            while time.monotonic() - accepted < 1 / MAX_RATE_HZ:
                await asyncio.sleep(1 / 100)
            link.on_pose(pose(4), None)
            link.on_pose(pose(4), None)  # An old sequence: a resend or one out of order counts for nothing.
            assert len(teleop.heads) == 2
            await socket.close()
    asyncio.run(run())
