"""The operator's headset steering the head (helm.xr): its gaze as yaw and pitch, one device's headset at a time, the
zero, Helm's socket and zero endpoint (login, token, origin), and a stalled loop never receiving poses."""
import asyncio
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm.access import Access, save_password
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import HEAD_POSE_ANSWER_S, TeleopSession
from robot_platform.helm.xr import FRESH_S, Headset, wrapped, yaw_pitch

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'


def about(axis, angle):
    """A unit quaternion (x, y, z, w) turning `angle` about the unit `axis`."""
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def times(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def gaze(yaw, pitch, roll=0.):
    """A headset turned `yaw` left, then looking `pitch` up, then rolled about its view."""
    return times(times(about((0, 1, 0), yaw), about((1, 0, 0), pitch)), about((0, 0, 1), roll))


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def test_the_gaze_turns_left_positive_and_up_positive_and_ignores_roll():
    assert yaw_pitch((0, 0, 0, 1)) == pytest.approx((0, 0))
    for yaw, pitch, roll in ((.4, 0, 0), (0, .3, 0), (-.9, -.5, 0), (1.2, .7, .6), (3., .2, -1.)):
        assert yaw_pitch(gaze(yaw, pitch, roll)) == pytest.approx((yaw, pitch), abs=1e-9)
    assert yaw_pitch([1.05 * v for v in gaze(.4, .2)]) == pytest.approx((.4, .2))  # Rounding in the page.
    for bad in ((0, 0, 1), (0, 0, 0, 0), (0, 0, 0, 2), ('a', 0, 0, 1), (math.nan, 0, 0, 1), None):
        with pytest.raises(ValueError):
            yaw_pitch(bad)
    assert wrapped(math.pi + .1) == pytest.approx(-math.pi + .1)


def test_poses_come_relative_to_the_zero_in_order_and_not_too_fast():
    clock = Clock()
    headset = Headset(clock)
    one = headset.connect('pc-session', 'Quest headset')
    assert headset.status()['connected'] and not headset.status()['tracking']
    with pytest.raises(ValueError, match='No headset pose'):
        headset.zero()
    assert headset.receive(one, {'type': 'pose', 'seq': 1, 'q': gaze(.5, .1)}) == pytest.approx((.5, .1))
    clock.now += .002
    assert headset.receive(one, {'type': 'pose', 'seq': 2, 'q': gaze(.6, .1)}) is None  # Faster than 120 Hz.
    clock.now += .02
    assert headset.receive(one, {'type': 'pose', 'seq': 2, 'q': gaze(.6, .1)}) == pytest.approx((.6, .1))
    assert headset.receive(one, {'type': 'pose', 'seq': 2, 'q': gaze(.7, .1)}) is None  # Not newer.
    for junk in ({'type': 'pose', 'seq': True, 'q': gaze(0, 0)}, {'type': 'pose', 'seq': 0, 'q': gaze(0, 0)},
                 {'type': 'hello'}, [1, 2]):
        assert headset.receive(one, junk) is None
    status = headset.zero()  # Where it looks now is straight ahead, level.
    assert (status['yaw'], status['pitch']) == pytest.approx((0, 0)) and status['zero']['yaw'] == pytest.approx(.6)
    clock.now += .02
    assert headset.receive(one, {'type': 'pose', 'seq': 3, 'q': gaze(-3., .3)}) == pytest.approx(
        (wrapped(-3. - .6), .2))  # The short way round.
    status = headset.status()
    assert status['tracking'] and status['device'] == 'Quest headset' and 45 < status['rate_hz'] < 50  # 22, 20 ms.
    clock.now += FRESH_S + .01
    assert not headset.status()['tracking'] and headset.status()['rate_hz'] is None
    with pytest.raises(ValueError, match='No headset pose'):
        headset.zero()


def test_one_device_steers_and_a_new_xr_session_restarts_the_zero():
    clock = Clock()
    headset = Headset(clock)
    quest = headset.connect('quest', 'Quest headset')
    headset.receive(quest, {'type': 'pose', 'seq': 1, 'q': gaze(.4, 0)})
    headset.zero(quest)
    with pytest.raises(ValueError, match='Quest headset is steering the head'):
        headset.connect('tablet', 'Tablet')
    again = headset.connect('quest', 'Quest headset')  # The same device reconnecting takes its stream back.
    assert headset.receive(quest, {'type': 'pose', 'seq': 9, 'q': gaze(0, 0)}) is None  # Its old connection.
    with pytest.raises(ValueError, match='Another connection'):
        headset.zero(quest)
    assert headset.status()['zero']['yaw'] == pytest.approx(.4)  # Kept across the reconnect.
    clock.now += .02
    assert headset.receive(again, {'type': 'pose', 'seq': 1, 'q': gaze(.5, 0)}) == pytest.approx((.1, 0))
    headset.start(again)  # A new XR session, or a recenter: its space starts at the heading, level.
    assert headset.status()['zero'] == {'yaw': 0., 'pitch': 0.} and not headset.status()['tracking']
    clock.now += .02
    assert headset.receive(again, {'type': 'pose', 'seq': 1, 'q': gaze(.05, -.1)}) == pytest.approx((.05, -.1))
    headset.disconnect(quest)  # An old connection closing leaves the live one.
    assert headset.status()['connected']
    headset.disconnect(again)
    assert not headset.status()['connected'] and headset.status()['device'] is None
    tablet = headset.connect('tablet', 'Tablet')  # Free: another device may steer, from its own zero.
    assert headset.status()['zero'] == {'yaw': 0., 'pitch': 0.} and headset.status()['device'] == 'Tablet'
    assert headset.receive(again, {'type': 'pose', 'seq': 5, 'q': gaze(0, 0)}) is None and tablet


def test_poses_reach_only_a_running_loop_that_answers_and_never_wait_for_a_send():
    sent = []
    loop = SimpleNamespace(_lock=threading.Lock(), _send_lock=threading.Lock(), active=True,
                           _view={'phase': 'running'}, _view_received_at=time.perf_counter(),
                           _conn=SimpleNamespace(send=sent.append))
    assert TeleopSession.head_pose(loop, .1, -.2) is True and sent == [('head_pose', .1, -.2)]
    loop._view_received_at = time.perf_counter() - HEAD_POSE_ANSWER_S - .1  # Quiet: a stalled loop.
    assert TeleopSession.head_pose(loop, .1, -.2) is False
    loop._view_received_at = time.perf_counter()
    with loop._send_lock:  # Another send under way: dropped, not waited for.
        assert TeleopSession.head_pose(loop, .1, -.2) is False
    loop._view = {'phase': 'starting'}
    assert TeleopSession.head_pose(loop, .1, -.2) is False
    loop._view, loop._conn = {'phase': 'running'}, SimpleNamespace(send=lambda message: (_ for _ in ()).throw(OSError()))
    assert TeleopSession.head_pose(loop, .1, -.2) is False and not loop._send_lock.locked()
    assert len(sent) == 1


class Teleop:
    active = False

    def __init__(self):
        self.poses = []

    def head_pose(self, yaw, pitch):
        self.poses.append((yaw, pitch))
        return True

    def snapshot(self):
        return {'phase': 'running', 'receiver': None, 'head': {'state': 'TELEOP', 'source': 'headset'}}

    def stop(self):
        pass

    def guide(self):
        return None


class Link:
    running = False

    def disconnect(self):
        pass


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    yield session
    session.close()


def test_the_socket_steers_the_head_and_zeroes_from_the_headset_or_the_teleop_tab(helm):
    from aiohttp import WSServerHandshakeError
    from aiohttp.test_utils import TestClient, TestServer
    teleop = Teleop()

    async def run():
        async with TestClient(TestServer(create_app(helm, Link(), teleop=teleop, token='secret',
                                                    simulation_only=True))) as client:
            page = await client.get('/xr')
            assert page.status == 200 and '/helm/xr.js' in await page.text()
            assert (await client.get('/helm/xr.js')).status == 200
            for path, headers in (('/api/helm/xr', {}), ('/api/helm/xr?token=wrong', {}),
                                  ('/api/helm/xr?token=secret', {'Origin': 'https://elsewhere'})):
                with pytest.raises(WSServerHandshakeError) as refused:
                    await client.ws_connect(path, headers=headers)
                assert refused.value.status == 403
            zero = await client.post('/api/helm/xr/zero', json={}, headers={'X-Helm-Token': 'secret'})
            assert zero.status == 422 and 'No headset pose' in (await zero.json())['error']
            ws = await client.ws_connect('/api/helm/xr?token=secret')
            await ws.send_json({'type': 'start'})
            assert (await ws.receive_json(timeout=2))['restarted'] is True
            await ws.send_json({'type': 'pose', 'seq': 1, 'q': gaze(.3, .2)})
            status = await ws.receive_json(timeout=2)  # The first pose, then every 15th: what the head does.
            assert status['steering'] is True and (status['head'], status['source']) == ('TELEOP', 'headset')
            assert teleop.poses == [pytest.approx((.3, .2))]
            await ws.send_json({'type': 'pose', 'seq': 2, 'q': [0, 0, 0]})
            assert 'four numbers' in (await ws.receive_json(timeout=2))['error']
            await ws.send_json({'type': 'zero'})  # A trigger pull in the headset.
            zeroed = await ws.receive_json(timeout=2)
            assert zeroed['zeroed'] is True and (zeroed['yaw'], zeroed['pitch']) == pytest.approx((0, 0))
            await asyncio.sleep(.02)
            await ws.send_json({'type': 'pose', 'seq': 3, 'q': gaze(.5, .2)})
            for _ in range(100):
                if len(teleop.poses) == 2:
                    break
                await asyncio.sleep(.01)
            assert teleop.poses[-1] == pytest.approx((.2, 0))
            state = await (await client.get('/api/helm/state')).json()
            assert state['xr']['connected'] and state['xr']['tracking'] and state['xr']['device'] == 'The headset'
            zero = await client.post('/api/helm/xr/zero', json={}, headers={'X-Helm-Token': 'secret'})
            assert zero.status == 200 and (await zero.json())['xr']['zero']['yaw'] == pytest.approx(.5)
            await ws.close()
            for _ in range(100):
                state = await (await client.get('/api/helm/state')).json()
                if not state['xr']['connected']:
                    break
                await asyncio.sleep(.01)
            assert not state['xr']['connected']
    asyncio.run(run())


def test_the_headset_logs_in_back_to_its_page_and_another_device_cannot_take_the_head(tmp_path):
    from aiohttp import ClientSession
    from aiohttp.test_utils import TestClient, TestServer
    save_password(tmp_path / 'access.json', 'minerva-helm')
    access = Access(tmp_path / 'access.json')
    quest = access.login('minerva-helm', device='Quest headset', address='192.168.8.60')
    tablet = access.login('minerva-helm', device='Tablet', address='192.168.8.61')
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', on_robot=True)

    async def run():
        app = create_app(session, Link(), teleop=Teleop(), token='secret', access=access)
        async with TestClient(TestServer(app)) as client:
            page = await client.get('/xr', allow_redirects=False)
            assert page.status == 302 and page.headers['Location'] == '/login?next=/xr'
            login = await client.get('/login')
            assert "get('next') === '/xr'" in await login.text()
            refused = await client.post('/api/helm/xr/zero', json={}, headers={'X-Helm-Token': 'secret'})
            assert refused.status == 401
            url = client.make_url('/api/helm/xr?token=secret')
            async with ClientSession(cookies={'helm_session': quest}) as on_quest, \
                    ClientSession(cookies={'helm_session': tablet}) as on_tablet:
                steering = await on_quest.ws_connect(url)
                await steering.send_json({'type': 'start'})
                assert (await steering.receive_json(timeout=2))['device'] == 'Quest headset'
                other = await on_tablet.ws_connect(url)
                busy = await other.receive_json(timeout=2)
                assert busy['busy'] is True and 'Quest headset is steering the head' in busy['error']
                await other.close()
                # The headset is not in control, and needs not be: the session the device in control engages
                # decides whether the head moves. Zeroing from the teleop tab, a change, needs control.
                view_only = await on_tablet.post(client.make_url('/api/helm/xr/zero'), json={},
                                                 headers={'X-Helm-Token': 'secret'})
                assert view_only.status == 403
                await steering.close()
    try:
        asyncio.run(run())
    finally:
        session.close()
