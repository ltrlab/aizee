"""The operator's headset steering the head (helm.xr): its gaze as yaw and pitch, one device's headset at a time, the
zero, Helm's socket and zero endpoint (login, token, origin), a stalled loop never receiving poses, and the wrist
cameras' mounts the headset puts their pictures on the model's wrists by."""
import asyncio
import json
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


def test_controllers_reach_only_a_running_headset_session_and_never_wait_for_a_send():
    sent = []
    hands = {'left': {'position': (0., 1.2, -.4), 'orientation': (0., 0., 0., 1.), 'grip': 1., 'trigger': 0.,
                      'hold': False, 'ui': False}}
    loop = SimpleNamespace(_lock=threading.Lock(), _send_lock=threading.Lock(), active=True,
                           _view={'phase': 'running', 'headset': {'loading': True}},
                           _view_received_at=time.perf_counter(), _conn=SimpleNamespace(send=sent.append))
    assert TeleopSession.hands(loop, hands, 3, .25) is True
    assert sent == [('hands', {'hands': hands, 'epoch': 3, 'forward': .25})]
    loop._view = {'phase': 'running', 'headset': None}  # A leader, replay or policy session: nothing to drag.
    assert TeleopSession.hands(loop, hands, 3, .25) is False
    loop._view = {'phase': 'running', 'headset': {}}
    loop._view_received_at = time.perf_counter() - HEAD_POSE_ANSWER_S - .1  # Quiet: a stalled loop.
    assert TeleopSession.hands(loop, hands, 3, .25) is False
    loop._view_received_at = time.perf_counter()
    with loop._send_lock:  # Another send under way: dropped, not waited for.
        assert TeleopSession.hands(loop, hands, 3, .25) is False
    assert len(sent) == 1


class Teleop:
    active = False

    def __init__(self):
        self.poses, self.hands_sent = [], []

    def head_pose(self, yaw, pitch):
        self.poses.append((yaw, pitch))
        return True

    def hands(self, hands, epoch, forward):
        self.hands_sent.append((hands, epoch, forward))
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
            text = await page.text()
            assert page.status == 200 and '/helm/xr.js' in text
            assert 'id="video"' in text and 'id="other-link"' in text  # It says which link, and offers the other.
            script = await (await client.get('/helm/xr.js')).text()
            assert '/login?next=${PAGE.page}' in script  # A login returns to the page as it was opened.
            assert "/login?next=/xr'" not in script
            # The headset's scene: three.js where the page's import map says, and the page's modules.
            assert '{"imports": {"three": "/static/vendor/three.module.js"}}' in text
            for path in ('/helm/xr.js', '/helm/xr_scene.js', '/helm/xr_camera.js', '/helm/xr_status.js',
                         '/helm/xr_ring.js', '/helm/xr_session.js', '/helm/xr_wrists.js', '/helm/camera_live.js',
                         '/static/vendor/three.module.js'):
                assert (await client.get(path)).status == 200, path
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
            assert status['head_fault'] is None
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
    quest = access.login('minerva-helm', device='Quest headset', address='192.0.2.60')
    tablet = access.login('minerva-helm', device='Tablet', address='192.0.2.61')
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', on_robot=True)

    async def run():
        app = create_app(session, Link(), teleop=Teleop(), token='secret', access=access)
        async with TestClient(TestServer(app)) as client:
            page = await client.get('/xr', allow_redirects=False)
            assert page.status == 302 and page.headers['Location'] == '/login?next=/xr'
            login = await client.get('/login')
            assert "next === '/xr' || next === '/xr?remote'" in await login.text()
            page = await client.get('/xr?remote', allow_redirects=False)
            assert page.status == 302 and page.headers['Location'] == '/login?next=/xr?remote'
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


def controller(p=(0.1, 1.2, -0.4), q=(0., 0., 0., 1.), grip=0., trigger=0., **more):
    return dict({'p': list(p), 'q': list(q), 'grip': grip, 'trigger': trigger}, **more)


def test_the_controllers_ride_the_pose_and_helm_keeps_each_hands_latest():
    # For the arms' grab and drag (teleop.vr_arms): each hand's grip pose, grip, trigger, B/Y and whether it works the
    # screens, with the time it came; a recenter or new XR session starts another epoch, where old hands mean nothing.
    clock = Clock()
    headset = Headset(clock)
    quest = headset.connect('quest', 'Quest headset')
    epoch = headset.status()['epoch']
    pose = {'type': 'pose', 'seq': 1, 'q': gaze(0, 0),
            'hands': {'left': controller(grip=1., trigger=.25, b=True), 'right': controller(q=(0, 0, 0, 1.04), ui=True)}}
    assert headset.receive(quest, pose) == pytest.approx((0, 0))
    hands, now_epoch = headset.hand_samples()
    assert now_epoch == epoch and set(hands) == {'left', 'right'}
    assert hands['left'] == {'position': pytest.approx((.1, 1.2, -.4)), 'orientation': (0., 0., 0., 1.), 'grip': 1.,
                             'trigger': .25, 'hold': True, 'ui': False, 'at': 100.}
    assert hands['right']['orientation'][3] == pytest.approx(1.) and hands['right']['ui'] is True  # Normalized.
    assert headset.status()['hands'] == {'left': {'tracking': True, 'grip': 1., 'trigger': .25, 'ui': False},
                                         'right': {'tracking': True, 'grip': 0., 'trigger': 0., 'ui': True}}
    clock.now += .02
    headset.receive(quest, {'type': 'pose', 'seq': 2, 'q': gaze(0, 0), 'hands': {'left': controller(grip=.5)}})
    hands, _ = headset.hand_samples()
    assert hands['left']['grip'] == .5 and hands['left']['at'] == 100.02 and hands['right']['at'] == 100.
    clock.now += FRESH_S
    assert headset.status()['hands']['right']['tracking'] is False and headset.status()['hands']['left']['tracking']
    headset.start(quest)  # A recenter: another space.
    assert headset.hand_samples() == ({}, epoch + 1) and headset.status()['hands'] == {}
    headset.disconnect(quest)
    assert headset.hand_samples()[0] == {}


@pytest.mark.parametrize('hands, error', [
    ({'middle': controller()}, 'left, right'),
    ([controller()], 'left, right'),
    ({'left': controller(p=(0, 0))}, 'three numbers'),
    ({'left': controller(p=(0, 11., 0))}, 'within 10 m'),
    ({'left': controller(q=(0, 0, 0, 2))}, 'unit quaternion'),
    ({'left': controller(q=(0, 0, 1))}, 'four numbers'),
    ({'left': controller(grip=1.5)}, '0 to 1'),
    ({'left': controller(trigger=-.1)}, '0 to 1'),
    ({'left': controller(b=1)}, 'true or false'),
    ({'left': dict(controller(), extra=1)}, 'p, q, grip and trigger'),
    ({'left': {'p': [0, 0, 0], 'q': [0, 0, 0, 1]}}, 'p, q, grip and trigger'),
])
def test_a_malformed_controller_refuses_the_whole_pose(hands, error):
    clock = Clock()
    headset = Headset(clock)
    quest = headset.connect('quest', 'Quest headset')
    with pytest.raises(ValueError, match=error):
        headset.receive(quest, {'type': 'pose', 'seq': 1, 'q': gaze(.2, 0), 'hands': hands})
    assert headset.status()['poses'] == 0 and headset.hand_samples()[0] == {}


def test_the_socket_takes_controllers_with_the_pose_and_helms_state_shows_each_hand(helm):
    from aiohttp.test_utils import TestClient, TestServer
    teleop = Teleop()

    async def run():
        async with TestClient(TestServer(create_app(helm, Link(), teleop=teleop, token='secret',
                                                    simulation_only=True))) as client:
            ws = await client.ws_connect('/api/helm/xr?token=secret')
            await ws.send_json({'type': 'start'})
            await ws.receive_json(timeout=2)
            # The page's largest message (both hands, rounded numbers) fits the socket's 1024-byte limit.
            hands = {hand: {'p': [-1.2345, 1.2345, -1.2345], 'q': [-.70711, .70711, -0.00001, .00002], 'grip': .123,
                            'trigger': .987, 'b': True, 'ui': False} for hand in ('left', 'right')}
            await ws.send_json({'type': 'pose', 'seq': 1, 'q': gaze(.3, .2), 'hands': hands})
            assert (await ws.receive_json(timeout=2))['steering'] is True
            await ws.send_json({'type': 'pose', 'seq': 2, 'q': gaze(.3, .2), 'hands': {'left': {'p': [0, 0]}}})
            assert 'controller' in (await ws.receive_json(timeout=2))['error'].lower()
            state = await (await client.get('/api/helm/state')).json()
            assert state['xr']['hands']['left'] == {'tracking': True, 'grip': .123, 'trigger': .987, 'ui': False}
            assert set(state['xr']['hands']) == {'left', 'right'}
            # The session gets the controllers as Helm checked them (the quaternion made unit), with the epoch and the
            # head zero's yaw; the refused pose sent none.
            assert len(teleop.hands_sent) == 1
            sent, epoch, forward = teleop.hands_sent[0]
            assert set(sent) == {'left', 'right'} and epoch == state['xr']['epoch'] and forward == 0.
            assert sent['left']['hold'] is True and 'at' not in sent['left']
            assert sum(v * v for v in sent['left']['orientation']) == pytest.approx(1.)
            await ws.close()
    asyncio.run(run())


def test_the_headset_puts_the_wrist_pictures_by_the_newest_solves_mounts_else_the_nominal(helm, tmp_path):
    import numpy as np
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.xr import NOMINAL_MOUNT, wrist_mounts
    from robot_platform.vision.rig import load_rig
    cameras = load_rig()['cameras']
    runs = tmp_path / 'runs'

    # No solve: each camera on its rig link, the simulation's mount (orthonormal, right-handed: optical axes), its
    # 85 degree vertical view across 640 x 480.
    nominal = wrist_mounts(cameras, runs)
    assert set(nominal) == {'left_wrist', 'right_wrist'}
    for role, mount in nominal.items():
        assert (mount['link'], mount['source']) == (cameras[role]['link'], 'nominal')
        assert mount['axis'] == [0., 1., 0.]  # The rig's picture axis: the gripper's, which the headset squares to.
        assert mount['matrix'] == [list(row) for row in NOMINAL_MOUNT]
        assert mount['fov_deg'] == pytest.approx([101.41, 85.], abs=.01)
    rotation = np.array(nominal['left_wrist']['matrix'])[:3, :3]
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=2e-3)
    assert np.linalg.det(rotation) == pytest.approx(1., abs=3e-3)
    # It looks along the link's y and down its z (simulation/task_scene.py: 37 degrees below y).
    assert rotation[:, 2] == pytest.approx([0, .795, -.606])

    def run(name, solved=None, recommended='plain'):
        folder = runs / name
        folder.mkdir(parents=True)
        (folder / 'stills.jsonl').write_text('', encoding='utf-8')
        if solved is not None:
            (folder / 'result.json').write_text(json.dumps({'recommended': recommended,
                                                            'models': {'plain': {'cameras': solved}}}))
    fitted = {'link': 'fitted_link', 'mount': {'matrix': [[1, 0, 0, .01], [0, 1, 0, .02], [0, 0, 1, .03], [0, 0, 0, 1]]},
              'field_of_view_deg': {'horizontal': 90.5, 'vertical': 72.25}}
    run('20261101T000000Z', {'left_wrist': fitted, 'right_wrist': fitted})
    run('20261103T000000Z')  # Stills only.
    run('20261104T000000Z', {}, recommended=None)  # No model solved.
    (runs / '20261105T000000Z').mkdir()
    (runs / '20261105T000000Z' / 'result.json').write_text('{not json')
    # The newest solved run's recommended model: each camera's mount, link and view.
    mounts = wrist_mounts(cameras, runs)
    # On another link than the rig's (the solve's J6-parent check), the rig's axis is not that link's: none.
    assert mounts['left_wrist'] == {'link': 'fitted_link', 'matrix': fitted['mount']['matrix'], 'fov_deg': [90.5, 72.25],
                                    'axis': None, 'source': 'calibration 20261101T000000Z'}
    on_rig = wrist_mounts({'left_wrist': {**cameras['left_wrist'], 'link': 'fitted_link'}}, runs)['left_wrist']
    assert on_rig['axis'] == [0., 1., 0.] and on_rig['source'] == 'calibration 20261101T000000Z'
    # A rig without a picture axis, or a malformed one: the picture as the camera looks.
    assert wrist_mounts({'left_wrist': {k: v for k, v in cameras['left_wrist'].items() if k != 'picture_axis'}}
                        )['left_wrist']['axis'] is None
    assert wrist_mounts({'left_wrist': {**cameras['left_wrist'], 'picture_axis': [0, 0]}})['left_wrist']['axis'] is None
    assert wrist_mounts({'left_wrist': {**cameras['left_wrist'], 'picture_axis': [0, 3, 4]}}
                        )['left_wrist']['axis'] == pytest.approx([0, .6, .8])
    # A newer solve supersedes it; a camera it lacks, or an unreadable mount, keeps the nominal one.
    run('20261102T000000Z', {'left_wrist': {**fitted, 'mount': {'matrix': [[1, 0], [0, 1]]}}})
    mounts = wrist_mounts(cameras, runs)
    assert (mounts['left_wrist']['source'], mounts['right_wrist']['source']) == ('nominal', 'nominal')

    # Helm's route: from its wrist calibration's runs, else the rig's nominal mounts.
    async def ask(calibration):
        async with TestClient(TestServer(create_app(helm, Link(), teleop=Teleop(), token='secret', simulation_only=True,
                                                    calibration=calibration))) as client:
            reply = await client.get('/api/helm/xr/wrists')
            assert reply.status == 200 and reply.headers['Cache-Control'] == 'no-store'
            return (await reply.json())['mounts']
    assert asyncio.run(ask(None)) == json.loads(json.dumps(nominal))
    (runs / '20261102T000000Z' / 'result.json').unlink()
    calibration = SimpleNamespace(config={'cameras': cameras}, root=runs, snapshot=dict)
    assert asyncio.run(ask(calibration))['right_wrist']['source'] == 'calibration 20261101T000000Z'
