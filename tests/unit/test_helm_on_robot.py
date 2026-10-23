"""Helm on Minerva: a login and one device in control (stopping always allowed), the robot connection that counts as
wired, Check's tether item, and teleop through the GELLO bridge (refused without one, its link bound on the tether)."""
import asyncio
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm.access import Access, save_password, stop_request
from robot_platform.helm.server import create_app
from robot_platform.helm.session import WIRED, HelmSession
from robot_platform.helm.teleop import TeleopSession
from robot_platform.ops.ssh import TETHER

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
TELEOP = ROOT / 'config/devices/minerva-teleop.json'


class Clock:
    def __init__(self):
        self.now = 1000.

    def __call__(self):
        return self.now


def test_a_password_logs_a_device_in_and_one_device_at_a_time_is_in_control(tmp_path):
    with pytest.raises(ValueError, match='at least 8'):
        save_password(tmp_path / 'access.json', 'short')
    save_password(tmp_path / 'access.json', 'minerva-helm')
    clock = Clock()
    access = Access(tmp_path / 'access.json', clock=clock)
    tablet = access.login('minerva-helm', device='Tablet', address='192.168.8.50')
    pc = access.login('minerva-helm', device='Operator PC', address='10.42.0.33')
    assert access.view(tablet) == {'enabled': True, 'device': 'Tablet', 'in_control': False, 'holder': None}
    access.claim(tablet)
    assert access.in_control(tablet) and not access.in_control(pc)
    assert access.view(pc)['holder']['device'] == 'Tablet'
    access.claim(pc)  # Taking control from another device.
    assert access.in_control(pc) and not access.in_control(tablet)
    access.logout(pc)
    assert access.view(tablet)['holder'] is None and access.session(pc) is None
    clock.now += 13 * 3600.
    assert access.session(tablet) is None  # Twelve hours unused: logged out.
    with pytest.raises(PermissionError, match='Log in first'):
        access.claim(tablet)


def test_wrong_passwords_wait_longer_each_time(tmp_path):
    save_password(tmp_path / 'access.json', 'minerva-helm')
    clock = Clock()
    access = Access(tmp_path / 'access.json', clock=clock)
    with pytest.raises(ValueError, match='Wrong password'):
        access.login('guess-one', device='x', address='192.168.8.99')
    with pytest.raises(PermissionError, match='try again'):
        access.login('minerva-helm', device='x', address='192.168.8.99')  # Even the right one, too soon.
    clock.now += 1.1
    with pytest.raises(ValueError):
        access.login('guess-two', device='x', address='192.168.8.99')
    clock.now += 1.1
    with pytest.raises(PermissionError):
        access.login('minerva-helm', device='x', address='192.168.8.99')  # Two failures: a two-second wait.
    clock.now += 1.
    assert access.login('minerva-helm', device='x', address='192.168.8.99')
    assert access.login('minerva-helm', device='y', address='192.168.8.98')  # Another address never waited.


def test_only_stopping_is_a_stop():
    assert stop_request('/api/helm/teleop/key', {'key': 'H'}) and stop_request('/api/helm/tune/key', {'key': 'X'})
    for path, payload in (('/api/helm/teleop/key', {'key': 'T'}), ('/api/helm/teleop/key', {'key': 'H', 'x': 1}),
                          ('/api/helm/teleop/start', {'key': 'H'}), ('/api/helm/teleop/key', None)):
        assert not stop_request(path, payload)


class Teleop:
    active = False

    def __init__(self):
        self.keys = []

    def key(self, key):
        self.keys.append(key)

    def stop(self):
        pass

    def snapshot(self):
        return {'phase': 'running', 'receiver': None}

    def guide(self):
        return None


class Link:
    running = False

    def disconnect(self):
        pass


def test_the_page_needs_a_login_and_changes_need_control_but_any_device_may_stop(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    save_password(tmp_path / 'access.json', 'minerva-helm')
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', on_robot=True)
    teleop = Teleop()

    async def scenario(client):
        page = await client.get('/', allow_redirects=False)
        assert page.status == 302 and page.headers['Location'] == '/login'
        state = await client.get('/api/helm/state')
        assert state.status == 401 and (await state.json())['login'] is True
        wrong = await client.post('/api/helm/login', json={'password': 'nope-nope', 'device': 'Tablet'})
        assert wrong.status == 401
        await asyncio.sleep(1.1)  # The wait after one wrong password.
        right = await client.post('/api/helm/login', json={'password': 'minerva-helm', 'device': 'Tablet'})
        assert right.status == 200 and 'helm_session' in right.cookies
        state = await (await client.get('/api/helm/state')).json()
        assert state['access'] == {'enabled': True, 'device': 'Tablet', 'in_control': False, 'holder': None}
        assert state['robot']['connection'] == 'robot' and state['robot']['address'] == '127.0.0.1'
        headers = {'X-Helm-Token': state['action_token']}
        refused = await client.post('/api/helm/teleop/key', json={'key': 'T'}, headers=headers)
        assert refused.status == 403 and (await refused.json())['view_only'] is True
        stop = await client.post('/api/helm/teleop/key', json={'key': 'H'}, headers=headers)
        assert stop.status != 403  # Disable reaches the session from a device that is not in control.
        claimed = await (await client.post('/api/helm/control/claim', json={}, headers=headers)).json()
        assert claimed['access']['in_control'] is True
        engaged = await client.post('/api/helm/teleop/key', json={'key': 'T'}, headers=headers)
        assert engaged.status != 403
        await client.post('/api/helm/logout', json={}, headers=headers)
        assert (await client.get('/api/helm/state')).status == 401

    async def run():
        app = create_app(session, Link(), teleop=teleop, token='secret', access=Access(tmp_path / 'access.json'),
                         simulation_only=False)
        async with TestClient(TestServer(app)) as client:
            await scenario(client)
    try:
        asyncio.run(run())
    finally:
        session.close()


def test_on_the_robot_the_connection_is_wired_and_check_passes_the_tether(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', on_robot=True)
    try:
        assert (session.connection, session.address, session.wired) == ('robot', '127.0.0.1', True)
        assert 'robot' in WIRED and 'wifi' not in WIRED
        with pytest.raises(ValueError):
            session.set_connection('wifi')  # There is no other way to the robot it runs on.
    finally:
        session.close()
    from robot_platform.ops import check
    status, detail = check._tether(SimpleNamespace(address='127.0.0.1', facts={}))
    assert status == check.PASS and 'Helm runs on Minerva' in detail


def test_teleop_on_the_robot_needs_the_bridge_and_binds_its_link_on_the_tether(tmp_path):
    helm = SimpleNamespace(profile_path=PROFILE, bindings={}, mapping={}, saved_limits={'joints': {}},
                           asset_id='minerva-production-01', connection='robot', address='127.0.0.1')
    bridge = SimpleNamespace(connected=False, serve_link=lambda *a, **k: None)
    made = []

    class Robot:
        def __init__(self, bundle, leader_profile, **options):
            from robot_platform.helm.edge import Inbox
            made.append(options)
            self.views, self.commands = Inbox(), SimpleNamespace(send=lambda message: None, close=lambda: None)

        def start(self):
            self.views.end('stand-in')

        def is_alive(self):
            return False

        def join(self, timeout=None):
            pass
    session = TeleopSession(helm, TELEOP, runs=tmp_path, target='jetson', edge_factory=Robot, bridge=bridge,
                            leaders=SimpleNamespace(saved=None, missing=lambda routes: []))
    helm.on_robot = True
    session.set_receiver('virtual')
    with pytest.raises(ValueError, match='never inside Helm'):
        session.set_target('pc')  # On Minerva the loop keeps to its own process on the control CPUs.
    with pytest.raises(ValueError, match='GELLO bridge is not connected'):
        session.start()
    session.start(replay={'episode': 's/e'})  # A replay needs no leaders, so no bridge.
    session.stop()
    bridge.connected = True
    session.start()
    try:
        assert made[-1]['serve_link'] is bridge.serve_link and made[-1]['link_bind'] == TETHER
    finally:
        session.stop()


# -- the operator's files, kept out of the release ---------------------------------------------------------------

def test_operator_files_start_from_the_release_and_a_release_change_wins_without_losing_a_save(tmp_path):
    from robot_platform.helm.operator_files import operator_files, prepare
    release, mine = tmp_path / 'release', tmp_path / 'operator'
    release.mkdir()
    files = {name: release / name for name in ('limits.json', 'servo.json', 'ranges.json')}
    files['limits.json'].write_text('limits v1')
    files['servo.json'].write_text('servo v1')
    assert prepare(files, mine) == []  # Seeded: the release's copies; ranges.json is in neither.
    assert (mine / 'limits.json').read_text() == 'limits v1' and not (mine / 'ranges.json').exists()
    (mine / 'limits.json').write_text('limits saved on Minerva')  # The operator saves; the release is unchanged.
    (mine / 'ranges.json').write_text('ranges swept on Minerva')
    assert prepare(files, mine) == []
    assert (mine / 'limits.json').read_text() == 'limits saved on Minerva'
    assert (mine / 'ranges.json').read_text() == 'ranges swept on Minerva'
    # A deploy brings committed copies: never saved over (servo) -> updated; saved over (limits) -> kept aside.
    files['servo.json'].write_text('servo v2')
    files['limits.json'].write_text('limits v2')
    notices = {notice['file']: notice for notice in prepare(files, mine, clock=lambda: 0.)}
    assert notices['servo.json']['kind'] == 'updated' and (mine / 'servo.json').read_text() == 'servo v2'
    assert notices['limits.json']['kind'] == 'replaced' and (mine / 'limits.json').read_text() == 'limits v2'
    kept = Path(notices['limits.json']['kept'])
    assert kept.parent == mine / 'replaced' and kept.read_text() == 'limits saved on Minerva'
    assert prepare(files, mine) == []  # Settled.
    # The operator saved exactly what the next release brings: nothing to keep aside.
    (mine / 'servo.json').write_text('servo v3')
    files['servo.json'].write_text('servo v3')
    assert prepare(files, mine) == [] and (mine / 'servo.json').read_text() == 'servo v3'
    found = operator_files(PROFILE, 'minerva-production-01', TELEOP)
    assert set(found) == {'minerva-production-01-joint-limits.json', 'minerva-production-01-servo.json',
                          'gello-dual-leader-ranges.json'}
    assert all(path.parent == PROFILE.parent.resolve() for path in found.values())


def test_helm_reads_and_saves_the_operator_files_in_their_folder(tmp_path):
    from robot_platform.control.servo_calibration import servo_path
    from robot_platform.helm.leaders import LeaderWorkspace
    from robot_platform.ops.check import Context
    folder = tmp_path / 'operator'
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', on_robot=True, operator_files=folder)
    try:
        assert session.limits_path == folder.resolve() / 'minerva-production-01-joint-limits.json'
    finally:
        session.close()
    assert servo_path(PROFILE, 'minerva-production-01', folder) == folder.resolve() / 'minerva-production-01-servo.json'
    assert servo_path(PROFILE, 'minerva-production-01') == PROFILE.parent.resolve() / 'minerva-production-01-servo.json'
    leaders = LeaderWorkspace(TELEOP, operator_files=folder)
    assert leaders.path == folder.resolve() / 'gello-dual-leader-ranges.json'
    context = Context(root=ROOT, profile=PROFILE, teleop=TELEOP, runs=tmp_path, limits=None, address=None, motors=False,
                      open_ssh=None, operator_files=folder)
    assert context.operator_files == folder


# -- the 3D view's meshes, converted on the build PC ---------------------------------------------------------------

def test_the_mesh_store_serves_converted_meshes_by_digest_and_revalidates(tmp_path):
    import gzip
    import hashlib
    import json
    import struct
    trimesh = pytest.importorskip('trimesh')
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.models import ModelProfile
    from robot_platform.viewer.server import ModelStore, create_app as viewer_app, mesh_payload, stored_mesh_name
    trimesh.creation.box().export(tmp_path / 'shape.stl')
    (tmp_path / 'machine.urdf').write_text('<robot name="m"><link name="base"><visual><geometry>'
                                           '<mesh filename="package://parts/shape.stl"/></geometry></visual></link></robot>')
    (tmp_path / 'model.json').write_text(json.dumps({'schema_version': 1, 'id': 'fixture', 'title': 'Box',
                                                     'urdf': 'machine.urdf', 'packages': {'parts': '.'}}))
    profile = ModelProfile.load(tmp_path / 'model.json')
    store = tmp_path / 'meshes'
    store.mkdir()
    payload = mesh_payload(tmp_path / 'shape.stl')
    sha = hashlib.sha256((tmp_path / 'shape.stl').read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='not in the mesh store'):
        ModelStore([profile], mesh_store=store).stored_mesh('fixture', 0)
    (store / stored_mesh_name(sha)).write_bytes(gzip.compress(payload, mtime=0))
    body, etag = ModelStore([profile], mesh_store=store).stored_mesh('fixture', 0)
    assert gzip.decompress(body) == payload and sha in etag
    assert struct.unpack('<II', payload[:8])[1] == 12  # A box: twelve triangles, all kept.

    async def scenario(client):
        first = await client.get('/api/models/fixture/meshes/0')
        assert first.status == 200 and await first.read() == payload  # Sent gzip-encoded, decoded by the client.
        assert first.headers['Content-Encoding'] == 'gzip' and first.headers['ETag'] == etag
        again = await client.get('/api/models/fixture/meshes/0', headers={'If-None-Match': etag})
        assert again.status == 304  # The browser keeps what it fetched.
        plain = await client.get('/api/models/fixture/meshes/0', headers={'Accept-Encoding': 'identity'})
        assert await plain.read() == payload and 'Content-Encoding' not in plain.headers

    async def run():
        async with TestClient(TestServer(viewer_app([profile], mesh_store=store))) as client:
            await scenario(client)
    asyncio.run(run())
