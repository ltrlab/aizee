"""Camera API and actual local REP/PUB contracts; no hardware, SSH or motor access."""
import asyncio
import base64
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import time

import msgpack
import pytest
import zmq

from robot_platform.helm import cameras as cam
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession

ROOT = Path(__file__).resolve().parents[2]
LIST = '''
 auto_exposure 0x009a0901 (menu) : min=0 max=3 default=3 value=1
    1: Manual Mode
    3: Aperture Priority Mode
 exposure_time_absolute 0x009a0902 (int) : min=1 max=5000 step=1 default=157 value=150
 gain 0x00980913 (int) : min=0 max=100 step=2 default=0 value=30
 white_balance_temperature_auto 0x0098090c (bool) : default=1 value=1
'''


def reported():
    return {name: {k: v for k, v in info.items() if k not in ('flags', 'options')}
            for name, info in cam.parse_controls(LIST).items()}


@pytest.mark.parametrize('name,value,error', [
    ('auto_exposure', 2, 'menu entry'), ('auto_exposure', True, 'integer'),
    ('gain', 31, 'step'), ('gain', -1, 'step'), ('gain', 102, 'step'),
    ('gain', 30., 'integer'), ('unknown', 1, 'Unknown'),
    ('white_balance_temperature_auto', 2, '0 or 1'),
])
def test_actual_sparse_menu_step_and_type_rejections(name, value, error):
    controls = cam.verified_controls(reported(), cam.parse_controls(LIST))
    with pytest.raises(ValueError, match=error):
        cam.validate_value(controls, name, value)


def test_menu_and_inactive_flags_come_from_device_not_min_max():
    actual = cam.parse_controls(LIST.replace('value=150', 'value=150 flags=inactive'))
    controls = cam.verified_controls(reported(), actual)
    assert controls['auto_exposure']['options'] == [
        {'value': 1, 'label': 'Manual Mode'}, {'value': 3, 'label': 'Aperture Priority Mode'}]
    cam.validate_value(controls, 'auto_exposure', 1)
    with pytest.raises(ValueError, match='inactive'):
        cam.validate_value(controls, 'exposure_time_absolute', 150)
    actual['gain']['step'] = 1
    with pytest.raises(RuntimeError, match='ranges do not match'):
        cam.verified_controls(reported(), actual)


@pytest.fixture
def rep(monkeypatch):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    port = socket.bind_to_random_port('tcp://127.0.0.1')
    monkeypatch.setitem(cam.ROLES, 'right_wrist', dict(cam.ROLES['right_wrist'], control_port=port))
    state = SimpleNamespace(controls=reported(), requests=[], apply=True, stop=threading.Event())
    def worker():
        while not state.stop.is_set():
            if not socket.poll(20):
                continue
            request = msgpack.unpackb(socket.recv(), raw=False)
            state.requests.append(request)
            if request['op'] == 'get_ctrls':
                result = {'ok': True, 'ctrls': state.controls}
            else:
                if hasattr(state, 'release_set'):
                    assert state.release_set.wait(4)
                if state.apply:
                    state.controls[request['name']]['value'] = request['value']
                result = {'ok': True}
            socket.send(msgpack.packb(result, use_bin_type=True))
    thread = threading.Thread(target=worker)
    thread.start()
    monkeypatch.setattr('shutil.which', lambda _: '/usr/bin/v4l2-ctl')
    monkeypatch.setattr(cam, '_device_controls', lambda config, reply: ('/dev/video-fixture',
        cam.verified_controls(reply, cam.parse_controls(LIST))))
    yield state
    state.stop.set()
    thread.join(1)
    socket.close(0)
    context.term()


def test_actual_rep_write_is_range_checked_and_read_back(rep):
    result = cam.remote_operation({'operation': 'set', 'role': 'right_wrist', 'name': 'gain', 'value': 42})
    assert result['readback'] == 42 and result['camera']['controls']['gain']['value'] == 42
    assert [r['op'] for r in rep.requests] == ['get_ctrls', 'set_ctrl', 'get_ctrls']
    with pytest.raises(ValueError, match='menu entry'):
        cam.remote_operation({'operation': 'set', 'role': 'right_wrist', 'name': 'auto_exposure', 'value': 2})
    assert [r['op'] for r in rep.requests].count('set_ctrl') == 1


def test_success_ack_without_readback_is_not_success(rep):
    rep.apply = False
    with pytest.raises(RuntimeError, match='readback did not confirm'):
        cam.remote_operation({'operation': 'set', 'role': 'right_wrist', 'name': 'gain', 'value': 42})


def test_missing_v4l2_is_explicit_and_no_request_is_sent(monkeypatch):
    monkeypatch.setattr('shutil.which', lambda _: None)
    monkeypatch.setattr(cam, '_request', lambda *args: pytest.fail('No request without v4l2'))
    state = cam._snapshot_role('left_wrist')
    assert not state['available'] and 'v4l2-ctl is not installed' in state['error']
    assert 'not supported' in cam._snapshot_role('head')['error']


@pytest.mark.parametrize('devices', [[], ['/dev/a', '/dev/b']])
def test_ambiguous_or_absent_usb_device_never_enables_writes(monkeypatch, devices):
    monkeypatch.setattr('shutil.which', lambda _: 'v4l2-ctl')
    monkeypatch.setattr(cam, '_devices', lambda _: devices)
    monkeypatch.setattr('subprocess.run', lambda *a, **k: SimpleNamespace(returncode=0, stdout=LIST))
    with pytest.raises(RuntimeError, match='missing or ambiguous'):
        cam._device_controls(cam.ROLES['right_wrist'], reported())


@pytest.mark.parametrize('camera_id,age,error', [('scene_cam', 0., None), ('wrong', 0., 'identity'), ('scene_cam', 5., 'stale')])
def test_actual_pub_preview_checks_identity_and_publisher_time(monkeypatch, camera_id, age, error):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    port = socket.bind_to_random_port('tcp://127.0.0.1')
    monkeypatch.setitem(cam.ROLES, 'head', dict(cam.ROLES['head'], frame_port=port))
    stop = threading.Event()
    jpeg = b'\xff\xd8fixture\xff\xd9'
    def publish():
        while not stop.wait(.01):
            header = {'camera_id': camera_id, 'timestamp': time.time() - age,
                      'color': {'format': 'jpeg'}, 'parts': ['color']}
            socket.send_multipart([msgpack.packb(header), jpeg])
    thread = threading.Thread(target=publish)
    thread.start()
    try:
        if error:
            with pytest.raises(ValueError, match=error):
                cam._preview('head')
        else:
            assert base64.b64decode(cam._preview('head')['jpeg']) == jpeg
    finally:
        stop.set()
        thread.join(1)
        socket.close(0)
        context.term()


class SSHChannel:
    def __init__(self, result):
        self.output = json.dumps(result).encode()
        self.closed = False
    def settimeout(self, value):
        assert value == 3.
    def shutdown_write(self): pass
    def recv_ready(self): return bool(self.output)
    def recv_stderr_ready(self): return False
    def recv(self, count):
        value, self.output = self.output[:count], self.output[count:]
        return value
    def exit_status_ready(self): return not self.output
    def recv_exit_status(self): return 0
    def close(self): self.closed = True


def test_pinned_profile_connection_and_cleanup_without_deployed_module(tmp_path):
    channel = SSHChannel({'ok': True, 'result': {'cameras': {}}})
    commands, source, connections = [], [], []
    ssh = SimpleNamespace(closed=False)
    def execute(command, timeout):
        commands.append(command)
        return SimpleNamespace(write=source.append, flush=lambda: None), SimpleNamespace(channel=channel), None
    ssh.exec_command = execute
    ssh.close = lambda: setattr(ssh, 'closed', True)
    def connect(profile, **kwargs):
        connections.append((profile, kwargs))
        return ssh
    manager = cam.CameraSettings(tmp_path/'profile.json', address=lambda: '10.42.0.1', open_ssh=connect)
    assert manager.snapshot() == {'cameras': {}}
    assert connections == [(tmp_path/'profile.json', {'address': '10.42.0.1', 'timeout': 3.})]
    assert commands == ['python3 -'] and "remote_main({'operation': 'snapshot'})" in source[0]
    assert ssh.closed and channel.closed
    assert not manager._lock.locked()


def test_identity_or_ssh_failure_becomes_explicit_camera_state(tmp_path):
    def denied(*a, **k):
        raise RuntimeError('Wrong robot computer')
    manager = cam.CameraSettings(tmp_path/'profile.json', address=lambda: '10.42.0.1', open_ssh=denied)
    assert all('Wrong robot' in state['error'] and not state['available'] for state in manager.snapshot()['cameras'].values())
    with pytest.raises(RuntimeError, match='Wrong robot'):
        manager.set('left_wrist', 'gain', 1)


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(ROOT/'config/devices/minerva-joint-calibration.json', runs=tmp_path/'runs',
        limits=tmp_path/'limits.json', seed=ROOT/'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


class FakeCameras:
    def __init__(self): self.calls = []
    def snapshot(self):
        self.calls.append(('get', threading.get_ident()))
        return {'cameras': {name: {'available': True, 'controls': {}} for name in cam.ROLES}}
    def set(self, role, name, value):
        self.calls.append(('set', threading.get_ident()))
        return {'readback': value}
    def frame(self, role):
        self.calls.append(('frame', threading.get_ident()))
        return b'jpeg'


class FakeTeleop:
    active = False
    task = {}
    def snapshot(self): return {'task': self.task}
    def guide(self): return None
    def stop(self): pass
    def start(self, **kwargs): self.active = True


def api(helm, cameras, scenario, *, simulation=False, teleop=None):
    from aiohttp.test_utils import TestClient, TestServer
    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), cameras=cameras,
                 teleop=teleop, simulation_only=simulation, token='secret'))) as client:
            await scenario(client)
    asyncio.run(run())


def test_routes_are_token_gated_off_loop_no_motor_reader(helm):
    backend = FakeCameras()
    loop_thread = threading.get_ident()
    async def scenario(client):
        response = await client.get('/api/helm/cameras')
        assert response.status == 200 and (await response.json())['can_apply']
        assert response.headers['Cache-Control'] == 'no-store'
        assert (await client.post('/api/helm/cameras/right_wrist', json={'name':'gain','value':1})).status == 403
        response = await client.post('/api/helm/cameras/right_wrist', json={'name':'gain','value':1}, headers={'X-Helm-Token':'secret'})
        assert response.status == 200 and (await response.json())['readback'] == 1
        response = await client.get('/api/helm/cameras/head/frame')
        assert response.content_type == 'image/jpeg' and await response.read() == b'jpeg'
        state = await (await client.get('/api/helm/state')).json()
        assert not state['live_view']['enabled'] and not state['link']['running']
    api(helm, backend, scenario)
    assert len(backend.calls) == 3 and all(thread != loop_thread for _, thread in backend.calls)


def test_simulation_routes_never_call_camera_backend(helm):
    backend = FakeCameras()
    async def scenario(client):
        for path in ('/api/helm/cameras', '/api/helm/cameras/head/frame'):
            assert (await client.get(path)).status == 422
        response = await client.post('/api/helm/cameras/right_wrist', json={'name':'gain','value':1}, headers={'X-Helm-Token':'secret'})
        assert response.status == 422
    api(helm, backend, scenario, simulation=True)
    assert backend.calls == []


@pytest.mark.parametrize('active,recording', [(True, {}), (False, {'active':True}), (False, {'state':'finishing'})])
def test_camera_mutation_blocked_by_session_or_capture(helm, active, recording):
    backend, teleop = FakeCameras(), FakeTeleop()
    teleop.active, teleop.task = active, {'recording':recording}
    async def scenario(client):
        state = await (await client.get('/api/helm/cameras')).json()
        assert not state['can_apply'] and 'Stop' in state['reason']
        response = await client.post('/api/helm/cameras/right_wrist', json={'name':'gain','value':1}, headers={'X-Helm-Token':'secret'})
        assert response.status == 422
    api(helm, backend, scenario, teleop=teleop)
    assert [call[0] for call in backend.calls] == ['get']


def test_pending_write_blocks_start_and_does_not_block_state(helm):
    backend, teleop = FakeCameras(), FakeTeleop()
    entered, release = threading.Event(), threading.Event()
    def change(*args):
        entered.set()
        assert release.wait(3)
        return {'readback':1}
    backend.set = change
    async def scenario(client):
        pending = asyncio.create_task(client.post('/api/helm/cameras/right_wrist', json={'name':'gain','value':1}, headers={'X-Helm-Token':'secret'}))
        try:
            for _ in range(100):
                if entered.is_set(): break
                await asyncio.sleep(.01)
            assert entered.is_set()
            assert (await client.get('/api/helm/state')).status == 200
            response = await client.post('/api/helm/teleop/start', json={}, headers={'X-Helm-Token':'secret'})
            assert response.status == 422 and not teleop.active
            assert 'camera change' in (await response.json())['error']
        finally:
            release.set()
            assert (await pending).status == 200
    api(helm, backend, scenario, teleop=teleop)


def test_no_live_cli_disables_automatic_reader_but_provides_lazy_cameras(monkeypatch, tmp_path):
    from aiohttp import web
    from robot_platform.helm import cli, server
    seen = {}
    monkeypatch.setattr(server, 'create_app', lambda *a, **kw: seen.update(kw))
    monkeypatch.setattr(web, 'run_app', lambda *a, **kw: None)
    monkeypatch.setattr(cam, 'CameraSettings', lambda *a, **kw: 'lazy-cameras')
    result = cli.main(['--profile', str(ROOT/'config/devices/minerva-joint-calibration.json'),
                       '--runs', str(tmp_path/'runs'), '--no-live'])
    assert result == 0 and seen['live'] is False and seen['cameras'] == 'lazy-cameras'
    assert seen['simulation_only'] is False


def test_stalled_ssh_result_closes_channel_and_releases_request_lock(tmp_path, monkeypatch):
    channel = SSHChannel({})
    channel.recv_ready = lambda: False
    channel.exit_status_ready = lambda: False
    client = SimpleNamespace(closed=False)
    client.exec_command = lambda *a, **kw: (SimpleNamespace(write=lambda _: None, flush=lambda: None), SimpleNamespace(channel=channel), None)
    client.close = lambda: setattr(client, 'closed', True)
    manager = cam.CameraSettings(tmp_path/'profile', address=lambda: '10.42.0.1', open_ssh=lambda *a, **kw: client)
    stamps = iter([0., 13.])
    monkeypatch.setattr(cam.time, 'monotonic', lambda: next(stamps))
    with pytest.raises(RuntimeError, match='timed out'):
        manager.set('right_wrist', 'gain', 10)
    assert client.closed and channel.closed and not manager._lock.locked()


def test_cancelled_http_write_keeps_start_fenced_until_remote_completion(helm):
    backend, teleop = FakeCameras(), FakeTeleop()
    entered, release = threading.Event(), threading.Event()
    def change(*args):
        entered.set()
        assert release.wait(3)
        return {'readback':1}
    backend.set = change
    class Request:
        headers = {'X-Helm-Token':'secret'}
        match_info = {'role':'right_wrist'}
        path = '/api/helm/cameras/right_wrist'
        async def json(self): return {'name':'gain','value':1}
    async def run():
        app = create_app(helm, SimulationLink(), cameras=backend, teleop=teleop, token='secret')
        handlers = {route.resource.canonical: route.handler for route in app.router.routes() if route.method == 'POST'}
        pending = asyncio.create_task(handlers['/api/helm/cameras/{role}'](Request()))
        try:
            for _ in range(100):
                if entered.is_set(): break
                await asyncio.sleep(.01)
            assert entered.is_set()
            pending.cancel()
            await asyncio.sleep(.01)
            with pytest.raises(ValueError, match='camera change'):
                await handlers['/api/helm/teleop/start'](Request())
            assert not teleop.active and not pending.done()
        finally:
            release.set()
            with pytest.raises(asyncio.CancelledError):
                await pending
    asyncio.run(run())


def test_delayed_legacy_set_remains_fenced_after_timeout_and_read_only_refresh(rep, tmp_path, monkeypatch):
    import ast
    rep.release_set = threading.Event()
    monkeypatch.setattr(cam, 'ROLES', {'right_wrist':cam.ROLES['right_wrist']})
    class LocalSSH:
        def exec_command(self, *args, **kwargs):
            source = []
            channel = SSHChannel({})
            def execute():
                request = ast.literal_eval(source[0].splitlines()[-1][len('remote_main('):-1])
                try:
                    response = {'ok':True, 'result':cam.remote_operation(request)}
                except Exception as exc:
                    response = {'ok':False, 'error':str(exc), 'write_uncertain':isinstance(exc, cam.CameraWriteUncertain)}
                channel.output = json.dumps(response).encode()
            channel.shutdown_write = execute
            return SimpleNamespace(write=source.append, flush=lambda:None), SimpleNamespace(channel=channel), None
        def close(self): pass
    manager = cam.CameraSettings(tmp_path/'profile', address=lambda:'local-fixture', open_ssh=lambda *a, **k: LocalSSH())
    try:
        with pytest.raises(cam.CameraWriteUncertain, match='outcome is uncertain'):
            manager.set('right_wrist', 'gain', 42)
        # The camera service has received the set request but has not applied it.
        assert rep.controls['gain']['value'] == 30
        assert manager.write_fault
    finally:
        rep.release_set.set()
    # An ordinary read after the delayed request applies does not authorize reuse.
    current = manager._call({'operation':'snapshot'})
    assert current['cameras']['right_wrist']['controls']['gain']['value'] == 42
    assert manager.write_fault
    with pytest.raises(ValueError, match='outcome is uncertain'):
        manager.set('right_wrist', 'gain', 44)
    assert [item['op'] for item in rep.requests].count('set_ctrl') == 1


def test_uncertain_write_blocks_session_start_capture_and_further_writes(helm):
    backend, teleop = FakeCameras(), FakeTeleop()
    backend.write_fault = 'Camera write outcome is uncertain; restart camera service and verify'
    async def scenario(client):
        state = await (await client.get('/api/helm/cameras')).json()
        assert not state['can_apply'] and 'uncertain' in state['reason']
        for path, body in [('/cameras/right_wrist', {'name':'gain','value':1}),
                           ('/teleop/start', {}), ('/task', {'action':'start_record'})]:
            response = await client.post('/api/helm'+path, json=body, headers={'X-Helm-Token':'secret'})
            assert response.status == 422 and 'uncertain' in (await response.json())['error']
        assert not teleop.active
    api(helm, backend, scenario, teleop=teleop)
    assert [call[0] for call in backend.calls] == ['get']


def test_remote_validation_rejection_does_not_latch_unknown_mutation(tmp_path):
    channel = SSHChannel({'ok':False, 'write_uncertain':False, 'error':'Value is not a reported camera menu entry'})
    client = SimpleNamespace(exec_command=lambda *a, **k: (SimpleNamespace(write=lambda _:None, flush=lambda:None),
                             SimpleNamespace(channel=channel), None), close=lambda:None)
    manager = cam.CameraSettings(tmp_path/'profile', address=lambda:'fixture', open_ssh=lambda *a, **k:client)
    with pytest.raises(ValueError, match='menu entry'):
        manager.set('right_wrist', 'auto_exposure', 2)
    assert manager.write_fault is None


def test_disconnect_while_sending_write_helper_latches_uncertainty(tmp_path):
    channel = SSHChannel({})
    def disconnected(_):
        raise OSError('link lost after sending some or all helper bytes')
    client = SimpleNamespace(exec_command=lambda *a, **k: (SimpleNamespace(write=disconnected, flush=lambda:None),
                             SimpleNamespace(channel=channel), None), close=lambda:None)
    manager = cam.CameraSettings(tmp_path/'profile', address=lambda:'fixture', open_ssh=lambda *a, **k:client)
    with pytest.raises(cam.CameraWriteUncertain, match='outcome is uncertain'):
        manager.set('right_wrist', 'gain', 42)
    assert manager.write_fault
