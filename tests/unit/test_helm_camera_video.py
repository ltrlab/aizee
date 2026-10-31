"""Persistent live video contracts using local sockets/child processes only."""
import asyncio
import io
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import msgpack
import pytest
import zmq
from PIL import Image

from robot_platform.helm.camera_video import CameraVideo, PacketDecoder, browser_packet
from robot_platform.helm.camera_video_worker import camera_packet, write_packet, MAX_HEADER, MAX_JPEG, ROLES
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession

ROOT = Path(__file__).resolve().parents[2]
buffer = io.BytesIO()
Image.new('RGB', (32, 24), 'green').save(buffer, format='JPEG')
JPEG = buffer.getvalue()


def header(**kwargs):
    return dict({'kind':'frame','role':'head','heartbeat':1,'source_age_s':.03,'source_timestamp':1000.}, **kwargs)


def packet(value, jpeg=JPEG):
    output = io.BytesIO()
    write_packet(output, value, jpeg)
    return output.getvalue()


def test_fragmented_packet_passthrough_and_bounded_header():
    encoded = packet(header())
    decoder, result = PacketDecoder(), []
    for index in range(0, len(encoded), 17):
        result.extend(decoder.feed(encoded[index:index+17]))
    assert result == [(header(), JPEG)] and not decoder.buffer
    with pytest.raises(ValueError, match='bound'):
        decoder.feed(struct.pack('!II', MAX_HEADER+1, 0))
    with pytest.raises(ValueError, match='bound'):
        PacketDecoder().feed(struct.pack('!II', 10, MAX_JPEG+1))
    binary = browser_packet({'role':'head','sequence':3,'age_s':.1}, JPEG)
    count = struct.unpack('!I', binary[:4])[0]
    assert json.loads(binary[4:4+count])['sequence'] == 3 and binary[4+count:] == JPEG


@pytest.mark.parametrize('identity,stamp,error', [('wrong', 1000., 'identity'), ('scene_cam', 998., 'stale'),
    ('scene_cam', float('nan'), 'missing'), ('scene_cam', 1001., 'future')])
def test_remote_validation_rejects_identity_and_stale_source(identity, stamp, error):
    raw = {'camera_id':identity,'timestamp':stamp,'color':{'format':'jpeg'},'parts':['color']}
    with pytest.raises(ValueError, match=error):
        camera_packet([msgpack.packb(raw), JPEG], 'scene_cam', 1000.)


def test_latest_cache_is_bounded_and_ssh_backlog_does_not_renew_age(tmp_path):
    clock = SimpleNamespace(now=10.)
    video = CameraVideo(tmp_path/'profile', address=lambda:'fixture', clock=lambda:clock.now)
    video._heartbeats.append((1, 9.9))
    for index in range(300):
        role = list(ROLES)[index % 3]
        video._accept(header(role=role,source_timestamp=1000.+index), JPEG)
    status, frames = video.snapshot()
    assert len(video._frames) == len(frames) == 3 and all(row['fresh'] for row in status['cameras'].values())
    assert all(row['sequence'] == 100 for row in status['cameras'].values())
    clock.now = 11.1
    # A delayed burst still references its old heartbeat, never the arrival clock.
    for _ in range(20):
        video._accept(header(), JPEG)
    status, frames = video.snapshot()
    assert not frames and not any(row['fresh'] for row in status['cameras'].values())
    video._heartbeats.append((2,11.1))
    video._accept(header(heartbeat=2),JPEG)
    assert video.snapshot()[1]['head'][0]['sequence'] == 101
    clock.now = 12.2
    assert video.snapshot()[1] == {}


def test_unknown_echo_nonfinite_age_or_invalid_role_are_rejected(tmp_path):
    video = CameraVideo(tmp_path/'profile', address=lambda:'fixture', clock=lambda:10.)
    video._heartbeats.append((1,9.9))
    for data in (header(heartbeat=2), header(source_age_s=float('nan')), header(role='unknown')):
        with pytest.raises(ValueError):
            video._accept(data,JPEG)
    assert not video.snapshot()[1]


class Channel:
    def __init__(self):
        self.buffer, self.lock = bytearray(), threading.Lock()
        self.closed = False
    def recv_ready(self):
        with self.lock: return bool(self.buffer)
    def recv_stderr_ready(self): return False
    def recv(self, size):
        with self.lock:
            data = bytes(self.buffer[:size]); del self.buffer[:size]
            return data
    def exit_status_ready(self): return self.closed
    def settimeout(self,value): pass
    def close(self): self.closed = True
    def send_heartbeat(self, line):
        if self.closed: raise OSError('closed')
        token = json.loads(line)['heartbeat']
        with self.lock:
            self.buffer.extend(packet({'kind':'status','heartbeat':token,'errors':{}}, b''))
            self.buffer.extend(packet(header(heartbeat=token)))


class Client:
    def __init__(self): self.channel = Channel(); self.closed = False
    def exec_command(self, command, timeout):
        assert command.startswith('python3 -u -c ') and 'set_ctrl' not in command
        return SimpleNamespace(write=self.channel.send_heartbeat,flush=lambda:None), SimpleNamespace(channel=self.channel), None
    def close(self): self.closed = True; self.channel.close()


def until(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.01)
    assert predicate(), 'condition did not become true'


def test_shared_subscriber_idle_shutdown_and_reconnect(tmp_path):
    clients = []
    def connect(*args,**kwargs):
        client=Client(); clients.append(client); return client
    video=CameraVideo(tmp_path/'profile',address=lambda:'fixture',open_ssh=connect)
    first, second=video.subscribe(),video.subscribe()
    try:
        until(lambda: bool(video.snapshot()[1]))
        assert len(clients)==1
        previous=video.snapshot()[0]['cameras']['head']['sequence']
        clients[0].channel.close()
        until(lambda:len(clients)>=2 and video.snapshot()[0]['cameras']['head']['sequence']>previous)
        video.unsubscribe(first)
        assert not clients[-1].closed
        video.unsubscribe(second)
        assert clients[-1].closed and video._thread is None and not video.snapshot()[1]
        third=video.subscribe()
        until(lambda:len(clients)==3 and bool(video.snapshot()[1]))
        video.unsubscribe(third)
    finally: video.close()
    assert all(c.closed for c in clients)
    with pytest.raises(RuntimeError,match='closed'): video.subscribe()


@pytest.mark.parametrize('end', ['eof','heartbeat'])
def test_actual_subscriber_exits_during_blocked_output_on_parent_loss(end):
    context=zmq.Context(); publisher=context.socket(zmq.PUB)
    port=publisher.bind_to_random_port('tcp://127.0.0.1')
    # Do not read stdout. Large frames fill its pipe; watchdog must still execute.
    code='from robot_platform.helm.camera_video_worker import run; run('+repr({'head':(port,'scene_cam')})+',heartbeat_timeout=.5)'
    process=subprocess.Popen([sys.executable,'-u','-c',code],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    stop=threading.Event()
    def heartbeat():
        while not stop.wait(.05):
            try: process.stdin.write(b'{"heartbeat":1}\n'); process.stdin.flush()
            except OSError: return
    thread=threading.Thread(target=heartbeat); thread.start()
    try:
        payload=b'\xff\xd8'+b'x'*(512*1024)+b'\xff\xd9'
        deadline=time.monotonic()+.8
        while time.monotonic()<deadline:
            raw={'camera_id':'scene_cam','timestamp':time.time(),'color':{'format':'jpeg'},'parts':['color']}
            publisher.send_multipart([msgpack.packb(raw),payload]); time.sleep(.02)
        stop.set(); thread.join(1)
        if end=='eof': process.stdin.close()
        assert process.wait(timeout=2)==0
    finally:
        stop.set(); thread.join(1)
        if process.poll() is None: process.kill(); process.wait()
        for handle in (process.stdin,process.stdout,process.stderr):
            if handle and not handle.closed: handle.close()
        publisher.close(0); context.term()


def test_actual_local_pub_relay_delivers_passthrough_frames_at_bounded_rate():
    context=zmq.Context(); publisher=context.socket(zmq.PUB)
    port=publisher.bind_to_random_port('tcp://127.0.0.1')
    code='from robot_platform.helm.camera_video_worker import run; run('+repr({'head':(port,'scene_cam')})+')'
    process=subprocess.Popen([sys.executable,'-u','-c',code],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    stop=threading.Event(); received=[]
    def read():
        decoder=PacketDecoder()
        while True:
            chunk=process.stdout.read1(65536)
            if not chunk: return
            for data,jpeg in decoder.feed(chunk):
                if jpeg: received.append((time.monotonic(),data,jpeg))
    reader=threading.Thread(target=read); reader.start()
    try:
        start=time.monotonic(); next_heartbeat=start
        while time.monotonic()-start<1.6:
            now=time.monotonic()
            if now>=next_heartbeat:
                process.stdin.write(b'{"heartbeat":3}\n'); process.stdin.flush(); next_heartbeat=now+.1
            raw={'camera_id':'scene_cam','timestamp':time.time(),'color':{'format':'jpeg'},'parts':['color']}
            publisher.send_multipart([msgpack.packb(raw),JPEG]); time.sleep(.015)
        assert 6<=len(received)<=17
        assert all(item[2]==JPEG and item[1]['heartbeat']==3 for item in received)
        assert all(b[0]-a[0]>.07 for a,b in zip(received,received[1:]))
    finally:
        process.stdin.close(); process.wait(timeout=2); reader.join(1)
        process.stdout.close(); process.stderr.close(); publisher.close(0); context.term()


@pytest.fixture
def helm(tmp_path):
    session=HelmSession(ROOT/'config/devices/minerva-joint-calibration.json',runs=tmp_path/'runs',
        limits=tmp_path/'limits',seed=ROOT/'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


class VideoFixture:
    def __init__(self): self.viewers=set(); self.closed=False; self.depth=[]
    def subscribe(self, depth=False):
        token=object(); self.viewers.add(token); self.depth.append(depth); return token
    def unsubscribe(self,token): self.viewers.discard(token)
    def close(self): self.closed=True; self.viewers.clear()
    def snapshot(self):
        return ({'type':'status','cameras':{'head':{'fresh':True,'age_s':.02,'sequence':1,'error':None}},
                 'target_fps':10,'stale_after_s':1.},
                {'head':({'role':'head','sequence':1,'age_s':.02,'source_timestamp':time.time()},JPEG)})


def test_websocket_auth_origin_echo_and_shutdown(helm):
    from aiohttp.test_utils import TestClient,TestServer
    from aiohttp import WSServerHandshakeError,WSMsgType
    video=VideoFixture()
    async def run():
        app=create_app(helm,SimulationLink(),camera_video=video,token='secret')
        async with TestClient(TestServer(app)) as client:
            for path,headers in [('/api/helm/cameras/live',{}),('/api/helm/cameras/live?token=secret',{'Origin':'https://elsewhere'})]:
                with pytest.raises(WSServerHandshakeError) as error: await client.ws_connect(path,headers=headers)
                assert error.value.status==403
            ws=await client.ws_connect('/api/helm/cameras/live?token=secret')
            message=await ws.receive(timeout=1)
            assert message.type==WSMsgType.TEXT and message.json()['viewer_heartbeat'] is None
            await ws.send_json({'type':'heartbeat','sequence':7})
            while True:
                message=await ws.receive(timeout=1)
                if message.type==WSMsgType.BINARY: break
            count=struct.unpack('!I',message.data[:4])[0]
            assert json.loads(message.data[4:4+count])['viewer_heartbeat']==7
            assert message.data[4+count:]==JPEG
            assert (await client.get('/api/helm/state')).status==200
            await ws.close()
            for _ in range(100):
                if not video.viewers: break
                await asyncio.sleep(.01)
            assert not video.viewers
    asyncio.run(run())
    assert video.closed


def test_simulation_cannot_subscribe(helm):
    from aiohttp.test_utils import TestClient,TestServer
    from aiohttp import WSServerHandshakeError
    video=VideoFixture()
    async def run():
        async with TestClient(TestServer(create_app(helm,SimulationLink(),camera_video=video,simulation_only=True,token='secret'))) as client:
            with pytest.raises(WSServerHandshakeError) as error:
                await client.ws_connect('/api/helm/cameras/live?token=secret')
            assert error.value.status==422 and not video.viewers
    asyncio.run(run())


def test_slow_websocket_has_bounded_send_and_does_not_block_state(helm,monkeypatch):
    from aiohttp import web
    from aiohttp.test_utils import TestClient,TestServer
    original=web.WebSocketResponse
    blocked=asyncio.Event()
    class SlowSocket(original):
        async def send_bytes(self,*a,**kw):
            blocked.set()
            await asyncio.sleep(5)
    monkeypatch.setattr(web,'WebSocketResponse',SlowSocket)
    video=VideoFixture()
    async def run():
        async with TestClient(TestServer(create_app(helm,SimulationLink(),camera_video=video,token='secret'))) as client:
            ws=await client.ws_connect('/api/helm/cameras/live?token=secret')
            await ws.send_json({'type':'heartbeat','sequence':1})
            await asyncio.wait_for(blocked.wait(),1)
            response=await asyncio.wait_for(client.get('/api/helm/state'),.25)
            assert response.status==200
            # Slow consumer is disconnected instead of building an application queue.
            deadline=asyncio.get_running_loop().time()+2
            while not ws.closed and asyncio.get_running_loop().time()<deadline:
                await ws.receive(timeout=2)
            assert ws.closed
    asyncio.run(run())
    assert not video.viewers


def test_stalled_video_shutdown_cannot_delay_guarded_motor_stop(helm, monkeypatch):
    from aiohttp import web
    from aiohttp.test_utils import TestClient, TestServer
    original=web.WebSocketResponse
    release=asyncio.Event()
    stop_called=threading.Event()
    class StalledClose(original):
        async def close(self,*a,**kw):
            await release.wait()
            return await super().close(*a,**kw)
    monkeypatch.setattr(web,'WebSocketResponse',StalledClose)
    teleop=SimpleNamespace(active=False, snapshot=lambda:{},guide=lambda:None,stop=stop_called.set)
    async def run():
        app=create_app(helm,SimulationLink(),teleop=teleop,camera_video=VideoFixture(),token='secret')
        async with TestClient(TestServer(app)) as client:
            ws=await client.ws_connect('/api/helm/cameras/live?token=secret')
            await ws.receive(timeout=1)
            shutdown=asyncio.create_task(app.shutdown())
            try:
                for _ in range(25):
                    if stop_called.is_set(): break
                    await asyncio.sleep(.01)
                assert stop_called.is_set(), 'Camera close delayed the guarded motor stop'
            finally:
                release.set()
                await asyncio.wait_for(shutdown,2)
                await ws.close()
    asyncio.run(run())


class MovingVideo(VideoFixture):
    """A camera whose frame is new at every look."""
    def __init__(self):
        super().__init__()
        self.sequence = 0

    def snapshot(self):
        self.sequence += 1
        status, _frames = super().snapshot()
        return status, {'head': ({'role': 'head', 'sequence': self.sequence, 'age_s': .02,
                                  'source_timestamp': time.time()}, JPEG)}


def test_a_slow_link_gets_fewer_frames_and_keeps_its_stream(helm, monkeypatch):
    from aiohttp import WSMsgType
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm import server
    backlog, seen = {'bytes': 0}, []

    def fake_backlog(transport):
        seen.append(transport)
        return backlog['bytes']
    monkeypatch.setattr(server, 'video_backlog', fake_backlog)

    async def run():
        loop = asyncio.get_running_loop()
        async with TestClient(TestServer(create_app(helm, SimulationLink(), camera_video=MovingVideo(),
                                                    token='secret'))) as client:
            ws = await client.ws_connect('/api/helm/cameras/live?token=secret')
            beats = iter(range(1, 10 ** 6))

            async def collect(seconds):
                frames, links, end = 0, [], loop.time() + seconds
                while loop.time() < end:
                    await ws.send_json({'type': 'heartbeat', 'sequence': next(beats)})  # The browser's heartbeats.
                    try:
                        message = await ws.receive(timeout=.3)
                    except asyncio.TimeoutError:
                        continue
                    if message.type == WSMsgType.BINARY:
                        frames += 1
                    elif message.type == WSMsgType.TEXT:
                        links.append(message.json().get('link'))
                    else:
                        break
                return frames, links
            fast, _ = await collect(1.)
            backlog['bytes'] = server.VIDEO_BACKLOG + 1  # The link falls behind, as Minerva's Wi-Fi did.
            slow, links = await collect(2.)
            assert not ws.closed and slow <= 1 and links[-1] == 'slow'  # Fewer frames, the stream kept.
            backlog['bytes'] = 0
            resumed, links = await collect(server.VIDEO_SLOW_S + .8)  # It says slow for VIDEO_SLOW_S after a skip.
            assert fast > 3 and resumed > 3 and links[-1] == 'ok'
            transport = next(item for item in seen if item is not None)
            assert transport.get_write_buffer_limits()[1] == server.VIDEO_HIGH_WATER  # Never paused under the backlog.
            import socket
            kernel = transport.get_extra_info('socket').getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
            assert server.VIDEO_SOCKET_BUFFER <= kernel <= 2 * server.VIDEO_SOCKET_BUFFER  # No second of video queued.
            await ws.close()
    asyncio.run(run())


class TwoCameras(VideoFixture):
    """The head and a wrist camera, each with a new frame at every look."""
    def __init__(self):
        super().__init__()
        self.sequence = 0

    def snapshot(self):
        self.sequence += 1
        status, _frames = super().snapshot()
        return status, {role: ({'role': role, 'sequence': self.sequence, 'age_s': .02, 'source_timestamp': time.time()},
                               JPEG) for role in ('head', 'left_wrist')}


def test_a_viewer_can_ask_for_some_cameras_only(helm):
    """The headset's screen asks for the head's frames (?roles=head): the wrists' never reach it."""
    from aiohttp import WSMsgType
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), camera_video=TwoCameras(),
                                                    token='secret'))) as client:
            async def roles(path):
                ws = await client.ws_connect(path)
                seen, frames = set(), 0
                for beat in range(1, 60):
                    await ws.send_json({'type': 'heartbeat', 'sequence': beat})
                    message = await ws.receive(timeout=1)
                    if message.type == WSMsgType.BINARY:
                        count = struct.unpack('!I', message.data[:4])[0]
                        seen.add(json.loads(message.data[4:4 + count])['role'])
                        frames += 1
                        if frames >= 8:
                            break
                await ws.close()
                assert frames >= 8
                return seen
            assert await roles('/api/helm/cameras/live?token=secret&roles=head') == {'head'}
            assert await roles('/api/helm/cameras/live?token=secret') == {'head', 'left_wrist'}
    asyncio.run(run())


def test_closing_the_last_viewer_tolerates_a_relay_already_gone():
    video = CameraVideo(ROOT / 'config/devices/minerva-joint-calibration.json', address=lambda: '127.0.0.1')

    class Gone:
        def close(self):
            raise EOFError()  # As paramiko does when its transport has ended.
    token = object()
    video._viewers.add(token)
    video._client, video._channel = Gone(), Gone()
    video.unsubscribe(token)
    assert not video._viewers and video._error == 'No camera viewers'


# The head camera's depth for the headset's 3D picture: compressed on the robot only while a headset asks.
DEPTH_K = {'fx': 4., 'fy': 5., 'cx': 3.5, 'cy': 2.5, 'width': 8, 'height': 6}
COLOR_K = {'fx': 40., 'fy': 42., 'cx': 31.5, 'cy': 23.5, 'width': 64, 'height': 48}
TO_COLOR = {'rotation': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], 'translation': [.015, 0., 0.]}


def depth_parts(depth, stamp=1000., registered=True):
    """The head camera's multipart: header, JPEG and its raw depth (camera_node's)."""
    import numpy as np
    raw = {'camera_id': 'scene_cam', 'timestamp': stamp, 'parts': ['color', 'depth'],
           'color': dict({'format': 'jpeg', 'width': 64, 'height': 48}, **({'intrinsics': COLOR_K} if registered else {})),
           'depth': dict({'format': 'uint16', 'width': depth.shape[1], 'height': depth.shape[0], 'intrinsics': DEPTH_K,
                          'intrinsics_frame': 'sensor_optical', 'scale': .001},
                         **({'to_color': TO_COLOR} if registered else {}))}
    return [msgpack.packb(raw), JPEG, np.ascontiguousarray(depth, dtype='<u2').tobytes()]


def test_the_relay_compresses_the_head_cameras_depth_for_a_headset():
    import numpy as np
    import zlib
    from robot_platform.helm.camera_video_worker import DEPTH_RANGE_M, head_depth
    depth = np.zeros((6, 8), dtype=np.uint16)
    depth[0:2, 0:2] = [[0, 900], [1200, 0]]  # The nearest reading of each 2x2 block,
    depth[0:2, 2:4] = [[100, 1000], [3000, 0]]  # its readings out of range set aside (0.15-2.5 m),
    depth[2:4, 0:2] = [[4000, 0], [0, 120]]  # and none left: no reading.
    depth[4:6, 6:8] = 2000
    meta, packed = head_depth(depth_parts(depth))
    near = np.frombuffer(zlib.decompress(packed), dtype='<u2').reshape(3, 4)
    expected = np.zeros((3, 4), dtype=np.uint16)
    expected[0, 0], expected[0, 1], expected[2, 3] = 900, 1000, 2000
    np.testing.assert_array_equal(near, expected)
    assert meta['bytes'] == len(packed) and meta['encoding'] == 'zlib-u16le' and meta['range_m'] == list(DEPTH_RANGE_M)
    assert (meta['width'], meta['height'], meta['scale']) == (4, 3, .001)
    # A half-size pixel's ray is its 2x2 block's centre ray.
    k = meta['intrinsics']
    for u, v in ((0, 0), (3, 2), (1, 1)):
        assert ((u - k['cx']) / k['fx'], (v - k['cy']) / k['fy']) == pytest.approx(
            ((2 * u + .5 - DEPTH_K['cx']) / DEPTH_K['fx'], (2 * v + .5 - DEPTH_K['cy']) / DEPTH_K['fy']))
    assert meta['color_intrinsics'] == COLOR_K and meta['to_color'] == TO_COLOR
    assert 'to_color' not in head_depth(depth_parts(depth, registered=False))[0]  # An older camera: depth only.
    assert head_depth([msgpack.packb({'parts': ['color']}), JPEG]) is None
    with pytest.raises(ValueError, match='Malformed'):
        head_depth(depth_parts(np.zeros((5, 8), dtype=np.uint16)))


def test_the_relay_keeps_to_the_background_cpus(tmp_path, monkeypatch):
    from robot_platform.helm import camera_video_worker
    moved = []
    monkeypatch.setattr(camera_video_worker.os, 'sched_setaffinity', lambda pid, cpus: moved.append((pid, cpus)),
                        raising=False)
    camera_video_worker.keep_off_control_cpus(tmp_path / 'none.json')  # No CPU split: it stays where it is.
    assert moved == []
    split = tmp_path / 'cpu-partition.json'
    split.write_text(json.dumps({'control': [4, 5], 'background': [0, 1, 2, 3]}), encoding='utf-8')
    camera_video_worker.keep_off_control_cpus(split)
    assert moved == [(0, {0, 1, 2, 3})]


def test_helm_serves_the_head_frame_with_its_depth_as_a_role_of_its_own(tmp_path):
    import numpy as np
    from robot_platform.helm.camera_video import DEPTH_ROLE
    from robot_platform.helm.camera_video_worker import head_depth
    video = CameraVideo(tmp_path/'profile', address=lambda: 'fixture', clock=lambda: 10.)
    video._heartbeats.append((1, 9.9))
    meta, packed = head_depth(depth_parts(np.full((6, 8), 1000, dtype=np.uint16)))
    video._accept(header(jpeg_bytes=len(JPEG), depth=meta), JPEG + packed)
    _status, frames = video.snapshot()
    assert frames['head'][1] == JPEG  # Every other viewer sees the head's picture as before.
    depth_header, payload = frames[DEPTH_ROLE]
    assert depth_header['role'] == DEPTH_ROLE and depth_header['sequence'] == frames['head'][0]['sequence']
    assert depth_header['jpeg_bytes'] == len(JPEG) and depth_header['depth'] == meta and payload == JPEG + packed
    video._accept(header(source_timestamp=1001.), JPEG)  # A frame without depth: no depth to serve.
    assert DEPTH_ROLE not in video.snapshot()[1]
    for bad in (header(jpeg_bytes=len(JPEG), depth=dict(meta, bytes=1)),
                header(jpeg_bytes=len(JPEG) + 5, depth=meta),
                header(jpeg_bytes=len(JPEG), depth=dict(meta, encoding='png')),
                header(role='left_wrist', jpeg_bytes=len(JPEG), depth=meta)):
        with pytest.raises(ValueError):
            video._accept(bad, JPEG + packed)


def test_helm_asks_the_relay_for_depth_only_while_a_headset_wants_it(tmp_path):
    lines = []

    class Watching(Channel):
        def send_heartbeat(self, line):
            lines.append(json.loads(line))
            super().send_heartbeat(line)

    class Watched(Client):
        def __init__(self):
            super().__init__()
            self.channel = Watching()
    video = CameraVideo(tmp_path/'profile', address=lambda: 'fixture', open_ssh=lambda *args, **kwargs: Watched())
    plain = video.subscribe()
    try:
        until(lambda: len(lines) >= 2)
        assert not any(line['depth'] for line in lines)
        headset = video.subscribe(depth=True)
        count = len(lines)
        until(lambda: len(lines) >= count + 2)
        assert lines[-1]['depth'] is True
        video.unsubscribe(headset)
        count = len(lines)
        until(lambda: len(lines) >= count + 2)
        assert lines[-1]['depth'] is False
        video.unsubscribe(plain)
    finally:
        video.close()


class DepthCamera(VideoFixture):
    """The head's picture, and the same frame with its depth."""
    def __init__(self):
        super().__init__()
        self.sequence = 0

    def snapshot(self):
        self.sequence += 1
        status, _frames = super().snapshot()
        base = {'sequence': self.sequence, 'age_s': .02, 'source_timestamp': time.time()}
        return status, {'head': (dict(base, role='head'), JPEG),
                        'head_depth': (dict(base, role='head_depth', jpeg_bytes=len(JPEG), depth={'bytes': 3}),
                                       JPEG + b'zzz')}


def test_a_headset_asking_for_depth_gets_the_heads_frames_with_their_depth(helm):
    from aiohttp import WSMsgType
    from aiohttp.test_utils import TestClient, TestServer
    video = DepthCamera()

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), camera_video=video,
                                                    token='secret'))) as client:
            ws = await client.ws_connect('/api/helm/cameras/live?token=secret&roles=head_depth')
            seen = []
            for beat in range(1, 60):
                await ws.send_json({'type': 'heartbeat', 'sequence': beat})
                message = await ws.receive(timeout=1)
                if message.type == WSMsgType.BINARY:
                    count = struct.unpack('!I', message.data[:4])[0]
                    seen.append((json.loads(message.data[4:4 + count])['role'], message.data[4 + count:]))
                    if len(seen) >= 4:
                        break
            await ws.close()
            assert seen and all(item == ('head_depth', JPEG + b'zzz') for item in seen)
            assert video.depth == [True]
    asyncio.run(run())


def test_the_actual_relay_sends_depth_after_the_heads_jpeg_while_asked():
    import numpy as np
    import zlib
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    port = publisher.bind_to_random_port('tcp://127.0.0.1')
    code = 'from robot_platform.helm.camera_video_worker import run; run(' + repr({'head': (port, 'scene_cam')}) + ')'
    process = subprocess.Popen([sys.executable, '-u', '-c', code], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    received = []

    def read():
        decoder = PacketDecoder()
        while True:
            chunk = process.stdout.read1(65536)
            if not chunk:
                return
            for data, payload in decoder.feed(chunk):
                if payload:
                    received.append((data, payload))
    reader = threading.Thread(target=read)
    reader.start()
    depth = np.full((6, 8), 1500, dtype=np.uint16)
    try:
        for depth_wanted in (False, True):
            count, start, next_heartbeat = len(received), time.monotonic(), 0.
            line = json.dumps({'heartbeat': 4, 'depth': depth_wanted}).encode() + b'\n'
            while time.monotonic() - start < 1.:
                if time.monotonic() >= next_heartbeat:
                    process.stdin.write(line)
                    process.stdin.flush()
                    next_heartbeat = time.monotonic() + .1
                publisher.send_multipart(depth_parts(depth, stamp=time.time()))
                time.sleep(.015)
            later = received[count + 2:]  # After the relay has the new heartbeat.
            assert later
            if not depth_wanted:
                assert all('depth' not in data and payload == JPEG for data, payload in later)
            else:
                for data, payload in later:
                    assert payload[:data['jpeg_bytes']] == JPEG and data['depth']['bytes'] == len(payload) - len(JPEG)
                    near = np.frombuffer(zlib.decompress(payload[len(JPEG):]), dtype='<u2')
                    assert near.shape == (12,) and (near == 1500).all()
    finally:
        process.stdin.close()
        process.wait(timeout=2)
        reader.join(1)
        process.stdout.close()
        process.stderr.close()
        publisher.close(0)
        context.term()
