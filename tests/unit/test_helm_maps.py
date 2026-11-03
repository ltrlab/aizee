"""Splat maps in Helm without hardware: the robot's head scene capture from the head camera's stream and the head's
pose feed (kept as the head turns and holds), read back by the splat trainer; Minerva's pose in each map's world;
the maps and captures Helm lists and serves; and the API."""
import asyncio
import io
import json
import math
from pathlib import Path
import sys
import threading
import time

import msgpack
import numpy as np
import pytest
import zmq
from PIL import Image

from robot_platform.helm import scene_capture as sc
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.maps import MapLibrary, column_major
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.neck import head
from robot_platform.neck.head_pose import ALIGNMENT_KIND, pose7, reference_digest
from robot_platform.vision.geometry import invert, transform

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/splat'))
import train_head_scene  # noqa: E402

COLOR_K = {'fx': 614.3, 'fy': 614.7, 'cx': 313.3, 'cy': 237., 'width': 640, 'height': 480}
DEPTH_K = {'fx': 382., 'fy': 382., 'cx': 322.9, 'cy': 238.1, 'width': 640, 'height': 480}
TO_COLOR = {'rotation': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], 'translation': [.0149, 0., 0.]}
REFERENCE = {'motors': [{'name': 'head_pan', 'raw_reference_rad': 3.1}]}
CAMERA_AT_ZERO = transform(np.array([[-1., 0., 0.], [0., 0., -1.], [0., -1., 0.]]), (.03, -.11, 1.27))
STEP_RAD, HOLD_S = math.radians(5.), .4  # The head turns in steps and holds between them.


def jpeg():
    buffer = io.BytesIO()
    Image.fromarray(np.full((480, 640, 3), 120, np.uint8)).save(buffer, format='JPEG')
    return buffer.getvalue()


def stream_head(stop, frames_endpoint, feed_endpoint, started):
    """The head camera's frames at 15 a second with depth and registration, and the head's feed at 100 Hz: the
    neck panning a step every HOLD_S, the camera's pose with it."""
    context = zmq.Context()
    frames, feed = context.socket(zmq.PUB), context.socket(zmq.PUB)
    frames.bind(frames_endpoint)
    feed.bind(feed_endpoint)
    picture, depth = jpeg(), np.full((480, 640), 900, '<u2').tobytes()
    try:
        k = 0
        while not stop.is_set():
            now = time.time()
            pan = STEP_RAD * math.floor((now - started) / HOLD_S)
            turn = np.eye(4)
            turn[:2, :2] = [[math.cos(pan), -math.sin(pan)], [math.sin(pan), math.cos(pan)]]
            feed.send_string(json.dumps({'joints': ['neck_z', 'neck_y'], 'zero': pose7(CAMERA_AT_ZERO),
                                         'samples': [[now, pan, -.5, *pose7(turn @ CAMERA_AT_ZERO)]]}))
            if k % 7 == 0:
                header = {'camera_id': 'scene_cam', 'timestamp': now, 'parts': ['color', 'depth'],
                          'color': {'format': 'jpeg', 'width': 640, 'height': 480, 'intrinsics': COLOR_K},
                          'depth': {'format': 'uint16', 'width': 640, 'height': 480, 'intrinsics': DEPTH_K,
                                    'scale': .001, 'to_color': TO_COLOR}}
                frames.send_multipart([msgpack.packb(header, use_bin_type=True), picture, depth])
            k += 1
            time.sleep(.01)
    finally:
        frames.close(0)
        feed.close(0)
        context.term()


def test_a_capture_keeps_steady_turned_frames_and_the_trainer_reads_it(tmp_path, monkeypatch):
    monkeypatch.setattr(sc, 'CAPTURE_ROOT', str(tmp_path / 'captures'))
    frames_endpoint, feed_endpoint = 'tcp://127.0.0.1:25574', 'tcp://127.0.0.1:25597'
    robot_in_world = transform(np.array([0., 0., .3]), (.4, -.2, 0.))
    stop = threading.Event()
    thread = threading.Thread(target=stream_head, args=(stop, frames_endpoint, feed_endpoint, time.time()),
                              daemon=True)
    thread.start()
    try:
        reply = sc.scene({'operation': 'scene', 'seconds': 2.5, 'fps': 3, 'world': 'board',
                          'folder': str(tmp_path / 'captures' / '20261101T000000Z'),
                          'robot_in_world': pose7(robot_in_world)},
                         frames_endpoint=frames_endpoint, feed_endpoint=feed_endpoint)
    finally:
        stop.set()
        thread.join(2)
    counts = reply['counts']
    assert 3 <= reply['frames'] <= 8 and counts['seen'] > reply['frames']
    assert counts['same_view'] + counts['rate'] + counts['turning'] > 0  # Holding, too soon, or mid-step.
    folder = Path(reply['folder'])
    scene = json.loads((folder / 'scene.json').read_text(encoding='utf-8'))
    assert scene['world'] == 'board' and scene['color_intrinsics'] == COLOR_K and scene['to_color'] == TO_COLOR
    pans = [frame['joints']['neck_z'] for frame in scene['frames']]
    assert all(b - a >= math.radians(3.) - 1e-9 for a, b in zip(pans, pans[1:]))  # Each turned on from the last.
    images, poses, color_k, depths = train_head_scene.load(folder)
    assert images.shape == (reply['frames'], 480, 640, 3) and color_k[0, 0] == pytest.approx(614.3)
    first = scene['frames'][0]
    np.testing.assert_allclose(poses[0], robot_in_world @ train_head_scene.pose_matrix(first['pose']), atol=1e-9)
    assert (depths[0] > 0).mean() > .3 and np.median(depths[0][depths[0] > 0]) == pytest.approx(.9, abs=.01)


def test_a_capture_needs_the_live_heads_feed_and_its_own_folder(tmp_path, monkeypatch):
    monkeypatch.setattr(sc, 'CAPTURE_ROOT', str(tmp_path / 'captures'))
    with pytest.raises(ValueError, match='saved under'):
        sc.scene({'seconds': 5, 'folder': str(tmp_path / 'elsewhere')})
    with pytest.raises(ValueError, match='seconds must be'):
        sc.scene({'seconds': 500, 'folder': str(tmp_path / 'captures' / 'x')})
    with pytest.raises(ValueError, match='feed is silent'):
        sc.scene({'seconds': 5, 'folder': str(tmp_path / 'captures' / 'x')}, feed_endpoint='tcp://127.0.0.1:25598')


# -- the library ----------------------------------------------------------------------------------------------------

BOARD = transform(np.array([0., 0., math.pi]), (.325, -.158, .512))  # The board's place in Minerva's frame.


def head_files(tmp_path, reference=REFERENCE, aligned_for=REFERENCE):
    files = head.files(tmp_path / 'operator', 'minerva-production-01')
    files['reference'].parent.mkdir(parents=True, exist_ok=True)
    files['reference'].write_text(json.dumps(reference), encoding='utf-8')
    if aligned_for is not None:
        files['camera'].write_text(json.dumps({
            'schema': 1, 'kind': ALIGNMENT_KIND, 'link': 'awg_rgb_camera_5',
            'reference_sha256': reference_digest(aligned_for),
            'mount': {'matrix': transform(np.array([1.5, -1.2, 1.2]), (-.11, -.08, -.05)).tolist()},
            'board_in_robot': {'matrix': BOARD.tolist()}}), encoding='utf-8')
    return files


class Session:
    def __init__(self, state='TELEOP', mode='live'):
        self.state, self.mode = state, mode

    def __call__(self):
        return dict(phase='running', receiver_choice={'mode': self.mode}, head={'state': self.state}, task={})


def library(tmp_path, files, session=None, remote=None):
    times = iter(range(1793000000, 1793999999, 7))
    return MapLibrary(teleop=session or Session(), root=tmp_path / 'maps', files=lambda: files, remote=remote,
                      clock=lambda: float(next(times)))


def test_minervas_pose_in_the_boards_world_comes_from_a_current_alignment(tmp_path):
    poses = library(tmp_path, head_files(tmp_path)).worlds()
    np.testing.assert_allclose(poses['robot'], np.eye(4))
    np.testing.assert_allclose(poses['board'], invert(BOARD), atol=1e-12)
    # Made through another head reference, or none saved: no board world.
    assert library(tmp_path / 'b', head_files(tmp_path / 'b', aligned_for={'motors': []})).worlds()['board'] is None
    assert library(tmp_path / 'c', head_files(tmp_path / 'c', aligned_for=None)).worlds()['board'] is None


def test_the_library_lists_maps_and_captures_newest_first_and_serves_a_map(tmp_path):
    maps = library(tmp_path, head_files(tmp_path))
    for name, world in (('20261101T010000Z', 'robot'), ('20261101T020000Z', 'board')):
        folder = tmp_path / 'maps' / 'maps' / name
        folder.mkdir(parents=True)
        (folder / 'map.splat').write_bytes(b'\0' * 64)
        (folder / 'map.json').write_text(json.dumps({'world': world, 'gaussians': 2, 'frames': 40,
                                                     'psnr_held_out': 31.2, 'capture': name}), encoding='utf-8')
    capture = tmp_path / 'maps' / 'captures' / '20261101T020000Z'
    capture.mkdir(parents=True)
    (capture / 'scene.json').write_text(json.dumps({'world': 'board', 'frames': [{}, {}, {}],
                                                    'captured_at': '2026-11-01T02:00:00Z'}), encoding='utf-8')
    (tmp_path / 'maps' / 'maps' / 'bad name!').mkdir()
    listing = maps.listing()
    assert [m['name'] for m in listing['maps']] == ['20261101T020000Z', '20261101T010000Z']
    assert listing['maps'][0] == dict(name='20261101T020000Z', world='board', gaussians=2, frames=40, psnr=31.2,
                                      capture='20261101T020000Z', bytes=64,
                                      trained_at=listing['maps'][0]['trained_at'])
    assert listing['captures'] == [dict(name='20261101T020000Z', frames=3, world='board',
                                        captured_at='2026-11-01T02:00:00Z', trained=True)]
    assert listing['world'] == 'board'
    assert listing['robot_in_world']['board'] == pytest.approx(column_major(invert(BOARD)))
    assert listing['robot_in_world']['robot'] == column_major(np.eye(4))
    assert maps.splat('20261101T010000Z').read_bytes() == b'\0' * 64
    with pytest.raises(ValueError, match='Not a map name'):
        maps.splat('../operator')
    with pytest.raises(FileNotFoundError):
        maps.splat('20261101T030000Z')
    # Column-major, as three.js reads a matrix: the translation in the last four.
    assert column_major(transform(np.eye(3), (1., 2., 3.)))[12:15] == [1., 2., 3.]


def test_a_capture_records_minervas_pose_in_the_world_it_will_be_drawn_in(tmp_path):
    requests = []
    remote = lambda request, timeout_s: requests.append((request, timeout_s)) or {  # noqa: E731
        'name': '20261101T000000Z', 'frames': 12, 'counts': {'seen': 60}, 'bytes': 3e6, 'world': request['world']}
    maps = library(tmp_path, head_files(tmp_path), remote=remote)
    state = maps.capture(30)
    request, timeout_s = requests[0]
    assert request['operation'] == 'scene' and request['seconds'] == 30 and timeout_s == 90.
    assert request['world'] == 'board' and request['robot_in_world'] == pytest.approx(pose7(invert(BOARD)))
    assert request['folder'].startswith(sc.CAPTURE_ROOT + '/')
    assert state['last']['frames'] == 12 and state['busy'] is False
    # Without an alignment the capture is in Minerva's own frame.
    maps = library(tmp_path / 'b', head_files(tmp_path / 'b', aligned_for=None), remote=remote)
    maps.capture(15)
    assert requests[-1][0]['world'] == 'robot' and requests[-1][0]['robot_in_world'] is None
    for session, message in ((Session(state='IDLE'), r'Enable the head \(E\)'), (Session(mode='virtual'), 'real head')):
        with pytest.raises(ValueError, match=message):
            library(tmp_path, head_files(tmp_path), session=session, remote=remote).capture(30)
    with pytest.raises(ValueError, match='5 to 120 seconds'):
        maps.capture(500)


# -- the API --------------------------------------------------------------------------------------------------------

@pytest.fixture
def helm(tmp_path):
    session = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=tmp_path / 'limits.json',
                          seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


def test_the_maps_api_serves_the_headset_and_captures_off_the_event_loop(helm, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    calls, loop_thread = [], threading.get_ident()
    remote = lambda request, timeout_s: calls.append(threading.get_ident()) or {'name': 'x', 'frames': 1}  # noqa: E731
    maps = library(tmp_path, head_files(tmp_path), remote=remote)
    folder = tmp_path / 'maps' / 'maps' / '20261101T010000Z'
    folder.mkdir(parents=True)
    (folder / 'map.splat').write_bytes(b'\1' * 32)

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), maps=maps, token='secret'))) as client:
            listing = await (await client.get('/api/helm/maps')).json()
            assert listing['maps'][0]['name'] == '20261101T010000Z' and listing['robot_in_world']['board']
            response = await client.get('/api/helm/maps/20261101T010000Z/map.splat')
            assert response.status == 200 and await response.read() == b'\1' * 32
            assert response.headers['Content-Type'] == 'application/octet-stream'
            assert (await client.get('/api/helm/maps/20261101T020000Z/map.splat')).status == 404
            assert (await client.post('/api/helm/maps/capture', json={'seconds': 15})).status == 403
            secret = {'X-Helm-Token': 'secret'}
            response = await client.post('/api/helm/maps/capture', json={'seconds': 15}, headers=secret)
            assert response.status == 200, await response.text()
            assert (await response.json())['maps']['last']['frames'] == 1
            assert (await client.post('/api/helm/maps/capture', json={}, headers=secret)).status >= 400
    asyncio.run(run())
    assert calls and all(thread != loop_thread for thread in calls)


def test_without_maps_the_api_says_why(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), token='secret'))) as client:
            response = await client.get('/api/helm/maps')
            assert response.status >= 400 and 'Helm on Minerva' in (await response.json())['error']
            assert (await (await client.get('/api/helm/state')).json())['maps'] is None
    asyncio.run(run())
