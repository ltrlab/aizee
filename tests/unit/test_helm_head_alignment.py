"""Helm's head camera alignment without hardware: capturing stills in a session with the live head, the solve and
saving the mount where the head's pose feed reads it, the robot helper's neck window over a still's frames, and the
API."""
import asyncio
import json
import math
from pathlib import Path
import threading
import time

import msgpack
import pytest
import zmq

from robot_platform.helm import calibration_still as cs
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.head_alignment import HeadAlignment
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.neck import head
from robot_platform.neck.head_pose import aligned_mount, reference_digest
from robot_platform.vision import head_alignment as ha
from robot_platform.vision.synthetic import SIM_PLACEMENT
from tests.unit.test_head_alignment import INTRINSICS, REFERENCE, apart, rig, synthetic, write_wrists  # noqa: F401

ROOT = Path(__file__).resolve().parents[2]


# -- Helm's flow ----------------------------------------------------------------------------------------------------

class Session:
    """A teleop snapshot with the head as a live session reports it."""

    def __init__(self, phase='running', mode='live', state='HOLD', recording='idle'):
        self.phase, self.mode, self.state, self.recording = phase, mode, state, recording

    def __call__(self):
        return dict(phase=self.phase, receiver_choice={'mode': self.mode, 'arm': 'head'},
                    head={'state': self.state, 'joints': {'neck_z': 0., 'neck_y': -.5}},
                    task={'recording': {'state': self.recording, 'active': self.recording == 'recording'}})


class Robot:
    """The robot helper's replies: the next synthetic still each time, its neck as the feed read it."""

    def __init__(self, stills, moved=.001):
        self.stills, self.moved, self.requests = list(stills), moved, []

    def __call__(self, request):
        self.requests.append(request)
        still = self.stills.pop(0)
        return dict(role='head', frames=request['frames'], first_stamp=1., last_stamp=1.5, size=[640, 480],
                    image=request['path'], tags={str(t): q.tolist() for t, q in still.corners.items()}, others=[],
                    duplicates=[], photometry={}, noise={'luma_sigma_median': .8, 'frames': 8}, thumbnail='anBn',
                    intrinsics=INTRINSICS, image_rotation_deg=0,
                    head={'joints': still.joints, 'moved_rad': self.moved, 'samples': 60})


def flow(tmp_path, session, robot, **options):
    files = head.files(tmp_path / 'operator', 'minerva-production-01')
    files['reference'].parent.mkdir(parents=True, exist_ok=True)
    files['reference'].write_text(json.dumps(REFERENCE), encoding='utf-8')
    times = iter(range(1793000000, 1793999999, 7))
    return HeadAlignment(None, address=lambda: '127.0.0.1', teleop=session, root=tmp_path / 'calibration' / 'head',
                         files=lambda: files, remote=robot,
                         solver=lambda folder: ha.solve_run(folder, wrists=options.get('wrists')),
                         clock=lambda: float(next(times)), **options), files


def test_a_head_still_needs_the_live_head_held_or_following(rig, tmp_path):
    for session, message in ((Session(phase='stopped'), 'running session'), (Session(mode='virtual'), 'real head'),
                             (Session(state='IDLE'), r'Enable the head \(E\)'),
                             (Session(recording='finishing'), 'Stop the recording')):
        alignment, _ = flow(tmp_path, session, Robot(synthetic(rig, count=5)))
        with pytest.raises(ValueError, match=message):
            alignment.capture()
    alignment, files = flow(tmp_path, Session(), Robot(synthetic(rig, count=5)))
    files['reference'].unlink()
    with pytest.raises(ValueError, match="Set the head's reference"):
        alignment.capture()


def test_a_head_still_records_the_neck_from_the_feed_and_refuses_a_moving_head(rig, tmp_path):
    stills = synthetic(rig, count=5)
    robot = Robot(stills)
    alignment, _ = flow(tmp_path, Session(state='TELEOP'), robot)
    state = alignment.capture()
    assert robot.requests[0]['head_pose'] == {'margin_s': .05} and robot.requests[0]['role'] == 'head'
    record = json.loads((alignment.run / 'stills.jsonl').read_text(encoding='utf-8'))
    assert record['joints'] == stills[0].joints and record['reference_sha256'] == reference_digest(REFERENCE)
    assert record['intrinsics'] == INTRINSICS and record['link'] == 'awg_rgb_camera_5' and 'thumbnail' not in record
    assert state['stills'] == state['tagged'] == 1 and state['last']['joints_deg']['neck_z'] == pytest.approx(
        math.degrees(stills[0].joints['neck_z']), abs=.01)
    assert alignment.thumbnail() == b'jpg'
    robot.moved = .004
    with pytest.raises(ValueError, match=r'moved 4\.0 mrad'):
        alignment.capture()
    assert len(alignment.stills) == 1 and 'hold it still' in alignment.snapshot()['error']
    alignment.undo()
    assert alignment.stills == [] and alignment.snapshot()['last'] is None
    with pytest.raises(ValueError, match='No head still'):
        alignment.undo()


def test_capture_solve_and_save_put_the_mount_where_the_heads_feed_reads_it(rig, tmp_path):
    alignment, files = flow(tmp_path, Session(), Robot(synthetic(rig, count=10) + synthetic(rig, count=10, seed=9)),
                            wrists=write_wrists(tmp_path / 'wrists', SIM_PLACEMENT))
    with pytest.raises(ValueError, match='Solve the run first'):
        alignment.save()
    for _ in range(10):
        alignment.capture()
    state = alignment.solve()
    assert state['result']['rms_px'] < .35 and state['result']['arms']['run'] == '20261031T000000Z'
    assert state['saved'] is None
    state = alignment.save()
    document = json.loads(files['camera'].read_text(encoding='utf-8'))
    mm, deg = apart(rig['truth'], aligned_mount(document, 'awg_rgb_camera_5', REFERENCE))
    assert mm < 1.5 and deg < .15
    assert state['saved']['current'] and state['saved']['run'] == alignment.run.name
    assert document['fit']['stills'] == 10 and document['board_in_robot']['matrix']
    # A second run saved over it keeps the first beside its own stills.
    first = alignment.run
    alignment.new_run()
    for _ in range(10):
        alignment.capture()
    alignment.solve()
    alignment.save()
    kept = list(alignment.run.glob('replaced-*-minerva-production-01-head-camera.json'))
    assert len(kept) == 1 and json.loads(kept[0].read_text(encoding='utf-8'))['run'] == first.name
    # A new head reference makes the saved alignment stale and refuses saving the old run's solve.
    files['reference'].write_text(json.dumps(dict(REFERENCE, motors=[])), encoding='utf-8')
    assert alignment.snapshot()['saved']['current'] is False
    with pytest.raises(ValueError, match='reference changed'):
        alignment.save()
    # Reopened, Helm shows the latest run and its solve.
    reopened, _ = flow(tmp_path, Session(), Robot([]))
    assert reopened.run == alignment.run and reopened.snapshot()['result']['stills'] == 10


# -- the robot helper's neck window ---------------------------------------------------------------------------------

def test_the_neck_window_covers_the_frames_or_refuses():
    samples = [[10. + k * .01, .2 + (.001 if k % 2 else 0.), -.5] for k in range(100)]
    window = cs.head_window(samples, ['neck_z', 'neck_y'], 10.3, 10.5, lag_s=.02, margin_s=.05)
    assert window['joints']['neck_z'] == pytest.approx(.2005, abs=1e-4) and window['moved_rad'] == pytest.approx(.001)
    assert window['samples'] == 31  # 10.23 .. 10.53.
    with pytest.raises(ValueError, match='did not cover'):
        cs.head_window(samples[:40], ['neck_z', 'neck_y'], 10.3, 10.5)  # The feed stops at 10.39.
    with pytest.raises(ValueError, match='silent'):
        cs.head_window([], None, 10.3, 10.5)


def test_a_head_still_reads_the_feed_over_its_frames(monkeypatch):
    frame_port, feed = 25564, 'tcp://127.0.0.1:25596'
    monkeypatch.setitem(cs.ROLES, 'head', {'frame_port': frame_port, 'camera_id': 'scene_cam'})
    stop = threading.Event()

    def publish():
        context = zmq.Context()
        frames, poses = context.socket(zmq.PUB), context.socket(zmq.PUB)
        frames.bind(f'tcp://127.0.0.1:{frame_port}')
        poses.bind(feed)
        try:
            k = 0
            while not stop.is_set():
                now = time.time()
                poses.send_string(json.dumps({'joints': ['neck_z', 'neck_y'], 'zero': [0.] * 7,
                                              'samples': [[now, .25, -.6, *([0.] * 7)]]}))
                if k % 3 == 0:
                    header = {'camera_id': 'scene_cam', 'timestamp': now, 'parts': ['color'], 'image_rotation_deg': 0,
                              'color': {'format': 'jpeg', 'intrinsics': INTRINSICS}}
                    frames.send_multipart([msgpack.packb(header, use_bin_type=True), b'jpeg'])
                k += 1
                time.sleep(.01)
        finally:
            frames.close(0)
            poses.close(0)
            context.term()

    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    try:
        frames, meta, pose = cs.frames_and_head('head', 4, {'margin_s': .05}, endpoint=feed)
    finally:
        stop.set()
        thread.join(2)
    assert len(frames) == 4 and meta[0] == {'intrinsics': INTRINSICS, 'image_rotation_deg': 0}
    assert pose['joints'] == {'neck_z': pytest.approx(.25), 'neck_y': pytest.approx(-.6)}
    assert pose['moved_rad'] == 0. and pose['samples'] >= 8


# -- the API --------------------------------------------------------------------------------------------------------

class FakeAlignment:
    def __init__(self):
        self.calls = []

    def snapshot(self):
        return {'run': None, 'stills': 0, 'busy': False}

    def thumbnail(self):
        return b'jpeg' if self.calls else None

    def __getattr__(self, name):
        if name in ('capture', 'undo', 'new_run', 'solve', 'save'):
            return lambda: self.calls.append((name, threading.get_ident()))
        raise AttributeError(name)


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=tmp_path / 'limits.json',
                          seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


def test_the_head_alignment_api_is_token_gated_and_runs_off_the_event_loop(helm):
    from aiohttp.test_utils import TestClient, TestServer
    fake, loop_thread = FakeAlignment(), threading.get_ident()

    async def run():
        app = create_app(helm, SimulationLink(), head_alignment=fake, token='secret')
        async with TestClient(TestServer(app)) as client:
            assert (await client.get('/api/helm/head-alignment/thumbnail')).status == 404
            assert (await client.post('/api/helm/head-alignment/capture', json={})).status == 403
            secret = {'X-Helm-Token': 'secret'}
            for action in ('capture', 'undo', 'new', 'solve', 'save'):
                response = await client.post(f'/api/helm/head-alignment/{action}', json={}, headers=secret)
                assert response.status == 200, await response.text()
                assert (await response.json())['head_alignment']['busy'] is False
            assert (await client.post('/api/helm/head-alignment/capture', json={'role': 'head'},
                                      headers=secret)).status >= 400
            assert (await client.post('/api/helm/head-alignment/move', json={}, headers=secret)).status == 404
            response = await client.get('/api/helm/head-alignment/thumbnail')
            assert response.content_type == 'image/jpeg' and await response.read() == b'jpeg'
    asyncio.run(run())
    assert [name for name, _ in fake.calls] == ['capture', 'undo', 'new_run', 'solve', 'save']
    assert all(thread != loop_thread for _, thread in fake.calls)


def test_without_head_alignment_the_api_says_why(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), token='secret'))) as client:
            response = await client.post('/api/helm/head-alignment/new', json={}, headers={'X-Helm-Token': 'secret'})
            assert response.status >= 400 and 'Helm on Minerva' in (await response.json())['error']
            assert (await (await client.get('/api/helm/state')).json())['head_alignment'] is None
    asyncio.run(run())
