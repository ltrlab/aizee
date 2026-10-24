"""Helm's wrist camera calibration without hardware: the robot helper's frame stream, stack and light readings (numpy
parts; detection needs OpenCV and was rehearsed on the robot), the still capture's stillness gate and run files, the
solve on synthetic stills, and the API."""
import asyncio
import json
import math
from pathlib import Path
import threading
import time

import msgpack
import numpy as np
import pytest
import zmq

from robot_platform.helm import calibration_still as cs
from robot_platform.helm.calibration import CameraCalibration
from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.vision.board import load_board
from robot_platform.vision.chain import Chain
from robot_platform.vision.rig import load_rig
from robot_platform.vision.solve import load_stills, solve_run
from robot_platform.vision.synthetic import sample_stills

ROOT = Path(__file__).resolve().parents[2]
ARM = [f'left_arm_j{k}' for k in range(1, 7)]


# -- the robot helper ---------------------------------------------------------------------------------------------

def publish(port, stop, packets):
    """Publish `packets()` (a list of (header, jpeg)) until stopped; SUB sockets join late, so keep going."""
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f'tcp://127.0.0.1:{port}')
    try:
        while not stop.is_set():
            for header, jpeg in packets():
                socket.send_multipart([msgpack.packb(header, use_bin_type=True), jpeg])
            time.sleep(.005)
    finally:
        socket.close(0)
        context.term()


def frame_header(stamp, camera_id='left_wrist'):
    return {'camera_id': camera_id, 'timestamp': stamp, 'parts': ['color'], 'color': {'format': 'jpeg'}}


@pytest.fixture
def stream(monkeypatch):
    port = 25563
    monkeypatch.setitem(cs.ROLES, 'left_wrist', {'frame_port': port, 'camera_id': 'left_wrist'})
    stop, threads = threading.Event(), []

    def start(packets):
        thread = threading.Thread(target=publish, args=(port, stop, packets), daemon=True)
        thread.start()
        threads.append(thread)
    yield start
    stop.set()
    for thread in threads:
        thread.join(2)


def test_collect_keeps_only_frames_stamped_after_the_request(stream):
    start = time.time()
    # Old frames first in every burst; only the fresh ones count.
    stream(lambda: [(frame_header(start - 5.), b'old'), (frame_header(time.time()), b'new')])
    frames = cs.collect('left_wrist', 3, fresh_after_s=0.)
    assert len(frames) == 3 and all(jpeg == b'new' and stamp >= start for stamp, jpeg in frames)


def test_collect_refuses_another_cameras_stream(stream):
    stream(lambda: [(frame_header(time.time(), 'right_wrist'), b'x')])
    with pytest.raises(ValueError, match='identity'):
        cs.collect('left_wrist', 2, fresh_after_s=0.)


def test_stack_averages_frames_and_measures_their_noise():
    rng = np.random.default_rng(0)
    scene = rng.uniform(40, 200, (24, 32, 3))
    frames = [np.clip(scene + rng.normal(0, 3., scene.shape), 0, 255).round().astype(np.uint8) for _ in range(40)]
    mean, sigma = cs.stack(frames)
    assert np.abs(mean - scene).mean() < 1. and mean.dtype == np.float32
    # Luma mixes three independent channels: sigma 3 * sqrt(.114^2 + .587^2 + .299^2), plus rounding.
    assert abs(float(np.median(sigma)) - 3 * math.sqrt(.114 ** 2 + .587 ** 2 + .299 ** 2)) < .3
    with pytest.raises(ValueError, match='size'):
        cs.stack([frames[0], frames[0][:-1]])


def test_photometry_reads_the_black_border_and_the_white_label():
    """A tag square of 6 cells (black border, here an all-black code) on a white label on a dark table."""
    image = np.full((200, 200, 3), 18., np.float32)
    image[30:170, 30:170] = (225., 230., 235.)  # The label (B, G, R): a slightly warm white.
    image[60:140, 60:140] = 20.  # The black square, 80 px across: cells of 13.3 px.
    quad = [(59.5, 59.5), (139.5, 59.5), (139.5, 139.5), (59.5, 139.5)]
    light = cs.photometry(image, np.full((200, 200), 1.5, np.float32), {3: quad})['3']
    assert light['black']['luma'] == pytest.approx(20, abs=.5) and light['black']['sigma'] == 1.5
    assert light['white']['bgr'] == pytest.approx([225, 230, 235], abs=.5)
    assert light['white']['r_over_g'] == pytest.approx(235 / 230, abs=.002) and light['contrast'] > 200
    # A tag whose label runs off the image keeps the points inside it.
    assert 'white' in cs.photometry(image, image[..., 0] * 0, {1: [(q[0] - 59, q[1]) for q in quad]})['1']


def test_stills_are_saved_only_as_png_under_the_calibration_folder(tmp_path, monkeypatch):
    monkeypatch.setattr(cs, 'CALIBRATION_ROOT', str(tmp_path / 'calibration'))
    assert cs._image_path(tmp_path / 'calibration' / 'run' / 'images' / 'a.png').name == 'a.png'
    for bad in (tmp_path / 'elsewhere.png', tmp_path / 'calibration' / '..' / 'x.png',
                tmp_path / 'calibration' / 'run' / 'a.jpg'):
        with pytest.raises(ValueError, match='under'):
            cs._image_path(bad)


def test_the_helper_answers_with_json_even_when_it_fails(capsys):
    cs.remote_main({'operation': 'move'})
    reply = json.loads(capsys.readouterr().out)
    assert reply == {'ok': False, 'error': 'ValueError: Unknown calibration operation'}


# -- the capture ----------------------------------------------------------------------------------------------------

class Session:
    """A teleop snapshot whose left arm reads `positions`, as a live session reports it."""

    def __init__(self, phase='running', state='HOLD', source='measured', recording='idle'):
        self.phase, self.state, self.source, self.recording = phase, state, source, recording
        self.positions = {joint: .1 * k for k, joint in enumerate(ARM)}
        self.stale = False
        self.drift = 0.  # Added to J2 on every read: an arm that keeps moving.

    def __call__(self):
        self.positions['left_arm_j2'] += self.drift
        rows = [dict(joint=j, position=p, source=self.source, stale=self.stale) for j, p in self.positions.items()]
        rows.append(dict(joint='right_arm_j1', position=0., source=self.source, stale=False))
        return dict(phase=self.phase, state=self.state, telemetry={'rows': rows},
                    task={'recording': {'state': self.recording, 'active': self.recording == 'recording'}})


def still_reply(request):
    return dict(role=request['role'], frames=request['frames'], first_stamp=1., last_stamp=1.1, size=[640, 480],
                image=request['path'], tags={'0': [[1, 2], [3, 4], [5, 6], [7, 8]]}, others=[7], duplicates=[],
                photometry={'0': {'white': {'luma': 201., 'r_over_g': 1.02, 'b_over_g': .97, 'bgr': [1, 2, 3],
                                            'sigma': 1.}, 'black': {'luma': 22., 'bgr': [1, 2, 3], 'sigma': .8}}},
                noise={'luma_sigma_median': .8, 'frames': request['frames']}, thumbnail='anBn')


def calibration(tmp_path, session, *, remote=still_reply, solver=None):
    times = iter(range(1792900000, 1792999999, 7))
    return CameraCalibration(None, address=lambda: '127.0.0.1', teleop=session, root=tmp_path / 'calibration',
                             remote=remote, solver=solver, clock=lambda: float(next(times)), sleep=lambda _: None)


def test_a_still_needs_a_live_session_holding_or_following(tmp_path):
    for session, message in ((Session(phase='stopped'), 'running teleop session'),
                             (Session(state='IDLE'), 'HOLD or TELEOP'),
                             (Session(source='simulated'), 'real arms'),
                             (Session(recording='recording'), 'Stop the recording'),
                             (Session(recording='finishing'), 'Stop the recording'),
                             (Session(), 'stale')):
        if message == 'stale':
            session.stale = True
        with pytest.raises(ValueError, match=message):
            calibration(tmp_path, session).capture('left_wrist')
    with pytest.raises(ValueError, match='Expected camera role'):
        calibration(tmp_path, Session()).capture('head')


def test_a_still_records_the_arm_and_what_the_robot_saw(tmp_path):
    requests = []
    cal = calibration(tmp_path, Session(state='TELEOP'), remote=lambda r: requests.append(r) or still_reply(r))
    state = cal.capture('left_wrist')
    request = requests[0]
    assert request['frames'] == 8 and request['tags'] == [0, 1, 2, 3, 4, 5] and request['family'] == 'DICT_4X4_50'
    assert Path(request['path']).as_posix().endswith('images/001-left_wrist.png')
    record = json.loads((cal.run / 'stills.jsonl').read_text(encoding='utf-8'))
    assert record['joints'] == {j: pytest.approx(.1 * k) for k, j in enumerate(ARM)}  # The left arm only.
    assert record['link'] == '1_1_02_eb571_960_stp_2' and record['tags'] == {'0': [[1, 2], [3, 4], [5, 6], [7, 8]]}
    assert 'thumbnail' not in record
    assert state['counts']['left_wrist'] == {'stills': 1, 'tagged': 1} and state['last']['thumbnail'] is True
    assert cal.thumbnail() == b'jpg'  # Served on its own, not in every state.
    assert state['last']['tags'] == [0] and state['last']['white'] == 201. and state['last']['r_over_g'] == 1.02


def test_an_arm_that_moves_spoils_the_still(tmp_path):
    session = Session()
    session.drift = .001  # 1 mrad a read: the first check (two reads apart) passes 1.5 mrad, not the whole still.
    cal = calibration(tmp_path, session)
    with pytest.raises(ValueError, match='moved .* while the frames'):
        cal.capture('left_wrist')
    session.drift = .002
    with pytest.raises(ValueError, match='moved .* before the frames'):
        cal.capture('left_wrist')
    assert cal.stills == [] and 'hold it still' in cal.snapshot()['error']


def test_undo_new_runs_and_reopening(tmp_path):
    cal = calibration(tmp_path, Session())
    for _ in range(3):
        cal.capture('left_wrist')
    first = cal.run
    cal.undo('left_wrist')
    assert [s['name'] for s in cal.stills] == ['001-left_wrist', '002-left_wrist']
    with pytest.raises(ValueError, match='No right_wrist still'):
        cal.undo('right_wrist')
    reopened = calibration(tmp_path, Session())
    assert reopened.run == first and len(reopened.stills) == 2
    reopened.new_run()
    assert reopened.run != first and reopened.stills == []
    assert reopened.snapshot()['counts']['left_wrist']['stills'] == 0


def test_solve_keeps_the_summary_and_reports_failure(tmp_path):
    cal = calibration(tmp_path, Session(), solver=lambda folder: (_ for _ in ()).throw(RuntimeError('Solve failed')))
    with pytest.raises(ValueError, match='Capture stills first'):
        cal.solve()
    cal.capture('left_wrist')
    with pytest.raises(RuntimeError, match='Solve failed'):
        cal.solve()
    assert cal.snapshot()['error'] == 'Solve failed' and not cal.snapshot()['solving']


# -- the solve ------------------------------------------------------------------------------------------------------

def write_synthetic_run(folder, count=12, seed=11):
    rig, board = load_rig(), load_board()
    rng = np.random.default_rng(seed)
    lines = []
    for name, camera in rig['cameras'].items():
        chain = Chain(rig['urdf'], camera['link'])
        for k, still in enumerate(sample_stills(board, chain, name, rng, count, noise_px=.2)):
            lines.append(json.dumps(dict(name=f'{k:03d}-{name}', camera=name, joints=still.joints,
                                         tags={str(t): q.tolist() for t, q in still.corners.items()})))
    lines.append(json.dumps(dict(name='099-left_wrist', camera='left_wrist', joints={}, tags={})))  # No tags seen.
    folder.mkdir(parents=True)
    (folder / 'stills.jsonl').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def test_solve_run_fits_both_models_and_checks_the_mount_link(tmp_path):
    run = tmp_path / 'run'
    write_synthetic_run(run)
    assert len(load_stills(run)) == 25
    result = solve_run(run)
    saved = json.loads((run / 'result.json').read_text(encoding='utf-8'))
    assert saved['recommended'] in ('plain', 'joint_offsets') and set(saved['models']) == {'plain', 'joint_offsets'}
    plain = saved['models']['plain']
    assert all(camera['rms_px'] < .4 for camera in plain['cameras'].values())
    # The synthetic cameras ride the gripper bodies, so J6's parent link fits far worse.
    for check in result['link_check'].values():
        assert check['parent_rms_px'] > 3 * check['rms_px']
    assert result['notes'] == []
    from robot_platform.helm.calibration import _summary
    summary = _summary(saved)
    assert summary['models']['plain']['cameras']['left_wrist']['stills'] == 12


# -- the API --------------------------------------------------------------------------------------------------------

@pytest.fixture
def helm(tmp_path):
    session = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=tmp_path / 'limits.json',
                          seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


class FakeCalibration:
    def __init__(self):
        self.calls = []

    def snapshot(self):
        return {'run': None, 'counts': {}, 'busy': False}

    def thumbnail(self):
        return b'jpeg' if self.calls else None

    def capture(self, role):
        self.calls.append(('capture', role, threading.get_ident()))

    def undo(self, role):
        self.calls.append(('undo', role, threading.get_ident()))

    def new_run(self):
        self.calls.append(('new', None, threading.get_ident()))

    def solve(self):
        self.calls.append(('solve', None, threading.get_ident()))


def test_the_api_is_token_gated_and_runs_off_the_event_loop(helm):
    from aiohttp.test_utils import TestClient, TestServer
    fake, loop_thread = FakeCalibration(), threading.get_ident()

    async def run():
        app = create_app(helm, SimulationLink(), calibration=fake, token='secret')
        async with TestClient(TestServer(app)) as client:
            assert (await client.get('/api/helm/calibration/thumbnail')).status == 404  # No still yet.
            assert (await client.post('/api/helm/calibration/capture', json={'role': 'left_wrist'})).status == 403
            secret = {'X-Helm-Token': 'secret'}
            for action, payload in (('capture', {'role': 'left_wrist'}), ('undo', {'role': 'right_wrist'}),
                                    ('new', {}), ('solve', {})):
                response = await client.post(f'/api/helm/calibration/{action}', json=payload, headers=secret)
                assert response.status == 200, await response.text()
                assert (await response.json())['calibration']['busy'] is False
            assert (await client.post('/api/helm/calibration/capture', json={}, headers=secret)).status >= 400
            assert (await client.post('/api/helm/calibration/solve', json={'x': 1}, headers=secret)).status >= 400
            assert (await client.post('/api/helm/calibration/move', json={}, headers=secret)).status == 404
            response = await client.get('/api/helm/calibration/thumbnail')
            assert response.content_type == 'image/jpeg' and await response.read() == b'jpeg'
            assert response.headers['Cache-Control'] == 'no-store'
    asyncio.run(run())
    assert [c[:2] for c in fake.calls] == [('capture', 'left_wrist'), ('undo', 'right_wrist'), ('new', None),
                                           ('solve', None)]
    assert all(thread != loop_thread for *_, thread in fake.calls)


def test_without_calibration_the_api_says_why(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), token='secret'))) as client:
            response = await client.post('/api/helm/calibration/new', json={}, headers={'X-Helm-Token': 'secret'})
            assert response.status >= 400 and 'Helm on Minerva' in (await response.json())['error']
            assert (await (await client.get('/api/helm/state')).json())['calibration'] is None
    asyncio.run(run())
