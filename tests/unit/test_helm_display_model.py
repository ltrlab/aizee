"""Helm's 3D view draws the head model (helm.display_model): Minerva with her neck, every guide for the active model
drawn on it by joint name, the neck at the head's measured angles while a session drives it, else at its zero, or
where the headset looks with Settings → Headset's preview on. The guard and the receivers keep the active model."""

import asyncio
from copy import deepcopy
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.display_model import DisplayModel, _same
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopSession
from robot_platform.helm.xr import Headset
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.ops.release import HEAD_MODEL, MODEL
from tests.unit.test_helm_xr import gaze

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'


@pytest.fixture(scope='module')
def models():
    return UrdfModel(ModelProfile.load(ROOT / MODEL)), UrdfModel(ModelProfile.load(ROOT / HEAD_MODEL))


def pose(active, **positions):
    return {
        'model': active.profile.id,
        'urdf_sha256': active.sha256,
        'live': True,
        'stream_id': 'helm-teleop',
        'positions': {name: positions.get(name, 0.0) for name in active.independent},
        'badge': 'LIVE',
    }


def test_the_head_model_is_the_active_one_plus_the_neck(models):
    active, shown = models
    display = DisplayModel(active, shown)
    assert set(display.extra) == {'neck_z', 'neck_y'} and display.profile.id == shown.profile.id
    drawn = display.guide(pose(active, left_arm_j2=0.4), {'neck_z': 0.3, 'neck_y': -0.2})
    assert (drawn['model'], drawn['urdf_sha256']) == (shown.profile.id, shown.sha256)
    assert set(drawn['positions']) == set(shown.independent)
    assert drawn['positions']['left_arm_j2'] == 0.4 and (
        drawn['positions']['neck_z'],
        drawn['positions']['neck_y'],
    ) == (0.3, -0.2)
    assert drawn['badge'] == 'LIVE' and drawn['stream_id'] == 'helm-teleop'  # The rest of the guide as it was.
    shown.positions(drawn['positions'])  # A pose the shown model takes.
    # No session: the neck at its zero; a reading past its range, or not a number, is not drawn as one.
    assert display.guide(pose(active))['positions']['neck_z'] == 0.0
    clamped = display.guide(pose(active), {'neck_z': 4.0, 'neck_y': math.nan})['positions']
    assert (clamped['neck_z'], clamped['neck_y']) == (shown.by_name['neck_z']['bounds'][1], 0.0)
    # Nothing to draw, or another model's guide: unchanged.
    assert display.guide(None) is None
    other = dict(pose(active), urdf_sha256='0' * 64)
    assert display.guide(other) is other
    # Joint telemetry: the same rows, stamped for the shown model.
    packet = {
        'model': active.profile.id,
        'urdf_sha256': active.sha256,
        'stream_id': 'reader',
        'sequence': 4,
        'rows': [{'motor': 'left_j2', 'joint': 'left_arm_j2', 'position': 0.4}],
    }
    stamped = display.telemetry(packet)
    assert (stamped['model'], stamped['urdf_sha256'], stamped['rows']) == (
        shown.profile.id,
        shown.sha256,
        packet['rows'],
    )
    assert display.telemetry(other) is other and display.telemetry(None) is None


def test_a_preview_pose_follows_the_headset_as_a_pose_of_its_own(models):
    active, shown = models
    display = DisplayModel(active, shown)
    reference = dict(
        pose(active), live=False, badge='NOT CONNECTED · reference pose shown', stream_id='reference', sequence=0
    )
    first = display.preview(reference, {'neck_z': 0.4, 'neck_y': -0.1}, 7)
    assert (first['positions']['neck_z'], first['positions']['neck_y']) == (0.4, -0.1)
    assert first['badge'] == 'NOT CONNECTED · reference pose shown · head: headset preview, no motors move'
    assert first['view'] == 'keep' and first['live'] is False  # The view keeps its framing; the arms stay as they were.
    # The view applies a pose once per stream and sequence: each headset pose is one of its own.
    assert display.preview(reference, {'neck_z': 0.5, 'neck_y': -0.1}, 8)['stream_id'] != first['stream_id']
    stopped = display.preview(reference, None, 9)['positions']  # Not tracking: the neck at its zero.
    assert (stopped['neck_z'], stopped['neck_y']) == (0.0, 0.0)
    assert display.preview(None, {'neck_z': 0.4}, 1) is None
    live = display.preview(dict(pose(active), badge=None), {'neck_z': 0.4}, 2)
    assert live['badge'] == 'LIVE · motors disabled · head: headset preview, no motors move'


def test_the_headset_gives_its_angles_only_while_it_tracks():
    clock = SimpleNamespace(now=100.0)
    headset = Headset(clock=lambda: clock.now)
    assert headset.angles() is None
    connection = headset.connect('quest', 'Quest headset')
    headset.start(connection)
    headset.receive(connection, {'type': 'pose', 'seq': 1, 'q': gaze(0.3, 0.2)})
    assert headset.angles() == pytest.approx({'neck_z': 0.3, 'neck_y': 0.2})
    clock.now += 1.0  # Quiet: not tracking.
    assert headset.angles() is None


def test_a_model_that_moves_an_active_joint_is_not_drawn_in_its_place(models):
    active, shown = models
    moved = deepcopy(shown.by_name)
    moved['left_arm_j3']['origin']['xyz'][0] += 0.001  # One millimetre off.
    with pytest.raises(ValueError, match='left_arm_j3'):
        DisplayModel(active, SimpleNamespace(by_name=moved, independent=shown.independent, profile=shown.profile))
    narrower = deepcopy(shown.by_name)
    narrower['right_arm_j1']['bounds'] = [-1.0, 1.0]
    with pytest.raises(ValueError, match='right_arm_j1'):
        DisplayModel(active, SimpleNamespace(by_name=narrower, independent=shown.independent, profile=shown.profile))
    # The same rotation written with -pi for pi is the same frame.
    joint = deepcopy(active.by_name['left_arm_j1'])
    flipped = deepcopy(joint)
    joint['origin']['rpy'], flipped['origin']['rpy'] = [math.pi, 0.0, 0.0], [-math.pi, 0.0, 0.0]
    assert _same(joint, flipped)
    logged = []
    assert DisplayModel.load(active, ROOT / 'urdf/minerva/missing/model.json', log=logged.append) is None
    assert DisplayModel.load(None, ROOT / HEAD_MODEL, log=logged.append) is None  # Never stops Helm.
    assert all('without the head' in line for line in logged) and len(logged) == 2


def test_the_session_hands_the_view_the_necks_angles_only_while_it_runs():
    loop = SimpleNamespace(
        _lock=threading.Lock(),
        active=True,
        _view={'phase': 'running', 'head': {'joints': {'neck_z': 0.2, 'neck_y': -0.1}}},
    )
    assert TeleopSession.head_joints(loop) == {'neck_z': 0.2, 'neck_y': -0.1}
    loop._view = {'phase': 'running', 'head': None}  # A session without a head.
    assert TeleopSession.head_joints(loop) is None
    loop._view = {'phase': 'stopped', 'head': {'joints': {'neck_z': 0.2, 'neck_y': -0.1}}}
    assert TeleopSession.head_joints(loop) is None
    loop._view, loop.active = {'phase': 'running', 'head': {'joints': {'neck_z': 0.2}}}, False
    assert TeleopSession.head_joints(loop) is None


class Teleop:
    active = False

    def __init__(self, model, joints):
        self.model, self.joints = model, joints

    def guide(self):
        return dict(
            pose(self.model, right_arm_j4=-0.5), updated_at=time.time(), source_age_seconds=0.0, feed_state='reading'
        )

    def head_joints(self):
        return self.joints

    def snapshot(self):
        return {'phase': 'running'}

    def stop(self):
        pass


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    yield session
    session.close()


def test_the_3d_view_draws_the_head_moving_with_the_session(helm, models):
    from aiohttp.test_utils import TestClient, TestServer

    _active, shown = models
    teleop = Teleop(helm.model, {'neck_z': 0.35, 'neck_y': -0.15})

    async def run():
        app = create_app(
            helm,
            SimulationLink(),
            teleop=teleop,
            token='secret',
            simulation_only=True,
            display=DisplayModel(helm.model, shown),
        )
        async with TestClient(TestServer(app)) as client:
            catalog = await (await client.get('/api/models')).json()
            assert catalog['selected'] == shown.profile.id and [m['id'] for m in catalog['models']] == [
                shown.profile.id
            ]
            metadata = await (await client.get(f'/api/models/{shown.profile.id}')).json()
            assert {'neck_z', 'neck_y'} <= set(metadata['independent'])
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert (drawn['model'], drawn['urdf_sha256']) == (shown.profile.id, shown.sha256)
            assert drawn['positions']['right_arm_j4'] == -0.5
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (0.35, -0.15)
            teleop.joints = None  # The session ended: the neck at its zero.
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (0.0, 0.0)
            # The joint telemetry the page relays to the view: for the drawn model.
            state = await (await client.get('/api/helm/state')).json()
            assert (state['telemetry']['model'], state['telemetry']['urdf_sha256']) == (shown.profile.id, shown.sha256)

    asyncio.run(run())


def test_the_preview_switch_turns_the_views_head_with_the_headset(helm, models):
    from aiohttp.test_utils import TestClient, TestServer

    _active, shown = models
    teleop = Teleop(helm.model, None)  # No session drives the head.
    teleop.head_pose = lambda yaw, pitch: False

    async def run():
        app = create_app(
            helm,
            SimulationLink(),
            teleop=teleop,
            token='secret',
            simulation_only=True,
            display=DisplayModel(helm.model, shown),
        )
        async with TestClient(TestServer(app)) as client:
            switch = lambda on: client.post('/api/helm/xr/preview', json={'on': on}, headers={'X-Helm-Token': 'secret'})
            state = await (await client.get('/api/helm/state')).json()
            assert (state['xr']['preview'], state['xr']['preview_available']) == (False, True)
            ws = await client.ws_connect('/api/helm/xr?token=secret')
            await ws.send_json({'type': 'start'})
            await ws.receive_json(timeout=2)
            await ws.send_json({'type': 'pose', 'seq': 1, 'q': gaze(0.35, -0.15)})
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['positions']['neck_z'] == 0.0  # The preview is off: the head at its zero.
            assert (await (await switch('yes')).json())['error'].startswith('Expected')
            assert (await (await switch(True)).json())['xr']['preview'] is True
            await ws.send_json({'type': 'pose', 'seq': 2, 'q': gaze(0.35, -0.15)})
            for _ in range(100):
                drawn = await (await client.get('/api/calibration-guide')).json()
                if drawn['positions']['neck_z']:
                    break
                await asyncio.sleep(0.01)
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == pytest.approx((0.35, -0.15))
            assert 'headset preview' in drawn['badge'] and drawn['view'] == 'keep'
            teleop.joints = {'neck_z': -0.2, 'neck_y': 0.05}  # A session drives the head: its measured angles show.
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (-0.2, 0.05)
            assert 'headset preview' not in (drawn.get('badge') or '')
            teleop.joints = None
            assert (await (await switch(False)).json())['xr']['preview'] is False
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['positions']['neck_z'] == 0.0
            await ws.close()

    asyncio.run(run())


def test_without_a_display_model_the_view_draws_the_active_one(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        app = create_app(
            helm, SimulationLink(), teleop=Teleop(helm.model, {'neck_z': 0.35}), token='secret', simulation_only=True
        )
        async with TestClient(TestServer(app)) as client:
            assert (await (await client.get('/api/models')).json())['selected'] == helm.model.profile.id
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['model'] == helm.model.profile.id and 'neck_z' not in drawn['positions']
            state = await (await client.get('/api/helm/state')).json()
            assert state['xr']['preview_available'] is False  # No head model: nothing to preview.
            refused = await client.post('/api/helm/xr/preview', json={'on': True}, headers={'X-Helm-Token': 'secret'})
            assert refused.status == 422 and 'no head model' in (await refused.json())['error']

    asyncio.run(run())
