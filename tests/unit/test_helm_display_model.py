"""Helm's 3D view draws the head model (helm.display_model): Minerva with her neck, every guide for the active model
drawn on it by joint name, the neck at the head's measured angles while a session drives it, else at its zero. The
guard and the receivers keep the active model."""
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
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.ops.release import HEAD_MODEL, MODEL

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'


@pytest.fixture(scope='module')
def models():
    return UrdfModel(ModelProfile.load(ROOT / MODEL)), UrdfModel(ModelProfile.load(ROOT / HEAD_MODEL))


def pose(active, **positions):
    return {'model': active.profile.id, 'urdf_sha256': active.sha256, 'live': True, 'stream_id': 'helm-teleop',
            'positions': {name: positions.get(name, 0.) for name in active.independent}, 'badge': 'LIVE'}


def test_the_head_model_is_the_active_one_plus_the_neck(models):
    active, shown = models
    display = DisplayModel(active, shown)
    assert set(display.extra) == {'neck_z', 'neck_y'} and display.profile.id == shown.profile.id
    drawn = display.guide(pose(active, left_arm_j2=.4), {'neck_z': .3, 'neck_y': -.2})
    assert (drawn['model'], drawn['urdf_sha256']) == (shown.profile.id, shown.sha256)
    assert set(drawn['positions']) == set(shown.independent)
    assert drawn['positions']['left_arm_j2'] == .4 and (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (.3, -.2)
    assert drawn['badge'] == 'LIVE' and drawn['stream_id'] == 'helm-teleop'  # The rest of the guide as it was.
    shown.positions(drawn['positions'])  # A pose the shown model takes.
    # No session: the neck at its zero; a reading past its range, or not a number, is not drawn as one.
    assert display.guide(pose(active))['positions']['neck_z'] == 0.
    clamped = display.guide(pose(active), {'neck_z': 4., 'neck_y': math.nan})['positions']
    assert (clamped['neck_z'], clamped['neck_y']) == (shown.by_name['neck_z']['bounds'][1], 0.)
    # Nothing to draw, or another model's guide: unchanged.
    assert display.guide(None) is None
    other = dict(pose(active), urdf_sha256='0' * 64)
    assert display.guide(other) is other
    # Joint telemetry: the same rows, stamped for the shown model.
    packet = {'model': active.profile.id, 'urdf_sha256': active.sha256, 'stream_id': 'reader', 'sequence': 4,
              'rows': [{'motor': 'left_j2', 'joint': 'left_arm_j2', 'position': .4}]}
    stamped = display.telemetry(packet)
    assert (stamped['model'], stamped['urdf_sha256'], stamped['rows']) == (shown.profile.id, shown.sha256, packet['rows'])
    assert display.telemetry(other) is other and display.telemetry(None) is None


def test_a_model_that_moves_an_active_joint_is_not_drawn_in_its_place(models):
    active, shown = models
    moved = deepcopy(shown.by_name)
    moved['left_arm_j3']['origin']['xyz'][0] += .001  # One millimetre off.
    with pytest.raises(ValueError, match='left_arm_j3'):
        DisplayModel(active, SimpleNamespace(by_name=moved, independent=shown.independent, profile=shown.profile))
    narrower = deepcopy(shown.by_name)
    narrower['right_arm_j1']['bounds'] = [-1., 1.]
    with pytest.raises(ValueError, match='right_arm_j1'):
        DisplayModel(active, SimpleNamespace(by_name=narrower, independent=shown.independent, profile=shown.profile))
    # The same rotation written with -pi for pi is the same frame.
    joint = deepcopy(active.by_name['left_arm_j1'])
    flipped = deepcopy(joint)
    joint['origin']['rpy'], flipped['origin']['rpy'] = [math.pi, 0., 0.], [-math.pi, 0., 0.]
    assert _same(joint, flipped)
    logged = []
    assert DisplayModel.load(active, ROOT / 'urdf/minerva/missing/model.json', log=logged.append) is None
    assert 'without the head' in logged[0]


def test_the_session_hands_the_view_the_necks_angles_only_while_it_runs():
    loop = SimpleNamespace(_lock=threading.Lock(), active=True,
                           _view={'phase': 'running', 'head': {'joints': {'neck_z': .2, 'neck_y': -.1}}})
    assert TeleopSession.head_joints(loop) == {'neck_z': .2, 'neck_y': -.1}
    loop._view = {'phase': 'running', 'head': None}  # A session without a head.
    assert TeleopSession.head_joints(loop) is None
    loop._view = {'phase': 'stopped', 'head': {'joints': {'neck_z': .2, 'neck_y': -.1}}}
    assert TeleopSession.head_joints(loop) is None
    loop._view, loop.active = {'phase': 'running', 'head': {'joints': {'neck_z': .2}}}, False
    assert TeleopSession.head_joints(loop) is None


class Teleop:
    active = False

    def __init__(self, model, joints):
        self.model, self.joints = model, joints

    def guide(self):
        return dict(pose(self.model, right_arm_j4=-.5), updated_at=time.time(), source_age_seconds=0.,
                    feed_state='reading')

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
    teleop = Teleop(helm.model, {'neck_z': .35, 'neck_y': -.15})

    async def run():
        app = create_app(helm, SimulationLink(), teleop=teleop, token='secret', simulation_only=True,
                         display=DisplayModel(helm.model, shown))
        async with TestClient(TestServer(app)) as client:
            catalog = await (await client.get('/api/models')).json()
            assert catalog['selected'] == shown.profile.id and [m['id'] for m in catalog['models']] == [shown.profile.id]
            metadata = await (await client.get(f'/api/models/{shown.profile.id}')).json()
            assert {'neck_z', 'neck_y'} <= set(metadata['independent'])
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert (drawn['model'], drawn['urdf_sha256']) == (shown.profile.id, shown.sha256)
            assert drawn['positions']['right_arm_j4'] == -.5
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (.35, -.15)
            teleop.joints = None  # The session ended: the neck at its zero.
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert (drawn['positions']['neck_z'], drawn['positions']['neck_y']) == (0., 0.)
            # The joint telemetry the page relays to the view: for the drawn model.
            state = await (await client.get('/api/helm/state')).json()
            assert (state['telemetry']['model'], state['telemetry']['urdf_sha256']) == (shown.profile.id, shown.sha256)
    asyncio.run(run())


def test_without_a_display_model_the_view_draws_the_active_one(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        app = create_app(helm, SimulationLink(), teleop=Teleop(helm.model, {'neck_z': .35}), token='secret',
                         simulation_only=True)
        async with TestClient(TestServer(app)) as client:
            assert (await (await client.get('/api/models')).json())['selected'] == helm.model.profile.id
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['model'] == helm.model.profile.id and 'neck_z' not in drawn['positions']
    asyncio.run(run())
