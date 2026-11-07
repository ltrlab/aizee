"""Helm's digital twin correction without hardware (helm.twin): Apply saves the proposal beside the head's files and
Remove takes it away, each one kept; the 3D views draw the measured arms and neck through it, only while it holds for
the references in use; a map's board turns with the head camera's poses; and the API. The correction itself is in
test_twin.py."""

import asyncio
import json
import math
from pathlib import Path
import time

import numpy as np
import pytest

from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.display_model import DisplayModel
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.twin import CHECK_S, PROPOSAL_S, TwinCorrection
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.neck import head
from robot_platform.ops.release import HEAD_MODEL
from robot_platform.vision import twin as vision_twin
from tests.unit.test_head_alignment import REFERENCE, rig  # noqa: F401
from tests.unit.test_twin import ARMS, camera_file, wrist_run

ROOT = Path(__file__).resolve().parents[2]
ASSET = 'minerva-production-01'


class Clock:
    def __init__(self):
        self.now = 1793000000.0

    def __call__(self):
        return self.now


class Teleop:
    """The teleop session's snapshot as the twin reads it, and its guide and neck as the 3D view draws them."""

    def __init__(self, model=None, phase='stopped', head_status=None, live=True):
        self.model, self.phase, self.head_status = model, phase, head_status
        self.receiver = {'mode': 'live' if live else 'virtual'}
        self.joints, self.measured = None, live
        self.active = False

    def snapshot(self):
        return {'phase': self.phase, 'head': self.head_status}

    def guide(self):
        if self.model is None:
            return None
        return {
            'model': self.model.profile.id,
            'urdf_sha256': self.model.sha256,
            'live': True,
            'measured': self.measured,
            'stream_id': 'helm-teleop',
            'sequence': 7,
            'badge': 'LIVE',
            'updated_at': time.time(),
            'source_age_seconds': 0.0,
            'feed_state': 'reading',
            'positions': {name: 0.1 for name in self.model.independent},
        }

    def head_joints(self):
        return self.joints

    def stop(self):
        pass


def setup(tmp_path, *, camera=None, recommended='joint_offsets', teleop=None, arms='arms-a'):
    """A twin flow over a wrist run, the head's files (a reference, and `camera` when given) and a clock."""
    files = head.files(tmp_path / 'operator', ASSET)
    files['reference'].parent.mkdir(parents=True, exist_ok=True)
    files['reference'].write_text(json.dumps(REFERENCE), encoding='utf-8')
    if camera is not None:
        files['camera'].write_text(json.dumps(camera), encoding='utf-8')
    state = {'arms': arms}
    clock = Clock()
    flow = TwinCorrection(
        wrists=wrist_run(tmp_path / 'calibration', recommended=recommended),
        files=lambda: files,
        path=lambda: vision_twin.path(tmp_path / 'operator', ASSET),
        arms_reference=lambda: state['arms'],
        teleop=(teleop or Teleop()).snapshot,
        history=tmp_path / 'calibration' / 'twin',
        clock=clock,
    )
    return flow, files, state, clock


def measured(pose_positions=None, **extra):
    return dict(
        {
            'live': True,
            'stream_id': 's',
            'sequence': 3,
            'badge': 'LIVE',
            'positions': dict(pose_positions or {'left_arm_j1': 0.1, 'right_arm_j5': -0.2, 'neck_z': 0.3}),
        },
        **extra,
    )


def test_apply_draws_the_measured_arms_through_the_zeros_and_remove_takes_it_away(rig, tmp_path):
    flow, _files, state, clock = setup(tmp_path)
    snapshot = flow.snapshot()
    assert snapshot['applied'] is None and snapshot['drawing'] == {'arms': 0, 'head': 0, 'notes': []}
    assert snapshot['proposal']['arms']['offsets_deg']['left_arm_j1'] == 0.4
    assert flow.draw(measured()) == measured()  # Nothing applied: the measured angles.
    snapshot = flow.apply()
    saved = json.loads(vision_twin.path(tmp_path / 'operator', ASSET).read_text(encoding='utf-8'))
    assert saved['kind'] == vision_twin.KIND and saved['arms']['reference_sha256'] == 'arms-a'
    assert snapshot['applied']['current'] and snapshot['drawing']['arms'] == 10 and not snapshot['busy']
    drawn = flow.draw(measured())
    # Drawn at the measured angle plus the zero: where the wrist solve puts the joint.
    assert drawn['positions']['left_arm_j1'] == pytest.approx(0.1 + math.radians(ARMS['left_wrist']['left_arm_j1']))
    assert drawn['positions']['right_arm_j5'] == pytest.approx(-0.2 + math.radians(ARMS['right_wrist']['right_arm_j5']))
    assert drawn['positions']['neck_z'] == 0.3  # No head part.
    assert drawn['stream_id'] == f's+twin{flow.revision}' and drawn['badge'] == 'LIVE · twin corrected'
    # Not readings: the reference pose, a virtual follower's: drawn as they are.
    for pose in (measured(live=False), measured(measured=False), {'positions': None}, None):
        assert flow.draw(pose) is pose
    flow.apply()  # Again: the one it replaces is kept.
    assert [p.name.split('-')[0] for p in (tmp_path / 'calibration' / 'twin').iterdir()] == ['replaced']
    # The arms zeroed again since: no longer drawn, and the card says why.
    state['arms'] = 'arms-b'
    clock.now += max(CHECK_S, PROPOSAL_S) + 0.1
    assert flow.draw(measured()) == measured()
    snapshot = flow.snapshot()
    assert snapshot['drawing']['arms'] == 0 and 'zeroed again' in snapshot['drawing']['notes'][0]
    assert not snapshot['applied']['current'] and snapshot['proposal']['arms'] is None
    snapshot = flow.clear()
    assert snapshot['applied'] is None and not vision_twin.path(tmp_path / 'operator', ASSET).exists()
    assert sorted(p.name.split('-')[0] for p in (tmp_path / 'calibration' / 'twin').iterdir()) == [
        'removed',
        'replaced',
    ]
    with pytest.raises(ValueError, match='No twin correction is applied'):
        flow.clear()
    with pytest.raises(ValueError, match='Nothing to correct: Arms: the arms were zeroed again'):
        flow.apply()
    assert 'Nothing to correct' in flow.snapshot()['error']


def test_the_neck_is_drawn_through_the_pan_zero_only_when_measured(rig, tmp_path):
    flow, files, _state, clock = setup(tmp_path, camera=camera_file(rig, 1.5), recommended='plain')
    flow.apply()
    assert flow.snapshot()['drawing'] == {'arms': 0, 'head': 1, 'notes': []}
    pan = math.radians(1.5)
    assert flow.draw(measured(), neck=True)['positions']['neck_z'] == pytest.approx(0.3 + pan, abs=1e-6)
    assert flow.draw(measured(), neck=False) == measured()  # The headset preview's angles, or the neck's zero.
    # The head camera aligned again since: its pan was compared against another alignment.
    files['camera'].write_text(json.dumps(camera_file(rig, 1.4)), encoding='utf-8')
    clock.now += CHECK_S + 0.1
    assert flow.draw(measured(), neck=True) == measured()
    assert 'aligned again' in flow.snapshot()['drawing']['notes'][0]


def test_the_head_cameras_turn_is_the_running_heads_else_the_saved_one(rig, tmp_path):
    teleop = Teleop()
    flow, _files, _state, _clock = setup(tmp_path, camera=camera_file(rig, 1.5), recommended='plain', teleop=teleop)
    np.testing.assert_allclose(flow.camera_turn(), np.eye(4))  # Nothing applied.
    flow.apply()
    saved = vision_twin.correction(rig['chain'], {'neck_z': math.radians(1.5)})
    np.testing.assert_allclose(flow.camera_turn(), saved, atol=1e-9)  # The next session's.
    # A session that started before it was applied: its camera's poses have none.
    teleop.phase, teleop.head_status = 'running', {'camera_offsets': {}}
    np.testing.assert_allclose(flow.camera_turn(), np.eye(4))
    teleop.head_status = {'camera_offsets': {'neck_z': 0.01}}
    np.testing.assert_allclose(flow.camera_turn(), vision_twin.correction(rig['chain'], {'neck_z': 0.01}), atol=1e-12)
    teleop.head_status = {'state': None}  # The head not built yet, or no camera feed: the saved one.
    np.testing.assert_allclose(flow.camera_turn(), saved, atol=1e-9)


def test_a_maps_board_turns_with_the_head_cameras_poses(rig, tmp_path):
    from robot_platform.helm.maps import MapLibrary
    from robot_platform.vision.geometry import invert

    camera = camera_file(rig, 1.5)
    flow, files, _state, _clock = setup(tmp_path, camera=camera, recommended='plain')
    maps = MapLibrary(
        teleop=Teleop().snapshot, root=tmp_path / 'maps', files=lambda: files, correction=flow.camera_turn
    )
    board = np.array(camera['board_in_robot']['matrix'])
    np.testing.assert_allclose(maps.worlds()['board'], invert(board), atol=1e-12)
    flow.apply()
    np.testing.assert_allclose(maps.worlds()['board'], invert(flow.camera_turn() @ board), atol=1e-12)


# -- the API --------------------------------------------------------------------------------------------------------


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(
        ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs', limits=tmp_path / 'limits.json'
    )
    yield session
    session.close()


def test_the_3d_views_draw_through_the_applied_correction(helm, rig, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer

    shown = UrdfModel(ModelProfile.load(ROOT / HEAD_MODEL))
    teleop = Teleop(helm.model)
    teleop.joints = {'neck_z': 0.2, 'neck_y': -0.3}
    flow, _files, _state, _clock = setup(tmp_path, camera=camera_file(rig, 1.5), teleop=teleop)

    async def run():
        app = create_app(
            helm, SimulationLink(), teleop=teleop, token='secret', display=DisplayModel(helm.model, shown), twin=flow
        )
        async with TestClient(TestServer(app)) as client:
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['positions']['left_arm_j1'] == 0.1 and drawn['positions']['neck_z'] == 0.2
            assert (await (await client.get('/api/helm/state')).json())['twin']['applied'] is None
            assert (await client.post('/api/helm/twin/apply', json={})).status == 403
            secret = {'X-Helm-Token': 'secret'}
            response = await client.post('/api/helm/twin/apply', json={}, headers=secret)
            assert response.status == 200, await response.text()
            assert (await response.json())['twin']['drawing'] == {'arms': 10, 'head': 1, 'notes': []}
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['positions']['left_arm_j1'] == pytest.approx(0.1 + math.radians(0.4))
            assert drawn['positions']['neck_z'] == pytest.approx(0.2 + math.radians(1.5), abs=1e-6)
            assert drawn['stream_id'].startswith('helm-teleop+twin') and drawn['model'] == shown.profile.id
            teleop.receiver = {'mode': 'virtual'}  # A virtual session's neck is not measured.
            teleop.measured = False
            drawn = await (await client.get('/api/calibration-guide')).json()
            assert drawn['positions']['left_arm_j1'] == 0.1 and drawn['positions']['neck_z'] == 0.2
            assert (await client.post('/api/helm/twin/apply', json={'x': 1}, headers=secret)).status >= 400
            assert (await client.post('/api/helm/twin/other', json={}, headers=secret)).status == 404
            response = await client.post('/api/helm/twin/clear', json={}, headers=secret)
            assert (await response.json())['twin']['applied'] is None

    asyncio.run(run())


def test_without_the_twin_the_api_says_why(helm):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), token='secret'))) as client:
            response = await client.post('/api/helm/twin/apply', json={}, headers={'X-Helm-Token': 'secret'})
            assert response.status >= 400 and 'Helm on Minerva' in (await response.json())['error']
            assert (await (await client.get('/api/helm/state')).json())['twin'] is None

    asyncio.run(run())
