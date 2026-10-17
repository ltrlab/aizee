"""The task scene's workcell: measured values carry their sources, and the scene judges the bucket it was given."""
import json
from pathlib import Path

import numpy as np
import pytest

from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.simulation.task_scene import CUBE_START, TABLE_TOP, MinervaTaskScene
from robot_platform.simulation.workcell import SYNTHETIC, Workcell, load_workcell, resolve, validate, workcell_path

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    return mapping


def test_synthetic_layout_keeps_the_constants_the_fixture_was_built_on():
    assert SYNTHETIC.table_top == TABLE_TOP == .28
    assert (SYNTHETIC.cube_start[0], SYNTHETIC.cube_start[1], SYNTHETIC.table_top + SYNTHETIC.cube_half + .002) \
        == CUBE_START
    assert resolve(None) is SYNTHETIC and resolve('synthetic') is SYNTHETIC
    assert SYNTHETIC.placeholders() == []
    assert cube_poses(GridSpec())[0]['x'] == CUBE_START[0]


def test_measured_file_names_a_source_for_every_value_and_lists_what_is_unmeasured():
    workcell = load_workcell(MEASURED)
    assert workcell.asset_id == 'minerva-production-01'
    assert workcell_path(ROOT / 'config/devices/minerva-joint-calibration.json', workcell.asset_id) == MEASURED
    # The user's measurements are values, not placeholders.
    assert workcell.table_top == pytest.approx(.664 - .152, abs=1e-3)
    assert workcell.cube_edge == pytest.approx(.0381) and workcell.bucket_inner == pytest.approx((.152, .305))
    measured = {'table.size', 'table.top_z', 'bucket.inner_size', 'cube.edge'}
    assert measured.isdisjoint(workcell.placeholders())
    assert {'table.center', 'bucket.center', 'bucket.height', 'cube.mass', 'cameras.head.pos'} <= \
        set(workcell.placeholders())
    assert all(workcell.sources[key] for key in workcell.sources)
    # The description crosses process boundaries unchanged.
    assert Workcell.from_description(json.loads(json.dumps(workcell.describe()))) == workcell


def test_a_value_without_a_source_is_refused(tmp_path):
    document = json.loads(MEASURED.read_text(encoding='utf-8'))
    document['table']['top_z'] = {'value': .5}
    path = tmp_path / 'bad-workcell.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError, match='named source'):
        load_workcell(path)
    with pytest.raises(ValueError, match='fit inside'):
        validate(Workcell.from_description(dict(SYNTHETIC.describe(), bucket_inner=[.03, .03])))


def test_grid_and_scene_follow_the_workcell():
    workcell = load_workcell(MEASURED)
    poses = cube_poses(GridSpec.for_workcell(workcell))
    assert (poses[0]['x'], poses[0]['y']) == workcell.cube_start[:2]
    (x0, x1), (y0, y1) = workcell.cube_region
    assert all(x0 <= p['x'] <= x1 and y0 <= p['y'] <= y1 for p in poses)
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        scene._q = np.zeros(14)
        state = scene.reset()
        for _ in range(6):
            state = scene.update(np.zeros(14), .05)
        # The cube rests on the measured table; the hanging arms do not reach it.
        assert state['cube_position'][2] == pytest.approx(workcell.table_top + workcell.cube_half, abs=2e-4)
        assert state['cube_contacts'] == ['task/table'] and not state['forbidden_contacts']
        assert scene.capabilities['workcell']['name'] == workcell.name
        assert scene.capabilities['workcell_placeholders'] == workcell.placeholders()
        assert any('not yet measured' in item for item in scene.capabilities['limitations'])
    finally:
        scene.close()


def _bucket_drop(workcell, offset):
    """Drop the cube from above the bucket at `offset` (bucket axes); the scene's verdict on where it settles."""
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        scene._q = np.zeros(14)
        scene.reset()
        c, s = np.cos(workcell.bucket_yaw), np.sin(workcell.bucket_yaw)
        x = workcell.bucket_center[0] + c * offset[0] - s * offset[1]
        y = workcell.bucket_center[1] + s * offset[0] + c * offset[1]
        scene.data.qpos[scene.cube_qadr:scene.cube_qadr + 3] = [x, y, workcell.table_top + workcell.bucket_height + .05]
        scene.mj.mj_forward(scene.model, scene.data)
        inside = touched = False
        for _ in range(40):
            state = scene.update(np.zeros(14), .05)
            touched = touched or 'task/bucket/bottom' in state['cube_contacts']
        along = np.array(state['cube_position'][:2]) - np.array(workcell.bucket_center)
        inside = abs(c * along[0] + s * along[1]) < workcell.bucket_inner[0] / 2 and \
            abs(-s * along[0] + c * along[1]) < workcell.bucket_inner[1] / 2
        return inside, touched, state
    finally:
        scene.close()


@pytest.mark.parametrize('yaw_deg', [0., 30.])
def test_a_rectangular_yawed_bucket_holds_the_cube_dropped_into_its_long_end(yaw_deg):
    measured = load_workcell(MEASURED)
    workcell = Workcell.from_description(dict(measured.describe(), bucket_yaw=np.radians(yaw_deg)))
    # Near the far end of the long side: inside a 0.305 m box, outside a square one of the short side.
    inside, touched, state = _bucket_drop(workcell, (0., .10))
    assert inside and touched
    # Settled on the bottom with no gripper: the scene sees it in the bucket, out of sequence, so no success.
    assert state['bucket_settled'] and not state['success'] and not state['forbidden_contacts']
    # The same distance across the short side is outside the 0.152 m box.
    inside, touched, state = _bucket_drop(workcell, (.10, 0.))
    assert not inside and not touched and not state['bucket_settled']
