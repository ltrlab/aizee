"""Driven grippers: the jaws move under the drive's MIT law and stop on what they hold. No receiver involved."""

import json
from pathlib import Path

import numpy as np
import pytest

from robot_platform.simulation.task_scene import MinervaTaskScene

FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']
LEFT, RIGHT = NAMES.index('left_gripper'), NAMES.index('right_gripper')
SCALE = FIXTURE['gripper_test_scale_m_per_motor_rad']
DRIVE = dict(inertia=0.002, friction=0.1, viscous=0.0, max_effort=14.0, source='test: nominal values')
START, LIFTED = (np.array(FIXTURE['waypoints'][k]['q']) for k in (0, 2))


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1.0, offset=0.0) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=SCALE, offset=0.0)
    return mapping


def command(position, kp=5.0, kd=0.2, effort=0.0):
    return dict(position=position, velocity=0.0, effort=effort, kp=kp, kd=kd)


@pytest.fixture
def scene():
    scene = MinervaTaskScene(
        fixture_map(), NAMES, width=64, height=48, grippers={'left_gripper': DRIVE, 'right_gripper': DRIVE}
    )
    scene._q = START.copy()
    scene.reset()  # The fixture's start: the open left jaws around the cube.
    yield scene
    scene.close()


def hold(scene, pose, commands, seconds):
    state = None
    for _ in range(round(seconds / 0.0025)):
        state = scene.step_driven(pose, pose, commands, 0.0025)
    return state


def test_the_cube_contact_forces_read_by_array_are_mujocos(scene):
    # Contact tracking reads the cube's contact forces by array (_normal_forces): bit for bit mj_contactForce's normal
    # force, for every contact of a gripper squeezing the cube on the table.
    hold(scene, START, {'left_gripper': command(1.0), 'right_gripper': command(START[RIGHT])}, 1.0)
    rows = np.arange(scene.data.ncon)
    expected, force = [], np.zeros(6)
    for i in rows:
        scene.mj.mj_contactForce(scene.model, scene.data, int(i), force)
        expected.append(force[0])
    assert len(rows) > 20 and scene._normal_forces(scene.data.contact, rows).tolist() == expected


def test_a_closing_gripper_stops_on_the_cube_and_squeezes_with_its_drive_torque(scene):
    assert scene.driven and scene.capabilities['gripper_drives']['left_gripper']['max_effort'] == 14.0
    assert scene.model.neq == 4 and scene.model.nu == 2  # Two transmissions and two mimic jaws; no attachment.
    state = hold(scene, START, {'left_gripper': command(1.0), 'right_gripper': command(START[RIGHT])}, 1.0)
    q, v, torque = state['left_gripper']
    # Commanded to 1.0, the drive stops where the pads meet the 36 mm cube, near the fixture's own 1.73 grasp.
    assert 1.73 < q < 1.80 and abs(v) < 1e-3
    assert torque == pytest.approx(5.0 * (1.0 - q), abs=0.02)
    snapshot = scene.snapshot()
    assert snapshot['jaw_contacts']['left'] == [0, 1] and not snapshot['forbidden_contacts']
    assert snapshot['grippers']['left_gripper']['q'] == pytest.approx(q)
    force, squeeze = np.zeros(6), []
    for i in range(scene.data.ncon):
        contact = scene.data.contact[i]
        if scene.cube_geom in (contact.geom1, contact.geom2):
            other = contact.geom2 if contact.geom1 == scene.cube_geom else contact.geom1
            if '/finger/' in scene.mj.mj_id2name(scene.model, scene.mj.mjtObj.mjOBJ_GEOM, other):
                scene.mj.mj_contactForce(scene.model, scene.data, i, force)
                squeeze.append((force[0], -contact.dist))
    # The drive torque through the transmission is the jaws' force together (the mimic jaw moves with the first, so
    # each squeezes with half); the pads stay at the cube's surface.
    total = sum(f for f, _ in squeeze)
    assert total == pytest.approx(abs(torque) / SCALE, rel=0.05)
    assert max(depth for _, depth in squeeze) < 0.0005
    # The right gripper, closing on nothing, reaches its command (within its friction band).
    state = hold(scene, START, {'left_gripper': command(1.0), 'right_gripper': command(-1.0)}, 1.0)
    assert state['right_gripper'][0] == pytest.approx(-1.0, abs=0.1 / 5 + 0.005)


def test_the_drive_torque_is_clipped_to_its_limit(scene):
    state = hold(scene, START, {'left_gripper': command(0.0, kp=50.0), 'right_gripper': None}, 0.5)
    assert state['left_gripper'][2] == pytest.approx(-14.0)
    assert state['right_gripper'][2] == 0.0  # Limp: no command, no torque.


def test_the_grip_lifts_the_cube_and_opening_drops_it(scene):
    grip = {'left_gripper': command(1.0), 'right_gripper': command(START[RIGHT])}
    hold(scene, START, grip, 0.5)
    for k in range(80):
        a, b = START + (LIFTED - START) * k / 80, START + (LIFTED - START) * (k + 1) / 80
        for j in range(4):
            scene.step_driven(a + (b - a) * j / 4, a + (b - a) * (j + 1) / 4, grip, 0.0025)
    snapshot = scene.snapshot()
    assert snapshot['first_gripper'] == 'left'
    assert snapshot['cube_position'][2] > scene.workcell.table_top + scene.workcell.cube_half + 0.03
    # Nothing holds the cube but the squeeze: open the jaws and it falls back to the table.
    hold(scene, LIFTED, {'left_gripper': command(3.0), 'right_gripper': command(START[RIGHT])}, 1.0)
    snapshot = scene.snapshot()
    assert snapshot['dropped'] and 'dropped_outside_bucket' in [e['kind'] for e in snapshot['events']]


def test_posing_a_driven_scene_places_the_drives_with_the_arms(scene):
    pose = START.copy()
    pose[LEFT] = 2.5
    scene.update(pose, 0.01)
    assert scene.gripper_state()['left_gripper'][0] == pytest.approx(2.5, abs=1e-6)
    with pytest.raises(RuntimeError, match='driven grippers'):
        MinervaTaskScene(fixture_map(), NAMES, width=64, height=48).step_driven(START, START, {}, 0.0025)
    with pytest.raises(ValueError, match='drive values'):
        MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, grippers={'left_gripper': DRIVE})
