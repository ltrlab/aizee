"""The scripted handoff demonstration plans reachable moves that clear the table and the bucket. No receiver.

The first tests plan the measured workcell's own layout and nominal start. The far-corner and turned-cube tests were
planned on the placeholder layout of 2026-10-17 and keep it (tests/fixtures/minerva_workcell_placeholder_20261017.json):
the measured box and the robot's interim place moved the cube region since."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.evaluation.demonstrator import DROPS, HANDOFF, PACE_STOP, HandoffDemonstrator, across
from robot_platform.evaluation.plants import _Stack, derive_stack
from robot_platform.simulation.workcell import Workcell, load_workcell

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
PLACEHOLDER_LAYOUT = ROOT / 'tests/fixtures/minerva_workcell_placeholder_20261017.json'


@pytest.fixture(scope='module')
def planned():
    workcell = load_workcell(MEASURED)
    stack = derive_stack()
    limits = _Stack(stack)
    context = SimpleNamespace(
        joint_names=stack['names'],
        joint_map=stack['derived']['joint_map'],
        lower=limits.lower,
        upper=limits.upper,
        home=limits.home,
        workcell=workcell,
        period_s=limits.period_s,
        lead=limits.lead,
        cube_start={'x': workcell.cube_start[0], 'y': workcell.cube_start[1], 'yaw': workcell.cube_start[2]},
    )
    demo = HandoffDemonstrator()
    demo.reset(context)
    yield demo, context, workcell
    demo.close()


def test_the_plan_grasps_the_cube_meets_at_the_handoff_and_drops_over_the_bucket(planned):
    demo, context, workcell = planned
    frames = {frame.label: frame for frame in demo.frames}
    order = [frame.label for frame in demo.frames]
    assert (
        order.index('left closes')
        < order.index('right closes')
        < order.index('left lets go')
        < order.index('right lets go')
    )
    ik = demo._ik
    cube = np.array([context.cube_start['x'], context.cube_start['y'], workcell.table_top + workcell.cube_half])
    middle, finger, closing = ik.frame('left', frames['left on the cube'].target)
    # Pads centred on the cube, fingers tilted down, jaws closing level.
    assert np.linalg.norm(middle - cube) < 1e-3 and finger[2] < -0.2 and abs(closing[2]) < 0.02
    left, left_finger, left_closing = ik.frame('left', frames['left holds the cube out'].target)
    right, right_finger, right_closing = ik.frame('right', frames['right on the cube'].target)
    handoff = np.array([HANDOFF[0], HANDOFF[1], workcell.table_top + HANDOFF[2]])
    # Both pads meet on the cube near the handoff point. The left's jaws close level and the right's vertically (its J5
    # a quarter turn from the level take): level jaws on perpendicular faces crossed their fingers behind the cube.
    assert np.linalg.norm(left - right) < 1e-3 and np.linalg.norm(left - handoff) < 0.012
    assert abs(left_closing[2]) < 0.05 and abs(right_closing[2]) > 0.99 and abs(left_closing @ right_closing) < 0.05
    drop, _, drop_closing = ik.frame('right', frames['right holds over the bucket'].target)
    (bx, by), (ix, iy) = workcell.bucket_center, workcell.bucket_inner
    assert abs(drop[0] - bx) < ix / 2 and abs(drop[1] - by) < iy / 2
    assert drop[2] > workcell.table_top + workcell.bucket_height + 0.05 and abs(drop_closing[2]) < 0.02
    # Every commanded point lies inside the range the direct receiver accepts.
    for frame in demo.frames:
        for point in frame.path or [frame.target]:
            assert np.all(point >= context.lower - 1e-9) and np.all(point <= context.upper + 1e-9), frame.label


def test_every_planned_move_clears_the_table_the_bucket_and_the_other_arm(planned):
    demo, _, _ = planned
    clearances = demo.plan_clearance()
    assert len(clearances) == sum(1 for frame in demo.frames if frame.kind == 'move')
    for label, distance, robot, other in clearances:
        assert distance > 0.001, f'{label}: {robot} comes {distance * 1000:.1f} mm from {other}'
    # No move spins a wrist the long way round (one took 41 s before the plan chose its wrist-roll branch).
    moves = [frame.duration for frame in demo.frames if frame.kind == 'move']
    assert max(moves) < 10.0 and 10.0 < sum(moves) < 40.0


def test_a_move_waits_for_an_arm_that_falls_behind(planned):
    demo, context, _ = planned
    demo._timeline(demo.frames)
    demo.bias = np.zeros(len(context.joint_names))
    arms = demo.arms['left'] + demo.arms['right']
    first = demo.frames[0]
    assert first.kind == 'move'
    # An arm stuck at the start: the plan leads each joint by at most PACE_STOP of its lead and one cycle's step, and
    # stays on the move.
    still = demo.ready.copy()
    commands = [demo({'t': k * 0.01, 'q': still})[0] for k in range(500)]
    beyond = max(float((np.abs(c[arms] - still[arms]) - PACE_STOP * context.lead[arms]).max()) for c in commands)
    assert beyond < 0.01
    assert demo.current == 0
    # An arm that keeps up with its commands finishes the move in its planned time.
    q, t = commands[-1].copy(), 5.0
    while demo.current == 0 and t < 5.0 + first.duration + 1.0:
        t += 0.01
        q = demo({'t': t, 'q': q})[0]
    assert demo.current == 1 and t - 5.0 <= first.duration + 0.02
    demo._timeline(demo.frames)


def test_from_the_far_corner_the_arms_go_to_the_handoff_one_at_a_time(planned):
    # From here, moving together, the left's wrist swept 4.9 mm through the right's fingertips on the way to the
    # handoff, so no plan cleared: the left brings the cube to the handoff first, and the right comes after.
    demo, context, _ = planned
    try:
        demo.reset(
            SimpleNamespace(
                **{
                    **vars(context),
                    'workcell': load_workcell(PLACEHOLDER_LAYOUT),
                    'cube_start': {'x': 0.043, 'y': -0.497, 'yaw': math.radians(16.5)},
                }
            )
        )
        labels = [frame.label for frame in demo.frames]
        assert labels.index('left to the handoff') < labels.index('right to the handoff') < labels.index('right turns')
        assert min(distance for _, distance, _, _ in demo.plan_clearance()) > 0.001
    finally:
        demo.reset(context)


@pytest.mark.parametrize('x, y, yaw', [(0.26, -0.50, -40.0), (0.02, -0.50, 40.0)])
def test_a_turned_cube_is_picked_by_a_pair_of_faces_the_left_reaches_and_every_move_clears(planned, x, y, yaw):
    # Before the second pair of faces and the climb to the handoff, the first could not be planned to clear (the
    # pads swung 14 mm into the table on the way to the handoff) and the second could not be picked at all.
    demo, context, _ = planned
    try:
        demo.reset(
            SimpleNamespace(
                **{
                    **vars(context),
                    'workcell': load_workcell(PLACEHOLDER_LAYOUT),
                    'cube_start': {'x': x, 'y': y, 'yaw': math.radians(yaw)},
                }
            )
        )
        assert min(distance for _, distance, _, _ in demo.plan_clearance()) > 0.001
        frames = {frame.label: frame for frame in demo.frames}
        _, finger, _ = demo._ik.frame('left', frames['left on the cube'].target)
        # The fingers point along one of the cube's face normals, turned at most 20 degrees off it.
        azimuth = math.degrees(math.atan2(finger[1], finger[0]))
        assert abs((azimuth - 270.0 - yaw + 45.0) % 90.0 - 45.0) <= 20.5
    finally:
        demo.reset(context)


def test_a_cuboid_is_taken_across_a_face_pair_the_jaws_open_across_and_the_choice_recorded(planned):
    """A block of the size the head camera fitted to the near real block (52 x 37 mm, 39 mm tall), at its place and
    turn: the left takes it across a pair of its side faces and the plan says which pair and how wide; the left's jaws
    stop at that width and the right's at the block's height."""
    demo, context, workcell = planned
    block = Workcell.from_description(dict(workcell.describe(), cube_size=[0.052, 0.037, 0.039]))
    try:
        demo.reset(SimpleNamespace(**{**vars(context), 'workcell': block}))
        assert min(distance for _, distance, _, _ in demo.plan_clearance()) > 0.001
        assert (
            demo.grasp['faces'] in ('x', 'y') and demo.grasp['width_m'] == {'x': 0.052, 'y': 0.037}[demo.grasp['faces']]
        )
        assert demo.grasp['height_m'] == 0.039
        # Each gripper is posed where its jaws stop on the block: the left at the pair's width, the right at the height.
        stops = dict(zip(('left', 'right'), demo._stops))
        for side, span in (('left', demo.grasp['width_m']), ('right', 0.039)):
            assert demo._gripper_gap(side, stops[side][1]) == pytest.approx(span, abs=1e-9)
        frames = {frame.label: frame for frame in demo.frames}
        index = list(context.joint_names).index('left_gripper')
        closes = frames['left closes'].target[index]
        assert (closes - stops['left'][1]) * (stops['left'][1] - stops['left'][2]) > 0  # Commanded past touching.
        assert demo.describe()['planned_grasp']['width_m'] == demo.grasp['width_m']
    finally:
        demo.reset(context)


def test_a_face_pair_too_wide_to_open_across_is_passed_over(planned):
    """A 95 x 60 mm block: the jaws open at most about 97 mm, so they cannot take it across its 95 mm faces with 5 mm
    to spare each side; the left only tries its 60 mm faces."""
    demo, context, workcell = planned
    block = Workcell.from_description(dict(workcell.describe(), cube_size=[0.095, 0.060, 0.040]))
    try:
        try:
            demo.reset(
                SimpleNamespace(
                    **{
                        **vars(context),
                        'workcell': block,
                        'cube_start': {'x': context.cube_start['x'], 'y': context.cube_start['y'], 'yaw': 0.0},
                    }
                )
            )
        except ValueError:
            pass  # Whether the whole task plans with so long a block is not the question here.
        assert demo._open['left'][1] < 0.095 + 2 * 0.005
        assert [across(block.block, 0.0, face) for face in demo._takeable(block.block, 0.0)] == [('y', 0.060)]
    finally:
        demo.reset(context)


def test_the_drops_start_with_the_one_the_layout_was_planned_with():
    assert (
        DROPS[0]['name'] == 'from the front' and DROPS[0]['clearance'] == 0.12 and DROPS[0]['lead'] == (0.0, 0.12, 0.0)
    )
    assert len({drop['name'] for drop in DROPS}) == len(DROPS)
