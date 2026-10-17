"""The scripted handoff demonstration plans reachable moves that clear the table and the bucket. No receiver."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.evaluation.demonstrator import HANDOFF, HandoffDemonstrator
from robot_platform.evaluation.plants import _Stack, derive_stack
from robot_platform.simulation.workcell import load_workcell

MEASURED = Path(__file__).resolve().parents[2] / 'config/devices/simulation/minerva-production-01-workcell.json'


@pytest.fixture(scope='module')
def planned():
    workcell = load_workcell(MEASURED)
    stack = derive_stack()
    limits = _Stack(stack)
    context = SimpleNamespace(joint_names=stack['names'], joint_map=stack['derived']['joint_map'], lower=limits.lower,
                              upper=limits.upper, home=limits.home, workcell=workcell, period_s=limits.period_s,
                              cube_start={'x': workcell.cube_start[0], 'y': workcell.cube_start[1], 'yaw': 0.})
    demo = HandoffDemonstrator()
    demo.reset(context)
    yield demo, context, workcell
    demo.close()


def test_the_plan_grasps_the_cube_meets_at_the_handoff_and_drops_over_the_bucket(planned):
    demo, context, workcell = planned
    frames = {frame.label: frame for frame in demo.frames}
    order = [frame.label for frame in demo.frames]
    assert order.index('left closes') < order.index('right closes') < order.index('left lets go') < \
        order.index('right lets go')
    ik = demo._ik
    cube = np.array([context.cube_start['x'], context.cube_start['y'], workcell.table_top + workcell.cube_half])
    middle, finger, closing = ik.frame('left', frames['left on the cube'].target)
    # Pads centred on the cube, fingers tilted down, jaws closing level.
    assert np.linalg.norm(middle - cube) < 1e-3 and finger[2] < -.2 and abs(closing[2]) < .02
    left, left_finger, left_closing = ik.frame('left', frames['left holds the cube out'].target)
    right, right_finger, right_closing = ik.frame('right', frames['right on the cube'].target)
    handoff = np.array([HANDOFF[0], HANDOFF[1], workcell.table_top + HANDOFF[2]])
    # Both pads meet on the cube at the handoff, closing on perpendicular faces.
    assert np.linalg.norm(left - handoff) < 1e-3 and np.linalg.norm(right - handoff) < 1e-3
    assert abs(left_closing @ right_closing) < .05
    drop, _, drop_closing = ik.frame('right', frames['right holds over the bucket'].target)
    (bx, by), (ix, iy) = workcell.bucket_center, workcell.bucket_inner
    assert abs(drop[0] - bx) < ix / 2 and abs(drop[1] - by) < iy / 2
    assert drop[2] > workcell.table_top + workcell.bucket_height + .05 and abs(drop_closing[2]) < .02
    # Every commanded point lies inside the range the direct receiver accepts.
    for frame in demo.frames:
        for point in frame.path or [frame.target]:
            assert np.all(point >= context.lower - 1e-9) and np.all(point <= context.upper + 1e-9), frame.label


def test_every_planned_move_clears_the_table_and_bucket(planned):
    demo, _, _ = planned
    clearances = demo.plan_clearance()
    assert len(clearances) == sum(1 for frame in demo.frames if frame.kind == 'move')
    for label, distance, robot, environment in clearances:
        assert distance > .001, f'{label}: {robot} comes {distance * 1000:.1f} mm from {environment}'
    # The start pose holds both grippers clear of the table and bucket.
    assert demo.ready is not None
    total = sum(frame.duration for frame in demo.frames)
    assert 20. < total < 60.
