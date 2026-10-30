"""The head's shoulder clearance (neck.clearance): the floor its recorded corners make, and steps that keep the head
above it on the way. Turned to a side and tilted all the way down, Minerva's head reaches its shoulder (2026-10-29)."""
import itertools

import pytest

from robot_platform.neck.clearance import Clearance, from_limits, section

# Tilting down reads negative here; turned 0.6 rad either way, the head clears the shoulder down to 0.4 rad.
CORNERS = [{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}]


def floor(corners=CORNERS, reach=.8, margin=.1):
    return Clearance(section(corners, margin_rad=margin), reach)


@pytest.mark.parametrize('corners, match', [
    ([{'pan': .6, 'tilt': -.4}], 'each side'),
    ([{'pan': .6, 'tilt': -.4}, {'pan': -.05, 'tilt': -.4}], 'turned to a side'),
    ([{'pan': .6, 'tilt': -.4}, {'pan': -.6, 'tilt': .4}], 'tilted down'),
    ([{'pan': .6, 'tilt': -.4}, {'pan': -.6, 'tilt': -.02}], 'tilted down'),
    ([{'pan': .6, 'tilt': float('nan')}, {'pan': -.6, 'tilt': -.4}], 'pan and a tilt'),
])
def test_corners_on_both_sides_tilted_down_the_same_way(corners, match):
    with pytest.raises(ValueError, match=match):
        section(corners)
    assert section(CORNERS)['down'] == -1 and section(CORNERS, margin_rad=0.)['margin_rad'] == 0.


def test_the_floor_runs_from_the_whole_tilt_range_ahead_to_each_corner_and_never_eases_off_outward():
    clear = floor([{'pan': .3, 'tilt': -.5}, {'pan': .6, 'tilt': -.7}, {'pan': -.6, 'tilt': -.4}])
    assert clear.depth(0.) == pytest.approx(.8)  # Straight ahead, its whole range down.
    assert clear.depth(.15) == pytest.approx(.6)  # Halfway to the first corner, less its margin.
    assert clear.depth(.3) == pytest.approx(.4)
    assert clear.depth(.6) == pytest.approx(.4) and clear.depth(1.2) == pytest.approx(.4)  # No easing off past it.
    assert clear.depth(-.6) == pytest.approx(.3) and clear.depth(-1.) == pytest.approx(.3)
    shallow = floor([{'pan': .6, 'tilt': -.15}, {'pan': -.6, 'tilt': -.15}], margin=.2)
    assert shallow.depth(.6) == 0.  # Level always allowed: a pan alone never reaches the shoulder.
    assert clear.clamp(.6, -.7) == (.6, pytest.approx(-.4)) and clear.clamp(.6, -.2) == (.6, -.2)
    assert clear.clear(0., -.8) and not clear.clear(.6, -.45) and clear.clear(.6, .9)


def test_a_clearance_belongs_to_its_reference():
    reference = {'id': 'head-1', 'motors': [{'name': 'head_pan', 'joint': 'neck_z', 'raw_reference_rad': 3.},
                                            {'name': 'head_tilt', 'joint': 'neck_y', 'raw_reference_rad': 3.}]}
    limits = {'reference_id': 'head-1', 'clearance': section(CORNERS),
              'joints': {'head_pan': {'min_rad': 1.5, 'max_rad': 4.5}, 'head_tilt': {'min_rad': 2.2, 'max_rad': 4.1}}}
    assert from_limits(limits, reference).reach == pytest.approx(.8)  # Its tilt limit on the down side.
    assert from_limits(dict(limits, clearance=None), reference) is None
    with pytest.raises(ValueError, match='another reference'):
        from_limits(dict(limits, reference_id='head-0'), reference)


def walk(clear, start, goal, limit=(.036, .036), cycles=200):
    poses = [start]
    for _ in range(cycles):
        poses.append(clear.step(poses[-1], goal, limit))
    return poses


def test_turning_while_looking_all_the_way_down_slides_up_the_floor_and_never_dips_under_it():
    clear = floor()
    poses = walk(clear, (0., -.75), (.9, -.8))  # Looking straight down, then turning far to the side.
    assert all(clear.clear(*pose) for pose in poses)
    assert poses[-1] == (pytest.approx(.9), pytest.approx(-.3))  # As far down as the floor allows there.
    steps = [(abs(b[0] - a[0]), abs(b[1] - a[1])) for a, b in zip(poses, poses[1:])]
    assert max(pan for pan, _ in steps) <= .036 + 1e-9 and max(tilt for _, tilt in steps) <= .036 + 1e-9
    # Between two commands the receiver ramps straight: those chords dip under the floor far less than its margin.
    for a, b in zip(poses, poses[1:]):
        for share in (.25, .5, .75):
            pan, tilt = a[0] + (b[0] - a[0]) * share, a[1] + (b[1] - a[1]) * share
            assert -tilt - clear.depth(pan) < .01


def test_a_head_below_the_floor_rises_straight_up_first():
    clear = floor()
    assert clear.step((.7, -.6), (0., 0.), (.036, .036)) == (.7, pytest.approx(-.564))
    poses = walk(clear, (.7, -.6), (0., -.8), cycles=30)
    assert clear.clear(*poses[-1]) and poses[8][0] == .7  # Out of the corner before it turns.


@pytest.mark.parametrize('pan_speed, tilt_speed', [(.2, .2), (.2, .05), (.05, .2), (.3, .1)])
def test_a_return_toward_straight_ahead_stays_clear_whatever_each_joint_speed(pan_speed, tilt_speed):
    # The lifecycle's Return moves each joint toward its zero at its own speed: the floor never eases off outward,
    # so moving inward and upward never meets it.
    clear = floor([{'pan': .3, 'tilt': -.5}, {'pan': .7, 'tilt': -.45}, {'pan': -.6, 'tilt': -.4}])
    for start in ((.7, -.35), (-.6, -.3), (.2, -.5), (.9, -.35), (0., -.8)):
        assert clear.clear(*start)
        pan, tilt = start
        for _ in itertools.count():
            if pan == 0. and tilt == 0.:
                break
            pan = max(0., pan - pan_speed * .04) if pan > 0 else min(0., pan + pan_speed * .04)
            tilt = min(0., tilt + tilt_speed * .04)
            assert clear.clear(pan, tilt)
