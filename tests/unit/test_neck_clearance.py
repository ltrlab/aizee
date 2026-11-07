"""The head's shoulder clearance (neck.clearance): the floor its recorded corners make, and steps that keep the head
above it on the way. Turned to a side and tilted all the way down, Minerva's head reaches its shoulder (2026-10-29)."""

import itertools

import pytest

from robot_platform.neck.clearance import Clearance, from_limits, section

# Tilting down reads negative here; turned 0.6 rad either way, the head clears the shoulder down to 0.4 rad.
CORNERS = [{'pan': -0.6, 'tilt': -0.4}, {'pan': 0.6, 'tilt': -0.4}]


def floor(corners=CORNERS, reach=0.8, margin=0.1):
    return Clearance(section(corners, margin_rad=margin), reach)


@pytest.mark.parametrize(
    'corners, match',
    [
        ([{'pan': 0.6, 'tilt': -0.4}], 'each side'),
        ([{'pan': 0.6, 'tilt': -0.4}, {'pan': -0.05, 'tilt': -0.4}], 'turned to a side'),
        ([{'pan': 0.6, 'tilt': -0.4}, {'pan': -0.6, 'tilt': 0.4}], 'tilted down'),
        ([{'pan': 0.6, 'tilt': -0.4}, {'pan': -0.6, 'tilt': -0.02}], 'tilted down'),
        ([{'pan': 0.6, 'tilt': float('nan')}, {'pan': -0.6, 'tilt': -0.4}], 'pan and a tilt'),
    ],
)
def test_corners_on_both_sides_tilted_down_the_same_way(corners, match):
    with pytest.raises(ValueError, match=match):
        section(corners)
    assert section(CORNERS)['down'] == -1 and section(CORNERS, margin_rad=0.0)['margin_rad'] == 0.0


def test_the_floor_runs_from_the_whole_tilt_range_ahead_to_each_corner_and_never_eases_off_outward():
    clear = floor([{'pan': 0.3, 'tilt': -0.5}, {'pan': 0.6, 'tilt': -0.7}, {'pan': -0.6, 'tilt': -0.4}])
    assert clear.depth(0.0) == pytest.approx(0.8)  # Straight ahead, its whole range down.
    assert clear.depth(0.15) == pytest.approx(0.6)  # Halfway to the first corner, less its margin.
    assert clear.depth(0.3) == pytest.approx(0.4)
    assert clear.depth(0.6) == pytest.approx(0.4) and clear.depth(1.2) == pytest.approx(0.4)  # No easing off past it.
    assert clear.depth(-0.6) == pytest.approx(0.3) and clear.depth(-1.0) == pytest.approx(0.3)
    shallow = floor([{'pan': 0.6, 'tilt': -0.15}, {'pan': -0.6, 'tilt': -0.15}], margin=0.2)
    assert shallow.depth(0.6) == 0.0  # Level always allowed: a pan alone never reaches the shoulder.
    assert clear.clamp(0.6, -0.7) == (0.6, pytest.approx(-0.4)) and clear.clamp(0.6, -0.2) == (0.6, -0.2)
    assert clear.clear(0.0, -0.8) and not clear.clear(0.6, -0.45) and clear.clear(0.6, 0.9)


def test_a_clearance_belongs_to_its_reference():
    reference = {
        'id': 'head-1',
        'motors': [
            {'name': 'head_pan', 'joint': 'neck_z', 'raw_reference_rad': 3.0},
            {'name': 'head_tilt', 'joint': 'neck_y', 'raw_reference_rad': 3.0},
        ],
    }
    limits = {
        'reference_id': 'head-1',
        'clearance': section(CORNERS),
        'joints': {'head_pan': {'min_rad': 1.5, 'max_rad': 4.5}, 'head_tilt': {'min_rad': 2.2, 'max_rad': 4.1}},
    }
    assert from_limits(limits, reference).reach == pytest.approx(0.8)  # Its tilt limit on the down side.
    assert from_limits(dict(limits, clearance=None), reference) is None
    with pytest.raises(ValueError, match='another reference'):
        from_limits(dict(limits, reference_id='head-0'), reference)


def walk(clear, start, goal, limit=(0.036, 0.036), cycles=200):
    poses = [start]
    for _ in range(cycles):
        poses.append(clear.step(poses[-1], goal, limit))
    return poses


def test_turning_while_looking_all_the_way_down_slides_up_the_floor_and_never_dips_under_it():
    clear = floor()
    poses = walk(clear, (0.0, -0.75), (0.9, -0.8))  # Looking straight down, then turning far to the side.
    assert all(clear.clear(*pose) for pose in poses)
    assert poses[-1] == (pytest.approx(0.9), pytest.approx(-0.3))  # As far down as the floor allows there.
    steps = [(abs(b[0] - a[0]), abs(b[1] - a[1])) for a, b in zip(poses, poses[1:])]
    assert max(pan for pan, _ in steps) <= 0.036 + 1e-9 and max(tilt for _, tilt in steps) <= 0.036 + 1e-9
    # Between two commands the receiver ramps straight: those chords dip under the floor far less than its margin.
    for a, b in zip(poses, poses[1:]):
        for share in (0.25, 0.5, 0.75):
            pan, tilt = a[0] + (b[0] - a[0]) * share, a[1] + (b[1] - a[1]) * share
            assert -tilt - clear.depth(pan) < 0.01


def test_a_head_below_the_floor_rises_straight_up_first():
    clear = floor()
    assert clear.step((0.7, -0.6), (0.0, 0.0), (0.036, 0.036)) == (0.7, pytest.approx(-0.564))
    poses = walk(clear, (0.7, -0.6), (0.0, -0.8), cycles=30)
    assert clear.clear(*poses[-1]) and poses[8][0] == 0.7  # Out of the corner before it turns.


@pytest.mark.parametrize('pan_speed, tilt_speed', [(0.2, 0.2), (0.2, 0.05), (0.05, 0.2), (0.3, 0.1)])
def test_a_return_toward_straight_ahead_stays_clear_whatever_each_joint_speed(pan_speed, tilt_speed):
    # The lifecycle's Return moves each joint toward its zero at its own speed: the floor never eases off outward,
    # so moving inward and upward never meets it.
    clear = floor([{'pan': 0.3, 'tilt': -0.5}, {'pan': 0.7, 'tilt': -0.45}, {'pan': -0.6, 'tilt': -0.4}])
    for start in ((0.7, -0.35), (-0.6, -0.3), (0.2, -0.5), (0.9, -0.35), (0.0, -0.8)):
        assert clear.clear(*start)
        pan, tilt = start
        for _ in itertools.count():
            if pan == 0.0 and tilt == 0.0:
                break
            pan = max(0.0, pan - pan_speed * 0.04) if pan > 0 else min(0.0, pan + pan_speed * 0.04)
            tilt = min(0.0, tilt + tilt_speed * 0.04)
            assert clear.clear(pan, tilt)


def test_a_head_rests_where_its_servos_read_inside_its_saved_limits():
    # Limp, the head droops (the user, 2026-10-30): a session starts where it rests, and its receiver checks the head
    # is still there (neck.head.read_rest, service_config's start pose).
    from robot_platform.neck import head
    from robot_platform.neck.feetech import FeetechBus
    from robot_platform.neck.sim import SimulatedServo, realtime_bus
    from robot_platform.ops.release import ROOT

    profile = head.load_profile(ROOT / head.PROFILE)
    pan, tilt = SimulatedServo(1, position=2060), SimulatedServo(2, position=1650)
    bus = realtime_bus([pan, tilt])
    limits = {
        'joints': {
            'head_pan': {'min_rad': 985 * head.RAD_PER_STEP, 'max_rad': 3055 * head.RAD_PER_STEP},
            'head_tilt': {'min_rad': 1418 * head.RAD_PER_STEP, 'max_rad': 2754 * head.RAD_PER_STEP},
        }
    }
    rest = head.read_rest(profile, limits, open_bus=lambda: FeetechBus(bus, timeout_s=0.005))
    assert rest == pytest.approx({'head_pan': 2060 * head.RAD_PER_STEP, 'head_tilt': 1650 * head.RAD_PER_STEP})
    reference = head.virtual_reference(profile)
    derived = {'config': {'joints': []}}
    references = head.service_config(
        dict(profile, bus={'device': '/dev/x', 'usb_serial': 'S', 'baud': 1000000}),
        reference,
        derived,
        mode='teleop_direct',
        start=rest,
    )['reference_positions']
    assert [r['raw_position_rad'] for r in references] == pytest.approx([rest['head_pan'], rest['head_tilt']])
    tilt.move_by_hand(1300)  # Far below its swept range: not where its setup left it.
    with pytest.raises(ValueError, match='Head tilt rests at .* outside its saved limits'):
        head.read_rest(profile, limits, open_bus=lambda: FeetechBus(bus, timeout_s=0.005))
