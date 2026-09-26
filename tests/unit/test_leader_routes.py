"""Range-to-range leader routes: a leader channel's full swept range spans its robot joint's."""
import math

import pytest

from robot_platform.teleop.absolute_mapping import AbsoluteLeaderMapper
from robot_platform.teleop.leader_routes import build_routes, unwrap_samples, wrap
from robot_platform.teleop.openrb import LeaderSample

# Motor space: shoulder cad = -motor + 0.1; the gripper opens as its motor angle grows.
JOINT_MAP = {'shoulder': {'actuator': 'arm_j1', 'scale': -1., 'offset': .1},
             'slide': {'actuator': 'gripper', 'scale': .01, 'offset': 0.}}
# The other arm's gripper slide is mirrored in CAD, but its motor still opens as it grows.
MIRRORED = {**JOINT_MAP, 'slide': {'actuator': 'gripper', 'scale': -.01, 'offset': 0.}}
OPENS = {'gripper': 1}
USABLE = {'arm_j1': (-.6, .3), 'gripper': (.4, 1.6)}
JOINTS = [{'name': 'arm_j1', 'lower': -1., 'upper': 1.}, {'name': 'gripper', 'lower': 0., 'upper': 2.}]
# Channel 1 spans 3.0 +/- 1.0 rad; the trigger (channel 7) spans 6.1 +/- 0.3 rad, across the encoder seam.
RANGES = {'left': [(3., 2.)] + [None] * 5 + [(6.1, .6)]}
CENTERS = {'left': [3.] + [None] * 5 + [6.1]}


def settings(sign=1, grip=1):
    return {'arm_j1': {'leader': 'left', 'channel': 1, 'sign': sign},
            'gripper': {'leader': 'left', 'channel': 7, 'sign': grip}}


def follow(routes, *, arm=3., trigger=6.1):
    # The encoder reports within one turn; the trigger's high end reads just past 0.
    positions = (arm, 1., 1., 1., 1., 1., trigger % (2 * math.pi))
    sample = {'left': LeaderSample(positions, (0,) * 7, 1, 10., 10., (0, 0, 1, 1), 1, 2)}
    target = AbsoluteLeaderMapper(JOINTS, routes).map(unwrap_samples(sample, CENTERS), now=10.001)
    return {row['name']: row['position'] for row in target}


def test_the_leaders_full_range_spans_the_robots_usable_range():
    routes = build_routes(settings(), RANGES, JOINT_MAP, USABLE)
    # Sign +1: leader up moves the CAD joint up, which is motor down (scale -1).
    assert follow(routes, arm=2.)['arm_j1'] == pytest.approx(.3)
    assert follow(routes, arm=3.)['arm_j1'] == pytest.approx(-.15)
    assert follow(routes, arm=4.)['arm_j1'] == pytest.approx(-.6)
    assert follow(routes, arm=4.5)['arm_j1'] == pytest.approx(-.6)  # Past the saved end: clamped.
    cad = [-follow(routes, arm=q)['arm_j1'] + .1 for q in (2., 3., 4.)]
    assert cad == sorted(cad)
    flipped = build_routes(settings(sign=-1), RANGES, JOINT_MAP, USABLE)
    assert follow(flipped, arm=2.)['arm_j1'] == pytest.approx(-.6)
    assert follow(flipped, arm=4.)['arm_j1'] == pytest.approx(.3)


def test_a_trigger_range_across_the_encoder_seam_spans_the_gripper():
    # Sign +1: the trigger angle increasing opens the gripper, whatever the CAD slide axis.
    for joint_map in (JOINT_MAP, MIRRORED):
        routes = build_routes(settings(), RANGES, joint_map, USABLE, OPENS)
        assert follow(routes, trigger=5.8)['gripper'] == pytest.approx(.4)
        assert follow(routes, trigger=6.1)['gripper'] == pytest.approx(1.)
        assert follow(routes, trigger=6.4)['gripper'] == pytest.approx(1.6)
        flipped = build_routes(settings(grip=-1), RANGES, joint_map, USABLE, OPENS)
        assert follow(flipped, trigger=5.8)['gripper'] == pytest.approx(1.6)
        assert follow(flipped, trigger=6.4)['gripper'] == pytest.approx(.4)


def test_routes_need_a_sign_a_channel_and_a_saved_range():
    with pytest.raises(ValueError, match='No leader route for gripper'):
        build_routes({'arm_j1': settings()['arm_j1']}, RANGES, JOINT_MAP, USABLE)
    with pytest.raises(ValueError, match='unknown leader or channel'):
        build_routes({**settings(), 'arm_j1': {'leader': 'left', 'channel': 8, 'sign': 1}}, RANGES, JOINT_MAP, USABLE)
    with pytest.raises(ValueError, match='sign must be'):
        build_routes(settings(sign=0), RANGES, JOINT_MAP, USABLE)
    unswept = {'left': RANGES['left'][:6] + [None]}
    with pytest.raises(ValueError, match='no saved range for the left leader channel 7'):
        build_routes(settings(), unswept, JOINT_MAP, USABLE)


def test_readings_unwrap_around_their_range_center():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    raw = LeaderSample((.1, 3. + 4 * math.pi, 1., 1., 1., 1., 1.), (0,) * 7, 1, 10., 10., (0, 0, 1, 1), 1, 2)
    centers = {'left': [6.2, 3.] + [None] * 5}
    unwrapped = unwrap_samples({'left': raw}, centers)['left'].positions_rad
    assert unwrapped[0] == pytest.approx(.1 + 2 * math.pi)  # Just past the seam.
    assert unwrapped[1] == pytest.approx(3.)  # Two extra turns from the servo's turn count.
    assert unwrapped[2:] == raw.positions_rad[2:]  # Channels without a range stay raw.


def test_an_arm_route_scales_straight_from_the_leader_zero_and_covers_the_robot_range():
    # Leader range 2..4, zero at 2.5 (0.5 below, 1.5 above); robot usable -0.6..0.3, reference 0
    # (0.6 below, 0.3 above). One scale for both directions, the larger side ratio: 0.6 / 0.5 = 1.2.
    anchors = {'arm_j1': (2.5, 0.), 'gripper': (6.1, .4)}
    routes = build_routes(settings(sign=-1), RANGES, JOINT_MAP, USABLE, OPENS, anchors)
    assert routes[0]['scale'] == pytest.approx(1.2) and routes[0]['gain'] == pytest.approx(1.2)
    assert follow(routes, arm=2.5)['arm_j1'] == pytest.approx(0.)  # The zero maps to the reference.
    assert follow(routes, arm=2.25)['arm_j1'] == pytest.approx(-.3)
    assert follow(routes, arm=2.)['arm_j1'] == pytest.approx(-.6)  # This end at the leader's end.
    assert follow(routes, arm=2.75)['arm_j1'] == pytest.approx(.3)  # This end early; the leader goes on.
    assert follow(routes, arm=4.)['arm_j1'] == pytest.approx(.3)
    assert follow(routes, arm=1.8)['arm_j1'] == pytest.approx(-.6)  # Past the recorded range: held.
    assert routes[0]['reach'] == pytest.approx(1.) and routes[0]['overtravel'] == pytest.approx(1.25)
    flipped = build_routes(settings(sign=1), RANGES, JOINT_MAP, USABLE, OPENS, anchors)
    assert flipped[0]['scale'] == pytest.approx(.6) and flipped[0]['reach'] == pytest.approx(1.)
    assert follow(flipped, arm=2.)['arm_j1'] == pytest.approx(.3)
    assert follow(flipped, arm=4.)['arm_j1'] == pytest.approx(-.6)
    # A zero two thirds along the leader range matches the reference's place in the robot range.
    matched = build_routes(settings(sign=-1), RANGES, JOINT_MAP, USABLE, OPENS, {'arm_j1': (2 + 4 / 3, 0.)})
    assert matched[0]['scale'] == pytest.approx(.45) and matched[0]['overtravel'] == pytest.approx(0.)
    assert follow(matched, arm=2.)['arm_j1'] == pytest.approx(-.6)
    assert follow(matched, arm=4.)['arm_j1'] == pytest.approx(.3)
    # Grippers map end to end; a zero outside a range is clamped onto its end.
    assert 'raw_zero' not in routes[1] and routes[1]['scale'] == pytest.approx(2.) and routes[1]['reach'] == 1.
    assert follow(routes, trigger=6.1)['gripper'] == pytest.approx(1.)
    outside = build_routes(settings(), RANGES, JOINT_MAP, USABLE, OPENS, {'arm_j1': (1.5, .9)})
    assert (outside[0]['raw_zero'], outside[0]['follower_zero']) == (2., .3)
