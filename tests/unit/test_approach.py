"""Braking curves in Python mirror the receiver's (rust/control_core/tests/controller.rs uses the same numbers)."""
import pytest

from robot_platform.control.approach import (APPROACH_FLOOR_RAD_S, COMMAND_HEADROOM, FRICTION_FF_FRACTION,
                                             FRICTION_SMOOTH_RAD_S, OVERSPEED_HEADROOM, approach_speed,
                                             command_speed, command_step, end_margin, friction_feedforward,
                                             lead_toward, limit_reserve, limit_speed, reserve_toward)

# The Rust fixture: limits +-3, cap 2 rad/s, 2 rad/s^2, bound 0.5; the box reacts in 40 + 20 + 10 ms (verdict,
# feedback timeout, reaction), a joint limit in 20 + 10 ms (the receiver checks feedback every tick).
JOINT = {'lower': -3., 'upper': 3., 'max_velocity': 2., 'max_tracking_error': .5, 'stop_acceleration': 2.}
REACTION = .07
LIMIT_REACTION = .03


def test_approach_speed_is_the_cap_in_open_travel_and_slows_to_the_floor_near_a_limit():
    assert approach_speed(JOINT, REACTION, 3.) == 2.
    assert reserve_toward(JOINT, REACTION, 3.) == pytest.approx(1.64)
    speed = approach_speed(JOINT, REACTION, 1.)
    assert APPROACH_FLOOR_RAD_S < speed < 2.
    assert reserve_toward(JOINT, REACTION, 1.) == pytest.approx(1.)  # The box reaches the limit, no further.
    assert approach_speed(JOINT, REACTION, .55) == APPROACH_FLOOR_RAD_S
    assert reserve_toward(JOINT, REACTION, .55) == pytest.approx(.5975)
    gripper = dict(JOINT, gripper=True)
    assert approach_speed(gripper, REACTION, .1) == 2. and reserve_toward(gripper, REACTION, .1) == 0.


def test_limit_speed_is_the_cap_in_open_travel_and_stops_the_joint_at_the_limit():
    assert limit_speed(JOINT, LIMIT_REACTION, 3.) == 2.
    # On the curve a joint stops exactly at the limit: no tracking bound, the receiver checks feedback.
    speed = limit_speed(JOINT, LIMIT_REACTION, .5)
    assert APPROACH_FLOOR_RAD_S < speed < 2.
    assert limit_reserve(JOINT, LIMIT_REACTION, .5) == pytest.approx(.5)
    # Within the floor's stopping margin, the floor.
    assert limit_speed(JOINT, LIMIT_REACTION, .05) == APPROACH_FLOOR_RAD_S
    assert limit_reserve(JOINT, LIMIT_REACTION, .05) == pytest.approx(.0775)
    gripper = dict(JOINT, gripper=True)
    assert limit_speed(gripper, LIMIT_REACTION, .01) == 2. and limit_reserve(gripper, LIMIT_REACTION, .01) == 0.


def test_targets_stay_the_floors_stopping_margin_inside_whatever_the_cap_or_bound():
    margin = end_margin(JOINT, LIMIT_REACTION)
    assert margin == pytest.approx(.0775 + 1e-6)
    assert end_margin(dict(JOINT, max_velocity=20.), LIMIT_REACTION) == margin
    assert end_margin(dict(JOINT, max_tracking_error=.05), LIMIT_REACTION) == margin
    assert end_margin(dict(JOINT, gripper=True), LIMIT_REACTION) == 0.
    # A cap under the floor is its own floor.
    assert end_margin(dict(JOINT, max_velocity=.2), LIMIT_REACTION) == pytest.approx(.2 * .03 + .04 / 4 + 1e-6)


def test_commands_keep_headroom_under_the_limit_curve_toward_an_end_and_the_cap_elsewhere():
    assert OVERSPEED_HEADROOM * COMMAND_HEADROOM == pytest.approx(1.)
    # Half a radian below the upper limit, moving up: the curve's speed there, with headroom.
    up = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)
    assert up == pytest.approx(COMMAND_HEADROOM * limit_speed(JOINT, LIMIT_REACTION, .5)) and up < 1.6
    # Moving away from that limit (5.5 rad from the other): the commanded cap.
    assert command_speed(JOINT, LIMIT_REACTION, 2.5, -1, 1.6) == 1.6


def test_the_pull_toward_an_end_is_what_damping_balances_at_the_speed_allowed_there():
    allowed = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)  # Half a radian from the end.
    assert lead_toward(JOINT, LIMIT_REACTION, -2., 1, .1, 1.6, 24., .87) == .1  # Open travel: the full lead.
    # Stiff and lightly damped (where J6 lunged): the pull its damping balances at the allowed speed.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, .1, 1.6, 240., .87) == pytest.approx(
        max(.1 * allowed / 1.6, allowed * .87 / 240.))
    # Softly held: the full lead, or it could not push past friction near the end.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, .1, 1.6, 8., 1.) == .1
    # Never less than the lead's share of the allowed speed, so the limit starts continuously.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, .1, 1.6, 1e6, .01) == pytest.approx(.1 * allowed / 1.6)
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, -1, .1, 1.6, 240., .87) == .1  # Away from that end.
    assert lead_toward(dict(JOINT, gripper=True), LIMIT_REACTION, 2.9, 1, .1, 1.6, 240., .87) == .1


def test_friction_feed_forward_is_a_share_of_the_measured_friction_in_the_direction_of_motion():
    assert friction_feedforward(.4, 1.) == pytest.approx(FRICTION_FF_FRACTION * .4)
    assert friction_feedforward(.4, -1.) == pytest.approx(-FRICTION_FF_FRACTION * .4)
    # Smoothly through zero speed, so a joint at rest gets none and a slow one little.
    assert friction_feedforward(.4, FRICTION_SMOOTH_RAD_S / 2) == pytest.approx(FRICTION_FF_FRACTION * .2)
    assert friction_feedforward(.4, 0.) == 0. and friction_feedforward(0., 1.) == 0.


def test_a_command_step_moves_no_faster_than_the_curve_allows_where_it_ends():
    # The receiver ramps a step at one speed all the way: taken where it starts, a step toward an end runs
    # faster than the curve allows where it ends.
    start = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)
    step = command_step(JOINT, LIMIT_REACTION, 2.5, 1, 1.6, .04)
    assert step < start * .04
    assert step == pytest.approx(.04 * command_speed(JOINT, LIMIT_REACTION, 2.5 + start * .04, 1, 1.6))
    # In open travel, the cap.
    assert command_step(JOINT, LIMIT_REACTION, 0., 1, 1.6, .04) == pytest.approx(1.6 * .04)
