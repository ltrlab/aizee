"""Braking curves in Python mirror the receiver's (rust/control_core/tests/controller.rs uses the same numbers)."""

import pytest

from robot_platform.control.approach import (
    APPROACH_FLOOR_RAD_S,
    ARRIVAL_CREEP_RAD_S,
    ARRIVAL_ZONE_RAD,
    COMMAND_HEADROOM,
    FRICTION_FF_FRACTION,
    FRICTION_SMOOTH_RAD_S,
    OVERSPEED_HEADROOM,
    approach_speed,
    arrival_factor,
    command_speed,
    command_step,
    end_margin,
    friction_feedforward,
    lead_toward,
    limit_reserve,
    limit_speed,
    reserve_toward,
)

# The Rust fixture: limits +-3, cap 2 rad/s, 2 rad/s^2, bound 0.5; the box reacts in 40 + 20 + 10 ms (verdict,
# feedback timeout, reaction), a joint limit in 20 + 10 ms (the receiver checks feedback every tick).
JOINT = {'lower': -3.0, 'upper': 3.0, 'max_velocity': 2.0, 'max_tracking_error': 0.5, 'stop_acceleration': 2.0}
REACTION = 0.07
LIMIT_REACTION = 0.03


def test_approach_speed_is_the_cap_in_open_travel_and_slows_to_the_floor_near_a_limit():
    assert approach_speed(JOINT, REACTION, 3.0) == 2.0
    assert reserve_toward(JOINT, REACTION, 3.0) == pytest.approx(1.64)
    speed = approach_speed(JOINT, REACTION, 1.0)
    assert APPROACH_FLOOR_RAD_S < speed < 2.0
    assert reserve_toward(JOINT, REACTION, 1.0) == pytest.approx(1.0)  # The box reaches the limit, no further.
    assert approach_speed(JOINT, REACTION, 0.55) == APPROACH_FLOOR_RAD_S
    assert reserve_toward(JOINT, REACTION, 0.55) == pytest.approx(0.5975)
    gripper = dict(JOINT, gripper=True)
    assert approach_speed(gripper, REACTION, 0.1) == 2.0 and reserve_toward(gripper, REACTION, 0.1) == 0.0


def test_limit_speed_is_the_cap_in_open_travel_and_stops_the_joint_at_the_limit():
    assert limit_speed(JOINT, LIMIT_REACTION, 3.0) == 2.0
    # On the curve a joint stops exactly at the limit: no tracking bound, the receiver checks feedback.
    speed = limit_speed(JOINT, LIMIT_REACTION, 0.5)
    assert APPROACH_FLOOR_RAD_S < speed < 2.0
    assert limit_reserve(JOINT, LIMIT_REACTION, 0.5) == pytest.approx(0.5)
    # Within the floor's stopping margin, the floor.
    assert limit_speed(JOINT, LIMIT_REACTION, 0.05) == APPROACH_FLOOR_RAD_S
    assert limit_reserve(JOINT, LIMIT_REACTION, 0.05) == pytest.approx(0.0775)
    gripper = dict(JOINT, gripper=True)
    assert limit_speed(gripper, LIMIT_REACTION, 0.01) == 2.0 and limit_reserve(gripper, LIMIT_REACTION, 0.01) == 0.0


def test_targets_stay_the_floors_stopping_margin_inside_whatever_the_cap_or_bound():
    margin = end_margin(JOINT, LIMIT_REACTION)
    assert margin == pytest.approx(0.0775 + 1e-6)
    assert end_margin(dict(JOINT, max_velocity=20.0), LIMIT_REACTION) == margin
    assert end_margin(dict(JOINT, max_tracking_error=0.05), LIMIT_REACTION) == margin
    assert end_margin(dict(JOINT, gripper=True), LIMIT_REACTION) == 0.0
    # A cap under the floor is its own floor.
    assert end_margin(dict(JOINT, max_velocity=0.2), LIMIT_REACTION) == pytest.approx(0.2 * 0.03 + 0.04 / 4 + 1e-6)


def test_commands_keep_headroom_under_the_limit_curve_toward_an_end_and_the_cap_elsewhere():
    assert OVERSPEED_HEADROOM * COMMAND_HEADROOM == pytest.approx(1.0)
    # Half a radian below the upper limit, moving up: the curve's speed there, with headroom.
    up = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)
    assert up == pytest.approx(COMMAND_HEADROOM * limit_speed(JOINT, LIMIT_REACTION, 0.5)) and up < 1.6
    # Moving away from that limit (5.5 rad from the other): the commanded cap.
    assert command_speed(JOINT, LIMIT_REACTION, 2.5, -1, 1.6) == 1.6


def test_the_pull_toward_an_end_is_what_damping_balances_at_the_speed_allowed_there():
    allowed = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)  # Half a radian from the end.
    assert lead_toward(JOINT, LIMIT_REACTION, -2.0, 1, 0.1, 1.6, 24.0, 0.87) == 0.1  # Open travel: the full lead.
    # Stiff and lightly damped (where J6 lunged): the pull its damping balances at the allowed speed.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, 0.1, 1.6, 240.0, 0.87) == pytest.approx(
        max(0.1 * allowed / 1.6, allowed * 0.87 / 240.0)
    )
    # Softly held: the full lead, or it could not push past friction near the end.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, 0.1, 1.6, 8.0, 1.0) == 0.1
    # Never less than the lead's share of the allowed speed, so the limit starts continuously.
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, 1, 0.1, 1.6, 1e6, 0.01) == pytest.approx(0.1 * allowed / 1.6)
    assert lead_toward(JOINT, LIMIT_REACTION, 2.5, -1, 0.1, 1.6, 240.0, 0.87) == 0.1  # Away from that end.
    assert lead_toward(dict(JOINT, gripper=True), LIMIT_REACTION, 2.9, 1, 0.1, 1.6, 240.0, 0.87) == 0.1


def test_friction_feed_forward_is_a_share_of_the_measured_friction_in_the_direction_of_motion():
    assert friction_feedforward(0.4, 1.0) == pytest.approx(FRICTION_FF_FRACTION * 0.4)
    assert friction_feedforward(0.4, -1.0) == pytest.approx(-FRICTION_FF_FRACTION * 0.4)
    # Smoothly through zero speed, so a joint at rest gets none and a slow one little.
    assert friction_feedforward(0.4, FRICTION_SMOOTH_RAD_S / 2) == pytest.approx(FRICTION_FF_FRACTION * 0.2)
    assert friction_feedforward(0.4, 0.0) == 0.0 and friction_feedforward(0.0, 1.0) == 0.0


def test_a_command_step_moves_no_faster_than_the_curve_allows_where_it_ends():
    # The receiver ramps a step at one speed all the way: taken where it starts, a step toward an end runs
    # faster than the curve allows where it ends.
    start = command_speed(JOINT, LIMIT_REACTION, 2.5, 1, 1.6)
    step = command_step(JOINT, LIMIT_REACTION, 2.5, 1, 1.6, 0.04)
    assert step < start * 0.04
    # The largest step whose speed is within the curve where it ends.
    allowed = command_speed(JOINT, LIMIT_REACTION, 2.5 + step, 1, 1.6)
    assert allowed * 0.999 <= step / 0.04 <= allowed + 1e-9
    # In open travel, the cap.
    assert command_step(JOINT, LIMIT_REACTION, 0.0, 1, 1.6, 0.04) == pytest.approx(1.6 * 0.04)


def test_commands_slow_to_rest_over_the_last_of_the_usable_range_and_still_arrive():
    # Right J2 reached its end at the curve's floor (0.4 rad/s) 0.01 rad behind its command, caught up at 0.6
    # and tripped. Over the last ARRIVAL_ZONE_RAD, commands slow to rest at the usable end instead.
    edge = JOINT['upper'] - end_margin(JOINT, LIMIT_REACTION)
    near = edge - ARRIVAL_ZONE_RAD / 4
    assert arrival_factor(JOINT, LIMIT_REACTION, near, 1) == pytest.approx(0.5)
    curve = command_speed(JOINT, LIMIT_REACTION, near, 1, 2.0)
    assert command_step(JOINT, LIMIT_REACTION, near, 1, 2.0, 0.04) / 0.04 <= 0.5 * curve + 1e-9
    assert arrival_factor(JOINT, LIMIT_REACTION, near, -1) == 1.0  # Away from that end it does not bind.
    # Before the last stretch the braking curve alone binds, whatever the speed: the largest step whose speed
    # is within the curve where it ends.
    far = edge - ARRIVAL_ZONE_RAD - 0.2
    assert arrival_factor(JOINT, LIMIT_REACTION, far, 1) == 1.0
    step = command_step(JOINT, LIMIT_REACTION, far, 1, 2.0, 0.04)
    assert step / 0.04 <= command_speed(JOINT, LIMIT_REACTION, far + step, 1, 2.0) + 1e-9
    assert step / 0.04 > 0.99 * command_speed(JOINT, LIMIT_REACTION, far + step, 1, 2.0)
    # Just short of the last stretch, a step is not cut to the creep because a full-speed one would pass it.
    short = edge - ARRIVAL_ZONE_RAD - 0.001
    step = command_step(JOINT, LIMIT_REACTION, short, 1, 2.0, 0.04)
    assert step > 0.04 * ARRIVAL_CREEP_RAD_S * 3
    # Commands still reach the edge (never slower than the creep), and no further.
    q, steps = edge - 0.1, 0
    while q < edge - 1e-9:
        q = min(edge, q + command_step(JOINT, LIMIT_REACTION, q, 1, 2.0, 0.04))
        steps += 1
    assert steps < 100
    assert command_step(JOINT, LIMIT_REACTION, edge, 1, 2.0, 0.04) == pytest.approx(0.04 * ARRIVAL_CREEP_RAD_S)
    assert arrival_factor(dict(JOINT, gripper=True), LIMIT_REACTION, 2.99, 1) == 1.0


def test_the_ramp_effort_is_a_share_of_the_inertia_times_the_change_of_speed_within_limits():
    from robot_platform.control.approach import ACCELERATION_FF_LIMIT, INERTIA_FF_FRACTION, ramp_effort

    # 0.9 kg m2 from 0.2 to 0.4 rad/s over a 40 ms ramp: 5 rad/s2.
    assert ramp_effort(0.9, 0.4, 0.2, 0.04, 0.0, 60.0) == pytest.approx(INERTIA_FF_FRACTION * 0.9 * 5.0)
    assert ramp_effort(0.9, 0.0, 0.4, 0.04, 0.0, 60.0) == pytest.approx(-INERTIA_FF_FRACTION * 0.9 * 10.0)
    # The acceleration fed forward is limited, and so is the effort with the command's own.
    assert ramp_effort(0.9, 4.0, 0.0, 0.04, 0.0, 60.0) == pytest.approx(
        INERTIA_FF_FRACTION * 0.9 * ACCELERATION_FF_LIMIT
    )
    assert ramp_effort(0.9, 4.0, 0.0, 0.04, 50.0, 60.0) == pytest.approx(0.9 * 60.0 - 50.0)
    # No inertia, or no ramp: nothing.
    assert ramp_effort(None, 0.4, 0.0, 0.04, 0.0, 60.0) == 0.0 and ramp_effort(0.9, 0.4, 0.0, 0.0, 0.0, 60.0) == 0.0
