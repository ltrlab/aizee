"""Braking curves in Python mirror the receiver's (rust/control_core/tests/controller.rs uses the same numbers)."""
import pytest

from robot_platform.control.approach import (APPROACH_FLOOR_RAD_S, COMMAND_HEADROOM, OVERSPEED_HEADROOM,
                                             approach_speed, command_speed, end_margin, reserve_toward)

# The Rust fixture: limits +-3, cap 2 rad/s, 2 rad/s^2, bound 0.5; reaction 40 + 20 + 10 ms.
JOINT = {'lower': -3., 'upper': 3., 'max_velocity': 2., 'max_tracking_error': .5, 'stop_acceleration': 2.}
REACTION = .07


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


def test_targets_stay_the_floors_stopping_margin_and_padding_inside_whatever_the_cap():
    margin = end_margin(JOINT, REACTION, .04)
    assert margin == pytest.approx(.5975 + .5 * .04 + 1e-6)
    assert end_margin(dict(JOINT, max_velocity=20.), REACTION, .04) == margin
    assert end_margin(dict(JOINT, gripper=True), REACTION, .04) == 0.
    # A cap under the floor is its own floor.
    assert end_margin(dict(JOINT, max_velocity=.2), REACTION, .04) == pytest.approx(.5 + .2 * .07 + .04 / 4 + .2 * .04
                                                                                     + 1e-6)


def test_commands_keep_headroom_under_the_curve_toward_an_end_and_the_cap_elsewhere():
    assert OVERSPEED_HEADROOM * COMMAND_HEADROOM == pytest.approx(1.)
    # One radian below the upper limit, moving up: the curve's speed there, with headroom.
    up = command_speed(JOINT, REACTION, 2., 1, 1.6)
    assert up == pytest.approx(COMMAND_HEADROOM * approach_speed(JOINT, REACTION, 1.))
    # Moving away from that limit (5 rad from the other): the commanded cap.
    assert command_speed(JOINT, REACTION, 2., -1, 1.6) == 1.6
