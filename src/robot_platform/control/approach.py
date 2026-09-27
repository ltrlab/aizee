"""Braking curves: how fast a joint may move toward the end of its travel.

Mirrors rust/control_core/src/collision.rs. In open travel a joint may move at its
receiver speed cap; nearer an end, only as fast as it can still stop short of it
after the receiver's reaction time, braking at its stop acceleration, with its
tracking bound in hand; never below APPROACH_FLOOR_RAD_S (or its cap). The
receiver enforces the curve on feedback while it drives the arm and keeps each
collision box's stopping reserve at the curve's speed, so a joint can use its
travel up to the stopping margin of the floor (end_margin) at any speed cap.

Commands carry headroom under what the receiver enforces: servos overshoot the
speed they follow. The receiver's cap is the commanded cap times
OVERSPEED_HEADROOM, and commands toward an end stay within COMMAND_HEADROOM of
the curve.
"""
import math

APPROACH_FLOOR_RAD_S = .5  # collision::APPROACH_FLOOR_RAD_S
OVERSPEED_HEADROOM = 1.25  # The receiver's speed cap over the commanded one.
COMMAND_HEADROOM = 1 / OVERSPEED_HEADROOM  # Commands toward an end: this share of the curve.


def reaction_s(receiver):
    """The receiver's reaction time (s): verdict lifetime, feedback timeout and reaction allowance.
    `receiver` has feedback_timeout_ms and collision.max_verdict_ms / reaction_ms (a derived
    receiver config, or a teleop profile's receiver block)."""
    policy = receiver['collision']
    return (policy['max_verdict_ms'] + receiver['feedback_timeout_ms'] + policy['reaction_ms']) / 1000


def approach_speed(joint, reaction, distance):
    """The fastest `joint` (a receiver joint: max_velocity is the receiver's cap) may move toward a
    limit `distance` away, given the receiver's `reaction` time (s)."""
    cap = joint['max_velocity']
    if joint.get('gripper'):
        return cap
    floor = min(APPROACH_FLOOR_RAD_S, cap)
    room = distance - joint['max_tracking_error']
    if not room > 0:
        return floor
    a = joint['stop_acceleration']
    v = a * (math.sqrt(reaction * reaction + 2 * room / a) - reaction)
    return min(cap, max(floor, v))


def reserve_toward(joint, reaction, distance):
    """The stopping reserve a box keeps toward a limit `distance` from the joint's extreme position."""
    if joint.get('gripper'):
        return 0.
    v = approach_speed(joint, reaction, distance)
    return joint['max_tracking_error'] + v * reaction + v * v / (2 * joint['stop_acceleration'])


def end_margin(joint, reaction, verdict_s):
    """How far inside a limit a joint's targets and positions stay: the stopping reserve at the approach
    floor, plus the envelope compiler's padding for motion while a verdict (`verdict_s`) is computed, plus
    a hair so a pose exactly on the edge still certifies. Nothing for a gripper."""
    if joint.get('gripper'):
        return 0.
    floor = min(APPROACH_FLOOR_RAD_S, joint['max_velocity'])
    return (joint['max_tracking_error'] + floor * reaction + floor * floor / (2 * joint['stop_acceleration'])
            + floor * verdict_s + 1e-6)


def lead_toward(joint, reaction, position, direction, lead, cap):
    """How far a command may lead a joint measured at `position` in `direction`: the full `lead` in open
    travel; toward an end, only in proportion to the speed commands may approach it there against the
    commanded `cap`.

    A lagging joint is pulled by its command's lead, not by the plan's speed: near an end a full lead drove
    right J6 in faster than the curve allows, and a joint stuck on static friction breaks free with all the
    pull its lead has built up (J6 lunged to 1.1 rad/s from a 0.06 rad lead, 2026-09-27). A stuck joint
    reads as slow as one lifting against gravity, so the lead shrinks by position alone. Until gravity
    feed-forward, a heavy joint may therefore lift short of an end."""
    if joint.get('gripper') or cap <= 0:
        return lead
    allowed = command_speed(joint, reaction, position, direction, cap)
    return lead if allowed >= cap else lead * allowed / cap


def command_speed(joint, reaction, position, direction, cap):
    """The fastest a command at `position` may move in `direction` (+1 or -1): `cap` (the commanded
    cap), and toward an end at most COMMAND_HEADROOM of the receiver's curve there."""
    if joint.get('gripper'):
        return cap
    distance = joint['upper'] - position if direction > 0 else position - joint['lower']
    return min(cap, COMMAND_HEADROOM * approach_speed(joint, reaction, distance))
