"""Braking curves: how fast a joint may move toward the end of its travel.

Mirrors rust/control_core/src/collision.rs. In open travel a joint may move at its
receiver speed cap; nearer an end, only as fast as it can still stop at it after the
receiver's limit reaction time (feedback timeout and reaction allowance: it checks
feedback every tick), braking at its stop acceleration; never below
APPROACH_FLOOR_RAD_S (or its cap). The receiver enforces the curve on feedback while
it drives the arm, so a joint uses its travel up to the floor's stopping margin
(end_margin) at any speed cap, and its hand-swept range is its travel (the receiver
lets it pass a swept end by OVERTRAVEL_RAD before it faults).

The collision box keeps its own, wider reserve (approach_speed, reserve_toward): the
tracking bound, then the reaction over the verdict's lifetime. It is clipped at the
limits, so it never takes travel away.

Commands carry headroom under what the receiver enforces: servos overshoot the
speed they follow. The receiver's cap is the commanded cap times
OVERSPEED_HEADROOM, and commands toward an end stay within COMMAND_HEADROOM of
the curve. Over the last few hundredths of a radian of the usable range they slow
to rest (arrival_factor) rather than stopping at its end from the curve's floor.
"""
import math

APPROACH_FLOOR_RAD_S = .5  # collision::APPROACH_FLOOR_RAD_S
OVERSPEED_HEADROOM = 1.25  # The receiver's speed cap over the commanded one.
COMMAND_HEADROOM = 1 / OVERSPEED_HEADROOM  # Commands toward an end: this share of the curve.
OVERTRAVEL_RAD = .02  # A joint may pass its hand-swept end by this much before the receiver faults.
FRICTION_FF_FRACTION = .7  # Friction feed-forward: this share of the measured friction, never all of it.
FRICTION_SMOOTH_RAD_S = .05  # Friction feed-forward reaches its share at this commanded speed.
ARRIVAL_ZONE_RAD = .04  # Commands slow to rest over the last this much of a joint's usable range,
ARRIVAL_CREEP_RAD_S = .05  # and no slower than this, so they arrive.


def reaction_s(receiver):
    """The collision box's reaction time (s): verdict lifetime, feedback timeout and reaction allowance.
    `receiver` has feedback_timeout_ms and collision.max_verdict_ms / reaction_ms (a derived receiver
    config, or a teleop profile's receiver block)."""
    policy = receiver['collision']
    return (policy['max_verdict_ms'] + receiver['feedback_timeout_ms'] + policy['reaction_ms']) / 1000


def limit_reaction_s(receiver):
    """The receiver's reaction time at a joint limit (s): the feedback timeout and reaction allowance."""
    return (receiver['feedback_timeout_ms'] + receiver['collision']['reaction_ms']) / 1000


def approach_speed(joint, reaction, distance):
    """The collision box's speed toward a limit `distance` away (joint: a receiver joint, whose
    max_velocity is the receiver's cap), given the box reaction time: slow enough to stop with the
    tracking bound in hand."""
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
    """The stopping reserve a collision box keeps toward a limit `distance` from the joint's extreme position."""
    if joint.get('gripper'):
        return 0.
    v = approach_speed(joint, reaction, distance)
    return joint['max_tracking_error'] + v * reaction + v * v / (2 * joint['stop_acceleration'])


def limit_speed(joint, reaction, distance):
    """The fastest `joint` may move toward a limit `distance` away and still stop at it, given the limit
    reaction time (collision::limit_speed)."""
    cap = joint['max_velocity']
    if joint.get('gripper'):
        return cap
    floor = min(APPROACH_FLOOR_RAD_S, cap)
    if not distance > 0:
        return floor
    a = joint['stop_acceleration']
    v = a * (math.sqrt(reaction * reaction + 2 * distance / a) - reaction)
    return min(cap, max(floor, v))


def limit_reserve(joint, reaction, distance):
    """How far a joint moving toward a limit `distance` away travels before it stops (collision::limit_reserve)."""
    if joint.get('gripper'):
        return 0.
    v = limit_speed(joint, reaction, distance)
    return v * reaction + v * v / (2 * joint['stop_acceleration'])


def end_margin(joint, reaction):
    """How far inside a limit a joint's targets stay: the stopping margin of its approach floor at the
    limit reaction time, plus a hair so a pose exactly on the edge still certifies. Nothing for a gripper."""
    if joint.get('gripper'):
        return 0.
    floor = min(APPROACH_FLOOR_RAD_S, joint['max_velocity'])
    return floor * reaction + floor * floor / (2 * joint['stop_acceleration']) + 1e-6


def command_speed(joint, reaction, position, direction, cap):
    """The fastest a command at `position` may move in `direction` (+1 or -1): `cap` (the commanded
    cap), and toward an end at most COMMAND_HEADROOM of the receiver's limit curve there, at the limit
    reaction time."""
    if joint.get('gripper'):
        return cap
    distance = joint['upper'] - position if direction > 0 else position - joint['lower']
    return min(cap, COMMAND_HEADROOM * limit_speed(joint, reaction, distance))


def arrival_factor(joint, reaction, position, direction):
    """The share of its speed a command at `position` keeps moving in `direction` toward the end of the joint's
    usable range: all of it, until the last ARRIVAL_ZONE_RAD; there, as the square root of the room left, so it
    slows to rest at the end rather than stopping there from its approach speed. With velocity fed forward, a
    joint lagging its command catches up at up to kp / kd times the lag on top of the command's speed: right J2
    reached its end 0.01 rad behind a command at the curve's floor (0.4 rad/s), ran at 0.6, over its limit
    curve, and tripped (2026-09-29)."""
    if joint.get('gripper'):
        return 1.
    distance = joint['upper'] - position if direction > 0 else position - joint['lower']
    room = distance - end_margin(joint, reaction)
    return 1. if room >= ARRIVAL_ZONE_RAD else math.sqrt(max(0., room) / ARRIVAL_ZONE_RAD)


def command_step(joint, reaction, position, direction, cap, period):
    """How far a command may move in `direction` from `position` in one `period` (s): the receiver ramps it
    at one speed all the way, so that speed must be within command_speed (and its arrival_factor) where the step
    ends, not only where it starts (the curve falls toward an end). Taken where it started, right J1 ran at
    1.5 rad/s into the part of its curve that allows 1.55, and its speed noise tripped the overspeed check (in
    rehearsal)."""
    def speed_at(q):
        speed = command_speed(joint, reaction, q, direction, cap)
        return min(speed, max(ARRIVAL_CREEP_RAD_S, speed * arrival_factor(joint, reaction, q, direction)))
    speed = speed_at(position)
    return period * min(speed, speed_at(position + direction * speed * period))


def lead_toward(joint, reaction, position, direction, lead, cap, kp, kd):
    """How far a command may lead a joint measured at `position` in `direction`, at stiffness `kp` and
    damping `kd`: the full `lead` in open travel; toward an end, no more than the pull its damping balances
    at the speed commands may approach the end there (allowed x kd / kp), nor less than the lead's share
    in proportion to that speed against the commanded `cap` (so the limit is continuous where it begins).

    A lagging joint is pulled by its command's lead, not by the plan's speed: near an end a full lead drove
    right J6 in faster than the curve allows, and a joint stuck on static friction breaks free with all the
    pull its lead has built up (J6 lunged to 1.1 rad/s from a 0.06 rad lead at kp 24, 2026-09-27). A stuck
    joint reads as slow as one lifting against gravity, so the limit is by position, not speed. A softly
    held joint keeps its full lead (at kp 8, J6 could not otherwise push past friction near its ends); a
    heavy joint may still lift short of an end until gravity feed-forward."""
    if joint.get('gripper') or cap <= 0:
        return lead
    allowed = command_speed(joint, reaction, position, direction, cap)
    if allowed >= cap:
        return lead
    balanced = allowed * kd / kp if kp > 0 else lead
    return min(lead, max(lead * allowed / cap, balanced))


def friction_feedforward(friction_nm, velocity):
    """Friction feed-forward for a commanded `velocity`: FRICTION_FF_FRACTION of the joint's measured
    friction, in the direction of motion, reaching it at FRICTION_SMOOTH_RAD_S."""
    if not friction_nm or not velocity:
        return 0.
    return FRICTION_FF_FRACTION * friction_nm * max(-1., min(1., velocity / FRICTION_SMOOTH_RAD_S))
