"""Direct velocity feed-forward: a smoothed command speed, never a step, a jump or a stale value.

Fed forward as one cycle's step, the command's speed pulsed the drives (Kd x each jump) while the leaders
repeated one sample in ten (2026-10-10, docs/development/DIRECT_TELEOP.md); manual tuning then ran position-only,
where Kd drags every motion. These pin the smoothed law that replaces both.
"""
import math

import pytest

from robot_platform.teleop.lifecycle import FEED_SAMPLES, TeleopLifecycle, direct_law

NAMES = ('left_arm_j1', 'left_arm_j2', 'left_gripper')
DT = .01


def joints():
    arm = dict(lower=-2., upper=2., hold_kp=500., hold_kd=20., max_velocity=1.5, align_speed=.5,
               shutdown_speed=.3, max_lead=.5, tolerance=.001, settled_velocity=.01, zero_position=0.,
               position_only=False, direct_velocity=True, velocity_feedforward=True, interpolation_s=0.,
               friction_nm=.5, inertia=.3, max_effort=60.)
    gripper = dict(arm, max_lead=4., direct_velocity=False, velocity_feedforward=False, gripper=True,
                   friction_nm=0., inertia=0.)
    return [dict(arm, name=NAMES[0]), dict(arm, name=NAMES[1]), dict(gripper, name=NAMES[2])]


def rows(now, positions, *, source=False):
    result = [dict(name=name, position=q, velocity=0., sampled_at=now, sequence=round(now * 10000) + 1)
              for name, q in zip(NAMES, positions)]
    return [{k: v for k, v in row.items() if k != 'velocity'} for row in result] if source else result


def engaged(pose=(0., 0., 0.), config=None):
    lifecycle = TeleopLifecycle(config or joints(), settle_time_s=0.)
    lifecycle.action('idle', rows(0., pose), now=0.)
    lifecycle.action('enable', rows(0., pose), now=0.)
    lifecycle.action('engage', rows(.01, pose), target=rows(.01, pose, source=True), now=.01)
    now = .01
    for _ in range(6):
        now += DT
        lifecycle.step(rows(now, pose), target=rows(now, pose, source=True), now=now)
    assert lifecycle.state == 'TELEOP'
    return lifecycle, now


def drive(lifecycle, now, goals):
    """Step through leader goals (one per cycle), the joints following their commands exactly."""
    sent = []
    for goal in goals:
        now += DT
        pose = lifecycle.target
        sent.append(lifecycle.step(rows(now, pose), target=rows(now, goal, source=True), now=now))
    return now, sent


def speeds(sent, joint=0):
    return [proposal['joints'][joint]['velocity'] for proposal in sent]


def test_a_steady_move_feeds_its_speed_forward_building_up_without_a_step():
    lifecycle, now = engaged()
    _, sent = drive(lifecycle, now, [(.3 * DT * k, -.2 * DT * k, .5) for k in range(1, 40)])
    v1, v2 = speeds(sent, 0), speeds(sent, 1)
    assert v1[-1] == pytest.approx(.3) and v2[-1] == pytest.approx(-.2)
    # The first move feeds part of its speed forward and builds to all of it over FEED_SAMPLES cycles.
    assert 0 < v1[0] < .3 and all(b >= a - 1e-12 for a, b in zip(v1, v1[1:]))
    assert v1[FEED_SAMPLES - 1] == pytest.approx(.3)
    # Only gravity is fed forward as effort (none here): no friction pulses, no acceleration ramps.
    assert all(c['effort'] == 0. for p in sent for c in p['joints'])
    assert all(all(e == 0. for e in p['ramp_efforts']) for p in sent)
    assert all(p['joints'][2]['velocity'] == 0. for p in sent)  # A gripper's squeeze is its lead; none.


def test_a_quantised_leader_count_feeds_a_bounded_ripple_not_a_pulse():
    lifecycle, now = engaged()
    count = .0015  # One leader count; it steps every third cycle (0.05 rad/s).
    _, sent = drive(lifecycle, now, [(count * (k // 3), 0., 0.) for k in range(1, 61)])
    v = speeds(sent)[FEED_SAMPLES:]
    assert sum(v) / len(v) == pytest.approx(.05, abs=.005)
    # One cycle's step would swing 0 -> 0.15 rad/s each count; the slope stays within 0.02-0.08.
    assert min(v) >= .02 and max(v) <= .08


def test_a_reversal_passes_through_zero_without_a_jump():
    lifecycle, now = engaged()
    # From rest, 0.2 rad either way of 0.2 at 0.5 Hz: 0.63 rad/s at its fastest, reversing twice.
    goals = [(.2 * (1 - math.cos(2 * math.pi * .5 * DT * k)), 0., 0.) for k in range(1, 300)]
    _, sent = drive(lifecycle, now, goals)
    v = speeds(sent)
    assert max(v) > .5 and min(v) < -.5
    # One cycle never changes the speed fed forward by much more than the motion's own acceleration allows.
    assert max(abs(b - a) for a, b in zip(v, v[1:])) <= .2 * (2 * math.pi * .5) ** 2 * DT * 1.6


def test_a_hold_or_a_lost_leader_feeds_nothing_and_resuming_starts_afresh():
    lifecycle, now = engaged()
    now, sent = drive(lifecycle, now, [(.3 * DT * k, 0., 0.) for k in range(1, 20)])
    assert speeds(sent)[-1] == pytest.approx(.3)
    pose = lifecycle.target
    now += DT
    held = lifecycle.action('disengage', rows(now, pose), now=now)
    assert all(c['velocity'] == 0. for c in held['joints'])
    now += DT
    assert all(c['velocity'] == 0. for c in lifecycle.step(rows(now, pose), now=now)['joints'])
    # Engage again from the hold: the speed fed forward builds from rest, not from before the hold.
    lifecycle.action('engage', rows(now + DT, pose), target=rows(now + DT, pose, source=True), now=now + DT)
    now += DT
    start = lifecycle.target[0]
    _, sent = drive(lifecycle, now, [(start + .3 * DT * k, 0., 0.) for k in range(1, 4)])
    assert speeds(sent)[0] < .3


def test_a_missing_leader_sample_feeds_nothing_that_cycle():
    lifecycle = TeleopLifecycle(joints(), settle_time_s=0., allow_repeated_source=True)
    pose = (0., 0., 0.)
    lifecycle.action('idle', rows(0., pose), now=0.)
    lifecycle.action('enable', rows(0., pose), now=0.)
    lifecycle.action('engage', rows(.01, pose), target=rows(.01, pose, source=True), now=.01)
    now = .01
    for _ in range(6):
        now += DT
        lifecycle.step(rows(now, pose), target=rows(now, pose, source=True), now=now)
    now, sent = drive(lifecycle, now, [(.3 * DT * k, 0., 0.) for k in range(1, 10)])
    now += DT
    missing = lifecycle.step(rows(now, lifecycle.target), now=now)  # No leader sample this cycle.
    assert missing['joints'][0]['velocity'] == 0.


def test_the_speed_fed_forward_never_exceeds_the_joints_cap():
    lifecycle, now = engaged()
    _, sent = drive(lifecycle, now, [(1.9, 0., 0.)] * 30)  # A leap: the command slews at its cap.
    assert max(abs(v) for v in speeds(sent)) <= 1.5 + 1e-12


def test_velocity_feedforward_is_only_a_direct_unramped_arm_law():
    for bad in (dict(position_only=True), dict(gripper=True), dict(interpolation_s=.02), dict(direct_velocity=False),
                dict(velocity_feedforward='yes')):
        config = joints()
        config[0].update(bad)
        with pytest.raises(ValueError, match='Velocity feed-forward|position_only'):
            TeleopLifecycle(config)
    assert direct_law(joints())
    mixed = joints()
    mixed[1]['velocity_feedforward'] = False
    assert not direct_law(mixed)
    position_only = [dict(j, position_only=True, velocity_feedforward=False) for j in joints()]
    assert direct_law(position_only)


def test_toward_an_end_inside_its_braking_zone_the_joint_follows_position_only():
    """The near-end lead (approach.lead_toward) bounds the speed a lead can pull by Kd's drag; fed forward, left J6
    caught up with its command at 1.65 rad/s where the receiver allowed 1.52 (live-20261015T021209Z)."""
    config = joints()
    curve = dict(lower=-2., upper=2., max_velocity=1.875, max_tracking_error=.4, stop_acceleration=16., reaction_s=.04)
    config[0]['approach'] = curve
    lifecycle, now = engaged((1.5, 0., 0.), config)
    # Up toward the end at 0.3 rad/s: fed forward in open travel, none from 0.1 s before the last ~0.18 rad.
    now, sent = drive(lifecycle, now, [(1.5 + .003 * k, 0., 0.) for k in range(1, 121)])
    fed = [(p['positions'][0], p['joints'][0]['velocity']) for p in sent]
    assert all(v > .25 for q, v in fed[FEED_SAMPLES:] if q < 1.76)
    assert all(v == 0. for q, v in fed if q > 1.80)  # 0.1 s at 0.3 rad/s before the curve.
    assert any(q > 1.85 for q, _ in fed)
    # Back away from the end, still inside the zone: fed forward again (only motion toward the end is damped).
    _, back = drive(lifecycle, now, [(1.86 - .003 * k, 0., 0.) for k in range(1, 15)])
    assert all(p['joints'][0]['velocity'] < -.25 for p in back[FEED_SAMPLES + 1:])
