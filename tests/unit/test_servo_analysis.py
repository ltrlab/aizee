"""Servo sweep analysis on records from a simulated joint (inertia, gravity, friction, position-only PD)."""

import math

import numpy as np
import pytest

from robot_platform.control.servo_analysis import (
    best_damping,
    braking,
    grid_check,
    inertia_fit,
    ramp_lag,
    recommend,
    speed_noise,
    step_response,
    torque_map,
)


def simulate(
    command, *, kp, kd, duration, q0=0.0, inertia=0.02, gravity=lambda q: 2.0 * math.sin(q), friction=0.3, dt=0.0025
):
    """Rows [t_ms, position, velocity, effort, command] for one joint, like the receiver's record."""
    rows, q, v, t = [], q0, 0.0, 0.0
    while t <= duration + 1e-9:
        c = command(t)
        torque = kp * (c - q) - kd * v
        net = torque - gravity(q)
        if abs(v) < 1e-4 and abs(net) <= friction:
            v = 0.0  # Stuck: static friction holds.
        else:
            v += (net - friction * math.copysign(1, v if abs(v) >= 1e-4 else net)) / inertia * dt
            q += v * dt
        rows.append([t * 1000, q, v, torque, c])
        t += dt
    return np.array(rows)


def ramp(start, end, speed, *, rest=0.3):
    """A commanded ramp as the sweep sends it: a 25 Hz staircase, then a rest at the end."""
    duration = abs(end - start) / speed

    def command(t):
        step = math.floor(t / 0.04) * 0.04
        return end if step >= duration else start + (end - start) * step / duration

    return command, duration + rest


def test_slow_ramps_both_ways_recover_gravity_and_friction():
    segments = []
    for start, end, direction in ((-0.5, 0.5, 1), (0.5, -0.5, -1)):
        command, duration = ramp(start, end, 0.1)
        segments.append(
            {
                'rows': simulate(command, kp=150.0, kd=10.0, duration=duration, q0=start),
                'direction': direction,
                'speed': 0.1,
            }
        )
    result = torque_map(segments, 0)
    assert result['friction'] == pytest.approx(0.3, abs=0.05)
    middle = [b for b in result['bins'] if abs(b['position']) < 0.4]
    assert middle and all(b['gravity'] == pytest.approx(2.0 * math.sin(b['position']), abs=0.1) for b in middle)
    assert result['gravity_max'] == pytest.approx(2.0 * math.sin(0.4), abs=0.2)


def test_step_responses_tell_underdamped_from_well_damped():
    def step(t):
        return 0.03 if t >= 0.1 else 0.0

    ringing = step_response(
        simulate(step, kp=150.0, kd=0.5, duration=1.0, gravity=lambda q: 0.0, friction=0.02), 0, target=0.03, start=0.0
    )
    damped = step_response(
        simulate(step, kp=150.0, kd=3.5, duration=1.0, gravity=lambda q: 0.0, friction=0.02), 0, target=0.03, start=0.0
    )
    assert ringing['overshoot'] > 0.3 and ringing['reversals'] >= 2
    assert damped['overshoot'] < 0.1 and damped['settle_s'] is not None and damped['settle_s'] < 0.3
    assert damped['steady_error'] < 0.002


def test_ramp_lag_grows_with_speed_and_braking_reports_lag_and_overshoot():
    lags = []
    for speed in (0.2, 0.8):
        command, duration = ramp(0.0, 0.6, speed)
        lags.append(ramp_lag(simulate(command, kp=150.0, kd=10.0, duration=duration), 0, speed=speed))
    assert lags[0]['speed'] == pytest.approx(0.2, rel=0.1) and lags[1]['speed'] == pytest.approx(0.8, rel=0.1)
    assert 0 < lags[0]['mean_lag'] < lags[1]['mean_lag']
    command, duration = ramp(0.0, 0.3, 0.8)
    stop = braking(simulate(command, kp=150.0, kd=10.0, duration=duration), 0, stop_at=0.3)
    assert stop['max_lag'] > 0 and stop['overshoot'] < 0.01


def test_ramp_lag_takes_the_cruise_at_the_commanded_speed_through_command_jitter():
    # The receiver ramps each 25 Hz command over 40 ms with its velocity fed forward. One command arrives a
    # 12 ms loop cycle late: the ramp pauses, then the next, on time, jumps ahead. That 40 ms ran at 1.47 rad/s
    # of a 1.2 rad/s cruise and was taken for the cruise, and a line fitted to it and extrapolated into the
    # braking end read lag that was never there (right J2, rehearsed and live, 2026-09-29).
    speed, acceleration, period, dt = 1.2, 8.0, 0.04, 0.0025
    ramp_s = speed / acceleration
    cruise_s = 0.6

    def profile(t):  # Accelerate, cruise, brake: (position, velocity).
        total = 2 * ramp_s + cruise_s
        t = min(max(t, 0.0), total)
        if t < ramp_s:
            return 0.5 * acceleration * t * t, acceleration * t
        if t < ramp_s + cruise_s:
            return 0.5 * speed * ramp_s + speed * (t - ramp_s), speed
        left = total - t
        return 0.5 * speed * ramp_s * 2 + speed * cruise_s - 0.5 * acceleration * left * left, acceleration * left

    arrivals = [k * period + (0.012 if k == 10 else 0.0) for k in range(60)]
    commands = [profile(k * period + period) for k in range(60)]
    rows, q, v, t = [], 0.0, 0.0, 0.0
    while t < 60 * period - 0.05:
        k = max(i for i, a in enumerate(arrivals) if a <= t) if t >= arrivals[0] else None
        if k is None:
            reference, velocity = 0.0, 0.0
        else:
            (target, rate), elapsed = commands[k], t - arrivals[k]
            reference, velocity = (target - rate * (period - elapsed), rate) if elapsed < period else (target, 0.0)
        torque = 150.0 * (reference - q) + 5.0 * (velocity - v)
        v += torque / 0.05 * dt
        q += v * dt
        rows.append([t * 1000, q, v, torque, reference])
        t += dt
    rows = np.array(rows)
    result = ramp_lag(rows, 0, speed=speed)
    assert result['speed'] == pytest.approx(speed, rel=0.03)
    assert result['max_lag'] == pytest.approx(np.max(rows[:, 4] - rows[:, 1]), abs=1e-9)  # The jump's, and no more.


def test_speed_noise_is_measured_while_holding():
    rows = np.zeros((100, 5))
    rows[:, 2] = np.where(np.arange(100) % 10 == 0, 0.2, 0.01)
    noise = speed_noise(rows, 0)
    assert noise['max'] == pytest.approx(0.2) and 0.06 < noise['rms'] < 0.07


def steps(overshoot, settle):
    return [
        {
            'overshoot': overshoot,
            'settle_s': settle,
            'steady_error': 0.001,
            'reversals': 0,
            'peak_effort': 1.0,
            'duration_s': 0.6,
        }
    ] * 2


def test_recommendation_takes_the_stiffest_passing_stiffness():
    torque = {'gravity_max': 2.0, 'friction': 0.3}
    grid = [
        {'kp': kp, 'kd': kd, 'steps': steps(over, 0.15), 'ramp': {'speed': 0.3, 'max_lag': 0.01}}
        for kp in (30.0, 60.0, 90.0, 150.0)
        for kd, over in ((2.0, 0.25), (6.0, 0.05))
    ]
    grid.append({'kp': 220.0, 'kd': 9.0, 'steps': steps(0.3, 0.15), 'ramp': {'speed': 0.3, 'max_lag': 0.01}})
    holds = {30.0: 0.08, 60.0: 0.03, 90.0: 0.025, 150.0: 0.015}
    speeds = [{'speed': s, 'mean_lag': s / 20, 'max_lag': s / 15, 'peak_effort': 1.0} for s in (0.2, 0.4, 0.6, 0.8)]
    decelerations = [{'deceleration': a, 'max_lag': a / 200, 'overshoot': a / 2000} for a in (2.0, 4.0, 8.0, 16.0)]
    result = recommend(
        'right_arm_j2',
        torque=torque,
        grid=grid,
        holds=holds,
        speeds=speeds,
        decelerations=decelerations,
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 220.0, 'kd': 15.0},
    )
    # 2.3 Nm within 0.04 rad needs kp >= 57.5; 220 overshoots 30%, and 150 is the stiffest that passes: it
    # follows and holds best.
    assert result['kp_floor'] == pytest.approx(57.5)
    assert (result['hold_kp'], result['hold_kd'], result['gains_met']) == (150.0, 6.0, True)
    assert any('stiffness 220 failed with every damping: overshot 30%' in note for note in result['notes'])
    # With gravity fed forward the position loop holds only its model's error, 20% of it here.
    fed = recommend(
        'right_arm_j2',
        torque=torque,
        grid=grid,
        holds=holds,
        speeds=speeds,
        decelerations=decelerations,
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 220.0, 'kd': 15.0},
        gravity_fed=True,
    )
    assert any(
        note.startswith('holding 0.70 Nm (20% of gravity (fed forward) + friction) within 0.040 rad needs kp >= 17.5')
        for note in fed['notes']
    )
    assert fed['kp_floor'] == pytest.approx(0.3 / 0.015)  # Settling against friction sets the floor now.
    assert result['max_velocity'] == 0.6  # Lagged 0.04 rad, just within half the bound; 0.8 lagged 0.053.
    assert result['stop_acceleration'] == 8.0  # 16 rad/s^2 lagged 0.08 rad.
    assert result['max_tracking_error'] == pytest.approx(0.08)  # Twice 0.04, capped at the current bound.


def test_a_setting_that_buzzes_at_rest_is_not_recommended():
    # The J2s at kd 21-29 rippled 0.14-0.21 Nm at rest and buzzed (2026-10-09): damping turns the drive's noisy
    # speed reading into torque, so the recommendation takes a setting that holds quietly.
    torque = {'gravity_max': 2.0, 'friction': 0.3}
    grid = [
        {
            'kp': 220.0,
            'kd': kd,
            'steps': steps(over, 0.15),
            'ramp': {'speed': 0.3, 'max_lag': 0.01},
            'ripple': {'std': ripple, 'p2p': 6 * ripple},
        }
        for kd, over, ripple in ((29.4, 0.0, 0.21), (21.0, 0.02, 0.14), (14.7, 0.05, 0.09))
    ]
    options = dict(
        torque=torque,
        grid=grid,
        holds={220.0: 0.01},
        speeds=[],
        decelerations=[],
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 220.0, 'kd': 21.0},
    )
    assert recommend('right_arm_j2', **options)['hold_kd'] == 29.4  # Least overshoot, without the limit.
    quiet = recommend('right_arm_j2', **options, ripple_limit=0.1)
    assert (quiet['hold_kp'], quiet['hold_kd']) == (220.0, 14.7)
    assert grid_check(grid[0], tolerance=0.03, limit=0.04, ripple_limit=0.1) == (
        'ripple',
        'torque rippled 0.21 Nm at rest (limit 0.10): it buzzes',
    )
    assert grid_check(grid[2], tolerance=0.03, limit=0.04, ripple_limit=0.1) is None


def test_recommendation_keeps_the_profile_gains_when_nothing_passes():
    result = recommend(
        'right_arm_j6',
        torque={'gravity_max': 5.0, 'friction': 1.0},
        grid=[{'kp': 8.0, 'kd': 1.0, 'steps': steps(0.5, None), 'ramp': None}],
        holds={},
        speeds=[],
        decelerations=[],
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 8.0, 'kd': 1.0},
    )
    assert result['gains_met'] is False and (result['hold_kp'], result['hold_kd']) == (8.0, 1.0)
    assert result['max_velocity'] is None and result['stop_acceleration'] is None


def test_a_speed_counts_only_when_both_directions_were_followed():
    # The first right J6 sweep (2026-09-27): 0.4 rad/s cruised at 0.35 and lagged 0.033 one way, 0.042 the other.
    torque = {'gravity_max': 0.156, 'friction': 0.104}
    grid = [{'kp': 16.0, 'kd': 0.71, 'steps': steps(0.03, 0.24), 'ramp': {'speed': 0.3, 'max_lag': 0.028}}]
    speeds = [
        dict(speed=0.1997, mean_lag=0.021, max_lag=0.028, peak_effort=0.36, target_speed=0.2, direction=1),
        dict(speed=0.1997, mean_lag=0.020, max_lag=0.0307, peak_effort=0.38, target_speed=0.2, direction=-1),
        dict(speed=0.3507, mean_lag=0.027, max_lag=0.0331, peak_effort=0.34, target_speed=0.4, direction=1),
        dict(speed=0.3573, mean_lag=0.026, max_lag=0.0417, peak_effort=0.40, target_speed=0.4, direction=-1),
    ]
    result = recommend(
        'right_arm_j6',
        torque=torque,
        grid=grid,
        holds={},
        speeds=speeds,
        decelerations=[],
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 8.0, 'kd': 1.0},
    )
    assert result['max_velocity'] == 0.2
    assert result['max_tracking_error'] == pytest.approx(2 * 0.0307)
    assert any('0.4 rad/s lagged over 0.040 rad' in note for note in result['notes'])


def test_a_speed_the_joint_did_not_reach_both_ways_does_not_count():
    # The complete J6 sweep (2026-09-27): commanded 1.0 rad/s, it lagged within the limit both ways but reached
    # only 0.75 rad/s moving up; 0.8 rad/s it reached both ways.
    torque = {'gravity_max': 0.14, 'friction': 0.106}
    grid = [{'kp': 24.0, 'kd': 0.61, 'steps': steps(0.03, 0.09), 'ramp': {'speed': 0.3, 'max_lag': 0.019}}]
    speeds = [
        dict(speed=0.713, max_lag=0.0484, target_speed=0.8, direction=1),
        dict(speed=0.795, max_lag=0.0302, target_speed=0.8, direction=-1),
        dict(speed=0.748, max_lag=0.0495, target_speed=1.0, direction=1),
        dict(speed=0.943, max_lag=0.0405, target_speed=1.0, direction=-1),
    ]
    result = recommend(
        'right_arm_j6',
        torque=torque,
        grid=grid,
        holds={},
        speeds=speeds,
        decelerations=[],
        bound=0.15,
        tolerance=0.03,
        profile_gains={'kp': 8.0, 'kd': 1.0},
    )
    assert result['max_velocity'] == 0.8
    assert any('1 rad/s lagged over 0.075 rad or fell short of it' in note for note in result['notes'])


def test_each_rejected_stiffness_says_why():
    torque = {'gravity_max': 0.156, 'friction': 0.104}
    resting = [dict(step, steady_error=0.018) for step in steps(0.0, 0.2)]
    grid = [
        {'kp': 24.0, 'kd': 0.5, 'steps': steps(0.03, 0.04), 'ramp': {'speed': 0.3, 'max_lag': 0.044}},
        {'kp': 16.8, 'kd': 0.59, 'steps': resting, 'ramp': {'speed': 0.3, 'max_lag': 0.034}},
        {'kp': 16.8, 'kd': 0.83, 'steps': steps(0.0, None), 'ramp': None},
        {'kp': 12.0, 'kd': 0.71, 'steps': steps(0.03, 0.24), 'ramp': {'speed': 0.3, 'max_lag': 0.028}},
    ]
    result = recommend(
        'right_arm_j6',
        torque=torque,
        grid=grid,
        holds={},
        speeds=[],
        decelerations=[],
        bound=0.08,
        tolerance=0.03,
        profile_gains={'kp': 8.0, 'kd': 1.0},
    )
    assert result['hold_kp'] == 12.0
    notes = ' | '.join(result['notes'])
    assert 'stiffness 24 failed with every damping: ramp lagged 0.044 rad (limit 0.040)' in notes
    assert (
        'stiffness 16.8 failed with every damping: rested 0.018 rad from the target (limit 0.015); did not come to rest'
    ) in notes
    assert grid_check(grid[3], tolerance=0.03, limit=0.04) is None


@pytest.mark.serial  # A 50 ms budget on this computer's clock: 127 ms beside the gate's other workers (2026-11-01).
def test_the_torque_map_of_a_wide_joint_takes_milliseconds():
    # It runs inside the sweep's control cycle. Right J1's characterisation (+1.35/-0.53 rad at 0.15 rad/s)
    # recorded tens of thousands of samples; summing them in Python missed the receiver's 80 ms deadline.
    import time

    rng = np.random.default_rng(3)
    segments = []
    for direction in (1, -1):
        for _ in range(8):
            n = 12000
            q = np.linspace(-0.5, 1.3, n)[::direction] + rng.normal(0, 1e-4, n)
            effort = 4 * np.sin(q) + 0.5 * direction + rng.normal(0, 0.05, n)
            rows = np.zeros((n, 9))
            rows[:, 0], rows[:, 1], rows[:, 3] = np.arange(n) * 2.5, q, effort
            segments.append({'rows': rows, 'direction': direction})
    started = time.perf_counter()
    result = torque_map(segments, 0)
    assert time.perf_counter() - started < 0.05
    assert result['friction'] == pytest.approx(0.5, abs=0.05)
    assert result['gravity_max'] == pytest.approx(4 * math.sin(1.3), abs=0.1)
    # Its gravity slice, for gravity feed-forward (control.gravity).
    assert result['gravity_fit']['A'] == pytest.approx(4.0, abs=0.05)
    assert result['gravity_fit']['C'] == pytest.approx(0.0, abs=0.05)
    # Over too short a travel, no fit.
    short = [dict(s, rows=s['rows'][np.abs(s['rows'][:, 1]) < 0.1]) for s in segments]
    assert torque_map(short, 0)['gravity_fit'] is None


def test_at_a_stiffness_the_fastest_well_damped_damping_is_chosen():
    # Right J2 at kp 220 (2026-09-30): kd 14.7 settled fastest but overshot 7%; with velocity fed forward it
    # lagged more at speed and overshot its stops. The well-damped kd 29.4 settles nearly as fast.
    def entry(kd, overshoot, settle):
        return {'kp': 220.0, 'kd': kd, 'steps': [{'overshoot': overshoot, 'settle_s': settle}] * 2}

    j2 = [entry(21.0, 0.06, 0.2875), entry(14.7, 0.073, 0.26), entry(29.4, 0.0, 0.2875)]
    assert best_damping(j2)['kd'] == 29.4
    # Right J1 at kp 150: the fastest, kd 5, overshot 0.5%, so it is still the one.
    j1 = [entry(20.0, 0.0, 0.345), entry(7.0, 0.0, 0.135), entry(5.0, 0.005, 0.092), entry(28.0, 0.0, 0.365)]
    assert best_damping(j1)['kd'] == 5.0
    # Right J3 at kp 215: friction left every step 14-20% past its target; the least overshooting is the one.
    j3 = [entry(14.0, 0.18, 0.21), entry(19.6, 0.1385, 0.2675), entry(3.5, 0.2024, 0.07)]
    assert best_damping(j3)['kd'] == 19.6
    # A step that never settled counts as slowest.
    loose = [entry(10.0, 0.08, 0.3), entry(8.0, 0.08, 0.25), entry(6.0, 0.09, None)]
    assert best_damping(loose)['kd'] == 8.0


def test_a_small_steps_overshoot_within_frictions_offset_does_not_fail_it():
    # Right J3's 18 mrad steps rested 2.5 mrad past their target (14%) at its best damping: friction, not ringing.
    def entry(overshoot, size):
        return {
            'kp': 215.0,
            'kd': 19.6,
            'failed': None,
            'ramp': None,
            'steps': [{'overshoot': overshoot, 'size': size, 'settle_s': 0.27, 'steady_error': 0.001}] * 2,
        }

    assert grid_check(entry(0.1385, 0.018), tolerance=0.03, limit=0.025) is None
    assert grid_check(entry(0.3, 0.018), tolerance=0.03, limit=0.025)[0] == 'overshoot'  # 5.4 mrad: too far.
    assert grid_check(entry(0.15, 0.05), tolerance=0.03, limit=0.025)[0] == 'overshoot'  # 7.5 mrad of a large step.


def test_the_inertia_is_fitted_from_moves_that_speed_up_and_slow_down():
    # Back and forth at up to 1 rad/s, speeding up and slowing down at 8 rad/s2, on a joint of 0.3 kg m2 with
    # gravity 2 sin q and 0.3 Nm of friction, followed by a stiff position loop (as the sweep's speed tests).
    def trapezoid(start, end, speed=1.0, acceleration=8.0):
        distance, sign = abs(end - start), math.copysign(1, end - start)
        ramp = speed / acceleration
        cruise = (distance - speed * ramp) / speed
        total = 2 * ramp + cruise

        def command(t):
            t = min(max(t, 0.0), total)
            if t < ramp:
                travelled = 0.5 * acceleration * t * t
            elif t < ramp + cruise:
                travelled = 0.5 * speed * ramp + speed * (t - ramp)
            else:
                left = total - t
                travelled = distance - 0.5 * acceleration * left * left
            return start + sign * travelled

        return command, total + 0.2

    segments = []
    for start, end in ((-0.6, 0.6), (0.6, -0.6), (-0.5, 0.7)):
        command, duration = trapezoid(start, end)
        segments.append(simulate(command, kp=400.0, kd=20.0, duration=duration, q0=start, inertia=0.3, friction=0.3))
    inertia = inertia_fit(segments, 0, gravity_fit={'A': 2.0, 'C': 0.0}, friction=0.3)
    assert inertia == pytest.approx(0.3, rel=0.15)
    assert inertia_fit(segments[:0], 0) is None  # Nothing to fit.
