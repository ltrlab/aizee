"""Servo sweep analysis on records from a simulated joint (inertia, gravity, friction, position-only PD)."""
import math

import numpy as np
import pytest

from robot_platform.control.servo_analysis import (braking, grid_check, ramp_lag, recommend, speed_noise,
                                                   step_response, torque_map)


def simulate(command, *, kp, kd, duration, q0=0., inertia=.02, gravity=lambda q: 2. * math.sin(q),
             friction=.3, dt=.0025):
    """Rows [t_ms, position, velocity, effort, command] for one joint, like the receiver's record."""
    rows, q, v, t = [], q0, 0., 0.
    while t <= duration + 1e-9:
        c = command(t)
        torque = kp * (c - q) - kd * v
        net = torque - gravity(q)
        if abs(v) < 1e-4 and abs(net) <= friction:
            v = 0.  # Stuck: static friction holds.
        else:
            v += (net - friction * math.copysign(1, v if abs(v) >= 1e-4 else net)) / inertia * dt
            q += v * dt
        rows.append([t * 1000, q, v, torque, c])
        t += dt
    return np.array(rows)


def ramp(start, end, speed, *, rest=.3):
    """A commanded ramp as the sweep sends it: a 25 Hz staircase, then a rest at the end."""
    duration = abs(end - start) / speed

    def command(t):
        step = math.floor(t / .04) * .04
        return end if step >= duration else start + (end - start) * step / duration
    return command, duration + rest


def test_slow_ramps_both_ways_recover_gravity_and_friction():
    segments = []
    for start, end, direction in ((-.5, .5, 1), (.5, -.5, -1)):
        command, duration = ramp(start, end, .1)
        segments.append({'rows': simulate(command, kp=150., kd=10., duration=duration, q0=start),
                         'direction': direction, 'speed': .1})
    result = torque_map(segments, 0)
    assert result['friction'] == pytest.approx(.3, abs=.05)
    middle = [b for b in result['bins'] if abs(b['position']) < .4]
    assert middle and all(b['gravity'] == pytest.approx(2. * math.sin(b['position']), abs=.1) for b in middle)
    assert result['gravity_max'] == pytest.approx(2. * math.sin(.4), abs=.2)


def test_step_responses_tell_underdamped_from_well_damped():
    def step(t):
        return .03 if t >= .1 else 0.
    ringing = step_response(simulate(step, kp=150., kd=.5, duration=1., gravity=lambda q: 0., friction=.02),
                            0, target=.03, start=0.)
    damped = step_response(simulate(step, kp=150., kd=3.5, duration=1., gravity=lambda q: 0., friction=.02),
                           0, target=.03, start=0.)
    assert ringing['overshoot'] > .3 and ringing['reversals'] >= 2
    assert damped['overshoot'] < .1 and damped['settle_s'] is not None and damped['settle_s'] < .3
    assert damped['steady_error'] < .002


def test_ramp_lag_grows_with_speed_and_braking_reports_lag_and_overshoot():
    lags = []
    for speed in (.2, .8):
        command, duration = ramp(0., .6, speed)
        lags.append(ramp_lag(simulate(command, kp=150., kd=10., duration=duration), 0, speed=speed))
    assert lags[0]['speed'] == pytest.approx(.2, rel=.1) and lags[1]['speed'] == pytest.approx(.8, rel=.1)
    assert 0 < lags[0]['mean_lag'] < lags[1]['mean_lag']
    command, duration = ramp(0., .3, .8)
    stop = braking(simulate(command, kp=150., kd=10., duration=duration), 0, stop_at=.3)
    assert stop['max_lag'] > 0 and stop['overshoot'] < .01


def test_speed_noise_is_measured_while_holding():
    rows = np.zeros((100, 5))
    rows[:, 2] = np.where(np.arange(100) % 10 == 0, .2, .01)
    noise = speed_noise(rows, 0)
    assert noise['max'] == pytest.approx(.2) and .06 < noise['rms'] < .07


def steps(overshoot, settle):
    return [{'overshoot': overshoot, 'settle_s': settle, 'steady_error': .001, 'reversals': 0,
             'peak_effort': 1., 'duration_s': .6}] * 2


def test_recommendation_takes_the_lowest_passing_stiffness_plus_one_step_of_margin():
    torque = {'gravity_max': 2., 'friction': .3}
    grid = [{'kp': kp, 'kd': kd, 'steps': steps(over, .15), 'ramp': {'speed': .3, 'max_lag': .01}}
            for kp in (30., 60., 90., 150.) for kd, over in ((2., .25), (6., .05))]
    holds = {30.: .08, 60.: .03, 90.: .025, 150.: .015}
    speeds = [{'speed': s, 'mean_lag': s / 20, 'max_lag': s / 15, 'peak_effort': 1.} for s in (.2, .4, .6, .8)]
    decelerations = [{'deceleration': a, 'max_lag': a / 200, 'overshoot': a / 2000} for a in (2., 4., 8., 16.)]
    result = recommend('right_arm_j2', torque=torque, grid=grid, holds=holds, speeds=speeds,
                       decelerations=decelerations, bound=.08, tolerance=.03,
                       profile_gains={'kp': 220., 'kd': 15.})
    # 2.3 Nm within 0.04 rad needs kp >= 57.5; 60 passes (its hold sagged 0.03 <= 0.04), so 90 with margin.
    assert result['kp_floor'] == pytest.approx(57.5)
    assert (result['hold_kp'], result['hold_kd'], result['gains_met']) == (90., 6., True)
    assert result['max_velocity'] == .6  # Lagged 0.04 rad, just within half the bound; 0.8 lagged 0.053.
    assert result['stop_acceleration'] == 8.  # 16 rad/s^2 lagged 0.08 rad.
    assert result['max_tracking_error'] == pytest.approx(.08)  # Twice 0.04, capped at the current bound.


def test_recommendation_keeps_the_profile_gains_when_nothing_passes():
    result = recommend('right_arm_j6', torque={'gravity_max': 5., 'friction': 1.},
                       grid=[{'kp': 8., 'kd': 1., 'steps': steps(.5, None), 'ramp': None}], holds={},
                       speeds=[], decelerations=[], bound=.08, tolerance=.03, profile_gains={'kp': 8., 'kd': 1.})
    assert result['gains_met'] is False and (result['hold_kp'], result['hold_kd']) == (8., 1.)
    assert result['max_velocity'] is None and result['stop_acceleration'] is None


def test_a_speed_counts_only_when_both_directions_were_followed():
    # The first right J6 sweep (2026-09-27): 0.4 rad/s cruised at 0.35 and lagged 0.033 one way, 0.042 the other.
    torque = {'gravity_max': .156, 'friction': .104}
    grid = [{'kp': 16., 'kd': .71, 'steps': steps(.03, .24), 'ramp': {'speed': .3, 'max_lag': .028}}]
    speeds = [dict(speed=.1997, mean_lag=.021, max_lag=.028, peak_effort=.36, target_speed=.2, direction=1),
              dict(speed=.1997, mean_lag=.020, max_lag=.0307, peak_effort=.38, target_speed=.2, direction=-1),
              dict(speed=.3507, mean_lag=.027, max_lag=.0331, peak_effort=.34, target_speed=.4, direction=1),
              dict(speed=.3573, mean_lag=.026, max_lag=.0417, peak_effort=.40, target_speed=.4, direction=-1)]
    result = recommend('right_arm_j6', torque=torque, grid=grid, holds={}, speeds=speeds, decelerations=[],
                       bound=.08, tolerance=.03, profile_gains={'kp': 8., 'kd': 1.})
    assert result['max_velocity'] == .2
    assert result['max_tracking_error'] == pytest.approx(2 * .0307)
    assert any('0.4 rad/s lagged over 0.040 rad in at least one direction' in note for note in result['notes'])


def test_each_rejected_stiffness_says_why():
    torque = {'gravity_max': .156, 'friction': .104}
    resting = [dict(step, steady_error=.018) for step in steps(0., .2)]
    grid = [{'kp': 8., 'kd': .5, 'steps': steps(.03, .04), 'ramp': {'speed': .3, 'max_lag': .044}},
            {'kp': 11.2, 'kd': .59, 'steps': resting, 'ramp': {'speed': .3, 'max_lag': .034}},
            {'kp': 11.2, 'kd': .83, 'steps': steps(0., None), 'ramp': None},
            {'kp': 16., 'kd': .71, 'steps': steps(.03, .24), 'ramp': {'speed': .3, 'max_lag': .028}}]
    result = recommend('right_arm_j6', torque=torque, grid=grid, holds={}, speeds=[], decelerations=[],
                       bound=.08, tolerance=.03, profile_gains={'kp': 8., 'kd': 1.})
    assert result['hold_kp'] == 16.
    notes = ' | '.join(result['notes'])
    assert 'stiffness 8 failed with every damping: ramp lagged 0.044 rad (limit 0.040)' in notes
    assert ('stiffness 11.2 failed with every damping: rested 0.018 rad from the target (limit 0.015); '
            'did not come to rest') in notes
    assert grid_check(grid[3], tolerance=.03, limit=.04) is None
