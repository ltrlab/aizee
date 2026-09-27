"""Measurements from a servo sweep's receiver record, and the settings they support.

A record is rows of [t_ms, then per joint: position, velocity, effort, commanded
position] at the receiver's tick rate (control.live_receiver / service Recorder).
Segments are the sweep's own annotations: which joint moved, how, with which
gains, over which rows. Everything here is pure arithmetic on those arrays, so it
runs the same on the robot computer during a sweep and in tests.

Commands are position-only, so a joint's servo torque is kp * lead - kd * speed:
it lags a moving target and sags under load. The recommendation keeps the static
load within half the tracking bound, the joint settling within half its
tolerance at the reference, and speeds and decelerations to what it followed
with at most half the bound of lag.
"""
from __future__ import annotations

import math

import numpy as np

POSITION, VELOCITY, EFFORT, COMMAND = range(4)


def column(rows, joint_index, field):
    return rows[:, 1 + 4 * joint_index + field]


def times_s(rows):
    return rows[:, 0] / 1000.


def position_speed(rows, joint_index, *, window_s=.02):
    """Speed from measured positions, averaged over about `window_s` (positions are clean
    where the drive's speed reading is not)."""
    t, q = times_s(rows), column(rows, joint_index, POSITION)
    if len(t) < 3:
        return np.zeros(len(t))
    speed = np.gradient(q, t)
    width = max(1, int(round(window_s / max(np.median(np.diff(t)), 1e-4))))
    if width > 1 and len(speed) > width:
        speed = np.convolve(speed, np.ones(width) / width, mode='same')
    return speed


# -- torque map -------------------------------------------------------------------

def torque_map(segments, joint_index, *, bin_rad=.02):
    """Gravity and friction by angle from slow ramps in both directions.

    Each segment: {'rows': array, 'direction': +1|-1}. Effort while moving up is
    gravity + friction; down, gravity - friction. Returns bins with both
    directions: [{'position', 'up', 'down', 'gravity', 'friction'}], plus the
    joint's worst gravity torque and its median friction.
    """
    # Torque weighted by forward displacement: a slow ramp moves in pulses (the command is a 25 Hz
    # staircase), and over a pass the inertia term integrates to the change in kinetic energy (~0)
    # while stuck samples, whose torque lies anywhere within friction, carry no weight.
    sums = {}
    for segment in segments:
        rows = segment['rows']
        if len(rows) < 8:
            continue
        position, effort = column(rows, joint_index, POSITION), column(rows, joint_index, EFFORT)
        step = np.diff(position) * segment['direction']
        forward = step > 0
        middle = ((position[:-1] + position[1:]) / 2)[forward]
        torque = ((effort[:-1] + effort[1:]) / 2)[forward]
        keys = np.round(middle / bin_rad).astype(int)
        for key, value, weight in zip(keys, torque, step[forward]):
            entry = sums.setdefault(int(key), {1: [0., 0.], -1: [0., 0.]})
            entry[segment['direction']][0] += float(value * weight)
            entry[segment['direction']][1] += float(weight)
    bins = []
    for key in sorted(sums):
        (up_sum, up_weight), (down_sum, down_weight) = sums[key][1], sums[key][-1]
        if up_weight > .25 * bin_rad and down_weight > .25 * bin_rad:
            up_mean, down_mean = up_sum / up_weight, down_sum / down_weight
            bins.append({'position': key * bin_rad, 'up': up_mean, 'down': down_mean,
                         'gravity': (up_mean + down_mean) / 2, 'friction': abs(up_mean - down_mean) / 2})
    if not bins:
        return {'bins': [], 'gravity_max': None, 'friction': None, 'span': None}
    worst = max(bins, key=lambda b: abs(b['gravity']))
    return {'bins': bins, 'gravity_max': abs(worst['gravity']), 'gravity_worst_at': worst['position'],
            'friction': float(np.median([b['friction'] for b in bins])),
            'span': (bins[0]['position'], bins[-1]['position'])}


# -- step response ------------------------------------------------------------------

def step_response(rows, joint_index, *, target, start, band_fraction=.1, band_floor_rad=.002):
    """A position step from `start` to `target`: overshoot past the target (fraction of
    the step); settling time (s), until the joint stays within 10% of the step (at least
    2 mrad) of where it comes to rest; steady error (rad), how far from the target it
    rests (friction stops a joint short); sign reversals of the error while settling;
    and the peak effort."""
    delta = target - start
    t = times_s(rows)
    position = column(rows, joint_index, POSITION)
    commanded = column(rows, joint_index, COMMAND)
    moved = np.flatnonzero(np.abs(commanded - start) > abs(delta) / 2)
    if not len(moved) or abs(delta) < 1e-6:
        return None
    t0 = t[moved[0]]
    after = t >= t0
    t, position = t[after] - t0, position[after]
    effort = column(rows, joint_index, EFFORT)[after]
    error = (target - position) * math.copysign(1, delta)  # Positive while short of the target.
    overshoot = max(0., float(-error.min()) / abs(delta))
    band = max(band_fraction * abs(delta), band_floor_rad)
    tail = position[int(len(position) * .75):]
    rest = float(np.mean(tail)) if len(tail) else float(position[-1])
    outside = np.flatnonzero(np.abs(position - rest) > band)
    settle = float(t[outside[-1]]) if len(outside) else 0.
    settled = len(outside) == 0 or outside[-1] < len(t) - len(tail)
    signs = np.sign(error[np.abs(error) > band_floor_rad])
    return {'overshoot': overshoot, 'settle_s': settle if settled else None,
            'steady_error': abs(target - rest) if len(tail) else None,
            'reversals': int(np.count_nonzero(np.diff(signs) != 0)) if len(signs) > 1 else 0,
            'peak_effort': float(np.max(np.abs(effort))) if len(effort) else None,
            'duration_s': float(t[-1]) if len(t) else 0.}


# -- ramps and braking ----------------------------------------------------------------

def ramp_lag(rows, joint_index, *, speed):
    """Lag behind the ideal constant-speed path while cruising.

    The commanded position is a 25 Hz staircase; the ideal path is the line fitted
    through it where it moves at `speed`. Returns mean and worst lag (rad, positive
    = behind) and the worst effort, or None without a cruise phase.
    """
    t = times_s(rows)
    commanded = column(rows, joint_index, COMMAND)
    position = column(rows, joint_index, POSITION)
    if len(t) < 8:
        return None
    # The move is where the command has left its start and not yet reached its end;
    # its middle 80% cruises (the staircase's own steps average out in the fit).
    moving = np.flatnonzero((commanded != commanded[0]) & (commanded != commanded[-1]))
    if len(moving) < 8:
        return None
    first, last = moving[0], moving[-1]
    margin = (last - first) // 10
    cruise = slice(first + margin, last - margin + 1)
    if cruise.stop - cruise.start < 8:
        return None
    slope, intercept = np.polyfit(t[cruise], commanded[cruise], 1)
    direction = math.copysign(1, slope)
    lag = (slope * t[cruise] + intercept - position[cruise]) * direction
    effort = column(rows, joint_index, EFFORT)[cruise]
    return {'speed': abs(float(slope)), 'mean_lag': float(np.mean(lag)), 'max_lag': float(np.max(lag)),
            'peak_effort': float(np.max(np.abs(effort)))}


def braking(rows, joint_index, *, stop_at):
    """How a joint followed a commanded deceleration to rest at `stop_at`: the
    worst lag while braking (rad) and the overshoot past the stop (rad)."""
    commanded = column(rows, joint_index, COMMAND)
    position = column(rows, joint_index, POSITION)
    if not len(commanded):
        return None
    direction = math.copysign(1, stop_at - commanded[0]) if stop_at != commanded[0] else 1.
    lag = (commanded - position) * direction
    overshoot = max(0., float(np.max((position - stop_at) * direction)))
    return {'max_lag': float(np.max(lag)), 'overshoot': overshoot}


def speed_noise(rows, joint_index):
    """The drive's speed reading while the joint is commanded to hold still: RMS and worst."""
    velocity = column(rows, joint_index, VELOCITY)
    if not len(velocity):
        return None
    return {'rms': float(np.sqrt(np.mean(velocity ** 2))), 'max': float(np.max(np.abs(velocity)))}


# -- recommendation ------------------------------------------------------------------------

def grid_check(entry, *, tolerance, limit, overshoot_limit=.1):
    """Why a stiffness x damping entry fails the recommendation's criteria, as (kind, text), or
    None when it passes: its steps must not overshoot, must come to rest within half the
    tolerance of the target, and its ramp must lag at most `limit`."""
    if entry.get('failed'):
        return 'guard', f"guard: {entry['failed']}"
    steps = [s for s in entry['steps'] if s]
    if not steps:
        return 'data', 'no step measured'
    overshoot = max(s['overshoot'] for s in steps)
    if overshoot > overshoot_limit:
        return 'overshoot', f'overshot {overshoot:.0%} of the step (limit {overshoot_limit:.0%})'
    if any(s['settle_s'] is None for s in steps):
        return 'rest', 'did not come to rest'
    error = max((s['steady_error'] or 0.) for s in steps)
    if error > .5 * tolerance:
        return 'error', f'rested {error:.3f} rad from the target (limit {.5 * tolerance:.3f})'
    ramp = entry.get('ramp')
    if ramp is not None and ramp['max_lag'] > limit:
        return 'ramp', f"ramp lagged {ramp['max_lag']:.3f} rad (limit {limit:.3f})"
    return None


def recommend(joint, *, torque, grid, holds, speeds, decelerations, bound, tolerance, profile_gains,
              overshoot_limit=.1, lag_fraction=.5):
    """Settings for one joint from its sweep results, with the reason for each.

    torque: torque_map() result. grid: [{'kp', 'kd', 'steps': [step_response...],
    'ramp': ramp_lag or None}] from the stiffness x damping ladder. holds: {kp:
    steady error at the worst-gravity point}. speeds: [ramp_lag results at the
    chosen gains]. decelerations: [{'deceleration', 'max_lag', 'overshoot'}].
    """
    notes = []
    limit = lag_fraction * bound
    load = (torque.get('gravity_max') or 0.) + (torque.get('friction') or 0.)
    friction = torque.get('friction') or 0.
    kp_static = load / limit if limit > 0 else math.inf
    kp_settle = friction / (.5 * tolerance) if tolerance > 0 else math.inf
    kp_floor = max(kp_static, kp_settle)
    notes.append(f'holding {load:.2f} Nm (gravity + friction) within {limit:.3f} rad needs kp >= {kp_static:.1f}; '
                 f'settling within {tolerance / 2:.3f} rad against {friction:.2f} Nm friction needs kp >= '
                 f'{kp_settle:.1f}')

    def passes(entry):
        return grid_check(entry, tolerance=tolerance, limit=limit, overshoot_limit=overshoot_limit) is None

    candidates = sorted({entry['kp'] for entry in grid if entry['kp'] >= kp_floor})
    chosen = None
    for kp in candidates:
        if kp in holds and holds[kp] is not None and holds[kp] > limit:
            notes.append(f'stiffness {kp:g}: sagged {holds[kp]:.3f} rad at the worst-gravity point')
            continue
        passing = [e for e in grid if e['kp'] == kp and passes(e)]
        if passing:
            best = min(passing, key=lambda e: max(s['settle_s'] for s in e['steps'] if s))
            chosen = {'kp': kp, 'kd': best['kd']}
            break
        # Why not, once per kind of failure, across its damping ratios.
        kinds = {}
        for entry in grid:
            if entry['kp'] == kp:
                kind, text = grid_check(entry, tolerance=tolerance, limit=limit, overshoot_limit=overshoot_limit)
                kinds.setdefault(kind, text)
        notes.append(f'stiffness {kp:g} failed with every damping: ' + '; '.join(list(kinds.values())[:2]))
    if chosen is None:
        notes.append('no tested stiffness and damping met the criteria; keeping the profile gains')
        chosen = dict(profile_gains)
        chosen['met'] = False
    else:
        # One ladder step of margin above the lowest passing stiffness, when it was tested and passed too.
        higher = [kp for kp in candidates if kp > chosen['kp']]
        if higher:
            margin_kp = higher[0]
            passing = [e for e in grid if e['kp'] == margin_kp and passes(e)]
            if passing:
                best = min(passing, key=lambda e: max(s['settle_s'] for s in e['steps'] if s))
                notes.append(f"lowest passing stiffness {chosen['kp']:.1f}; one ladder step of margin")
                chosen = {'kp': margin_kp, 'kd': best['kd']}
        chosen['met'] = True
    # A speed counts only when every direction tested at it (and at every slower speed) was followed.
    def commanded(result):
        return result.get('target_speed') or result.get('speed')

    tested = [s for s in speeds if s and commanded(s) is not None]
    failed = [commanded(s) for s in tested if s.get('max_lag') is None or s['max_lag'] > limit]
    followed = [s for s in tested if s.get('max_lag') is not None and s['max_lag'] <= limit
                and not any(f <= commanded(s) + 1e-9 for f in failed)]
    speed = max((commanded(s) for s in followed), default=None)
    if speed is None:
        notes.append('no tested speed was followed within half the tracking bound both ways')
    faster = [f for f in failed if speed is None or f > speed]
    if faster:
        notes.append(f'{min(faster):g} rad/s lagged over {limit:.3f} rad in at least one direction')
    braked = [d for d in decelerations if d and d['max_lag'] <= limit and d['overshoot'] <= limit / 2]
    deceleration = max((d['deceleration'] for d in braked), default=None)
    at_speed = [s['max_lag'] for s in followed if speed is not None and commanded(s) >= speed * .9]
    tracking = None
    if at_speed:
        tracking = min(bound, max(.03, 2 * max(at_speed)))
        notes.append(f'worst lag at {speed:.2f} rad/s was {max(at_speed):.3f} rad; tracking bound twice that '
                     f'(at least 0.03, at most the current {bound:.3f})')
    return {'joint': joint, 'hold_kp': chosen['kp'], 'hold_kd': chosen['kd'], 'gains_met': chosen['met'],
            'max_velocity': speed, 'stop_acceleration': deceleration, 'max_tracking_error': tracking,
            'kp_floor': kp_floor, 'load_nm': load, 'friction_nm': friction, 'notes': notes}
