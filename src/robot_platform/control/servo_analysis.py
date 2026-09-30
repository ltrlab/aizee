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
COMMAND_PERIOD_S = .04  # The sweep sends a command every 40 ms.
GRAVITY_FIT_SPAN_RAD = .3  # A gravity slice is fitted over at least this much travel.
GRAVITY_RESIDUAL = .2  # With gravity fed forward, the share of it a joint's position loop still holds.
WELL_DAMPED = .02  # At a stiffness, dampings overshooting a step at most this share more than the least do.
OVERSHOOT_FLOOR_RAD = .004  # A step's overshoot counts against it only beyond this (friction's offset).


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
    joint's worst gravity torque, its median friction and its gravity slice
    gravity = A sin(q) + C (`gravity_fit`, over at least GRAVITY_FIT_SPAN_RAD; the
    reference pose q = 0 hangs straight down, control.gravity).
    """
    # Torque weighted by forward displacement: a slow ramp moves in pulses (the command is a 25 Hz
    # staircase), and over a pass the inertia term integrates to the change in kinetic energy (~0)
    # while stuck samples, whose torque lies anywhere within friction, carry no weight. Summed by bin
    # with numpy: it runs inside the sweep's control cycle, and a wide joint records tens of thousands
    # of samples (right J1 missed the receiver's 80 ms deadline summing them in Python, 2026-09-28).
    keys, directions, torques, weights = [], [], [], []
    for segment in segments:
        rows = segment['rows']
        if len(rows) < 8:
            continue
        position, effort = column(rows, joint_index, POSITION), column(rows, joint_index, EFFORT)
        step = np.diff(position) * segment['direction']
        forward = step > 0
        keys.append(np.round(((position[:-1] + position[1:]) / 2)[forward] / bin_rad).astype(int))
        torques.append(((effort[:-1] + effort[1:]) / 2)[forward])
        weights.append(step[forward])
        directions.append(np.full(int(forward.sum()), segment['direction']))
    sums = {1: {}, -1: {}}
    if keys:
        keys, directions = np.concatenate(keys), np.concatenate(directions)
        torques, weights = np.concatenate(torques), np.concatenate(weights)
        lowest = int(keys.min()) if len(keys) else 0  # Bins are small integers: count them directly.
        for direction in (1, -1):
            mask = directions == direction
            if mask.any():
                index = keys[mask] - lowest
                weighted = np.bincount(index, weights=torques[mask] * weights[mask])
                total = np.bincount(index, weights=weights[mask])
                sums[direction] = {int(i) + lowest: (float(weighted[i]), float(total[i]))
                                   for i in np.flatnonzero(total)}
    bins = []
    for key in sorted(set(sums[1]) & set(sums[-1])):
        (up_sum, up_weight), (down_sum, down_weight) = sums[1][key], sums[-1][key]
        if up_weight > .25 * bin_rad and down_weight > .25 * bin_rad:
            up_mean, down_mean = up_sum / up_weight, down_sum / down_weight
            bins.append({'position': key * bin_rad, 'up': up_mean, 'down': down_mean,
                         'gravity': (up_mean + down_mean) / 2, 'friction': abs(up_mean - down_mean) / 2})
    if not bins:
        return {'bins': [], 'gravity_max': None, 'friction': None, 'span': None, 'gravity_fit': None}
    worst = max(bins, key=lambda b: abs(b['gravity']))
    positions = np.array([b['position'] for b in bins])
    fit = None
    if len(bins) >= 5 and positions[-1] - positions[0] >= GRAVITY_FIT_SPAN_RAD:
        (a, c), *_ = np.linalg.lstsq(np.column_stack([np.sin(positions), np.ones_like(positions)]),
                                     np.array([b['gravity'] for b in bins]), rcond=None)
        fit = {'A': float(a), 'C': float(c)}
    return {'bins': bins, 'gravity_max': abs(worst['gravity']), 'gravity_worst_at': worst['position'],
            'friction': float(np.median([b['friction'] for b in bins])),
            'span': (bins[0]['position'], bins[-1]['position']), 'gravity_fit': fit}


def inertia_fit(segments, joint_index, *, gravity_fit=None, friction=0.):
    """A joint's inertia (kg m2) from moves that speed up and slow down (its speed tests, braking and passes):
    the drive's torque less its gravity slice (A sin q + C) and friction in the direction of motion, against
    its acceleration, both taken from positions differenced over a command period and centred on the same
    sample. None without enough accelerating samples, or when the fit is not a plausible inertia.

    Without its inertia fed forward, a heavy joint lags while it accelerates: right J2 (0.6 kg m2 fitted this
    way, 2026-09-30) lagged 0.035-0.041 rad at 1.2 rad/s, about 0.13 s into each cruise, and its speed stopped
    at 1.0 rad/s, while cruising it lagged 0.01."""
    residuals, accelerations = [], []
    for rows in segments:
        if rows is None or len(rows) < 8:
            continue
        t = times_s(rows)
        window = max(1, int(round(COMMAND_PERIOD_S / max(float(np.median(np.diff(t))), 1e-4))))
        if len(t) < 2 * window + 8:
            continue
        position = column(rows, joint_index, POSITION)
        effort = column(rows, joint_index, EFFORT)
        # Speed over a period centred on sample i: (p[i + w] - p[i - w]) / (t[i + w] - t[i - w]); acceleration
        # the same way from those speeds, so both are centred on sample i.
        span = t[2 * window:] - t[:-2 * window]
        speed = (position[2 * window:] - position[:-2 * window]) / np.maximum(span, 1e-6)
        middle = slice(window, len(t) - window)  # Samples the speeds are centred on.
        speed_t = t[middle]
        if len(speed) < 2 * window + 2:
            continue
        acceleration = (speed[2 * window:] - speed[:-2 * window]) / np.maximum(
            speed_t[2 * window:] - speed_t[:-2 * window], 1e-6)
        centre = slice(2 * window, len(t) - 2 * window)
        v, a = speed[window:len(speed) - window], acceleration
        q, torque = position[centre], effort[centre]
        count = min(len(v), len(a), len(q), len(torque))
        v, a, q, torque = v[:count], a[:count], q[:count], torque[:count]
        gravity = (gravity_fit['A'] * np.sin(q) + gravity_fit['C']) if gravity_fit else 0.
        residual = torque - gravity - friction * np.sign(v) * (np.abs(v) > .05)
        use = (np.abs(v) > .05) & (np.abs(a) > 1.) & (np.abs(a) < 60.)
        residuals.append(residual[use])
        accelerations.append(a[use])
    if not residuals:
        return None
    residual, acceleration = np.concatenate(residuals), np.concatenate(accelerations)
    if len(acceleration) < 100 or np.ptp(acceleration) < 4.:
        return None
    inertia, _ = np.polyfit(acceleration, residual, 1)
    return float(inertia) if 0. < inertia < 10. else None


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
    return {'overshoot': overshoot, 'size': abs(delta), 'settle_s': settle if settled else None,
            'steady_error': abs(target - rest) if len(tail) else None,
            'reversals': int(np.count_nonzero(np.diff(signs) != 0)) if len(signs) > 1 else 0,
            'peak_effort': float(np.max(np.abs(effort))) if len(effort) else None,
            'duration_s': float(t[-1]) if len(t) else 0.}


# -- ramps and braking ----------------------------------------------------------------

def ramp_lag(rows, joint_index, *, speed):
    """Lag behind the command while cruising.

    The cruise is where the command moves at its top speed, measured over each command
    period (the receiver's ramp through the 25 Hz commands, or their staircase), not its
    accelerating and braking ends: a sample counts when the periods before and after it
    both do. The top is the commanded `speed` once the command reached it: commands
    arrive a loop cycle early or late, so the ramp briefly runs up to half again faster
    or pauses, and the fastest period alone is no cruise. The slope of the line fitted
    through the command there is the speed reached; the lag is the command's lead over
    the measured position, which is what the receiver's tracking bound limits (a line
    extrapolated into the braking end read J2 0.041 rad behind where it lagged 0.024).
    Returns that speed, the mean and worst lag (rad, positive = behind) and the worst
    effort, or None without a cruise phase.
    """
    t = times_s(rows)
    commanded = column(rows, joint_index, COMMAND)
    position = column(rows, joint_index, POSITION)
    if len(t) < 8:
        return None
    window = max(1, int(round(COMMAND_PERIOD_S / max(float(np.median(np.diff(t))), 1e-4))))
    if len(t) < 2 * window + 8:
        return None
    rate = np.abs(commanded[window:] - commanded[:-window]) / np.maximum(t[window:] - t[:-window], 1e-6)
    top = min(float(np.max(rate)), speed or math.inf)
    if not top > 0:
        return None
    fast = np.flatnonzero(rate >= .9 * top)  # rate[i] spans samples i to i + window.
    cruise = slice(int(fast[0]) + window, int(fast[-1]) + 1)
    if cruise.stop - cruise.start < 8:
        return None
    slope, _ = np.polyfit(t[cruise], commanded[cruise], 1)
    direction = math.copysign(1, slope)
    lag = (commanded[cruise] - position[cruise]) * direction
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
    # A few thousandths of a radian past a small step is friction's offset where the joint sticks, not ringing:
    # right J3's 18 mrad steps rested 2.5-3.6 mrad past at every damping, 14-20%, and no stiffness passed.
    if overshoot > overshoot_limit and max(s['overshoot'] * s.get('size', math.inf) for s in steps) > OVERSHOOT_FLOOR_RAD:
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


def static_load(torque, *, gravity_fed=False):
    """The torque a joint's position loop holds at rest: its friction and its gravity, or with gravity fed
    forward, GRAVITY_RESIDUAL of it (the model's error)."""
    return ((torque.get('gravity_max') or 0.) * (GRAVITY_RESIDUAL if gravity_fed else 1.)
            + (torque.get('friction') or 0.))


def best_damping(entries):
    """Of grid entries at one stiffness (each with its steps), the one to use: the fastest to settle of those
    well damped, overshooting a step at most WELL_DAMPED more than the least of them does (what is left of the
    least is friction's offset, not ringing). With velocity fed forward, a lightly damped joint lags more as it
    speeds up and overshoots its stops: right J2 at kp 220 settled fastest at kd 14.7 (7% overshoot, 0% at
    kd 29.4) but lagged 0.042 rad at 1.2 rad/s and overshot every stop by 0.02-0.03 rad; at kd 21 (2%) it
    lagged 0.033 and braked at 32 rad/s2. Right J1's fastest, kd 5, overshot 0.5%."""
    def worst(entry, key, default):
        values = [step[key] for step in entry['steps'] if step and step.get(key) is not None]
        return max(values) if values else default
    least = min(worst(e, 'overshoot', 0.) for e in entries)
    return min(entries, key=lambda e: (worst(e, 'overshoot', 0.) > least + WELL_DAMPED,
                                       worst(e, 'settle_s', math.inf)))


def recommend(joint, *, torque, grid, holds, speeds, decelerations, bound, tolerance, profile_gains,
              overshoot_limit=.1, lag_fraction=.5, gravity_fed=False):
    """Settings for one joint from its sweep results, with the reason for each.

    torque: torque_map() result. grid: [{'kp', 'kd', 'steps': [step_response...],
    'ramp': ramp_lag or None}] from the stiffness x damping ladder. holds: {kp:
    steady error at the worst-gravity point}. speeds: [ramp_lag results at the
    chosen gains]. decelerations: [{'deceleration', 'max_lag', 'overshoot'}].
    gravity_fed: the joint's gravity is fed forward, so its position loop holds only
    the model's error of it.
    """
    notes = []
    limit = lag_fraction * bound
    load = static_load(torque, gravity_fed=gravity_fed)
    friction = torque.get('friction') or 0.
    kp_static = load / limit if limit > 0 else math.inf
    kp_settle = friction / (.5 * tolerance) if tolerance > 0 else math.inf
    kp_floor = max(kp_static, kp_settle)
    held = f'{GRAVITY_RESIDUAL:.0%} of gravity (fed forward)' if gravity_fed else 'gravity'
    notes.append(f'holding {load:.2f} Nm ({held} + friction) within {limit:.3f} rad needs kp >= {kp_static:.1f}; '
                 f'settling within {tolerance / 2:.3f} rad against {friction:.2f} Nm friction needs kp >= '
                 f'{kp_settle:.1f}')

    def passes(entry):
        return grid_check(entry, tolerance=tolerance, limit=limit, overshoot_limit=overshoot_limit) is None

    # The stiffest rung that passes: it follows and holds best, and the criteria keep it settled and unshaken.
    candidates = sorted({entry['kp'] for entry in grid if entry['kp'] >= kp_floor}, reverse=True)
    chosen = None
    for kp in candidates:
        if kp in holds and holds[kp] is not None and holds[kp] > limit:
            notes.append(f'stiffness {kp:g}: sagged {holds[kp]:.3f} rad at the worst-gravity point')
            continue
        passing = [e for e in grid if e['kp'] == kp and passes(e)]
        if passing:
            best = best_damping(passing)
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
        notes.append(f"stiffest passing stiffness {chosen['kp']:g}")
        chosen['met'] = True
    # A speed counts only when every direction tested at it (and at every slower speed) was followed: within
    # the lag limit, and reaching the speed (the sweep's plan waits for a joint that cannot keep up).
    def commanded(result):
        return result.get('target_speed') or result.get('speed')

    def missed(result):
        return (result.get('max_lag') is None or result['max_lag'] > limit
                or (result.get('target_speed') and (result.get('speed') or 0.) < .85 * result['target_speed']))

    tested = [s for s in speeds if s and commanded(s) is not None]
    failed = [commanded(s) for s in tested if missed(s)]
    followed = [s for s in tested if not missed(s) and not any(f <= commanded(s) + 1e-9 for f in failed)]
    speed = max((commanded(s) for s in followed), default=None)
    if speed is None:
        notes.append('no tested speed was followed within half the tracking bound both ways')
    faster = [f for f in failed if speed is None or f > speed]
    if faster:
        notes.append(f'{min(faster):g} rad/s lagged over {limit:.3f} rad or fell short of it in at least one '
                     'direction')
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
