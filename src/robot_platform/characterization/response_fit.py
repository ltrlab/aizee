"""Fit a joint's response from receiver recordings and replay it; offline and read-only.

Per joint, with the arm's other joints held (a sweep moves one joint at a time):

  mechanics  I a = tau - g(q) - b v - f sgn(v),   g(q) = A sin q + B cos q + C
  drive      tau(t) = k * (kp (q* - q) + kd (v* - v) + tau_ff), its command d late

1. Mechanics, from any recording: least squares on position and the drive's reported effort (as tau), both
   low-passed by the same zero-phase filter; v and a are central differences of the filtered position, so
   both sides of the regression carry the same frequency content (Savitzky-Golay derivative filters do not:
   their implicit smoothing differs from the smoothing filter's, and inertia came out 6-23% high on synthetic
   recordings, growing with the window). Every regressor is that filter applied to its raw signal: the
   friction sign (of a lightly filtered speed) and the gravity terms too; tanh of the filtered speed instead
   put a sharp sign against filtered torque at each reversal and found 27% of an exact joint's Coulomb
   friction. The effort is first aligned to the position:
   friction and damping absorb any offset between them at about kp x offset (one tick moved the viscous term
   by +0.36 Nm s/rad on an exact synthetic joint), and a held command makes an end-of-tick effort sample stand
   for torque applied up to a tick earlier. The offset is searched over EFFORT_SHIFTS on the fit blocks.
   A drive that samples its torque at a fixed point of each tick, while its command is held and the joint
   moves, reports kp x speed x (that point's offset from mid-tick) on top of the torque that moves the joint.
   That term looks like viscous friction. Where a recording varies its gains (a sweep's stiffness ladder), a
   kp x speed regressor separates the two; with constant or unknown gains the viscous term includes it, and
   the report says so (a synthetic joint sampled at each tick's end read 0.15 for 0.40 Nm s/rad).
2. Drive, from recordings that carry each tick's whole command (the recorder since 5852fe2): the delay d
   and scale k that best turn the recorded command, at the drive's own measured state, into its reported
   effort. The state is not delayed; the command is (it arrives late; the drive's loop uses fresh readings).
3. Replay: windows re-simulated from their measured start. Open loop under the reported effort (any
   recording), and closed loop under the recorded commands through the fitted drive (what the simulator does),
   each against the measured position and a naive baseline.
4. Closed-loop fit, from recordings with commands: inertia, viscous and Coulomb friction, the command delay
   and a gravity scale chosen so that the closed loop, simulated under the recorded commands, reproduces the
   measured positions of the fit windows (least squares). It never reads the effort, whose timing the real
   drives leave uncertain: on a right J2 sweep the effort fit's inertia ran from 0.65 to 1.33 kg m2 across
   plausible effort lags with almost the same residual, while its gravity (A 18.3 Nm) did not move. This is
   the fit the simulator should take; stages 1 and 2 seed it.

Every fit uses alternate BLOCK_S blocks of the recording and is scored on the others. Recordings from before
the command block exist only for stage 1 and the open-loop replay; the report says so instead of guessing.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np

TICK_S = .0025  # The receiver's period.
CUTOFF_HZ = 8.  # Zero-phase low-pass on position and effort alike (4th-order Butterworth, run both ways).
EDGE_S = .25  # Rows this close to a run's ends carry the filter's start-up transient and are dropped.
BLOCK_S = 10.  # Alternate blocks fit and test.
V_EPS = .005  # rad/s: the friction sign's smoothing, before it is filtered like everything else.
SIGN_CUTOFF_HZ = 30.  # The speed whose sign gives friction's direction: lightly filtered position, differenced.
STICK_V = 1e-4  # rad/s: a replayed joint below this sticks while its net torque is within friction.
SUBSTEPS = 5
OPEN_WINDOW_S = .25
CLOSED_WINDOW_S = 1.
FIT_WINDOWS = 120  # Closed-loop fit windows, spread evenly over the fit blocks.
FIT_STRIDE = 4  # Every 4th tick of a window enters the closed-loop fit's residual.
MAX_DELAY_TICKS = 24  # 60 ms
DELAY_STEP_TICKS = .25
DEAD_EFFORT_NM = .01  # A reported effort spanning less than this over a whole recording carries no torque (left J1).
MIN_INERTIA = 1e-3  # kg m2: below this a fitted joint is not simulated (it would not be a joint of Minerva's).
OTHERS_MOVED_RAD = .05  # Another joint of the recording moving more than this breaks the one-joint gravity model.
BOOTSTRAP = 64
EFFORT_SHIFTS = np.round(np.arange(-4., 4.0001, .1), 3)  # ticks: the effort as reported at t + shift stands for t.
PARAMS = ('inertia', 'viscous', 'friction', 'gravity_sin', 'gravity_cos', 'gravity_offset')
COMMAND_FIELDS = ('command_velocity', 'command_effort', 'kp', 'kd')
GAIN_SPREAD = .05  # kp varying this much (std / mean, drive on) separates the effort-sampling term.


@dataclass
class Joint:
    name: str
    t: np.ndarray  # s, the receiver clock
    q: np.ndarray
    v: np.ndarray  # the drive's own velocity reading
    effort: np.ndarray  # the drive's reported torque
    command: np.ndarray  # commanded position
    commands: dict | None  # COMMAND_FIELDS -> arrays, or None (a recording from before the command block)


def load_session(path):
    """{'joints': {name: Joint}, 'provenance': {...}} from a recording's columns.json and samples.f32."""
    path = Path(path)
    columns = json.loads((path / 'columns.json').read_text(encoding='utf-8'))['columns']
    raw = (path / 'samples.f32').read_bytes()
    values = np.frombuffer(raw, dtype=np.float32)
    if columns[0] != 't_ms' or values.size % len(columns):
        raise ValueError(f'{path}: samples.f32 does not hold whole rows of its {len(columns)} columns')
    data = values.reshape(-1, len(columns)).astype(float)
    index = {name: i for i, name in enumerate(columns)}
    t = data[:, 0] / 1000.
    if len(t) > 1 and np.any(np.diff(t) <= 0):
        raise ValueError(f'{path}: recorder time does not increase')
    names = [c[:-len('.position')] for c in columns if c.endswith('.position')]
    joints = {}
    for name in names:
        fields = [index[f'{name}.{f}'] for f in ('position', 'velocity', 'effort', 'command')]
        commands = None
        if all(f'{name}.{f}' in index for f in COMMAND_FIELDS):
            commands = {f: data[:, index[f'{name}.{f}']] for f in COMMAND_FIELDS}
            if np.isnan(commands['kp']).all():
                commands = None  # Padded from an older receiver: unknown, not zero.
        joints[name] = Joint(name, t, *(data[:, i] for i in fields), commands)
    gaps = np.diff(t) > 1.5 * TICK_S
    return {'joints': joints, 'provenance': {
        'session': str(path), 'samples_sha256': hashlib.sha256(raw).hexdigest(), 'rows': len(t),
        'span_s': float(t[-1] - t[0]) if len(t) else 0., 'gaps': int(gaps.sum()),
        'command_block': any(j.commands is not None for j in joints.values())}}


def travel(joint):
    return float(np.ptp(joint.q)) if len(joint.q) else 0.


def runs(t):
    """Index ranges of contiguous ticks (a dropped recorder row splits a run)."""
    breaks = np.flatnonzero(np.diff(t) > 1.5 * TICK_S) + 1
    edges = np.concatenate([[0], breaks, [len(t)]])
    return [(int(a), int(b)) for a, b in zip(edges[:-1], edges[1:])]


def _filter(cutoff_hz=CUTOFF_HZ):
    from scipy.signal import butter
    return butter(4, cutoff_hz, fs=1 / TICK_S, output='sos')


def _long_runs(joint):
    edge = int(round(EDGE_S / TICK_S))
    return [(start, end) for start, end in runs(joint.t) if end - start >= 4 * edge], edge


def aligned_effort(joint, shift_ticks):
    """The reported effort moved by `shift_ticks` (the effort reported at t + shift stands for t)."""
    if not shift_ticks:
        return joint.effort
    return np.interp(joint.t + shift_ticks * TICK_S, joint.t, joint.effort)


def filtered_effort(joint, shift_ticks=0.):
    """The aligned effort, filtered as smoothed() filters position, at smoothed()'s rows."""
    from scipy.signal import sosfiltfilt
    sos, (spans, edge) = _filter(), _long_runs(joint)
    effort = aligned_effort(joint, shift_ticks)
    return np.concatenate([sosfiltfilt(sos, effort[start:end])[edge:end - start - edge] for start, end in spans])


def smoothed(joint, shift_ticks=0.):
    """Rows away from run edges, with filtered position, its velocity and acceleration, and filtered effort."""
    from scipy.signal import sosfiltfilt
    sos, light, (spans, edge) = _filter(), _filter(SIGN_CUTOFF_HZ), _long_runs(joint)
    if not spans:
        raise ValueError(f'{joint.name}: no contiguous run long enough to filter')
    columns = {key: [] for key in ('index', 'q', 'v', 'a', 'sign', 'sin', 'cos')}
    for start, end in spans:
        raw = joint.q[start:end]
        qf = sosfiltfilt(sos, raw)
        vf = np.gradient(qf, TICK_S)
        speed = np.gradient(sosfiltfilt(light, raw), TICK_S)
        inside = slice(edge, end - start - edge)
        values = [('index', np.arange(start, end)), ('q', qf), ('v', vf), ('a', np.gradient(vf, TICK_S)),
                  ('sign', sosfiltfilt(sos, np.tanh(speed / V_EPS))), ('sin', sosfiltfilt(sos, np.sin(raw))),
                  ('cos', sosfiltfilt(sos, np.cos(raw)))]
        if joint.commands is not None:
            values.append(('kpv', sosfiltfilt(sos, joint.commands['kp'][start:end] * speed)))
        for key, value in values:
            columns.setdefault(key, []).append(value[inside])
    s = {key: np.concatenate(value) for key, value in columns.items() if value}
    if joint.commands is not None:
        kp = joint.commands['kp'][enabled(joint)]
        s['sampling_separable'] = bool(len(kp) and kp.mean() > 0 and kp.std() / kp.mean() > GAIN_SPREAD)
    return dict(s, tau=filtered_effort(joint, shift_ticks), shift_ticks=float(shift_ticks))


def align(joint, s, fit):
    """The effort shift whose mechanics fit leaves the least torque residual on the fit blocks, and its rows."""
    X = design(s)[fit]
    best = None
    for shift in EFFORT_SHIFTS:
        y = filtered_effort(joint, shift)[fit]
        residual = y - X @ np.linalg.lstsq(X, y, rcond=None)[0]
        rms = float(np.sqrt(np.mean(residual ** 2)))
        if best is None or rms < best[0]:
            best = (rms, float(shift))
    s = dict(s, tau=filtered_effort(joint, best[1]), shift_ticks=best[1])
    at_edge = best[1] in (EFFORT_SHIFTS[0], EFFORT_SHIFTS[-1])
    return s, at_edge


def blocks(t, index):
    """True for rows in fit blocks, False for held-out ones."""
    return (np.floor((t[index] - t[0]) / BLOCK_S).astype(int) % 2) == 0


def design(s):
    """Regressors for PARAMS (and the effort-sampling term where separable), each the filtered raw signal."""
    columns = [s['a'], s['v'], s['sign'], s['sin'], s['cos'], np.ones_like(s['a'])]
    if s.get('sampling_separable'):
        columns.append(s['kpv'])
    return np.column_stack(columns)


def gravity(params, q):
    return params['gravity_sin'] * np.sin(q) + params['gravity_cos'] * np.cos(q) + params['gravity_offset']


def fit_mechanics(joint, s, fit, *, seed=1):
    X, y = design(s), s['tau']
    names = PARAMS + (('effort_sampling_s',) if s.get('sampling_separable') else ())
    theta = np.linalg.lstsq(X[fit], y[fit], rcond=None)[0]
    params = dict(zip(names, map(float, theta)))
    # Uncertainty from refits on resampled fit blocks (residuals are correlated within a block).
    block = np.floor((joint.t[s['index']] - joint.t[0]) / BLOCK_S).astype(int)
    fit_blocks = np.unique(block[fit])
    rng = np.random.default_rng(seed)
    draws = []
    for _ in range(BOOTSTRAP):
        chosen = rng.choice(fit_blocks, size=len(fit_blocks))
        rows = np.concatenate([np.flatnonzero(block == b) for b in chosen])
        draws.append(np.linalg.lstsq(X[rows], y[rows], rcond=None)[0])
    spread = np.std(draws, axis=0) if len(fit_blocks) > 1 else np.full(len(names), np.nan)

    def score(rows):
        residual = y[rows] - X[rows] @ theta
        return {'rms_nm': float(np.sqrt(np.mean(residual ** 2))),
                'r2': float(1 - np.sum(residual ** 2) / np.sum((y[rows] - y[rows].mean()) ** 2))}
    flags = []
    if params['inertia'] <= 0 or params['friction'] < 0:
        flags.append('implausible: negative inertia or friction')
    if params['inertia'] > 0 and spread[0] / params['inertia'] > .2:
        flags.append('inertia poorly excited (bootstrap spread over 20%)')
    if not s.get('sampling_separable'):
        flags.append("viscous includes the drive's effort-sampling term (gains constant or not recorded)")
    return {'params': params, 'bootstrap_std': dict(zip(names, map(float, spread))),
            'effort_sampling_tick': (-params['effort_sampling_s'] / TICK_S if 'effort_sampling_s' in params
                                     else None),
            'effort_shift_ms': s.get('shift_ticks', 0.) * TICK_S * 1000,
            'fit': score(fit), 'holdout': score(~fit), 'flags': flags}


def drive_torque(joint, delay_ticks, *, scale=1.):
    """Each tick's torque from the command d ticks earlier, at the drive's measured state now."""
    c = joint.commands
    t = joint.t
    late = t - delay_ticks * TICK_S

    def at(x):
        return np.interp(late, t, x)
    return scale * (at(c['kp']) * (at(joint.command) - joint.q) + at(c['kd']) * (at(c['command_velocity']) - joint.v)
                    + at(c['command_effort']))


def enabled(joint):
    c = joint.commands
    return (c['kp'] > 0) | (c['kd'] > 0) | (c['command_effort'] != 0)


def fit_drive(joint):
    if joint.commands is None:
        return {'unavailable': 'the recording lacks each tick\'s command (recorded before 5852fe2)'}
    fit = (np.floor((joint.t - joint.t[0]) / BLOCK_S).astype(int) % 2) == 0
    on = enabled(joint)
    on &= np.concatenate([[False] * MAX_DELAY_TICKS, on[:-MAX_DELAY_TICKS]]) if len(on) > MAX_DELAY_TICKS else on
    best = None
    for d in np.arange(0., MAX_DELAY_TICKS + 1e-9, DELAY_STEP_TICKS):
        x = drive_torque(joint, d)
        rows = on & fit
        A = np.column_stack([x[rows], np.ones(rows.sum())])
        (k, c0), *_ = np.linalg.lstsq(A, joint.effort[rows], rcond=None)
        rms = float(np.sqrt(np.mean((joint.effort[rows] - k * x[rows] - c0) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, d, float(k), float(c0))
    rms, d, k, c0 = best
    rows = on & ~fit
    residual = joint.effort[rows] - k * drive_torque(joint, d)[rows] - c0
    return {'delay_ms': d * TICK_S * 1000, 'delay_ticks': d, 'scale': k, 'offset_nm': c0, 'fit_rms_nm': rms,
            'holdout_rms_nm': float(np.sqrt(np.mean(residual ** 2))), 'enabled_rows': int(on.sum())}


def _integrate(params, q, v, torque, static, h):
    """One substep for every window at once: Coulomb friction with sticking, as simulation.arm_plant."""
    net = torque - gravity(params, q) - params['viscous'] * v
    moving = np.abs(v) >= STICK_V
    stuck = ~moving & (np.abs(net) <= static)
    direction = np.where(moving, np.sign(v), np.sign(net))
    v = np.where(stuck, 0., v + (net - params['friction'] * direction) / params['inertia'] * h)
    return q + v * h, v


def _windows(joint, s, rows_ok, length_s):
    """Start rows (indices into s) of held-out windows whose ticks are all contiguous and allowed."""
    n = int(round(length_s / TICK_S))
    index = s['index']
    starts = []
    i = 0
    while i + n < len(index):
        span = index[i:i + n + 1]
        if span[-1] - span[0] == n and rows_ok[span].all():
            starts.append(i)
            i += n
        else:
            i += 1
    return np.array(starts, dtype=int), n


def _summary(errors):
    worst = np.max(np.abs(errors), axis=1)
    rms = np.sqrt(np.mean(errors ** 2, axis=1))
    return {'windows': len(errors), 'rms_mrad_p50': float(np.median(rms) * 1000),
            'max_mrad_p50': float(np.median(worst) * 1000), 'max_mrad_p95': float(np.percentile(worst, 95) * 1000)}


def replay_open(joint, s, mechanics, holdout_rows):
    """Held-out windows driven by the reported effort, against the measured position; the baseline coasts at the
    starting velocity."""
    params = mechanics['params']
    if params['inertia'] < MIN_INERTIA:
        return {'unavailable': f"inertia {params['inertia']:.2g} kg m2 is not a joint's"}
    starts, n = _windows(joint, s, holdout_rows, OPEN_WINDOW_S)
    if not len(starts):
        return {'unavailable': 'no held-out window'}
    rows = s['index'][starts]
    q, v = s['q'][starts].copy(), s['v'][starts].copy()
    static = max(params['friction'], 0.)
    sim, coast = np.empty((len(starts), n)), np.empty((len(starts), n))
    effort = aligned_effort(joint, s.get('shift_ticks', 0.))
    sampling = params.get('effort_sampling_s', 0.)
    for k in range(n):
        # The torque that moves the joint: the reported effort without the drive's sampling term.
        torque = effort[rows + k] - (sampling * joint.commands['kp'][rows + k] * v if sampling else 0.)
        for _ in range(SUBSTEPS):
            q, v = _integrate(params, q, v, torque, static, TICK_S / SUBSTEPS)
        sim[:, k] = q
        coast[:, k] = s['q'][starts] + s['v'][starts] * (k + 1) * TICK_S
    measured = np.stack([joint.q[rows + k + 1] for k in range(n)], axis=1)
    return {'window_s': OPEN_WINDOW_S, 'model': _summary(sim - measured), 'baseline_coast': _summary(coast - measured)}


def simulate_closed(joint, s, starts, n, params, delay_ticks, *, scale=1., offset=0.):
    """Each window's positions, simulated from its measured start under the recorded commands, d ticks late."""
    c, t = joint.commands, joint.t
    rows = s['index'][starts]
    # The step to row r applies the command of t_r - d, as the drive fit reads the torque reported at row r.
    late = t[rows[:, None] + 1 + np.arange(n)[None, :]] - delay_ticks * TICK_S
    q_star, v_star, ff, kp, kd = (np.interp(late, t, x) for x in (joint.command, c['command_velocity'],
                                                                  c['command_effort'], c['kp'], c['kd']))
    q, v = s['q'][starts].copy(), s['v'][starts].copy()
    static = max(params['friction'], 0.)
    sim = np.empty((len(starts), n))
    for k in range(n):
        for _ in range(SUBSTEPS):
            torque = scale * (kp[:, k] * (q_star[:, k] - q) + kd[:, k] * (v_star[:, k] - v) + ff[:, k]) + offset
            q, v = _integrate(params, q, v, torque, static, TICK_S / SUBSTEPS)
        sim[:, k] = q
    return sim


def _measured(joint, s, starts, n, source=None):
    rows = s['index'][starts]
    return (joint.q if source is None else source)[rows[:, None] + 1 + np.arange(n)[None, :]]


def replay_closed(joint, s, mechanics, drive, holdout_rows, *, scale=None, offset=None):
    """Held-out windows under the recorded commands through the drive (the simulator's job), against the
    measured position; the baseline is a joint that follows its command exactly (the virtual receiver)."""
    if joint.commands is None or 'unavailable' in drive:
        return {'unavailable': drive.get('unavailable', 'no drive fit')}
    starts, n = _windows(joint, s, holdout_rows & enabled(joint), CLOSED_WINDOW_S)
    if not len(starts):
        return {'unavailable': 'no held-out window with the drive enabled throughout'}
    sim = simulate_closed(joint, s, starts, n, mechanics['params'], drive['delay_ticks'],
                          scale=drive['scale'] if scale is None else scale,
                          offset=drive['offset_nm'] if offset is None else offset)
    measured = _measured(joint, s, starts, n)
    return {'window_s': CLOSED_WINDOW_S, 'model': _summary(sim - measured),
            'baseline_follows_command': _summary(_measured(joint, s, starts, n, joint.command) - measured)}


def fit_closed_loop(joint, s, mechanics, drive, fit_rows, holdout_rows):
    """Inertia, viscous and Coulomb friction, command delay and gravity scale from the measured positions alone
    (the drive applies its command law exactly: scale 1, no offset), then the held-out replay with them."""
    from scipy.optimize import least_squares
    if joint.commands is None or 'unavailable' in drive:
        return {'unavailable': drive.get('unavailable', 'no drive fit')}
    starts, n = _windows(joint, s, fit_rows & enabled(joint), CLOSED_WINDOW_S)
    if len(starts) < 10:
        return {'unavailable': f'{len(starts)} fit windows with the drive enabled throughout (10 needed)'}
    starts = starts[np.unique(np.linspace(0, len(starts) - 1, min(FIT_WINDOWS, len(starts))).astype(int))]
    measured = _measured(joint, s, starts, n)[:, FIT_STRIDE - 1::FIT_STRIDE]
    seed = mechanics['params']
    gravity_keys = ('gravity_sin', 'gravity_cos', 'gravity_offset')

    def model(x):
        params = dict(seed, inertia=math.exp(x[0]), viscous=x[1], friction=math.exp(x[2]),
                      **{key: x[4] * seed[key] for key in gravity_keys})
        return params, x[3]

    def residual(x):
        params, delay = model(x)
        return (simulate_closed(joint, s, starts, n, params, delay)[:, FIT_STRIDE - 1::FIT_STRIDE] - measured).ravel()
    x0 = [math.log(seed['inertia'] if seed['inertia'] > 0 else .5), seed['viscous'],
          math.log(max(seed['friction'], .01)), float(np.clip(drive['delay_ticks'], 0., MAX_DELAY_TICKS)), 1.]
    lower = [math.log(1e-3), -5., math.log(1e-3), 0., .5]
    upper = [math.log(50.), 50., math.log(50.), float(MAX_DELAY_TICKS), 1.5]
    x0 = list(np.clip(x0, lower, upper))
    result = least_squares(residual, x0, bounds=(lower, upper), diff_step=1e-3, x_scale='jac', max_nfev=200)
    params, delay = model(result.x)
    fitted = {'params': {key: params[key] for key in PARAMS}, 'delay_ticks': float(delay),
              'delay_ms': float(delay * TICK_S * 1000), 'gravity_scale': float(result.x[4]),
              'fit_windows': len(starts), 'fit_rms_mrad': float(np.sqrt(np.mean(result.fun ** 2)) * 1000),
              'converged': bool(result.success), 'evaluations': int(result.nfev)}
    fitted['replay_closed_loop'] = replay_closed(joint, s, {'params': params}, dict(drive, delay_ticks=delay),
                                                 holdout_rows, scale=1., offset=0.)
    return fitted


def fit_joint(session, name, *, servo=None):
    joint = session['joints'][name]
    others = {n: round(travel(j), 4) for n, j in session['joints'].items()
              if n != name and n.split('_')[0] == name.split('_')[0] and 'gripper' not in n}
    s = smoothed(joint)
    fit = blocks(joint.t, s['index'])
    holdout_rows = np.zeros(len(joint.t), dtype=bool)
    holdout_rows[s['index'][~fit]] = True
    s, at_edge = align(joint, s, fit)
    mechanics = fit_mechanics(joint, s, fit)
    if at_edge:
        mechanics['flags'].append('effort alignment at the edge of its search range')
    if any(v > OTHERS_MOVED_RAD for v in others.values()):
        mechanics['flags'].append('other joints moved: the one-joint gravity model does not hold')
    if np.ptp(joint.effort) < DEAD_EFFORT_NM:
        mechanics['flags'].insert(0, 'the drive reported a constant effort: no torque feedback to fit')
    drive = fit_drive(joint)
    report = {
        'joint': name,
        'excitation': {'travel_rad': travel(joint), 'speed_max': float(np.max(np.abs(s['v']))),
                       'acceleration_p99': float(np.percentile(np.abs(s['a']), 99)),
                       'effort_range_nm': [float(joint.effort.min()), float(joint.effort.max())],
                       'others_travel_rad': others},
        'rows': {'smoothed': len(s['index']), 'fit': int(fit.sum()), 'holdout': int((~fit).sum())},
        'mechanics': mechanics, 'drive': drive,
        'replay_open_loop': replay_open(joint, s, mechanics, holdout_rows),
        'replay_closed_loop': replay_closed(joint, s, mechanics, drive, holdout_rows),
    }
    fit_rows = np.zeros(len(joint.t), dtype=bool)
    fit_rows[s['index'][fit]] = True
    report['closed_loop_fit'] = fit_closed_loop(joint, s, mechanics, drive, fit_rows, holdout_rows)
    saved = (((servo or {}).get('joints') or {}).get(name) or {}).get('measured') or {}
    torque = saved.get('torque') or {}
    if torque:
        report['servo_sweep_fit'] = {'inertia': torque.get('inertia'), 'friction': torque.get('friction'),
                                     'gravity_fit': torque.get('gravity_fit')}
    return report


def fit_session(path, joints=None, *, servo=None, min_travel_rad=.2):
    session = load_session(path)
    names = joints or [n for n, j in session['joints'].items() if travel(j) >= min_travel_rad]
    return {'schema': 1, 'kind': 'joint_response_fit', 'provenance': session['provenance'],
            'model': {'mechanics': 'I a = tau - (A sin q + B cos q + C) - b v - f sgn(v)',
                      'drive': 'tau = k (kp (q* - q) + kd (v* - v) + tau_ff), command d late',
                      'cutoff_hz': CUTOFF_HZ, 'block_s': BLOCK_S, 'v_eps': V_EPS},
            'joints': {name: fit_joint(session, name, servo=servo) for name in names}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session', type=Path, help='a recording folder with columns.json and samples.f32')
    parser.add_argument('--joint', action='append', help='fit these joints (default: every joint that moved)')
    parser.add_argument('--servo', type=Path, help='a saved servo calibration to compare against')
    parser.add_argument('--output', type=Path, help='write the report here (never overwritten)')
    args = parser.parse_args()
    servo = json.loads(args.servo.read_text(encoding='utf-8')) if args.servo else None
    report = fit_session(args.session, args.joint, servo=servo)
    text = json.dumps(report, indent=1, allow_nan=True) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(text)
    for name, joint in report['joints'].items():
        p, h = joint['mechanics']['params'], joint['mechanics']['holdout']
        print(f"{name}: I {p['inertia']:.4f} kg m2, b {p['viscous']:.3f}, f {p['friction']:.3f} Nm, "
              f"gravity {p['gravity_sin']:.2f} sin q + {p['gravity_cos']:.2f} cos q + {p['gravity_offset']:.2f}; "
              f"held-out torque rms {h['rms_nm']:.3f} Nm (R2 {h['r2']:.3f}); {', '.join(joint['mechanics']['flags'])}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
