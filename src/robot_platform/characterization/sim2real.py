"""Compare the simulator's response with the real arm's, on real recordings only; read-only.

The simulator is the one `robot-platform helm --sim` drives (simulation.minerva_sim): an ArmPlant joint per motor
with its Tune sweep's inertia, friction and gravity, and the rehearsals' nominal values where a sweep measured
none. It is built here from the same files Helm uses: the calibration profile, the servo calibration beside it and
the runs folder's confirmed motor directions.

Each 1 s window of a real recording (since release 5852fe2, which records every drive's whole command per tick) is
replayed through that plant one joint at a time:
- the joint starts from its measured state (position and speed of the 8 Hz-filtered measured position);
- every 2.5 ms tick it takes the command the real drive took: position, velocity, feed-forward effort, kp and kd;
- the arm's other joints stay at their measured positions, so the gravity coupling is the real one.
The simulated position is compared with the measured one. A joint that follows its command exactly (Helm's
virtual receiver) is the baseline.

Calibration fits that plant's parameters (inertia, Coulomb and breakaway friction, damping, a gravity scale and a
command delay) to real responses on alternate 10 s blocks and scores them on the held-out blocks. A parameter is
as good as the real response it reproduces: nothing here is checked against simulated ground truth.

Windows overlapping a disabled drive, a recorder gap, or the last WINDOW_FAULT_S before a FAULT event are left out.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from robot_platform.characterization.response_fit import BLOCK_S, TICK_S, load_session, smoothed
from robot_platform.simulation.arm_plant import SUBSTEPS

WINDOW_S = 1.
STRIDE = 4  # Every 4th tick of a fit window enters the calibration residual.
FIT_WINDOWS = 150  # At most this many fit windows per joint, spread evenly.
WINDOW_FAULT_S = 1.  # Windows ending this close before a FAULT event are left out, and all after it.
STICK_V = 1e-4  # As ArmPlant.
MOVING_SPEED = .05  # rad/s: a window whose command moves faster than this somewhere counts as moving.
MAX_DELAY_TICKS = 8.
CALIBRATED = ('inertia', 'friction', 'static', 'viscous', 'gravity_scale', 'delay_ticks')


def simulator(calibration_profile, runs, *, servo=None):
    """The plant helm --sim builds: per motor its ArmPlant spec and gravity terms, and their sources' identity."""
    from robot_platform.control.gravity import gravity_model
    from robot_platform.control.joint_calibration import load_plan
    from robot_platform.control.servo_calibration import load_servo, servo_path
    from robot_platform.ops.check import direction_session
    from robot_platform.simulation.minerva_sim import joint_specs
    calibration_profile = Path(calibration_profile)
    _plan, _paths, device, reference, model, _pose, bindings = load_plan(calibration_profile)
    session = direction_session(Path(runs), device['asset_id'], bindings)
    if session is None or 'preview_mapping' not in session:
        raise ValueError(f'No confirmed direction session for this calibration in {runs}')
    servo_file = Path(servo) if servo else servo_path(calibration_profile, device['asset_id'])
    calibration = load_servo(servo_file)
    names = [m['name'] for m in reference['motors']]
    specs, nominal = joint_specs(names, calibration)
    terms = gravity_model(model, session['preview_mapping'], reference['motors'], (calibration or {}).get('gravity'))
    return {'specs': {s['name']: s for s in specs}, 'terms': terms, 'nominal': nominal,
            'identity': {'calibration_profile': str(calibration_profile.resolve()),
                         'servo': str(servo_file.resolve()),
                         'servo_sha256': hashlib.sha256(servo_file.read_bytes()).hexdigest(),
                         'directions': session['bindings']}}


def _json_lines(path):
    """Each JSON object line; a blank, cut-off or zero-filled line (a session's last write when it ended uncleanly,
    as live-20261015T223850Z's trace) is skipped."""
    with Path(path).open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if isinstance(row, dict):
                yield row


def fault_times_ms(path):
    """Receiver times (ms) of the session's FAULT events, mapped from the host clock through the direct trace.
    [-inf] when the session faulted but never had a request accepted to place the fault by."""
    path = Path(path)
    events = path / 'events.jsonl'
    if not events.is_file():
        return []
    faults = [row['t'] for row in _json_lines(events) if row.get('lifecycle') == 'FAULT']
    if not faults:
        return []
    latency = path / 'direct-latency.jsonl'
    pairs = []
    if latency.is_file():
        for row in _json_lines(latency):
            if (row.get('receiver_accepted_ms') is not None and row.get('started_at') is not None
                    and row.get('completed_at') is not None):
                pairs.append(((row['started_at'] + row['completed_at']) / 2, row['receiver_accepted_ms']))
    if not pairs:
        return [-math.inf]  # A fault at an unknown time: no window of the recording can be trusted.
    host, receiver = np.array(pairs).T
    offset = float(np.median(receiver - host * 1000))
    return sorted(t * 1000 + offset for t in faults)


class Replay:
    """One joint of one recording, ready to replay: its commands, measured response and the arm's other joints."""

    def __init__(self, session, motor, terms, faults_ms=()):
        joints = session['joints']
        self.motor, self.joint = motor, joints[motor]
        if self.joint.commands is None:
            raise ValueError(f'{motor}: the recording lacks each tick\'s command (made before release 5852fe2)')
        self.t = self.joint.t
        self.terms = terms or ()
        missing = [m for term in self.terms for m in term[1] if m not in joints]
        if missing:
            raise ValueError(f'{motor}: gravity needs {sorted(set(missing))}, not in this recording')
        self.context = {m: joints[m].q for term in self.terms for m in term[1] if m != motor}
        s = smoothed(self.joint)
        self.q0 = np.full(len(self.t), np.nan)
        self.v0 = np.full(len(self.t), np.nan)
        self.q0[s['index']], self.v0[s['index']] = s['q'], s['v']
        c = self.joint.commands
        self.commands = (self.joint.command, c['command_velocity'], c['command_effort'], c['kp'], c['kd'])
        self.faults_ms = sorted(faults_ms)

    def windows(self, length_s=WINDOW_S):
        """Start rows of non-overlapping windows with the drive commanded and the clock contiguous throughout, a
        known starting state, and nothing near or after a fault; and whether each falls in a fit block."""
        n = int(round(length_s / TICK_S))
        on = (self.commands[3] > 0) | (self.commands[4] > 0)
        contiguous = np.concatenate([[True], np.diff(self.t) < 1.5 * TICK_S])
        first_fault = (self.faults_ms[0] / 1000 - WINDOW_FAULT_S) if self.faults_ms else math.inf
        starts, r = [], int(MAX_DELAY_TICKS) + 1
        while r + n < len(self.t):
            span = slice(r, r + n + 1)
            if (on[span].all() and contiguous[r + 1:r + n + 1].all() and np.isfinite(self.q0[r])
                    and self.t[r + n] < first_fault):
                starts.append(r)
                r += n
            else:
                r += 1
        starts = np.array(starts, dtype=int)
        fit = (np.floor((self.t[starts] - self.t[0]) / BLOCK_S).astype(int) % 2) == 0
        return starts, fit, n

    def _late(self, x, rows, delay):
        """x at fractional rows (rows - delay), linearly between recorded ticks."""
        position = rows - delay
        low = np.floor(position).astype(int)
        frac = position - low
        return x[low] * (1 - frac) + x[np.minimum(low + 1, len(x) - 1)] * frac

    def simulate(self, starts, n, params, delay=0.):
        """Positions [windows, n] after each tick: ArmPlant's equations (simulation.arm_plant.ArmPlant.step),
        vectorised over windows. The step to row r + 1 takes the command recorded at row r, as the simulator's
        plant takes the receiver's latest command each tick; `delay` (ticks) takes it that much older."""
        rows = starts[:, None] + np.arange(n)[None, :]
        q_star, v_star, ff, kp, kd = (self._late(x, rows, delay) for x in self.commands)
        context = {m: q[rows] for m, q in self.context.items()}
        q, v = self.q0[starts].copy(), self.v0[starts].copy()
        inertia, friction = params['inertia'], params['friction']
        static, viscous, scale = params['static'], params['viscous'], params.get('gravity_scale', 1.)
        h = TICK_S / SUBSTEPS
        out = np.empty((len(starts), n))
        for k in range(n):
            for _ in range(SUBSTEPS):
                holding = 0.
                for coefficient, angle in self.terms:
                    holding = holding + coefficient * np.sin(sum(m * (q if motor == self.motor else context[motor][:, k])
                                                                 for motor, m in angle.items()))
                torque = kp[:, k] * (q_star[:, k] - q) + kd[:, k] * (v_star[:, k] - v) + ff[:, k]
                net = torque - scale * holding - viscous * v
                moving = np.abs(v) >= STICK_V
                stuck = ~moving & (np.abs(net) <= static)
                direction = np.copysign(1., np.where(moving, v, net))
                v = np.where(stuck, 0., v + (net - friction * direction) / inertia * h)
                q = q + v * h
            out[:, k] = q
        return out

    def measured(self, starts, n):
        return self.joint.q[starts[:, None] + 1 + np.arange(n)[None, :]]

    def follows(self, starts, n):
        """A joint that is exactly where its command puts it (Helm's virtual receiver)."""
        return self.joint.command[starts[:, None] + 1 + np.arange(n)[None, :]]

    def moving(self, starts, n):
        speed = self.commands[1][starts[:, None] + np.arange(n)[None, :]]
        step = np.abs(np.diff(self.joint.command[starts[:, None] + np.arange(n + 1)[None, :]], axis=1)) / TICK_S
        return (np.abs(speed) > MOVING_SPEED).any(axis=1) | (step > MOVING_SPEED).any(axis=1)


def score(errors, moving):
    """Position error (simulated or baseline minus measured) over windows, in mrad."""
    worst = np.max(np.abs(errors), axis=1) * 1000
    rms = np.sqrt(np.mean(errors ** 2, axis=1)) * 1000

    def part(mask):
        if not mask.any():
            return None
        return {'windows': int(mask.sum()), 'rms_mrad_p50': float(np.median(rms[mask])),
                'max_mrad_p50': float(np.median(worst[mask])), 'max_mrad_p95': float(np.percentile(worst[mask], 95))}
    return {'all': part(np.ones(len(errors), dtype=bool)), 'moving': part(moving), 'holding': part(~moving)}


def calibrate(replays, base):
    """Least-squares fit of CALIBRATED on the fit windows of every replay of one joint."""
    from scipy.optimize import least_squares
    usable = [(replay, starts, n) for replay, starts, n in replays if len(starts)]
    total = sum(len(starts) for _, starts, _ in usable)
    if total < 10:
        return {'unavailable': f'{total} fit windows (10 needed)'}
    if total > FIT_WINDOWS:  # Spread evenly over every recording's fit windows.
        usable = [(replay, starts[np.unique(np.linspace(0, len(starts) - 1,
                                                        max(2, round(FIT_WINDOWS * len(starts) / total))).astype(int))], n)
                  for replay, starts, n in usable]
    sets = [(replay, starts, n, replay.measured(starts, n)[:, STRIDE - 1::STRIDE]) for replay, starts, n in usable]

    def unpack(x):
        friction = math.exp(x[1])
        return {'inertia': math.exp(x[0]), 'friction': friction, 'static': friction + math.exp(x[2]),
                'viscous': x[3], 'gravity_scale': x[4]}, x[5]

    def residual(x):
        params, delay = unpack(x)
        return np.concatenate([(r.simulate(s, n, params, delay)[:, STRIDE - 1::STRIDE] - m).ravel()
                               for r, s, n, m in sets])
    x0 = [math.log(base['inertia']), math.log(max(base['friction'], 1e-3)),
          math.log(max(base['static'] - base['friction'], 1e-3)), max(base['viscous'], 0.), 1., 0.]
    lower = [math.log(1e-4), math.log(1e-3), math.log(1e-4), 0., .5, 0.]
    upper = [math.log(20.), math.log(20.), math.log(20.), 100., 1.5, MAX_DELAY_TICKS]
    result = least_squares(residual, np.clip(x0, lower, upper), bounds=(lower, upper), diff_step=1e-3,
                           x_scale='jac', max_nfev=300)
    params, delay = unpack(result.x)
    at_bound = [name for name, value, lo, hi in zip(('inertia', 'friction', 'static', 'viscous', 'gravity_scale',
                                                     'delay_ticks'), result.x, lower, upper)
                if abs(value - lo) < 1e-6 or abs(value - hi) < 1e-6]
    return {'params': dict(params, delay_ticks=float(delay), delay_ms=float(delay * TICK_S * 1000)),
            'fit_windows': int(sum(len(s) for _, s, _, _ in sets)),
            'fit_rms_mrad': float(np.sqrt(np.mean(result.fun ** 2)) * 1000), 'converged': bool(result.success),
            'at_bounds': at_bound, 'evaluations': int(result.nfev)}


def compare(recordings, simulator_, motors=None, *, calibrate_joints=True):
    """Per motor: the current simulator and a calibrated one against the real responses' held-out windows."""
    sessions = []
    for path in recordings:
        session = load_session(path)
        sessions.append((Path(path), session, fault_times_ms(path)))
    names = motors or [m for m in simulator_['specs'] if any(m in s['joints'] for _, s, _ in sessions)]
    report = {'schema': 1, 'kind': 'sim2real_comparison',
              'recordings': [dict(s['provenance'], faults_receiver_ms=['unplaced: every window left out'
                                                                       if math.isinf(x) else x for x in f])
                             for _, s, f in sessions],
              'simulator': dict(simulator_['identity'], nominal=simulator_['nominal']),
              'method': {'window_s': WINDOW_S, 'block_s': BLOCK_S, 'tick_s': TICK_S, 'plant': 'ArmPlant equations',
                         'context': "the arm's other joints at their measured positions"}, 'joints': {}}
    for motor in names:
        spec = simulator_['specs'][motor]
        current = {k: spec[k] for k in ('inertia', 'friction', 'static', 'viscous')}
        replays, held = [], []
        travel = 0.
        for path, session, faults in sessions:
            if motor not in session['joints'] or session['joints'][motor].commands is None:
                continue
            replay = Replay(session, motor, simulator_['terms'].get(motor), faults)
            starts, fit, n = replay.windows()
            travel = max(travel, float(np.ptp(session['joints'][motor].q)))
            replays.append((replay, starts[fit], n))
            held.append((replay, starts[~fit], n))
        entry = {'current_params': current, 'nominal': simulator_['nominal'].get(motor, []), 'travel_rad': travel,
                 'windows': {'fit': int(sum(len(s) for _, s, _ in replays)),
                             'holdout': int(sum(len(s) for _, s, _ in held))}}
        if not entry['windows']['holdout']:
            entry['unavailable'] = 'no held-out window with the drive commanded'
            report['joints'][motor] = entry
            continue

        def evaluate(params, delay=0.):
            errors, moving = [], []
            for replay, starts, n in held:
                if len(starts):
                    errors.append(replay.simulate(starts, n, params, delay) - replay.measured(starts, n))
                    moving.append(replay.moving(starts, n))
            return score(np.concatenate(errors), np.concatenate(moving))
        moving = np.concatenate([r.moving(s, n) for r, s, n in held if len(s)])
        baseline = np.concatenate([r.follows(s, n) - r.measured(s, n) for r, s, n in held if len(s)])
        entry['follows_command'] = score(baseline, moving)
        entry['current_simulator'] = evaluate(dict(current, gravity_scale=1.))
        if calibrate_joints and not motor.endswith('gripper'):
            fitted = calibrate(replays, current)
            if 'params' in fitted:
                fitted['holdout'] = evaluate(fitted['params'], fitted['params']['delay_ticks'])
            entry['calibrated'] = fitted
        elif motor.endswith('gripper'):
            entry['calibrated'] = {'unavailable': 'grippers grasp objects the simulator does not have'}
        report['joints'][motor] = entry
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recordings', type=Path, nargs='+', help='recording folders (columns.json, samples.f32)')
    parser.add_argument('--profile', type=Path, default=Path('config/devices/minerva-joint-calibration.json'))
    parser.add_argument('--runs', type=Path, required=True, help="Helm's runs folder with the confirmed directions")
    parser.add_argument('--servo', type=Path, help='servo calibration (default: beside the profile, as Helm)')
    parser.add_argument('--joint', action='append', help='only these motors')
    parser.add_argument('--no-calibration', action='store_true')
    parser.add_argument('--output', type=Path, help='write the report here (never overwritten)')
    args = parser.parse_args()
    report = compare(args.recordings, simulator(args.profile, args.runs, servo=args.servo), args.joint,
                     calibrate_joints=not args.no_calibration)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(report, indent=1, allow_nan=True) + '\n')
    for motor, entry in report['joints'].items():
        if 'unavailable' in entry:
            print(f'{motor}: {entry["unavailable"]}')
            continue

        def mrad(part):
            return f"{part['all']['max_mrad_p50']:.1f}" if part and part.get('all') else '-'
        line = (f"{motor}: {entry['windows']['holdout']} held-out windows; max error p50 (mrad): follows command "
                f"{mrad(entry['follows_command'])}, current sim {mrad(entry['current_simulator'])}")
        calibrated = entry.get('calibrated') or {}
        if 'holdout' in calibrated:
            p = calibrated['params']
            line += (f", calibrated {mrad(calibrated['holdout'])} (I {p['inertia']:.3f}, f {p['friction']:.2f}, "
                     f"static {p['static']:.2f}, b {p['viscous']:.2f}, g x{p['gravity_scale']:.2f}, "
                     f"delay {p['delay_ms']:.1f} ms{', at bounds ' + ', '.join(calibrated['at_bounds']) if calibrated['at_bounds'] else ''})")
        print(line)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
