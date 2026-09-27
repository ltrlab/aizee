"""An attended live sweep that steps through servo settings joint by joint and measures the result.

For each joint in turn, with every other joint of the arm held at its reference
(hanging) pose at the profile's gains:
  1. Characterise: slow ramps out and back at the profile's gains, widening step by
     step toward the joint's usable range, holding at each end. Gives gravity and
     friction by angle (servo_analysis.torque_map).
  2. Stiffness x damping ratio: position steps and a short ramp at the reference
     for each rung of a stiffness ladder and each damping ratio (damping scales as
     sqrt(stiffness), times a multiplier). Low stiffness is tried only at the
     reference; a stiffness is held at the worst-gravity point only where the
     torque map says its sag stays within half the tracking bound.
  3. Speeds and braking at the chosen gains: trapezoidal moves at increasing
     speeds, then braking at increasing decelerations.
  4. Verify: the whole verified range at the chosen settings.

Every move is planned first and checked against the collision model along its
path (a pair that comes within the clearance and was not already that close at
the reference pose stops it). Each cycle the lead of a command over the measured
position is capped, and a guard abandons the task when any joint lags by more
than a share of its tracking bound (before the receiver's own fault), pushes too
hard or runs hot, or the collision check is not clear. The task's joint then
holds where it is at the profile's gains and returns to the reference, and the
sweep carries on with what the evidence allows.

The engine is pure: each cycle it takes the receiver rows recorded since the
last and returns the next command (a target and gains per joint). The loop that
owns the receiver (helm.tune) sends it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from fnmatch import fnmatchcase
import math

import numpy as np

from . import servo_analysis as analysis

DEFAULTS = {
    'characterize_speed': .15,
    'amplitudes': [.05, .1, .2, .35, .55, .8, 1.1, 1.5, 2.],
    'hold_s': .6,
    'rest_s': .3,
    'kp_ladder': [1., .7, .5, .35, .25, .18, .12],
    'damping': [1., .7, 1.4, .5],
    'step_fraction': .6,
    'grid_ramp_rad': .1,
    'grid_ramp_speed': .3,
    'speeds': [.2, .4, .6, .8, 1., 1.2, 1.6, 2., 2.5],
    'decelerations': [2., 4., 8., 16., 32.],
    'acceleration': 4.,
    'cruise_s': .3,
    'lead_fraction': .6,
    'guard_fraction': .75,
    'effort_fraction': .5,
    'temperature_c': 55.,
    'clearance_m': .02,
    'path_step_rad': .02,
}
JOINT_KEYS = ('kp_ceiling', 'speed_ceiling')
PERIOD_S = .04


def sweep_settings(profile, motor):
    """The sweep block's defaults, then every joint entry matching `motor` in order."""
    block = profile.get('sweep') or {}
    merged = dict(DEFAULTS)
    merged.update(block.get('defaults') or {})
    for entry in block.get('joints') or []:
        if fnmatchcase(motor, entry['match']):
            merged.update({k: v for k, v in entry.items() if k != 'match'})
    return merged


def damping_for(kp, *, reference, multiplier):
    """Damping at stiffness `kp` with the reference pair's damping ratio times `multiplier`
    (a second-order joint's damping ratio goes as kd / sqrt(kp))."""
    kp_ref, kd_ref = reference
    return kd_ref * math.sqrt(kp / kp_ref) * multiplier


def sweep_profile(profile, motors, gains):
    """The teleop profile for a sweep session: each joint's stiffness, damping and speed
    ceilings raised to what the sweep may try. `gains`: {motor: (hold_kp, hold_kd)}."""
    from copy import deepcopy
    result = deepcopy(profile)
    overrides = []
    for motor in motors:
        settings = sweep_settings(profile, motor)
        kp_ref, kd_ref = gains[motor]
        ceiling = settings.get('kp_ceiling') or kp_ref
        kd_ceiling = damping_for(ceiling, reference=(kp_ref, kd_ref), multiplier=max(settings['damping']))
        entry = {'match': motor, 'max_kp': max(ceiling, kp_ref), 'max_kd': max(kd_ceiling, kd_ref)}
        if settings.get('speed_ceiling'):
            entry['max_velocity'] = settings['speed_ceiling']
        overrides.append(entry)
    result['motion']['joints'] = list(result['motion']['joints']) + overrides
    return result


class Abort(Exception):
    """A guard abandoned the current task."""


class SweepStopped(Exception):
    """The operator asked the sweep to finish."""


@dataclass
class Segment:
    joint: str
    kind: str
    params: dict
    t0: float
    t1: float | None = None
    rows: np.ndarray | None = None


@dataclass
class JointReport:
    motor: str
    verified: dict = field(default_factory=lambda: {1: 0., -1: 0.})
    events: list = field(default_factory=list)
    torque: dict | None = None
    grid: list = field(default_factory=list)
    holds: dict = field(default_factory=dict)
    speeds: list = field(default_factory=list)
    decelerations: list = field(default_factory=list)
    noise: dict | None = None
    verification: dict | None = None
    recommendation: dict | None = None
    status: str = 'pending'


class Recording:
    """The receiver's rows, a rolling window for segments and guards, optionally appended to a file."""

    def __init__(self, names, *, window_s=90., sink=None):
        self.names = list(names)
        self.width = 1 + 4 * len(self.names)
        self.window_ms = window_s * 1000
        self.chunks = deque()
        self.sink = sink  # A binary file: float32 rows.
        self.rows_total = self.dropped = 0
        self.last_t = None

    def add(self, rows, dropped=0):
        rows = np.asarray(rows, dtype=float).reshape(-1, self.width)
        self.dropped += int(dropped)
        if not len(rows):
            return rows
        if self.last_t is not None:
            rows = rows[rows[:, 0] > self.last_t]  # Never the same tick twice.
            if not len(rows):
                return rows
        self.chunks.append(rows)
        self.rows_total += len(rows)
        self.last_t = float(rows[-1, 0])
        while self.chunks and self.chunks[0][-1, 0] < self.last_t - self.window_ms:
            self.chunks.popleft()
        if self.sink is not None:
            self.sink.write(rows.astype(np.float32).tobytes())
        return rows

    def between(self, t0, t1):
        if not self.chunks:
            return np.empty((0, self.width))
        rows = np.concatenate(list(self.chunks))
        return rows[(rows[:, 0] >= t0) & (rows[:, 0] <= t1)]


def result_rows(result, names):
    """A receiver result's recorder rows, or one row from its status and feedback (the virtual
    harness, which steps once per request and keeps no record)."""
    recent = result.get('recent')
    if recent:
        return recent, result.get('recent_dropped', 0)
    status, feedback = result.get('status') or {}, result.get('feedback') or []
    if not feedback or 'now_ms' not in status:
        return [], 0
    row = [float(status['now_ms'])]
    commands = status.get('joints') or [{}] * len(feedback)
    for sample, command in zip(feedback, commands):
        row += [sample['position'], sample['velocity'], sample['effort'], command.get('position', sample['position'])]
    return [row], 0


class PathChecker:
    """Checks a planned move of one joint against the collision model, relative to the reference pose.

    Keep-out shells of neighbouring parts already touch at the reference; a pair
    counts only when it comes within the clearance and was not that close there.
    """

    def __init__(self, backend, joint_map, *, home, fixed, cad_limits, clearance_m, step_rad, scope='all'):
        from .guarded_config import to_cad
        self.backend, self.joint_map, self.home = backend, joint_map, dict(home)
        self.fixed, self.cad_limits, self.to_cad = dict(fixed), cad_limits, to_cad
        self.clearance, self.step, self.scope = clearance_m, step_rad, scope
        self.baseline = set(self._near(self.home))

    def _near(self, motors):
        positions = dict(self.fixed)
        positions.update(self.to_cad(self.joint_map, motors, self.cad_limits))
        result = self.backend.proximity(positions, threshold=self.clearance, scope=self.scope)
        return {tuple(sorted(pair['shapes'])): pair['distance_m'] for pair in result['near_pairs']}

    def clear(self, motor, start, end):
        """(True, None) if the move stays clear, else (False, 'shape and shape at q rad')."""
        count = max(1, math.ceil(abs(end - start) / self.step))
        pose = dict(self.home)
        for k in range(1, count + 1):
            pose[motor] = start + (end - start) * k / count
            new = {pair: d for pair, d in self._near(pose).items() if pair not in self.baseline}
            if new:
                pair, distance = min(new.items(), key=lambda item: item[1])
                return False, f'{pair[0]} within {distance * 1000:.0f} mm of {pair[1]} at {pose[motor]:+.2f} rad'
        return True, None


class SweepEngine:
    """The sweep's plan and guards, one command per cycle. See the module docstring."""

    def __init__(self, joints, *, home, gains, settings, sweep, checker=None, window_s=.12, verdict_s=.08,
                 labels=None, period_s=PERIOD_S):
        self.limits = {joint['name']: joint for joint in joints}  # The receiver's joint limits.
        self.names = [joint['name'] for joint in joints]
        self.home, self.gains = dict(home), dict(gains)  # gains: {motor: (hold_kp, hold_kd)}.
        self.settings = settings  # {motor: sweep_settings(...)}.
        self.sweep = list(sweep)  # The motors to sweep, in order.
        self.checker, self.window_s, self.verdict_s = checker, window_s, verdict_s
        self.labels = dict(labels or {})
        self.period = period_s
        self.recording = Recording(self.names)
        self.reports = {motor: JointReport(motor) for motor in self.sweep}
        self.segments = []
        self.targets = dict(self.home)  # Commanded position per joint (before the lead cap).
        self.velocity = {name: 0. for name in self.names}  # Commanded speed per joint.
        self.test_gains = {}  # {motor: (kp, kd)} overriding the profile's while a test runs.
        self.current = None  # The motor under test.
        self.task = 'starting'
        self.recovering = False
        self.stopping = False
        self.done = False
        self.events = []
        self.latest = None
        self.temperatures = {}
        self.collision_clear = True
        self._plan = self._run()
        self._now = None

    # -- observations and guards ------------------------------------------------------

    def observe(self, rows, *, dropped=0, temperatures=None, collision_clear=None):
        new = self.recording.add(rows, dropped)
        if len(new):
            self.latest = new[-1]
            self._fresh = new
        else:
            self._fresh = None
        if temperatures:
            self.temperatures = dict(temperatures)
        if collision_clear is not None:
            self.collision_clear = collision_clear
        return new

    def measured(self, motor):
        if self.latest is None:
            return None
        return float(self.latest[1 + 4 * self.names.index(motor)])

    def _guard(self):
        """The reason to abandon the running task, or None."""
        rows = getattr(self, '_fresh', None)
        if rows is None or self.recovering or self.current is None:
            return None
        for i, name in enumerate(self.names):
            limit, settings = self.limits[name], self.settings.get(name) or self.settings[self.current]
            lag = np.max(np.abs(rows[:, 4 + 4 * i] - rows[:, 1 + 4 * i]))
            bound = settings['guard_fraction'] * limit['max_tracking_error']
            if lag > bound:
                return f'{self._label(name)} lagged {lag:.3f} rad (guard {bound:.3f})'
            effort = np.max(np.abs(rows[:, 3 + 4 * i]))
            if effort > settings['effort_fraction'] * limit['max_effort']:
                return f"{self._label(name)} pushed {effort:.1f} Nm (guard {settings['effort_fraction'] * limit['max_effort']:.1f})"
            temperature = self.temperatures.get(name)
            if temperature is not None and temperature > settings['temperature_c']:
                return f"{self._label(name)} at {temperature:.0f} C (guard {settings['temperature_c']:.0f})"
        if not self.collision_clear:
            return 'the collision check of the commanded pose was not clear'
        return None

    def _label(self, motor):
        return self.labels.get(motor, motor)

    # -- command ----------------------------------------------------------------------

    def command(self, now):
        """The next command: {'targets': [...], 'gains': [(kp, kd), ...], 'task': str}, or None when done."""
        self._now = now
        if self.done:
            return None
        reason = self._guard()
        try:
            if reason is not None:
                self._event(reason)
                self._plan.throw(Abort(reason))
            elif self.stopping and not getattr(self, '_stop_sent', False):
                self._stop_sent = True
                self._plan.throw(SweepStopped())
            else:
                next(self._plan)
        except StopIteration:
            self.done = True
            self.task = 'stopped' if self.stopping else 'finished'
            return None
        return self._compose()

    def stop(self):
        """Finish: the current joint returns to the reference, then the sweep ends."""
        self.stopping = True

    def _compose(self):
        targets, gains = [], []
        for name in self.names:
            target = self.targets[name]
            limit = self.limits[name]
            q = self.measured(name)
            settings = self.settings.get(name) or self.settings[self.current or self.sweep[0]]
            if q is not None and name == self.current:
                lead = settings['lead_fraction'] * limit['max_tracking_error']
                target = min(q + lead, max(q - lead, target))  # Never lead the joint by more than the cap.
            target = min(limit['upper'], max(limit['lower'], target))
            targets.append(target)
            gains.append(self.test_gains.get(name, self.gains[name]))
        return {'targets': targets, 'gains': gains, 'task': self.task}

    def _event(self, text):
        entry = {'motor': self.current, 'task': self.task, 'text': text}
        self.events.append(entry)
        if self.current in self.reports:
            self.reports[self.current].events.append(entry)

    # -- motion primitives (generators: one yield per cycle) ---------------------------------

    def _cycles(self, seconds):
        return max(1, round(seconds / self.period))

    def _hold(self, motor, seconds):
        for _ in range(self._cycles(seconds)):
            self.velocity[motor] = 0.
            yield

    def _go(self, motor, target, speed, *, acceleration=None, deceleration=None, stop=True):
        """Move the commanded position to `target` at `speed`, accelerating and braking at the given
        rates (instant when None). Ends at rest unless stop=False (then at cruise speed)."""
        start = self.targets[motor]
        direction = math.copysign(1, target - start) if target != start else 1.
        v = self.velocity[motor] * direction if self.velocity[motor] * direction > 0 else 0.
        while True:
            remaining = (target - self.targets[motor]) * direction
            if remaining <= 1e-9:
                break
            dt = self.period
            braking = deceleration if deceleration else math.inf
            if stop and deceleration and v * v / (2 * deceleration) >= remaining:
                v = max(0., v - braking * dt)
                if v == 0.:
                    v = min(speed, math.sqrt(2 * deceleration * remaining))  # Creep the last bit.
            elif acceleration:
                v = min(speed, v + acceleration * dt)
            else:
                v = speed
            step = min(remaining, v * dt)
            self.targets[motor] += step * direction
            self.velocity[motor] = v * direction if (not stop or step < remaining) else 0.
            yield
        self.targets[motor] = target
        if stop:
            self.velocity[motor] = 0.

    def _measure(self, motor, kind, generator, **params):
        segment = Segment(motor, kind, params, t0=self.recording.last_t or 0.)
        yield from generator
        segment.t1 = self.recording.last_t or segment.t0
        segment.rows = self.recording.between(segment.t0, segment.t1)
        self.segments.append(segment)
        return segment

    def _recover(self, motor):
        """After a guard: hold the joint where it is at the profile's gains, then return it."""
        self.recovering = True
        try:
            self.test_gains.pop(motor, None)
            q = self.measured(motor)
            if q is not None:
                self.targets[motor] = q
            self.velocity[motor] = 0.
            yield from self._hold(motor, .5)
            yield from self._go(motor, self.home[motor], self.settings[motor]['characterize_speed'])
            yield from self._hold(motor, self.settings[motor]['rest_s'])
        finally:
            self.recovering = False

    # -- the plan ---------------------------------------------------------------------------

    def _run(self):
        try:
            for motor in self.sweep:
                self.current = motor
                report = self.reports[motor]
                report.status = 'running'
                try:
                    yield from self._joint(motor, report)
                    report.status = 'done'
                except SweepStopped:
                    report.status = 'stopped'
                    self.recovering = True
                    yield from self._recover(motor)
                    raise
                finally:
                    self.test_gains.pop(motor, None)
            self.task = 'finished'
            self.current = None
        except SweepStopped:
            self.task = 'stopped'
            self.current = None

    def _task(self, label):
        self.task = label

    def _guarded(self, motor, generator):
        """Run a task; on a guard, recover and report False."""
        try:
            yield from generator
            return True
        except Abort:
            yield from self._recover(motor)
            return False

    def _allowed(self, motor, speed):
        """The positions a joint may reach moving at `speed`: its limits less the receiver's stopping
        reserve at that speed (a stop plans braking from the measured speed)."""
        limit = self.limits[motor]
        if limit.get('gripper'):
            return limit['lower'], limit['upper']
        reserve = (limit['max_tracking_error'] + speed * self.window_s
                   + speed ** 2 / (2 * limit['stop_acceleration']) + speed * self.verdict_s)
        return limit['lower'] + reserve, limit['upper'] - reserve

    def _clear(self, motor, start, end):
        if self.checker is None:
            return True, None
        return self.checker.clear(motor, start, end)

    def _joint(self, motor, report):
        settings, home = self.settings[motor], self.home[motor]
        slow = settings['characterize_speed']
        # Start from the reference.
        self._task(f'{self._label(motor)}: to the reference')
        yield from self._go(motor, home, slow)
        yield from self._hold(motor, settings['rest_s'])
        # 1. Characterise, widening both ways.
        char = []
        for direction in (1, -1):
            low, high = self._allowed(motor, slow)
            edge = high if direction > 0 else low
            for amplitude in settings['amplitudes']:
                target = home + direction * amplitude
                last = (target - edge) * direction >= 0
                if last:
                    target = edge
                if (target - home) * direction <= report.verified[direction] + 1e-6:
                    break
                clear, why = self._clear(motor, home, target)
                if not clear:
                    self._event(f'stopped widening: {why}')
                    break
                self._task(f'{self._label(motor)}: characterise {direction * (target - home):+.2f} rad')
                segments = []

                def out_and_back(target=target, direction=direction, segments=segments):
                    segments.append((yield from self._measure(motor, 'ramp', self._go(motor, target, slow),
                                                              direction=direction, speed=slow)))
                    segments.append((yield from self._measure(motor, 'hold', self._hold(motor, settings['hold_s']),
                                                              position=target)))
                    segments.append((yield from self._measure(motor, 'ramp', self._go(motor, home, slow),
                                                              direction=-direction, speed=slow)))
                    yield from self._hold(motor, settings['rest_s'])
                ok = yield from self._guarded(motor, out_and_back())
                char += [s for s in segments if s.kind == 'ramp']
                if not ok:
                    break
                report.verified[direction] = (target - home) * direction
                if last:
                    break
        index = self.names.index(motor)
        report.torque = analysis.torque_map([{'rows': s.rows, 'direction': s.params['direction'],
                                              'speed': s.params['speed']} for s in char if s.rows is not None], index)
        # 2. Stiffness x damping at the reference, stiff first; stop lowering once a rung fails every ratio.
        kp_ref, kd_ref = self.gains[motor]
        ceiling = settings.get('kp_ceiling') or kp_ref
        limit = self.limits[motor]
        bound, lead = limit['max_tracking_error'], settings['lead_fraction'] * limit['max_tracking_error']
        step = settings['step_fraction'] * lead
        load = (report.torque.get('gravity_max') or 0.) + (report.torque.get('friction') or 0.)
        ladder = sorted({round(f * ceiling, 3) for f in settings['kp_ladder']}, reverse=True)
        for kp in ladder:
            passed_any = False
            for multiplier in settings['damping']:
                kd = min(limit['max_kd'], damping_for(kp, reference=(kp_ref, kd_ref), multiplier=multiplier))
                self._task(f'{self._label(motor)}: stiffness {kp:g}, damping {kd:.2f}')
                entry = {'kp': kp, 'kd': kd, 'multiplier': multiplier, 'steps': [], 'ramp': None, 'failed': None}
                ok = yield from self._guarded(motor, self._grid(motor, kp, kd, step, load, entry))
                if not ok:
                    entry['failed'] = report.events[-1]['text'] if report.events else 'guard'
                report.grid.append(entry)
                passed_any = passed_any or (ok and all(s and s['overshoot'] < 1. for s in entry['steps']))
            if not passed_any:
                self._event(f'stiffness {kp:g} failed with every damping ratio; not trying lower')
                break
        # Holding at the worst-gravity point, only where the predicted sag is within half the bound.
        worst = report.torque.get('gravity_worst_at')
        if worst is not None and report.torque.get('gravity_max'):
            for kp in sorted({e['kp'] for e in report.grid if not e['failed']}):
                if load / kp > .5 * bound:
                    continue
                best = self._best_damping(report, kp)
                if best is None:
                    continue
                self._task(f'{self._label(motor)}: hold at {worst:+.2f} rad with stiffness {kp:g}')
                holding = {}
                ok = yield from self._guarded(motor, self._hold_test(motor, worst, kp, best, holding))
                report.holds[kp] = holding.get('error') if ok else None
        gains = analysis.recommend(motor, torque=report.torque, grid=report.grid, holds=report.holds, speeds=[],
                                   decelerations=[], bound=bound, tolerance=self._tolerance(motor),
                                   profile_gains={'kp': kp_ref, 'kd': kd_ref})
        chosen = (gains['hold_kp'], gains['hold_kd'])
        # 3. Speeds, then braking, at the chosen gains.
        self.test_gains[motor] = chosen
        span_low = home - report.verified[-1]
        span_high = home + report.verified[1]
        followed = None
        for speed in [s for s in settings['speeds'] if s <= limit['max_velocity'] + 1e-9]:
            low, high = self._allowed(motor, speed)
            low, high = max(low, span_low), min(high, span_high)
            accel = settings['acceleration']
            need = speed * speed / accel + speed * settings['cruise_s']
            if high - low < need:
                self._event(f'{speed:g} rad/s needs {need:.2f} rad of verified room; stopping the speed ladder')
                break
            self._task(f'{self._label(motor)}: {speed:g} rad/s')
            results = []
            ok = yield from self._guarded(motor, self._speed_test(motor, low, high, speed, accel, results))
            report.speeds += results
            if not ok or any(r is None or r['max_lag'] > .5 * bound for r in results):
                break
            followed = speed
        if followed:
            low, high = self._allowed(motor, followed)
            low, high = max(low, span_low), min(high, span_high)
            for deceleration in settings['decelerations']:
                self._task(f'{self._label(motor)}: braking at {deceleration:g} rad/s2 from {followed:g} rad/s')
                results = []
                ok = yield from self._guarded(motor, self._brake_test(motor, low, high, followed, deceleration,
                                                                      settings['acceleration'], results))
                report.decelerations += results
                if not ok or any(r is None or r['max_lag'] > .5 * bound for r in results):
                    break
        report.recommendation = analysis.recommend(
            motor, torque=report.torque, grid=report.grid, holds=report.holds, speeds=report.speeds,
            decelerations=report.decelerations, bound=bound, tolerance=self._tolerance(motor),
            profile_gains={'kp': kp_ref, 'kd': kd_ref})
        # 4. Verify the whole verified range at the recommended settings.
        recommendation = report.recommendation
        self.test_gains[motor] = (recommendation['hold_kp'], recommendation['hold_kd'])
        speed = min(recommendation['max_velocity'] or slow, limit['max_velocity'])
        self._task(f'{self._label(motor)}: verify')
        verification = {}
        ok = yield from self._guarded(motor, self._verify(motor, span_low, span_high, speed, verification))
        verification['passed'] = bool(ok)
        report.verification = verification
        self.test_gains.pop(motor, None)
        self._task(f'{self._label(motor)}: back to the reference')
        yield from self._go(motor, home, slow)
        yield from self._hold(motor, settings['rest_s'])

    def _tolerance(self, motor):
        return self.settings[motor].get('tolerance') or .03

    def _best_damping(self, report, kp):
        entries = [e for e in report.grid if e['kp'] == kp and not e['failed'] and e['steps']
                   and all(s and s['settle_s'] is not None for s in e['steps'])]
        if not entries:
            return None
        best = min(entries, key=lambda e: (max(s['overshoot'] for s in e['steps']) > .1,
                                           max(s['settle_s'] for s in e['steps'])))
        return best['kd']

    # -- tests ---------------------------------------------------------------------------------

    def _grid(self, motor, kp, kd, step, load, entry):
        home, settings = self.home[motor], self.settings[motor]
        index = self.names.index(motor)
        self.test_gains[motor] = (kp, kd)
        try:
            yield from self._hold(motor, settings['rest_s'])
            noise = yield from self._measure(motor, 'hold', self._hold(motor, settings['rest_s']), position=home)
            if entry.get('noise') is None and noise.rows is not None and len(noise.rows):
                entry['noise'] = analysis.speed_noise(noise.rows, index)
            for target in (home + step, home, home - step, home):
                start = self.targets[motor]

                def jump(target=target):
                    self.targets[motor] = target
                    self.velocity[motor] = 0.
                    yield from self._hold(motor, settings['hold_s'])
                segment = yield from self._measure(motor, 'step', jump(), start=start, target=target)
                entry['steps'].append(analysis.step_response(segment.rows, index, target=target, start=start)
                                      if segment.rows is not None and len(segment.rows) else None)
            # A short ramp, no farther than the torque map lets this stiffness hold within half the lead cap.
            limit = self.limits[motor]
            reach = settings['grid_ramp_rad']
            if load > 0:
                reach = min(reach, .5 * settings['lead_fraction'] * limit['max_tracking_error'] * kp / load * reach
                            if load * reach > 0 else reach)
            low, high = self._allowed(motor, settings['grid_ramp_speed'])
            target = min(high, home + reach)
            if target - home > 2 * step:
                clear, why = self._clear(motor, home, target)
                if clear:
                    speed = settings['grid_ramp_speed']
                    ramp = yield from self._measure(motor, 'ramp', self._go(motor, target, speed), speed=speed,
                                                    direction=1)
                    yield from self._go(motor, home, speed)
                    yield from self._hold(motor, settings['rest_s'])
                    if ramp.rows is not None and len(ramp.rows):
                        entry['ramp'] = analysis.ramp_lag(ramp.rows, index, speed=speed)
        finally:
            self.test_gains[motor] = self.gains[motor]

    def _hold_test(self, motor, position, kp, kd, result):
        settings = self.settings[motor]
        index = self.names.index(motor)
        clear, why = self._clear(motor, self.home[motor], position)
        if not clear:
            self._event(f'hold test skipped: {why}')
            return
        yield from self._go(motor, position, settings['characterize_speed'])
        self.test_gains[motor] = (kp, kd)
        try:
            yield from self._hold(motor, .3)
            segment = yield from self._measure(motor, 'hold', self._hold(motor, 1.), position=position, kp=kp)
            if segment.rows is not None and len(segment.rows):
                q = analysis.column(segment.rows, index, analysis.POSITION)
                result['error'] = float(np.mean(np.abs(position - q[len(q) // 2:])))
        finally:
            self.test_gains[motor] = self.gains[motor]
        yield from self._go(motor, self.home[motor], settings['characterize_speed'])
        yield from self._hold(motor, settings['rest_s'])

    def _speed_test(self, motor, low, high, speed, acceleration, results):
        index = self.names.index(motor)
        settings = self.settings[motor]
        for end in (high, low):
            clear, why = self._clear(motor, self.targets[motor], end)
            if not clear:
                self._event(f'speed test shortened: {why}')
                return
            segment = yield from self._measure(motor, 'ramp', self._go(motor, end, speed, acceleration=acceleration,
                                                                       deceleration=acceleration),
                                               speed=speed, direction=1 if end == high else -1)
            yield from self._hold(motor, settings['rest_s'])
            results.append(analysis.ramp_lag(segment.rows, index, speed=speed)
                           if segment.rows is not None and len(segment.rows) else None)
        yield from self._go(motor, self.home[motor], settings['characterize_speed'])

    def _brake_test(self, motor, low, high, speed, deceleration, acceleration, results):
        index = self.names.index(motor)
        settings = self.settings[motor]
        yield from self._go(motor, low, settings['characterize_speed'])
        yield from self._hold(motor, settings['rest_s'])
        braking_distance = speed * speed / (2 * deceleration)
        brake_at = high - braking_distance
        if brake_at <= low + speed * speed / (2 * acceleration):
            self._event(f'no room to reach {speed:g} rad/s before braking at {deceleration:g} rad/s2')
            return
        clear, why = self._clear(motor, low, high)
        if not clear:
            self._event(f'braking test skipped: {why}')
            return
        yield from self._go(motor, brake_at, speed, acceleration=acceleration, stop=False)
        segment = yield from self._measure(motor, 'brake', self._go(motor, high, speed, deceleration=deceleration),
                                           speed=speed, deceleration=deceleration, stop_at=high)
        yield from self._hold(motor, settings['hold_s'])
        if segment.rows is not None and len(segment.rows):
            stop = analysis.braking(self.recording.between(segment.t0, self.recording.last_t), index, stop_at=high)
            if stop is not None:
                stop['deceleration'] = deceleration
            results.append(stop)
        yield from self._go(motor, self.home[motor], settings['characterize_speed'])
        yield from self._hold(motor, settings['rest_s'])

    def _verify(self, motor, low, high, speed, result):
        index = self.names.index(motor)
        settings = self.settings[motor]
        lags = []
        for end in (high, low, self.home[motor]):
            clear, why = self._clear(motor, self.targets[motor], end)
            if not clear:
                self._event(f'verification shortened: {why}')
                break
            segment = yield from self._measure(motor, 'ramp', self._go(motor, end, speed,
                                                                       acceleration=settings['acceleration'],
                                                                       deceleration=settings['acceleration']),
                                               speed=speed)
            yield from self._hold(motor, settings['hold_s'])
            if segment.rows is not None and len(segment.rows):
                lag = np.abs(analysis.column(segment.rows, index, analysis.COMMAND)
                             - analysis.column(segment.rows, index, analysis.POSITION))
                lags.append(float(lag.max()))
        result.update(speed=speed, max_lag=max(lags) if lags else None, span=(low, high))

    # -- views -----------------------------------------------------------------------------

    def progress(self):
        done = sum(report.status in ('done', 'stopped') for report in self.reports.values())
        return {'task': self.task, 'motor': self.current, 'joints_done': done, 'joints': len(self.sweep),
                'recovering': self.recovering, 'events': self.events[-8:], 'rows': self.recording.rows_total,
                'dropped': self.recording.dropped}

    def summary(self):
        """Per joint: the recommendation, the measurements behind it and what happened."""
        result = {}
        for motor, report in self.reports.items():
            result[motor] = {
                'status': report.status, 'verified': {'up': report.verified[1], 'down': report.verified[-1]},
                'torque': None if report.torque is None else {k: report.torque.get(k) for k in
                                                              ('gravity_max', 'gravity_worst_at', 'friction', 'span')},
                'grid': [{k: e.get(k) for k in ('kp', 'kd', 'multiplier', 'failed')}
                         | {'overshoot': max((s['overshoot'] for s in e['steps'] if s), default=None),
                            'settle_s': max((s['settle_s'] or 0. for s in e['steps'] if s), default=None),
                            'ramp_lag': (e.get('ramp') or {}).get('max_lag')} for e in report.grid],
                'holds': {str(kp): error for kp, error in report.holds.items()},
                'speeds': report.speeds, 'decelerations': report.decelerations,
                'verification': report.verification, 'recommendation': report.recommendation,
                'events': report.events}
        return result
