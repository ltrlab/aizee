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
import time

import numpy as np

from . import servo_analysis as analysis
from robot_platform.control.approach import command_speed, end_margin

DEFAULTS = {
    'characterize_speed': .15,
    'amplitudes': [.05, .1, .2, .35, .55, .8, 1.1, 1.5, 2.],
    'hold_s': .6,
    'rest_s': .3,
    'kp_ladder': [1., .7, .5, .35, .25, .18, .12],
    'damping': [1., .7, 1.4, .5, .35, .25],
    'step_fraction': .6,
    'grid_ramp_rad': .1,
    'grid_ramp_speed': .3,
    'speeds': [.2, .3, .4, .5, .6, .8, 1., 1.2, 1.6, 2., 2.5],
    'decelerations': [2., 4., 8., 16., 32.],  # Speeds step up to each joint's cap (or speed_ceiling, opt-in).
    'acceleration': 4.,
    'cruise_s': .3,
    'lead_fraction': .6,
    'guard_fraction': .75,
    'effort_fraction': .5,
    'temperature_c': 55.,
    'clearance_m': .02,
    'path_step_rad': .02,
    'check_span_rad': .3,  # Verify mode: the out-and-back for friction, each way.
    'usable_floor': .5,  # A loosened bound or cap must leave at least this share of the swept travel usable.
    # Headroom under the receiver's overspeed limit (its speed cap, 3 readings in a row): a joint overshoots the
    # speed it follows, and a stiff, lightly damped step peaks at about its size times its natural frequency.
    'speed_headroom': .8,  # Speeds (tested, verified, checked) at most this share of the cap.
    'max_step_rad': .03,  # Step tests no larger than this (J6 stepped 0.029 rad cleanly up to kp 24).
}
JOINT_KEYS = ('kp_ceiling', 'speed_ceiling', 'bound_ceiling')
PERIOD_S = .04
STALL_S = 3.  # A joint that cannot follow its plan (or come to rest) for this long ends the task.
REST_BAND_RAD = .003  # At rest: the joint moved less than this over REST_S.
REST_S = .2
GAIN_RAMP_S = .3  # Stiffness and damping change over this long, never in one step.
CHECK_BUDGET_S = .01  # Collision queries per cycle; a path is checked over several cycles.


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


def sweep_profile(profile, motors, gains, *, travel=None, sweep=None):
    """The teleop profile for a sweep session: each joint holds at `gains` ({motor: (hold_kp,
    hold_kd)}, the profile's or a saved calibration's) with its stiffness and damping ceilings
    raised to what the sweep may try. The swept joints (`sweep`, default all) also get their
    tracking bound and speed cap raised toward bound_ceiling and speed_ceiling; given `travel`
    ({motor: its row of derive_receiver's report}), only as far as the usable range stays at least
    usable_floor of the swept travel, since the receiver's stopping reserve grows with both."""
    from copy import deepcopy
    from robot_platform.control.guarded_config import motion_settings
    result = deepcopy(profile)
    overrides = []
    for motor in motors:
        settings = sweep_settings(profile, motor)
        kp_ref, kd_ref = gains[motor]
        ceiling = settings.get('kp_ceiling') or kp_ref
        kd_ceiling = damping_for(ceiling, reference=(kp_ref, kd_ref), multiplier=max(settings['damping']))
        entry = {'match': motor, 'hold_kp': kp_ref, 'hold_kd': kd_ref,
                 'max_kp': max(ceiling, kp_ref), 'max_kd': max(kd_ceiling, kd_ref)}
        if sweep is None or motor in sweep:
            try:
                motion = motion_settings(profile['motion'], motor)
            except ValueError:  # A partial profile: only the ceilings apply, as they are.
                if settings.get('speed_ceiling'):
                    entry['max_velocity'] = settings['speed_ceiling']
                if settings.get('bound_ceiling'):
                    entry['max_tracking_error'] = settings['bound_ceiling']
            else:
                bound, cap = loosened(motion, settings, (travel or {}).get(motor), profile['receiver'])
                if bound != motion['max_tracking_error']:
                    entry['max_tracking_error'] = bound
                if cap != motion['max_velocity']:
                    entry['max_velocity'] = cap
        overrides.append(entry)
    result['motion']['joints'] = list(result['motion']['joints']) + overrides
    return result


def loosened(motion, settings, row, receiver):
    """(tracking bound, speed cap) for a sweep session: raised toward the joint's bound_ceiling
    (first: it is the joint's push, as the command's lead is capped at a share of it) and
    speed_ceiling, never lowered. With `row` (the joint's derived report row), each only as far as
    its usable range stays at least usable_floor of its swept travel; a gripper has no reserve."""
    from robot_platform.control.guarded_config import usable_margin
    bound0, cap0 = motion['max_tracking_error'], motion['max_velocity']
    bound_ceiling = max(bound0, settings.get('bound_ceiling') or bound0)
    cap_ceiling = max(cap0, settings.get('speed_ceiling') or cap0)
    if row is None or motion.get('gripper') is True:
        return bound_ceiling, cap_ceiling
    # The speed cap no longer costs travel (joints slow near the ends); the bound still does.
    lower, upper = row['operational']
    floor = settings['usable_floor'] * (row['swept'][1] - row['swept'][0])

    def usable(bound):
        return upper - lower - 2 * usable_margin(dict(motion, max_tracking_error=bound), receiver)
    if usable(bound0) < floor:
        return bound0, cap_ceiling  # Already under the floor: no bound to spend.
    steps = max(0, int(round((bound_ceiling - bound0) / .01)))
    bound = max((b for b in [bound0 + k * (bound_ceiling - bound0) / steps for k in range(1, steps + 1)]
                 if usable(b) >= floor), default=bound0) if steps else bound0
    return round(bound, 4), cap_ceiling


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
    check: dict | None = None
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
        chunks = [chunk for chunk in self.chunks if chunk[-1, 0] >= t0 and chunk[0, 0] <= t1]
        if not chunks:
            return np.empty((0, self.width))
        rows = np.concatenate(chunks)
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

    def samples(self, start, end):
        count = max(1, math.ceil(abs(end - start) / self.step))
        return [start + (end - start) * k / count for k in range(1, count + 1)]

    def check(self, motor, position):
        """None if the pose with `motor` at `position` (the rest at the reference) is clear, else why."""
        pose = dict(self.home, **{motor: position})
        new = {pair: d for pair, d in self._near(pose).items() if pair not in self.baseline}
        if not new:
            return None
        pair, distance = min(new.items(), key=lambda item: item[1])
        return f'{pair[0]} within {distance * 1000:.0f} mm of {pair[1]} at {position:+.2f} rad'

    def clear(self, motor, start, end):
        """(True, None) if the move stays clear, else (False, why), in one go (tests and tools)."""
        for position in self.samples(start, end):
            why = self.check(motor, position)
            if why:
                return False, why
        return True, None


class SweepEngine:
    """The sweep's plan and guards, one command per cycle. See the module docstring."""

    def __init__(self, joints, *, home, gains, settings, sweep, checker=None, window_s=.12, verdict_s=.08,
                 labels=None, period_s=PERIOD_S, mode='tune', calibration=None, travel=None, receiver=None):
        """mode 'tune' steps the settings; 'verify' (Minerva Check) checks each joint at the settings it has
        (`gains`, and each joint's speed cap), against its saved sweep (`calibration`: {motor: entry}) if any."""
        if mode not in ('tune', 'verify'):
            raise ValueError('The sweep tunes or verifies')
        self.mode, self.calibration = mode, dict(calibration or {})
        # For the results: each joint's derived report row today, and the receiver's reserve settings.
        self.travel, self.receiver = dict(travel or {}), receiver
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

    def begin(self, positions):
        """Start from the measured pose ({motor: rad}); the sweep brings every joint to the reference slowly."""
        self.targets.update({name: float(q) for name, q in positions.items() if name in self.targets})

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
            low, high = self._allowed(name)
            target = min(high, max(low, target))
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
        waited = 0.
        while True:
            remaining = (target - self.targets[motor]) * direction
            if remaining <= 1e-9:
                break
            dt = self.period
            behind = self._behind(motor, direction)
            if behind is not None and behind >= self._lead(motor):
                # The command is pinned at its lead cap: the plan waits for the joint rather than run
                # ahead of it (a plan far ahead turns into a lunge the moment the joint gets more push).
                waited += dt
                if waited >= STALL_S:
                    why = (f'{self._label(motor)} could not follow its plan: {behind:.3f} rad behind for '
                           f'{STALL_S:g} s')
                    if self.recovering:
                        self._event(why + '; holding where it is')
                        return
                    self._event(why)
                    raise Abort(why)
                yield
                continue
            waited = 0.
            braking = deceleration if deceleration else math.inf
            if stop and deceleration and v * v / (2 * deceleration) >= remaining:
                v = max(0., v - braking * dt)
                if v == 0.:
                    v = min(speed, math.sqrt(2 * deceleration * remaining))  # Creep the last bit.
            elif acceleration:
                v = min(speed, v + acceleration * dt)
            else:
                v = speed
            v = min(v, self._toward_end(motor, direction))  # Slower toward an end: the braking curve.
            step = min(remaining, v * dt)
            self.targets[motor] += step * direction
            self.velocity[motor] = v * direction if (not stop or step < remaining) else 0.
            yield
        self.targets[motor] = target
        if stop:
            self.velocity[motor] = 0.

    def _lead(self, motor):
        """How far a command may lead the measured position of the joint under test."""
        return (self.settings.get(motor) or self.settings[self.sweep[0]])['lead_fraction'] \
            * self.limits[motor]['max_tracking_error']

    def _behind(self, motor, direction):
        """How far the plan is ahead of the joint in the direction of travel (the joint under test only,
        whose command is capped), or None."""
        q = self.measured(motor) if motor == self.current else None
        return None if q is None else (self.targets[motor] - q) * direction

    def _arrive(self, motor):
        """Hold until the joint has come to rest within its lead of its commanded position."""
        waited, history = 0., []
        window = self._cycles(REST_S) + 1
        while True:
            q = self.measured(motor)
            if q is not None:
                history = (history + [q])[-window:]
                if (len(history) == window and max(history) - min(history) < REST_BAND_RAD
                        and abs(self.targets[motor] - q) <= self._lead(motor)):
                    return
            waited += self.period
            if waited >= STALL_S:
                why = (f'{self._label(motor)} did not come to rest at its commanded position within '
                       f'{STALL_S:g} s')
                self._event(why)
                raise Abort(why)
            self.velocity[motor] = 0.
            yield

    def _stiffen(self, motor, gains, seconds=GAIN_RAMP_S):
        """Change the joint's stiffness and damping once it is at rest, over `seconds`: a joint that is
        still moving or behind its command lunges when its stiffness jumps."""
        yield from self._arrive(motor)
        old = self.test_gains.get(motor, self.gains[motor])
        steps = self._cycles(seconds)
        for k in range(1, steps + 1):
            f = k / steps
            self.test_gains[motor] = (old[0] + (gains[0] - old[0]) * f, old[1] + (gains[1] - old[1]) * f)
            self.velocity[motor] = 0.
            yield
        self.test_gains[motor] = tuple(gains)

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

    def _home_all(self):
        """Every joint to its reference together, each at its characterisation speed."""
        self._task('to the reference pose')
        moves = [self._go(name, self.home[name], (self.settings.get(name) or self.settings[self.sweep[0]])
                          ['characterize_speed']) for name in self.names]
        while moves:
            for move in list(moves):
                try:
                    next(move)
                except StopIteration:
                    moves.remove(move)
            if moves:
                yield
        yield from self._hold(self.sweep[0], .5)

    def _run(self):
        try:
            yield from self._home_all()
            for motor in self.sweep:
                self.current = motor
                report = self.reports[motor]
                report.status = 'running'
                try:
                    yield from (self._verify_joint(motor, report) if self.mode == 'verify'
                                else self._joint(motor, report))
                    report.status = 'done'
                except Abort:
                    report.status = 'stopped'  # Its reason is in the events; the next joint goes on.
                    yield from self._recover(motor)
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

    def _allowed(self, motor, speed=None):
        """Where a joint's target and measured position may be: its limits less the stopping margin of its
        approach floor (control.approach; the receiver refuses an envelope that cannot hold the joint's
        reserve at the speed it may approach a limit). This is teleop's usable range."""
        limit = self.limits[motor]
        if limit.get('gripper'):
            return limit['lower'], limit['upper']
        margin = end_margin(limit, self.window_s, self.verdict_s)
        return limit['lower'] + margin, limit['upper'] - margin

    def _toward_end(self, motor, direction):
        """How fast the plan may move `motor` in `direction` from its commanded position: within the
        receiver's braking curve toward that end, with headroom."""
        limit = self.limits[motor]
        return command_speed(limit, self.window_s, self.targets[motor], direction, limit['max_velocity'])

    def _clear(self, motor, start, end):
        """(clear, why) for a planned move, checking about CHECK_BUDGET_S of poses per cycle and holding
        in between, so the loop keeps the receiver's lease and collision verdict fresh."""
        if self.checker is None:
            return True, None
        deadline = time.perf_counter() + CHECK_BUDGET_S
        for position in self.checker.samples(start, end):
            why = self.checker.check(motor, position)
            if why:
                return False, why
            if time.perf_counter() >= deadline:
                self.velocity[motor] = 0.
                yield
                deadline = time.perf_counter() + CHECK_BUDGET_S
        return True, None

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
                clear, why = yield from self._clear(motor, home, target)
                if not clear:
                    self._event(f'stopped widening: {why}')
                    break
                self._task(f'{self._label(motor)}: characterise {target - home:+.2f} rad')
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
        step = min(settings['step_fraction'] * lead, settings['max_step_rad'])
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
        yield from self._stiffen(motor, chosen)
        span_low = home - report.verified[-1]
        span_high = home + report.verified[1]
        followed = None
        for speed in [s for s in settings['speeds'] if s <= settings['speed_headroom'] * limit['max_velocity'] + 1e-9]:
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
            if not ok or any(r['max_lag'] is None or r['max_lag'] > .5 * bound
                             or (r['speed'] or 0.) < .85 * speed for r in results):
                break  # Lagged, or could not reach the speed (the plan waited for it).
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
        # 4. Verify the whole verified range at the recommended settings. The pass across the range is the
        # last word on lag: over half the bound, it steps down to the next slower tested speed (twice at most).
        recommendation = report.recommendation
        yield from self._stiffen(motor, (recommendation['hold_kp'], recommendation['hold_kd']))
        top = recommendation['max_velocity']
        tries = ([s for s in settings['speeds'] if s <= min(top, limit['max_velocity']) + 1e-9][::-1][:3]
                 if top else []) or [slow]
        for speed in tries:
            self._task(f'{self._label(motor)}: verify at {speed:g} rad/s')
            verification = {}
            ok = yield from self._guarded(motor, self._verify(motor, span_low, span_high, speed, verification))
            lag = verification.get('max_lag')
            if not ok or lag is None:
                verification['reason'] = ('a guard stopped the pass across the range' if not ok
                                          else 'no lag measured on the pass across the range')
                ok = False
                break
            if lag <= .5 * bound:
                break
            ok = False
            verification['reason'] = (f'the pass across the range at {speed:g} rad/s lagged {lag:.3f} rad, over '
                                      f'half the tracking bound ({.5 * bound:.3f})')
            recommendation['notes'].append(verification['reason'])
        if ok and top:
            if speed < top - 1e-9:
                recommendation['notes'].append(f'speed stepped down to {speed:g} rad/s by the pass across the range')
                recommendation['max_velocity'] = speed
            lags = [r['max_lag'] for r in report.speeds
                    if r.get('max_lag') is not None and abs((r.get('target_speed') or 0.) - speed) < 1e-9]
            tracking = min(bound, max(.03, 2 * max(lags + [lag])))
            if tracking > (recommendation['max_tracking_error'] or 0.) + 1e-9:
                recommendation['notes'].append(f'the pass across the range lagged {lag:.3f} rad; tracking bound '
                                               f'{tracking:.3f}')
            recommendation['max_tracking_error'] = tracking
        verification['passed'] = bool(ok)
        report.verification = verification
        self.test_gains.pop(motor, None)
        self._task(f'{self._label(motor)}: back to the reference')
        yield from self._go(motor, home, slow)
        yield from self._hold(motor, settings['rest_s'])

    def _verify_joint(self, motor, report):
        """Minerva Check's live sweep of one joint: a short out-and-back for friction, then a pass across
        its range at its speed, judged against half its tracking bound and its saved sweep."""
        settings, home, limit = self.settings[motor], self.home[motor], self.limits[motor]
        slow, bound = settings['characterize_speed'], limit['max_tracking_error']
        saved = self.calibration.get(motor)
        measured = (saved or {}).get('measured') or {}
        self._task(f'{self._label(motor)}: to the reference')
        yield from self._go(motor, home, slow)
        yield from self._hold(motor, settings['rest_s'])
        low, high = self._allowed(motor)
        char = []
        for direction in (1, -1):
            room = (high - home) if direction > 0 else (home - low)
            amplitude = min(settings['check_span_rad'], .8 * room)
            if amplitude < .02:
                continue
            target = home + direction * amplitude
            clear, why = yield from self._clear(motor, home, target)
            if not clear:
                self._event(f'friction check skipped: {why}')
                continue
            self._task(f'{self._label(motor)}: friction {direction * amplitude:+.2f} rad')
            segments = []

            def out_and_back(target=target, direction=direction, segments=segments):
                segments.append((yield from self._measure(motor, 'ramp', self._go(motor, target, slow),
                                                          direction=direction, speed=slow)))
                yield from self._hold(motor, settings['rest_s'])
                segments.append((yield from self._measure(motor, 'ramp', self._go(motor, home, slow),
                                                          direction=-direction, speed=slow)))
                yield from self._hold(motor, settings['rest_s'])
            if (yield from self._guarded(motor, out_and_back())):
                report.verified[direction] = amplitude
            char += segments
        index = self.names.index(motor)
        report.torque = analysis.torque_map([{'rows': s.rows, 'direction': s.params['direction'],
                                              'speed': s.params['speed']} for s in char if s.rows is not None], index)
        # The pass: the range the joint was swept over (or the friction span), at its speed.
        swept = measured.get('verified') or {}
        up = min(swept.get('up', report.verified[1]), high - home)
        down = min(swept.get('down', report.verified[-1]), home - low)
        speed = min((saved or {}).get('max_velocity') or .5 * limit['max_velocity'],
                    settings['speed_headroom'] * limit['max_velocity'])
        self._task(f'{self._label(motor)}: pass at {speed:.2f} rad/s')
        verification = {}
        ok = yield from self._guarded(motor, self._verify(motor, home - down, home + up, speed, verification))
        verification['passed'] = bool(ok)
        report.verification = verification
        lag, friction = verification.get('max_lag'), report.torque.get('friction')
        reasons = ['guard: ' + event['text'] for event in report.events]
        if lag is None:
            reasons.append('no pass across the range')
        elif lag > .5 * bound:
            reasons.append(f'lagged {lag:.3f} rad (half the tracking bound is {.5 * bound:.3f})')
        swept_lag = (measured.get('verification') or {}).get('max_lag')
        if lag is not None and swept_lag is not None and lag > 1.5 * swept_lag + .01:
            reasons.append(f'lagged {lag:.3f} rad, up from {swept_lag:.3f} when swept')
        swept_friction = (measured.get('torque') or {}).get('friction')
        if friction is not None and swept_friction is not None and friction > 1.5 * swept_friction + .1:
            reasons.append(f'friction {friction:.2f} Nm, up from {swept_friction:.2f} when swept')
        report.check = {'passed': not reasons, 'reasons': reasons, 'lag': lag, 'friction': friction,
                        'speed': speed, 'calibrated': saved is not None}
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
        yield from self._stiffen(motor, (kp, kd))
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
                clear, why = yield from self._clear(motor, home, target)
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
        clear, why = yield from self._clear(motor, self.home[motor], position)
        if not clear:
            self._event(f'hold test skipped: {why}')
            return
        yield from self._go(motor, position, settings['characterize_speed'])
        yield from self._stiffen(motor, (kp, kd))
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
            clear, why = yield from self._clear(motor, self.targets[motor], end)
            if not clear:
                self._event(f'speed test shortened: {why}')
                return
            segment = yield from self._measure(motor, 'ramp', self._go(motor, end, speed, acceleration=acceleration,
                                                                       deceleration=acceleration),
                                               speed=speed, direction=1 if end == high else -1)
            yield from self._hold(motor, settings['rest_s'])
            lag = (analysis.ramp_lag(segment.rows, index, speed=speed)
                   if segment.rows is not None and len(segment.rows) else None)
            results.append(dict(lag or {'speed': None, 'mean_lag': None, 'max_lag': None, 'peak_effort': None},
                                target_speed=speed, direction=1 if end == high else -1))
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
        clear, why = yield from self._clear(motor, low, high)
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
            clear, why = yield from self._clear(motor, self.targets[motor], end)
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

    def _usable(self, motor, report):
        """The joint's usable range today and at the recommended settings (rad), and its swept travel."""
        row, rec = self.travel.get(motor), report.recommendation
        if row is None or self.receiver is None or rec is None or self.limits[motor].get('gripper'):
            return None
        from robot_platform.control.guarded_config import usable_margin
        limit = self.limits[motor]
        lower, upper = row['operational']
        settings = {'max_tracking_error': rec.get('max_tracking_error') or limit['max_tracking_error'],
                    'max_velocity': rec.get('max_velocity') or limit['max_velocity'],
                    'stop_acceleration': rec.get('stop_acceleration') or limit['stop_acceleration']}
        return {'swept': row['swept'][1] - row['swept'][0], 'today': row['usable'][1] - row['usable'][0],
                'recommended': max(0., upper - lower - 2 * usable_margin(settings, self.receiver))}

    def summary(self):
        """Per joint: the recommendation, the measurements behind it and what happened."""
        result = {}
        for motor, report in self.reports.items():
            result[motor] = {
                'label': self._label(motor), 'current': {'hold_kp': self.gains[motor][0], 'hold_kd': self.gains[motor][1],
                                                         **{key: self.limits[motor][key] for key in (
                                                             'max_tracking_error', 'max_velocity', 'stop_acceleration')},
                                                         **((self.travel.get(motor) or {}).get('today') or {})},
                'status': report.status, 'verified': {'up': report.verified[1], 'down': report.verified[-1]},
                'torque': None if report.torque is None else {k: report.torque.get(k) for k in
                                                              ('gravity_max', 'gravity_worst_at', 'friction', 'span')},
                'grid': [{k: e.get(k) for k in ('kp', 'kd', 'multiplier', 'failed')}
                         | {'overshoot': max((s['overshoot'] for s in e['steps'] if s), default=None),
                            # None: a step never came to rest.
                            'settle_s': (max(s['settle_s'] for s in e['steps'])
                                         if e['steps'] and all(s and s['settle_s'] is not None for s in e['steps'])
                                         else None),
                            'steady_error': max((s['steady_error'] for s in e['steps']
                                                 if s and s['steady_error'] is not None), default=None),
                            'ramp_lag': (e.get('ramp') or {}).get('max_lag'),
                            'why_not': (analysis.grid_check(e, tolerance=self._tolerance(motor),
                                                            limit=.5 * self.limits[motor]['max_tracking_error'])
                                        or (None, None))[1]} for e in report.grid],
                'holds': {str(kp): error for kp, error in report.holds.items()},
                'speeds': report.speeds, 'decelerations': report.decelerations,
                'verification': report.verification, 'recommendation': report.recommendation,
                'check': report.check, 'usable': self._usable(motor, report), 'events': report.events}
        return result
