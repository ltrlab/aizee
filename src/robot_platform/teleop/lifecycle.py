"""Pure GUI teleop phases and bounded absolute targets; no device ownership.

Proposals are not acknowledgements or permission to enable hardware. A guarded
adapter must certify and acknowledge every batch, and call fault() on failure.
"""
from copy import deepcopy
from collections import deque
import math
import time

from robot_platform.control.approach import command_step, end_margin, friction_feedforward, lead_toward, ramp_effort
from robot_platform.control.gravity import gravity_effort


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)



# Settling speed is judged over at least this long, whatever the loop rate: one 20 ms cycle's change in position
# reads the drives' position steps and noise as twice the speed of a 40 ms one, and alignment never settled.
SETTLE_WINDOW_S = .04

class TeleopLifecycle:
    """Shared semantic operations, independent of each GUI's key bindings."""
    def __init__(self, joints, *, feedback_max_age_s=.03, target_max_age_s=.05,
                 max_skew_s=.02, max_step_s=.08, settle_time_s=.1, max_source_misses=2,
                 allow_repeated_source=False, clock=time.perf_counter):
        if not isinstance(joints, (list, tuple)) or not joints:
            raise ValueError('A complete ordered joint configuration is required')
        self._joints = tuple(deepcopy(joints))
        names = []
        for joint in self._joints:
            if not isinstance(joint, dict) or not isinstance(joint.get('name'), str) or not joint['name']:
                raise ValueError('Every joint needs a unique name')
            names.append(joint['name'])
            fields = ('lower', 'upper', 'hold_kp', 'hold_kd', 'max_velocity', 'align_speed',
                      'shutdown_speed', 'max_lead', 'tolerance', 'settled_velocity', 'zero_position')
            if any(not _finite(joint.get(key)) for key in fields):
                raise ValueError('Joint limits, gains, speeds and tolerances must be finite')
            if (joint['lower'] >= joint['upper'] or not joint['lower'] <= joint['zero_position'] <= joint['upper']
                    or joint['hold_kp'] < 0 or joint['hold_kd'] < 0
                    or any(joint[key] <= 0 for key in ('max_velocity', 'align_speed', 'shutdown_speed',
                                                     'max_lead', 'tolerance', 'settled_velocity'))
                    or max(joint['align_speed'], joint['shutdown_speed']) > joint['max_velocity']):
                raise ValueError('Invalid joint interval, reference, gains or motion bounds')
        if len(set(names)) != len(names):
            raise ValueError('Joint names must be unique')
        for joint in self._joints:
            for term in joint.get('gravity') or ():
                if (len(term) != 2 or not _finite(term[0]) or not isinstance(term[1], dict)
                        or any(name not in names or term[1][name] not in (-1, 1) for name in term[1])):
                    raise ValueError('Gravity terms must be finite and name this lifecycle\'s joints')
        self.names = tuple(names)
        values = (feedback_max_age_s, target_max_age_s, max_skew_s, max_step_s)
        if (any(not _finite(value) or value <= 0 for value in values)
                or not _finite(settle_time_s) or settle_time_s < 0
                or max_skew_s > min(feedback_max_age_s, target_max_age_s)
                or type(allow_repeated_source) is not bool
                or type(max_source_misses) is not int or not 0 <= max_source_misses <= 10):
            raise ValueError('Finite positive freshness, skew and cycle bounds are required')
        self.feedback_max_age_s, self.target_max_age_s = feedback_max_age_s, target_max_age_s
        self.max_skew_s, self.max_step_s, self.settle_time_s = max_skew_s, max_step_s, settle_time_s
        self.max_source_misses = max_source_misses
        self.allow_repeated_source = allow_repeated_source
        self.clock = clock
        self.state, self.reason, self.target = 'DISABLED', None, None
        self._last_time, self._settled_since = None, None
        self._last_measured = None  # Positions of the previous cycle, for the lead.
        self._history = deque()  # (time, positions) of recent cycles, for settling speed.
        self._feedback, self._source = None, None
        self._misses = 0
        self._velocities = None  # This cycle's slewed targets' velocities, for the next proposal.
        self._sent_velocities = None  # The last move's velocities, for the acceleration fed forward.
        self._before = None  # (target, sent velocities) before this cycle's proposal, for reduce().
        self._limits = {}
        self._proposed = None  # (target, velocities) of this cycle's move, as first proposed.

    @property
    def joints(self):
        return deepcopy(list(self._joints))

    def _now(self, now):
        now = self.clock() if now is None else now
        if not _finite(now):
            raise ValueError('Finite monotonic time required')
        return now

    def _read(self, rows, now, *, feedback):
        label = 'feedback' if feedback else 'mapped source'
        if (not isinstance(rows, (list, tuple)) or len(rows) != len(self.names)
                or any(not isinstance(row, dict) for row in rows)
                or tuple(row.get('name') for row in rows) != self.names):
            raise ValueError(f'Complete ordered {label} required')
        max_age = self.feedback_max_age_s if feedback else self.target_max_age_s
        previous = self._feedback if feedback else self._source
        for index, (row, limit) in enumerate(zip(rows, self._joints)):
            if (not _finite(row.get('position')) or not limit['lower'] <= row['position'] <= limit['upper']
                    or not _finite(row.get('sampled_at')) or not 0 <= now-row['sampled_at'] < max_age
                    or type(row.get('sequence')) is not int or not 0 < row['sequence'] < (1 << 32)
                    or row.get('fault') or row.get('valid', True) is not True
                    or feedback and not _finite(row.get('velocity'))):
                raise ValueError(f'Invalid, out-of-range, faulted or stale {label}: {limit["name"]}')
            if previous is not None:
                before = previous[index]
                delta = (row['sequence']-before['sequence']) & 0xffffffff
                if delta == 0:
                    keys = ('position', 'sampled_at', 'velocity') if feedback else ('position', 'sampled_at')
                    if any(row.get(key) != before.get(key) for key in keys):
                        raise ValueError(f'{label} changed without a new source sequence')
                elif delta >= 0x80000000 or row['sampled_at'] < before['sampled_at']:
                    raise ValueError(f'{label} sequence or timestamp moved backwards')
        stamps = [row['sampled_at'] for row in rows]
        if max(stamps)-min(stamps) > self.max_skew_s:
            raise ValueError(f'{label} exceeds the whole-robot skew bound')
        result = deepcopy(list(rows))
        if feedback:
            self._feedback = result
        else:
            self._source = result
        return result

    def _hold_measured(self, measured):
        """A target holding the measured pose, inside each joint's usable range: a joint may settle a hair
        past a command at the edge of its range, and the receiver refuses a command within its stopping margin."""
        held = []
        for row, joint in zip(measured, self._joints):
            q, curve = row['position'], joint.get('approach')
            if curve:
                margin = end_margin(curve, curve['reaction_s'])
                q = max(curve['lower'] + margin, min(curve['upper'] - margin, q))
            held.append(q)
        return tuple(held)

    def _proposal(self, operation, *, original=True):
        positions = list(self.target) if self.target is not None else None
        zero_gains = self.state == 'IDLE'
        # A slewed move carries its velocity (the receiver ramps it over its interpolation period) and
        # friction feed-forward in that direction; any other target (a snap to the measured pose) none.
        velocities = self._velocities if operation == 'move' and not zero_gains else None
        if velocities is None:
            self._limits = {}
        self._velocities = None
        velocities = velocities or (0.,) * len(self._joints)
        if original:
            self._proposed = (tuple(positions), tuple(velocities)) if operation == 'move' and positions else None
        commands = None if positions is None else [
            dict(position=q, velocity=v, effort=0. if zero_gains else self._effort(joint, v, positions),
                 kp=0. if zero_gains else joint['hold_kp'], kd=0. if zero_gains else joint['hold_kd'])
            for q, v, joint in zip(positions, velocities, self._joints)]
        # The effort that accelerates each joint to its command's velocity from the last move's, which the
        # receiver adds only while it ramps to the command (a stop never holds it).
        previous = self._sent_velocities or (0.,) * len(self._joints)
        ramps = None
        if commands is not None and operation == 'move' and not zero_gains:
            ramps = [ramp_effort(joint.get('inertia'), command['velocity'], before, joint.get('interpolation_s'),
                                 command['effort'], joint.get('max_effort', math.inf))
                     for command, before, joint in zip(commands, previous, self._joints)]
        self._sent_velocities = tuple(velocities) if operation == 'move' and not zero_gains else None
        return dict(state=self.state, operation=operation, joint_names=list(self.names),
                    positions=positions, joints=commands, reason=self.reason, ramp_efforts=ramps,
                    limits=deepcopy(self._limits))

    def _effort(self, joint, velocity, positions):
        """Feed-forward: friction in the direction of motion and the arm's gravity at the commanded pose,
        within 90% of the joint's effort limit."""
        effort = friction_feedforward(joint.get('friction_nm', 0.), velocity)
        if joint.get('gravity'):
            effort += gravity_effort(joint['gravity'], dict(zip(self.names, positions)))
        limit = .9 * joint.get('max_effort', math.inf)
        return max(-limit, min(limit, effort))

    def fault(self, reason):
        """Latch a fault and clear proposals; receiver recovery is external."""
        self.state, self.reason, self.target = 'FAULT', str(reason) or 'Unspecified guarded failure', None
        self._settled_since = None
        self._source = None
        return self._proposal('disable')

    def recover(self, feedback, held, *, now=None):
        """After a fault the receiver braked the arm to a hold, and the adapter re-armed it there (`held`: the
        positions it holds): shut down from there, lowering the arm to the reference, then disable. The
        fault's reason stays; nothing follows the leaders again."""
        if self.state != 'FAULT':
            raise ValueError(f'Cannot recover from {self.state}')
        now = self._now(now)
        measured = self._read(feedback, now, feedback=True)
        if len(held) != len(self.names) or not all(_finite(q) for q in held):
            raise ValueError('Complete held positions required')
        self.target = tuple(float(q) for q in held)
        self._last_measured = tuple(row['position'] for row in measured)
        self._history = deque([(now, self._last_measured)])
        self._last_time, self._settled_since, self._source, self._misses = now, None, None, 0
        self._velocities = None
        self.state = 'SHUTDOWN'
        return self._proposal('move')

    def reduce(self, fraction):
        """The adapter could not certify this cycle's move (its collision box was not clear): propose it at
        `fraction` of its step and its speed instead, from the target last sent (0: hold that target). The next
        cycle steps on from what was sent. A hold, alignment or shutdown carries on."""
        if (self.state not in ('HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN') or self._proposed is None
                or self._before is None or self._before[0] is None):
            raise ValueError(f'No move to reduce from {self.state}')
        if not _finite(fraction) or not 0 <= fraction < 1:
            raise ValueError('A move is reduced to a fraction in [0, 1)')
        (previous, sent), (proposed, velocities) = self._before, self._proposed
        self.target = tuple(p + fraction * (q - p) for p, q in zip(previous, proposed))
        self._sent_velocities = sent  # The ramp effort accelerates from what was sent before.
        self._velocities = tuple(fraction * v for v in velocities)
        self._settled_since = None
        return self._proposal('move', original=False)

    def action(self, action, feedback=None, *, target=None, now=None):
        """Apply Idle/Enable/Engage/Hold/Shutdown/Disable GUI intent.

        Invalid operator transitions do not mutate state. Valid actions with
        invalid observations fault and raise; the adapter must dispatch stop.
        """
        allowed = {
            'idle': {'DISABLED', 'IDLE', 'HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN'},
            'enable': {'IDLE'}, 'engage': {'IDLE', 'HOLD'},
            'disengage': {'HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN'},
            'shutdown': {'IDLE', 'HOLD', 'ENGAGING', 'TELEOP'},
            'disable': {'DISABLED', 'IDLE', 'HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN', 'FAULT'},
        }
        if action not in allowed or self.state not in allowed[action]:
            raise ValueError(f'Cannot {action} from {self.state}')
        if action == 'disable':
            if self.state != 'FAULT':
                self.state, self.reason = 'DISABLED', None
            self.target, self._source, self._settled_since, self._last_time = None, None, None, None
            self._last_measured = None
            self._history.clear()
            return self._proposal('disable')
        try:
            now = self._now(now)
            if self._last_time is not None and now < self._last_time:
                raise ValueError('Lifecycle clock moved backwards')
            measured = self._read(feedback, now, feedback=True)
            if action == 'engage':
                self._read(target, now, feedback=False)
            self._before = (self.target, self._sent_velocities)
            old_state = self.state
            self.target = self._hold_measured(measured)
            self._last_measured = tuple(row['position'] for row in measured)
            self._history = deque([(now, self._last_measured)])
            self._last_time, self._settled_since, self.reason = now, None, None
            self._misses = 0
            self.state = {'idle': 'IDLE', 'enable': 'HOLD', 'engage': 'ENGAGING',
                          'disengage': 'HOLD', 'shutdown': 'SHUTDOWN'}[action]
            if action != 'engage':
                self._source = None
            operation = 'idle' if action == 'idle' else 'enable' if old_state == 'IDLE' else 'move'
            return self._proposal(operation)
        except (ValueError, TypeError, KeyError) as exc:
            self.fault(str(exc))
            raise

    def _settled(self, measured, goal, now, dt):
        # Speed from measured positions at least SETTLE_WINDOW_S apart, not the drives'
        # speed readings: at rest those read up to ~0.2 rad/s on Minerva, while
        # positions stay put to a few ten-thousandths of a radian.
        # A gripper trails its command, as the receiver lets it: it rests short of a target by its friction over
        # its stiffness, or on what it holds (the left one rested 0.074 rad from its reference, and Engage and
        # Shutdown never settled, 2026-10-09). Its command must have reached the goal and it must have stopped,
        # wherever it stopped.
        then = next(((t, positions) for t, positions in reversed(self._history) if now - t >= SETTLE_WINDOW_S), None)
        arrived = then is not None and all(
            abs((command if joint.get('gripper') else row['position'])-q) <= joint['tolerance']
            and abs(row['position']-p)/(now - then[0]) <= joint['settled_velocity']
            for row, q, p, joint, command in zip(measured, goal, then[1], self._joints, self.target))
        if not arrived:
            self._settled_since = None
            return False
        if self._settled_since is None:
            self._settled_since = now
        return now-self._settled_since >= self.settle_time_s

    def _approach(self, measured, goal, speed, dt, now):
        proposed, velocities = [], []
        before = self._last_measured or tuple(row['position'] for row in measured)
        sent = self._sent_velocities or (0.,) * len(self._joints)
        for previous, row, target, joint, last, was in zip(self.target, measured, goal, self._joints, before, sent):
            desired, reasons = target, []
            up = down = joint[speed]
            lead_up = lead_down = joint['max_lead']
            # The receiver ramps each command from the last one over its interpolation period: a cycle
            # moves at most that period's worth, so a late cycle never asks for more than the speed.
            period = joint.get('interpolation_s') or 0.
            # Direct targets apply immediately; velocity/acceleration bounds still
            # use the actual producer period, independently of receiver interpolation.
            velocity_period = period or (dt if joint.get('direct_velocity') else 0.)
            step = min(dt, period) if period > 0 else dt
            curve = joint.get('approach')
            if curve:  # Toward an end, within the receiver's braking curve (with headroom) all along the step.
                if step > 0:
                    up = command_step(curve, curve['reaction_s'], previous, 1, up, step) / step
                    down = command_step(curve, curve['reaction_s'], previous, -1, down, step) / step
                # And pull no harder than that speed warrants: the lead toward an end shrinks with it.
                reaction, cap = curve['reaction_s'], joint['max_velocity']
                gains = (joint['hold_kp'], joint['hold_kd'])
                lead_up = lead_toward(curve, reaction, row['position'], 1, joint['max_lead'], cap, *gains)
                lead_down = lead_toward(curve, reaction, row['position'], -1, joint['max_lead'], cap, *gains)
            # The lead is taken from where the joint will be once the receiver has ramped to this command: its
            # measured position carried on, in the direction it moves, at its speed between the last two cycles
            # over the sample's age and the interpolation period. Taken from the measured position alone, the
            # ramp and the sample's age (about 80 ms) held right J1 to 0.57 rad/s at a 0.048 rad lead.
            ahead = (row['position'] - last) / dt * (max(0., now - row['sampled_at']) + period) if dt > 0 else 0.
            up_from, down_from = row['position'] + max(0., ahead), row['position'] + min(0., ahead)
            lead_lower, lead_upper = down_from-lead_down, up_from+lead_up
            if self.allow_repeated_source:
                # Construct endpoints inside the exact bound. Otherwise subtracting
                # a rounded endpoint from feedback can exceed max_lead by one ULP
                # and falsely trip the unchanged strict HOLD check on source loss.
                lead_lower = math.nextafter(lead_lower, down_from)
                lead_upper = math.nextafter(lead_upper, up_from)
            # A target beyond a shrunken lead may come back toward the joint at once: that only eases the pull.
            low = max(joint['lower'], lead_lower, min(previous-down*step, lead_upper))
            high = min(joint['upper'], lead_upper, max(previous+up*step, lead_lower))
            if low > high:
                raise ValueError(f'Measured pose exceeds command lead/slew bounds: {joint["name"]}')
            if not joint['lower'] <= desired <= joint['upper']:
                reasons.append('joint_limit')
            if not down_from-lead_down <= desired <= up_from+lead_up:
                reasons.append('tracking_lead')
            if not previous-down*step <= desired <= previous+up*step:
                reasons.append('speed_or_limit_approach')
            if curve and velocity_period > 0 and dt > 0:
                # Speeding up no faster than the joint's qualified braking, in either direction; slowing toward
                # the goal is never held back, so a command never runs past where the leader stopped. A step from
                # rest to 1.5 rad/s asked right J2 for 41 Nm in 10 ms, and gravity carried it to 1.98 rad/s,
                # over the receiver's 1.875 cap (rehearsal, 2026-10-04).
                rate = curve['stop_acceleration'] * dt
                target = max(previous + (min(was, 0.) - rate) * velocity_period,
                             min(previous + (max(was, 0.) + rate) * velocity_period, target))
                if target != desired:
                    reasons.append('acceleration')
            q = max(low, min(high, target))
            proposed.append(q)
            self._limits[joint['name']] = reasons if abs(q-desired) > 1e-12 else []
            # The velocity it is ramped at, fed forward (a snap back toward the joint no faster than the cap).
            velocities.append(0. if velocity_period <= 0 else
                              max(-joint['max_velocity'], min(joint['max_velocity'], (q-previous)/velocity_period)))
        self._velocities = tuple(velocities)
        return tuple(proposed)

    def step(self, feedback=None, *, target=None, now=None):
        """Propose a fresh complete batch; mapped targets are absolute coordinates."""
        if self.state == 'FAULT':
            return self._proposal('disable')
        if self.state == 'DISABLED':
            return self._proposal('none')
        try:
            now = self._now(now)
            dt = now-self._last_time
            if not 0 <= dt <= self.max_step_s:
                raise ValueError('Lifecycle scheduler stalled or clock moved backwards')
            measured = self._read(feedback, now, feedback=True)
            self._before = (self.target, self._sent_velocities)
            operation = 'move'
            if self.state == 'IDLE':
                self.target = tuple(row['position'] for row in measured)
                operation = 'idle'
            elif self.state == 'HOLD':
                if any(abs(q-row['position']) > joint['max_lead']
                       for q, row, joint in zip(self.target, measured, self._joints)):
                    raise ValueError('Measured pose exceeds the holding lead bound')
            elif self.state in {'ENGAGING', 'TELEOP'} and target is None:
                # Missing source: hold the last bounded target, never extrapolate.
                self._misses += 1
                self._settled_since = None
                if self._misses > self.max_source_misses:
                    # Stop following. Resuming needs an explicit new Engage.
                    self.state = 'HOLD'
                    self.reason = 'Leader input lost; holding measured pose until Engage'
                    self.target = self._hold_measured(measured)
                    self._source, self._misses = None, 0
                elif any(abs(q-row['position']) > joint['max_lead']
                         for q, row, joint in zip(self.target, measured, self._joints)):
                    raise ValueError('Measured pose exceeds the holding lead bound')
            elif self.state in {'ENGAGING', 'TELEOP'}:
                previous_source = self._source
                source = self._read(target, now, feedback=False)
                repeated = previous_source is not None and any(
                    row['sequence'] == previous['sequence'] for row, previous in zip(source, previous_source))
                if dt > 0 and repeated and not self.allow_repeated_source:
                    raise ValueError('Mapped source did not advance')
                self._misses = self._misses + 1 if repeated else 0
                if self._misses > self.max_source_misses:
                    self.state = 'HOLD'
                    self.reason = 'Leader input lost; holding measured pose until Engage'
                    self.target = self._hold_measured(measured)
                    self._source, self._misses = None, 0
                else:
                    # Direct loops can outpace USB samples. Continue the bounded
                    # approach to the same observed goal, without extrapolating it
                    # or refreshing its sequence/age. A duplicate is not a stop.
                    goal = [row['position'] for row in source]
                    speed = 'align_speed' if self.state == 'ENGAGING' else 'max_velocity'
                    self.target = self._approach(measured, goal, speed, dt, now)
                    if self.state == 'ENGAGING' and self._settled(measured, goal, now, dt):
                        self.state = 'TELEOP'
            elif self.state == 'SHUTDOWN':
                goal = [joint['zero_position'] for joint in self._joints]
                self.target = self._approach(measured, goal, 'shutdown_speed', dt, now)
                if self._settled(measured, goal, now, dt):
                    self.state, self.target = 'DISABLED', None
                    operation = 'disable'
            self._last_time = now
            self._last_measured = tuple(row['position'] for row in measured)
            self._history.append((now, self._last_measured))
            while len(self._history) > 2 and now - self._history[1][0] >= 5 * SETTLE_WINDOW_S:
                self._history.popleft()
            return self._proposal(operation)
        except (ValueError, TypeError, KeyError) as exc:
            self.fault(str(exc))
            raise
