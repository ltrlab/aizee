"""Pure GUI teleop phases and bounded absolute targets; no device ownership.

Proposals are not acknowledgements or permission to enable hardware. A guarded
adapter must certify and acknowledge every batch, and call fault() on failure.
"""
from copy import deepcopy
import math
import time

from robot_platform.control.approach import command_speed, end_margin, friction_feedforward, lead_toward
from robot_platform.control.gravity import gravity_effort


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class TeleopLifecycle:
    """Shared semantic operations, independent of each GUI's key bindings."""
    def __init__(self, joints, *, feedback_max_age_s=.03, target_max_age_s=.05,
                 max_skew_s=.02, max_step_s=.08, settle_time_s=.1, max_source_misses=2,
                 clock=time.perf_counter):
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
                or type(max_source_misses) is not int or not 0 <= max_source_misses <= 10):
            raise ValueError('Finite positive freshness, skew and cycle bounds are required')
        self.feedback_max_age_s, self.target_max_age_s = feedback_max_age_s, target_max_age_s
        self.max_skew_s, self.max_step_s, self.settle_time_s = max_skew_s, max_step_s, settle_time_s
        self.max_source_misses = max_source_misses
        self.clock = clock
        self.state, self.reason, self.target = 'DISABLED', None, None
        self._last_time, self._settled_since = None, None
        self._last_measured = None  # Positions of the previous cycle, for settling speed.
        self._feedback, self._source = None, None
        self._misses = 0
        self._velocities = None  # This cycle's slewed targets' velocities, for the next proposal.

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

    def _proposal(self, operation):
        positions = list(self.target) if self.target is not None else None
        zero_gains = self.state == 'IDLE'
        # A slewed move carries its velocity (the receiver ramps it over its interpolation period) and
        # friction feed-forward in that direction; any other target (a snap to the measured pose) none.
        velocities = self._velocities if operation == 'move' and not zero_gains else None
        self._velocities = None
        velocities = velocities or (0.,) * len(self._joints)
        commands = None if positions is None else [
            dict(position=q, velocity=v, effort=0. if zero_gains else self._effort(joint, v, positions),
                 kp=0. if zero_gains else joint['hold_kp'], kd=0. if zero_gains else joint['hold_kd'])
            for q, v, joint in zip(positions, velocities, self._joints)]
        return dict(state=self.state, operation=operation, joint_names=list(self.names),
                    positions=positions, joints=commands, reason=self.reason)

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
            return self._proposal('disable')
        try:
            now = self._now(now)
            if self._last_time is not None and now < self._last_time:
                raise ValueError('Lifecycle clock moved backwards')
            measured = self._read(feedback, now, feedback=True)
            if action == 'engage':
                self._read(target, now, feedback=False)
            old_state = self.state
            self.target = self._hold_measured(measured)
            self._last_measured = tuple(row['position'] for row in measured)
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
        # Speed from the measured positions of consecutive cycles, not the drives'
        # speed readings: at rest those read up to ~0.2 rad/s on Minerva, while
        # positions stay put to a few ten-thousandths of a radian.
        previous = self._last_measured
        arrived = previous is not None and dt > 0 and all(
            abs(row['position']-q) <= joint['tolerance']
            and abs(row['position']-p)/dt <= joint['settled_velocity']
            for row, q, p, joint in zip(measured, goal, previous, self._joints))
        if not arrived:
            self._settled_since = None
            return False
        if self._settled_since is None:
            self._settled_since = now
        return now-self._settled_since >= self.settle_time_s

    def _approach(self, measured, goal, speed, dt):
        proposed, velocities = [], []
        for previous, row, target, joint in zip(self.target, measured, goal, self._joints):
            up = down = joint[speed]
            lead_up = lead_down = joint['max_lead']
            curve = joint.get('approach')
            if curve:  # Toward an end, within the receiver's braking curve (with headroom).
                up = command_speed(curve, curve['reaction_s'], previous, 1, up)
                down = command_speed(curve, curve['reaction_s'], previous, -1, down)
                # And pull no harder than that speed warrants: the lead toward an end shrinks with it.
                reaction, cap = curve['reaction_s'], joint['max_velocity']
                gains = (joint['hold_kp'], joint['hold_kd'])
                lead_up = lead_toward(curve, reaction, row['position'], 1, joint['max_lead'], cap, *gains)
                lead_down = lead_toward(curve, reaction, row['position'], -1, joint['max_lead'], cap, *gains)
            # The receiver ramps each command from the last one over its interpolation period: a cycle
            # moves at most that period's worth, so a late cycle never asks for more than the speed.
            period = joint.get('interpolation_s') or 0.
            step = min(dt, period) if period > 0 else dt
            # A target beyond a shrunken lead may come back toward the joint at once: that only eases the pull.
            low = max(joint['lower'], row['position']-lead_down, min(previous-down*step, row['position']+lead_up))
            high = min(joint['upper'], row['position']+lead_up, max(previous+up*step, row['position']-lead_down))
            if low > high:
                raise ValueError(f'Measured pose exceeds command lead/slew bounds: {joint["name"]}')
            q = max(low, min(high, target))
            proposed.append(q)
            # The velocity it is ramped at, fed forward (a snap back toward the joint no faster than the cap).
            velocities.append(0. if period <= 0 else
                              max(-joint['max_velocity'], min(joint['max_velocity'], (q-previous)/period)))
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
                self._misses = 0
                previous_source = self._source
                source = self._read(target, now, feedback=False)
                if dt > 0 and previous_source is not None and any(
                        row['sequence'] == previous['sequence'] for row, previous in zip(source, previous_source)):
                    raise ValueError('Mapped source did not advance')
                goal = [row['position'] for row in source]
                speed = 'align_speed' if self.state == 'ENGAGING' else 'max_velocity'
                self.target = self._approach(measured, goal, speed, dt)
                if self.state == 'ENGAGING' and self._settled(measured, goal, now, dt):
                    self.state = 'TELEOP'
            elif self.state == 'SHUTDOWN':
                goal = [joint['zero_position'] for joint in self._joints]
                self.target = self._approach(measured, goal, 'shutdown_speed', dt)
                if self._settled(measured, goal, now, dt):
                    self.state, self.target = 'DISABLED', None
                    operation = 'disable'
            self._last_time = now
            self._last_measured = tuple(row['position'] for row in measured)
            return self._proposal(operation)
        except (ValueError, TypeError, KeyError) as exc:
            self.fault(str(exc))
            raise
