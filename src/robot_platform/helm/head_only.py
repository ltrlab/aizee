"""Live · Head: the head alone in a teleop session (the Teleop tab's Head choice), for its bring-up with the headset.

The head's own receiver (helm.head_teleop) under the session's keys, with none of the arms' motors: the arms'
receivers are never opened, so their drives stay as they are and Helm's live view keeps reading them. Idle (I) takes
the head's receiver with its torque off, Enable (E) holds the head where it is, Engage (T) aligns it to the headset and
follows it, T again holds it, Return (X) takes it to its zero and then disables it, and Disable (H) removes its torque
at once. As with the arms, a head disabled after it was powered needs a new session. The pedals step it the same way
(C: Idle, Enable, Engage, then Return; B: hold and resume); A, recording, has nothing to record.

The loop (helm.teleop.run_loop) drives this core as it drives TeleopCore, one head cycle each 40 ms period.
"""
from collections import deque
from copy import deepcopy
import time

from robot_platform.teleop.guarded_lifecycle import minerva_action
from .head_teleop import PERIOD_S, HeadTeleop
from .pedals import Pedals

ENGAGED = ('ENGAGING', 'TELEOP')
# Operator transitions the lifecycle refuses from other states: said, never sent (a refused one faults the head).
ALLOWED = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ENGAGED,
           'shutdown': ('IDLE', 'HOLD', *ENGAGED)}
CYCLE_HISTORY = 100
STATES = {'IDLE': 'torque off. Enable (E) holds the head where it is.',
          'HOLD': 'holding. Engage (T) aligns it to the headset, then follows it.',
          'ENGAGING': 'aligning to the headset.',
          'TELEOP': 'following the headset. T holds it; Return (X) takes it to its zero.',
          'SHUTDOWN': 'returning to its zero, then disabling.'}


def head_only(receiver):
    """Whether a session's receiver choice is the head alone ({'mode': 'live', 'arm': 'head'})."""
    return isinstance(receiver, dict) and receiver.get('arm') == 'head'


class HeadOnlyCore:
    """The loop body of a head-only session, from the session bundle's 'head' and 'receiver' (live, or virtual for a
    rehearsal on the native harness). `open_head` builds the HeadTeleop; tests inject theirs."""

    collision_mode = None  # The loop's plain schedule: one cycle a period, each with a full view.

    def __init__(self, bundle, *, open_head=None, clock=time.perf_counter):
        self.bundle, self.clock = deepcopy(bundle), clock
        receiver = bundle.get('receiver') or {}
        if not head_only(receiver) or receiver.get('mode') not in ('live', 'virtual'):
            raise ValueError('A head-only session drives the head alone, live or on the virtual receiver')
        if bundle.get('head') is None:
            raise ValueError('The head is not set up: save its reference, limits and shoulder clearance in Limits → '
                             'Head first')
        self.live = receiver['mode'] == 'live'
        self.period_s = PERIOD_S
        self.head = (open_head or (lambda head: HeadTeleop(head, live=self.live, clock=clock)))(bundle['head'])
        self.pedals = Pedals()
        self.phase, self.fault = 'starting', None
        self.message = (f'Starting {self._name}: its receiver, the servos limp until Idle (I)…' if self.live
                        else f'Starting {self._name}: no motors move…')
        self.scheduling = {}
        self._tuning_result = None  # No motor tuning here; the loop reads it.
        self._powered = False  # A live head disabled after it was powered needs a new session.
        self._land = self._closing = False
        self.sequence = 0
        self.cycles = deque(maxlen=CYCLE_HISTORY)
        self._last_start, self.largest_gap_ms = None, 0.

    @property
    def _name(self):
        return 'LIVE HEAD' if self.live else 'VIRTUAL HEAD'

    def build(self):
        self.head.build()
        self.phase = 'running'
        self.message = (f'{self._name} · the servos are powered from Idle (I); then Enable (E) holds the head and Engage '
                        '(T) follows the headset. Return (X) takes it to its zero; Disable (H) removes its torque.')

    # -- the cycle ---------------------------------------------------------------------------

    def tick(self, key, started):
        if key:
            self.pedals.invalidate(ended=key in ('H', 'X'))
        head = self.head
        if self.fault:
            if key == 'H':
                self._disable_now()
            return
        previous = head.state
        action = minerva_action(key, previous) if key else None
        notice = None
        if self._land:
            # Closing: a powered head returns to its zero and is disabled (as X) before its receiver closes.
            self._land = False
            if previous in ('HOLD', *ENGAGED) and action != 'disable':
                action = 'shutdown'
        if action in ALLOWED and previous not in ALLOWED[action]:
            notice = f'[{key}] Not available from {previous}. Start with Idle (I), then Enable (E).'
            action = None
        if self.live and self._powered and previous == 'DISABLED' and action not in (None, 'disable'):
            notice = f'[{key}] The head was disabled. Stop the session and start a new one to power it again.'
            action = None
        try:
            record = head.cycle(action, started)
        except Exception:
            self.fault = head.fault
            self.message = (f'{self._name} FAULT · {self.fault} · Disable (H) removes its torque; stop the session and '
                            'start it again.')
            return
        if record is not None:
            self.sequence += 1
            self.cycles.append({'started_at': record.get('started_at'), 'completed_at': record.get('completed_at')})
            if self._last_start is not None:
                self.largest_gap_ms = max(self.largest_gap_ms, (started - self._last_start) * 1000)
            self._last_start = started
        current = head.state
        self._powered = self._powered or current not in (None, 'DISABLED')
        if notice:
            self.message = notice
        elif current != previous:
            self.message = f'{self._name} · {current} · ' + STATES.get(current, (
                'torque off. Stop the session; a new one powers it again.' if self.live and self._powered
                else 'torque off.'))

    def _disable_now(self):
        try:
            self.head.session.disable_now()
        except Exception as exc:
            self.message = (f'{self._name} FAULT · {self.fault} · Disable could not be confirmed: '
                            f'{str(exc) or type(exc).__name__}. Use the foot switch; the session remains faulted.')
        else:
            self.message = (f'{self._name} FAULT · {self.fault} · Disabled at your request: the head is limp where it '
                            'is. Stop the session and start it again.')

    def head_pose(self, yaw, pitch):
        """The headset's latest yaw and pitch (helm.xr), as the loop received it."""
        self.head.pose(yaw, pitch)

    # -- closing -------------------------------------------------------------------------------

    def land(self):
        """Before the session closes: a powered head returns to its zero, then is disabled (as X does). True while
        there is a head to return."""
        self._closing = True
        self.pedals.invalidate(ended=True)
        if self.head.session is None:
            return False
        if not self.fault and self.head.state in ('HOLD', *ENGAGED):
            self._land = True
        return self.landing

    @property
    def landing(self):
        """Whether the loop is still returning the head before it may close its receiver."""
        if self.head.session is None or self.fault:
            return False
        return self._land or self.head.state == 'SHUTDOWN'

    def helm_gone(self, reason):
        """Helm closed the channel or went silent: the loop lands and closes, as when asked to stop."""

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason
        self.message = f'Teleop stopped: {reason}'

    def close(self):
        try:
            self.head.close()
        except Exception as exc:
            if self.phase != 'fault':
                self.fail(str(exc) or type(exc).__name__)
        if self.phase != 'fault':
            self.phase = 'stopped'
            self.message = ('Teleop stopped. The head is disabled and its receiver closed.' if self.live
                            else 'Teleop stopped. The virtual head receiver is closed.')

    # -- what the arms' sessions offer, which a head-only one does not -------------------------------

    def task_action(self, action):
        raise ValueError('A head-only session records nothing: choose Both arms to capture the task')

    def set_grip_feedback(self, payload):
        raise ValueError('A head-only session has no grippers')

    def route(self, action, motor):
        raise ValueError("A head-only session has no leader routes: the headset steers the head")

    def set_zero(self):
        raise ValueError("A head-only session has no leaders: Zero headset sets the head's straight ahead")

    def queue_tuning(self, payload):
        raise ValueError('Motor tuning applies to the arms')

    # -- the pedals -------------------------------------------------------------------------------

    def _pedal_context(self):
        return dict(phase=self.phase, state=self.head.state, fault=self.fault, recording_fault=None,
                    spent=bool(self.live and self._powered), closing=self._closing, task=None)

    def prepare_pedal(self, payload):
        return self.pedals.prepare(payload, self._pedal_context(), self.clock())

    def finish_pedal(self, intent, *, error=None, command_id=None):
        if error is None and intent['pedal'] != 'a':
            record = self.head.session.last_cycle if self.head.session is not None else None
            if not record or record.get('accepted') is not True or record.get('skipped'):
                error = 'Pedal command dispatch was not confirmed'
        return self.pedals.finish(intent, self._pedal_context(), error=error, command_id=command_id)

    # -- views -------------------------------------------------------------------------------------

    def guide(self):
        """No pose of its own: the 3D view draws the arms as Helm's live view reads them, the neck as this session
        measures it (TeleopSession.head_joints)."""
        return None

    def view(self):
        head = self.head
        status = head.status() if head.session is not None else {'state': None, 'fault': head.fault}
        last = (head.receiver.last or {}) if head.receiver is not None else {}
        durations = sorted((c['completed_at'] - c['started_at']) * 1000 for c in self.cycles
                           if c.get('completed_at') is not None and c.get('started_at') is not None)
        return {'phase': self.phase, 'message': self.message, 'fault': self.fault, 'recording_fault': None,
                'trial': None, 'position_only': False, 'velocity_feedforward': False,
                'state': status['state'], 'receiver': (last.get('status') or {}).get('state'),
                'manual_tuning_ack': None, 'manual_tuning_pending': False,
                'live': {'arm': 'head', 'motors_enabled': last.get('hardware_enabled'),
                         'watchdog': last.get('watchdog'), 'reference': last.get('reference')} if self.live else None,
                'input_error': None, 'late_targets': 0, 'collision': {'mode': 'disabled'},
                'task': None, 'grip_feedback': None, 'replay': None, 'policy': None, 'head': status,
                'head_only': True, 'scheduling': self.scheduling, 'joints': [],
                'cycle_ms': {'p50': durations[(len(durations) - 1) // 2],
                             'p99': durations[min(len(durations), -(-99 * len(durations) // 100)) - 1],
                             'max': durations[-1], 'overruns': sum(d > self.period_s * 1000 for d in durations),
                             'largest_gap': self.largest_gap_ms} if durations else None,
                'pedals': self.pedals.view(self._pedal_context(), self.clock())}
