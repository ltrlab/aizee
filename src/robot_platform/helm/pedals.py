"""Bounded pedal intent policy. The control worker, not browser state, decides actions.

Offers last 750 ms on the worker clock. This is an input freshness fence only;
receiver leases, feedback age, collision bounds and motion laws are unchanged.
Recording and motion offers have independent revisions so a slow writer cannot
prevent pause/shutdown. No request is queued or automatically retried here.
"""
from collections import deque
import uuid

OFFER_S = .75
PEDALS = frozenset('abc')


class Pedals:
    def __init__(self):
        self.session_id = uuid.uuid4().hex
        self.paused = self.has_engaged = self.ended = False
        self.last_result = None
        self._seen = deque(maxlen=32)
        self._motion = self._capture = None
        self._signatures = {'motion': None, 'capture': None}
        self._revisions = {'motion': 0, 'capture': 0}
        self._expires = {'motion': 0., 'capture': 0.}

    def invalidate(self, *, ended=False):
        self.paused = False
        self.ended = self.ended or ended
        self._motion = None
        for group in self._revisions:
            self._revisions[group] += 1
            self._expires[group] = 0.

    def capture_command(self, action, result):
        if action in ('start_record', 'stop_record'):
            self._capture = (result['id'], action)
            self._revisions['capture'] += 1
            self._expires['capture'] = 0.

    def _sync(self, context):
        state = context.get('state')
        if state in ('ENGAGING', 'TELEOP'):
            self.has_engaged = True
        if state != 'HOLD' or context.get('fault') or context.get('recording_fault'):
            self.paused = False
        task = context.get('task') or {}
        command = task.get('last_command') or {}
        if (self._capture and command.get('id') == self._capture[0]
                and command.get('state') in ('complete', 'failed')):
            self._capture = None
        record = task.get('recording') or {}
        motion = (context.get('phase'), state, bool(context.get('fault')),
                  bool(context.get('recording_fault')), bool(context.get('spent')),
                  bool(context.get('closing')), self.paused, self.has_engaged, self.ended)
        capture = (motion, bool(task.get('capture_ready')), bool(task.get('error')),
                   record.get('state'), bool(record.get('active')), bool(record.get('complete')),
                   self._capture, command.get('id'), command.get('state'))
        for group, signature in (('motion', motion), ('capture', capture)):
            if signature != self._signatures[group]:
                self._signatures[group] = signature
                self._revisions[group] += 1
                self._expires[group] = 0.

    def _actions(self, c):
        state = c.get('state')
        task = c.get('task') or {}
        rec = task.get('recording') or {}
        pending = task.get('last_command') or {}
        running = c.get('phase') == 'running' and not c.get('closing') and not self.ended
        healthy = running and not c.get('fault') and not c.get('recording_fault')
        stop_record = bool(rec.get('active'))
        a = 'stop_record' if stop_record else 'start_record'
        a_busy = (self._capture is not None or pending.get('state') == 'pending'
                  or rec.get('state') in ('starting', 'stopping', 'finishing'))
        a_ok = (healthy and bool(task) and not task.get('error') and not a_busy
                and (stop_record or (state in ('HOLD', 'TELEOP') and task.get('capture_ready') is True)))
        b = 'pause' if state in ('ENGAGING', 'TELEOP') else 'resume'
        b_ok = healthy and (state in ('ENGAGING', 'TELEOP') or (state == 'HOLD' and self.paused))
        if state == 'DISABLED':
            action = 'idle'
        elif state == 'IDLE':
            action = 'enable'
        elif state == 'HOLD' and not self.has_engaged and not c.get('recording_fault'):
            action = 'engage'
        else:
            action = 'shutdown'
        c_ok = (running and not c.get('fault') and state in ('DISABLED', 'IDLE', 'HOLD', 'TELEOP')
                and not (state == 'DISABLED' and c.get('spent'))
                and (not c.get('recording_fault') or action == 'shutdown'))
        moving = self._motion is not None
        unfinished = (bool(rec.get('active')) or self._capture is not None
                      or pending.get('state') == 'pending' or rec.get('state') in ('starting', 'stopping', 'finishing'))
        labels = {'idle': 'Idle', 'enable': 'Enable / Hold', 'engage': 'Engage / Align',
                  'pause': 'Pause / Hold', 'resume': 'Resume / Align', 'shutdown': 'Shutdown to reference',
                  'start_record': 'Start recording', 'stop_record': 'Finish recording'}
        result = {}
        for pedal, name, enabled, reason in (
                ('a', a, a_ok, 'Wait for recording acknowledgement, ready cameras and Hold or Teleop'),
                ('b', b, b_ok and not moving, 'Pause while aligning/following; resume only a pedal-paused Hold'),
                ('c', action, c_ok and not moving, 'Wait for the current transition; faults cannot enable or resume')):
            if pedal == 'c' and state == 'DISABLED' and (self.ended or c.get('spent')):
                reason = 'Session finished. Stop this session and start a new one to enable again.'
            group = 'capture' if pedal == 'a' else 'motion'
            result[pedal] = {'action': name, 'label': labels[name], 'enabled': bool(enabled),
                             'reason': None if enabled else reason, 'revision': self._revisions[group],
                             'warning': ('Shutdown aborts unfinished recording. Finish with A and wait for Complete to save.'
                                         if name == 'shutdown' and unfinished else None)}
        return result

    def view(self, context, now):
        self._sync(context)
        for group in self._expires:
            if now >= self._expires[group]:
                self._revisions[group] += 1
                self._expires[group] = now + OFFER_S
        return {'session_id': self.session_id, 'revision': self._revisions['motion'],
                'paused': self.paused, 'has_engaged': self.has_engaged, 'ended': self.ended,
                'busy': self._motion is not None, 'actions': self._actions(context),
                'last_result': self.last_result}

    def prepare(self, payload, context, now):
        if (not isinstance(payload, dict) or set(payload) != {'session_id', 'revision', 'press_id', 'pedal'}
                or payload.get('pedal') not in PEDALS or type(payload.get('revision')) is not int
                or type(payload.get('press_id')) is not int or not 0 < payload['press_id'] < 2**53):
            raise ValueError('Expected a pedal edge with session_id, revision, press_id and a, b or c')
        if payload['session_id'] != self.session_id:
            raise ValueError('Pedal belongs to a different teleop session')
        edge = (payload['pedal'], payload['revision'], payload['press_id'])
        if edge in self._seen:
            raise ValueError('Duplicate or reordered pedal edge; release and press again')
        self._seen.append(edge)
        self._sync(context)
        pedal = payload['pedal']
        group = 'capture' if pedal == 'a' else 'motion'
        if payload['revision'] != self._revisions[group] or now >= self._expires[group]:
            raise ValueError('Pedal offer is stale; review current state and press again')
        offered = self._actions(context)[pedal]
        if not offered['enabled']:
            raise ValueError(offered['reason'])
        intent = dict(payload, action=offered['action'])
        if pedal != 'a':
            self._motion = intent
        # Consume only this domain; an A edge never invalidates pause/shutdown.
        self._revisions[group] += 1
        self._expires[group] = 0.
        return intent

    def finish(self, intent, context, *, error=None, command_id=None):
        action, state = intent['action'], context.get('state')
        expected = {'idle': 'IDLE', 'enable': 'HOLD', 'engage': 'ENGAGING', 'pause': 'HOLD',
                    'resume': 'ENGAGING', 'shutdown': 'SHUTDOWN'}
        if error is None and action in expected:
            allowed = {expected[action]}
            if action in ('engage', 'resume'):
                allowed.add('TELEOP')
            if action == 'shutdown':
                allowed.add('DISABLED')
            if context.get('fault') or state not in allowed:
                error = 'Pedal transition was not confirmed by the control worker'
        if error is None:
            if action == 'pause':
                self.paused = True
            elif action in ('resume', 'engage'):
                self.paused = False
                self.has_engaged = True
            elif action == 'shutdown':
                self.ended = True
                self.paused = False
        self._motion = None
        self.last_result = {key: intent[key] for key in ('press_id', 'pedal', 'action')}
        self.last_result.update(accepted=error is None, error=error)
        if command_id is not None:
            self.last_result['command_id'] = command_id
        self._sync(context)
        return dict(self.last_result)
