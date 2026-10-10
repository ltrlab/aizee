"""Collision-disabled lifecycle with one receiver exchange per following cycle.

The previous reply supplies both feedback and a one-use receiver clock anchor.
No queued targets, retries, geometry backend or collision verdicts are involved.
"""
from copy import deepcopy
import math
import time

from robot_platform.control.guarded import GuardRejected
from .guarded_lifecycle import GuardedLifecycleSession


class DirectLifecycleSession(GuardedLifecycleSession):
    def __init__(self, controller, config, lifecycle, snapshot, *, period_s=.02,
                 cycle_budget_s=.035, clock=time.perf_counter):
        names = tuple(j['name'] for j in config['joints'])
        if (not config.get('direct') or config.get('collision') is not None
                or controller.identity != config['identity'] or controller.group != config['group']
                or controller.joint_names != names or lifecycle.names != names
                or not all(math.isfinite(v) and 0 < v <= lifecycle.max_step_s
                           for v in (period_s, cycle_budget_s))):
            raise ValueError('Matching explicit direct configuration and valid timing required')
        for row, limit in zip(lifecycle.joints, config['joints']):
            if (any(row[k] != limit[k] for k in ('name', 'lower', 'upper', 'hold_kp', 'hold_kd'))
                    or row['max_velocity'] > limit['max_velocity']):
                raise ValueError('Lifecycle bounds and gains must match receiver configuration')
        self.controller, self.lifecycle, self.snapshot, self.clock = controller, lifecycle, snapshot, clock
        self.compiler = None
        self.period_s, self.cycle_budget_s = period_s, cycle_budget_s
        self._verdict_s = cycle_budget_s
        self.owned = False
        self.stop_error = self.last_cycle = self.last_feedback = self.blocked = None
        self._cached = None
        self._samples = None
        self._offer_received = None
        self.command_ms = config['direct']['command_ms']

    def _cache(self, reply, started):
        sample = self.snapshot()
        if sample['reply'] != reply or sample['status'] != reply['status']:
            raise ValueError('Feedback must be from the same receiver reply')
        # The synchronous transport owns this completed result. Only feedback
        # enters the next control cycle; recorder history and duplicate status
        # trees stay in the recording path. Keep an owned copy of the feedback.
        feedback = deepcopy(sample['feedback'])
        # Stamp conservatively at request start, not reply arrival. Never refresh
        # the cached sample's age on a later loop iteration.
        rows = self._rows(reply, feedback, started)
        if self._samples is not None:
            old_rows, old_feedback = self._samples
            for row, raw, old, previous in zip(rows, feedback, old_rows, old_feedback):
                if row['sequence'] == old['sequence']:
                    if any(raw[k] != previous[k] for k in ('position', 'velocity', 'sampled_at_ms')):
                        raise ValueError('Receiver feedback changed without a new sequence')
                    row['sampled_at'] = old['sampled_at']
        self._samples = (deepcopy(rows), deepcopy(feedback))
        self._offer_received = self.clock()
        self._cached = (reply, rows, feedback)
        return self._cached

    def _offer(self):
        started = self.clock()
        return self._cache(self.controller.freshness_offer(), started)

    def recover(self):
        status = self.controller.poll()
        if status['state'] == 'stopping':
            return False
        if status['state'] != 'stopped':
            raise RuntimeError(f"The receiver is {status['state']} ({status['reason']})")
        if not self.controller.owned:
            self.controller.acquire()
        self.owned = True
        status = self.controller.arm()
        _, rows, _ = self._offer()
        self.lifecycle.recover(rows, [j['position'] for j in status['joints']], now=self.clock())
        self.last_feedback = deepcopy(rows)
        return True

    def cycle(self, *, action=None, target=None, started_at=None, tuning=None):
        started = self.clock() if started_at is None else started_at
        record = dict(action=action, accepted=False, started_at=started, collision_mode='disabled')
        self.last_cycle = record
        exchanges = self.controller.exchanges
        previous_tuning = None
        try:
            if tuning is not None:
                if self.lifecycle.state != 'HOLD' or action is not None:
                    raise ValueError('Hold before applying motor tuning')
                previous_tuning = (deepcopy(self.lifecycle._joints), deepcopy(self.lifecycle._manual), self.lifecycle._manual_hold)
                self.lifecycle.configure_manual(tuning)
            if action == 'disable':
                record.update(proposal=self.disable(), accepted=True)
                self._cached = None
                return record
            if self.lifecycle.state == 'FAULT':
                raise RuntimeError('Explicit receiver recovery required after a fault')
            if not self.owned and action == 'idle':
                status = self.controller.acquire()
                self.owned = True
                self._cached = None
                if status['state'] != 'disarmed':
                    raise ValueError('Receiver must be disarmed before Idle')
            # Outside active following, refresh feedback/authority explicitly.
            offered, rows, feedback = self._cached or self._offer()
            self.last_feedback = deepcopy(rows)
            now = self.clock()
            # Preserve the observations even if proposal construction faults.
            # Otherwise the first divergent cycle loses precisely its evidence.
            record.update(feedback=feedback, target=deepcopy(target), mapped_at=now,
                          lifecycle_before=self.lifecycle.state,
                          feedback_age_ms=[(now-row['sampled_at'])*1000 for row in rows])
            proposal = (self.lifecycle.action(action, rows, target=target, now=now) if action
                        else self.lifecycle.step(rows, target=target, now=now))
            record.update(proposal=deepcopy(proposal))
            operation = proposal['operation']
            self._deadline(started, target)
            rpc_started = self.clock()
            if operation == 'move':
                lifetime = self.command_ms
                if target and self.lifecycle.state in ('ENGAGING', 'TELEOP'):
                    expires = min(row['sampled_at'] for row in target) + self.lifecycle.target_max_age_s
                    # Offer issuance was no later than reply receipt. Anchoring the
                    # remaining lifetime there gives a conservative receiver deadline
                    # without comparing the two hosts' clocks, including an RPC stall.
                    lifetime = min(lifetime, math.floor((expires - self._offer_received) * 1000))
                if lifetime <= 0:
                    raise ValueError('Leader target expired before dispatch')
                try:
                    reply = self.controller.direct_update(proposal['joints'], offered, valid_for_ms=lifetime,
                                                          ramp_efforts=proposal.get('ramp_efforts'),
                                                          update_hold_gains=tuning is not None)
                except GuardRejected as exc:
                    # A settling check may change after the UI preflight. This
                    # definitive rejection precedes native offer consumption and
                    # gain commit; keep holding with the old settings. An unknown
                    # outcome, loss of authority or receiver fault still faults.
                    if not (previous_tuning is not None and exc.reason == 'invalid_state'
                            and exc.status.get('reason') == 'ok'
                            and exc.status.get('state') in ('holding', 'executing')
                            and self.controller.owned and self.lifecycle.state == 'HOLD'):
                        raise
                    self.lifecycle._joints, self.lifecycle._manual, self.lifecycle._manual_hold = previous_tuning
                    previous_tuning = None
                    proposal = self.lifecycle._proposal('move')
                    record.update(proposal=deepcopy(proposal), tuning_rejected=
                                  'The arm is still settling; settings were not applied. Wait in Hold and apply again.')
                    self._deadline(started, target)
                    # No new offer, refreshed lifetime or retry of the edit.
                    reply = self.controller.direct_update(proposal['joints'], offered, valid_for_ms=lifetime,
                                                          ramp_efforts=proposal.get('ramp_efforts'))
                previous_tuning = None  # Receiver acknowledged the whole batch, including fallback gains.
                self._cache(reply, rpc_started)
                status = reply['status']
            else:
                self._cached = None
                if operation == 'none':
                    return record
                if not self.owned:
                    raise ValueError('Idle must acquire ownership before commands')
                lease = offered['status'].get('lease') or {}
                if offered['offer']['expires_at_ms'] > lease.get('expires_at_ms', -1):
                    self.controller.renew_with(offered)
                if operation == 'idle':
                    status = self.controller.idle()
                elif operation == 'enable':
                    status = self.controller.arm()
                elif operation == 'disable':
                    status = self.controller.disable()
                    self.owned = False
                else:
                    raise ValueError(f'Unknown lifecycle operation {operation}')
            record.update(accepted=True, status=status, rpc_ms=(self.clock() - rpc_started) * 1000,
                          receiver_accepted_ms=status['now_ms'],
                          request_sequence=(status.get('lease') or {}).get('last_sequence'),
                          receiver_applied=status.get('last_applied_target'))
            return record
        except Exception as exc:
            if previous_tuning is not None:
                self.lifecycle._joints, self.lifecycle._manual, self.lifecycle._manual_hold = previous_tuning
            record['failure'] = str(exc)
            self.fault(exc)
            self._cached = None
            raise
        finally:
            record.update(completed_at=self.clock(), stop_error=self.stop_error,
                          lifecycle_after=self.lifecycle.state,
                          rpc_count=self.controller.exchanges - exchanges)
