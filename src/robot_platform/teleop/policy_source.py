"""A policy's action chunks as the teleop loop's target, in place of the leaders (Helm's policy mode).

The policy runs in its own process (helm.policy_worker) and answers an observation (the arms' measured positions
and the cameras, taken at the loop's clock `obs_time`) with a chunk: rows of receiver motor positions, row k meant
for obs_time + k * action_dt (evaluation.runner's contract). Inference takes real time on the robot, so a chunk
arrives late: the source starts it at the row for now and never replays the rows already past.

What the arms do, by lifecycle state:
- HOLD: the source asks for one chunk from where the arms are and offers its first row, unmoving: Engage aligns to
  it at the lifecycle's alignment speed, as it aligns to the leaders.
- TELEOP: the rollout. The source asks for a new chunk whenever none is pending and follows the newest in real
  time, crossfading from the one it replaces over `blend_s` so a disagreement between chunks is never a step.
- A chunk is followed only while it is fresh: within `max_age_s` of its observation (past its last row it holds
  that row). Without one the source gives no target, and the lifecycle treats that as it treats lost leaders.
- Anything else (disabled, idle, aligning, holding after a pause) asks for nothing new.

The loop sees an ordinary target: every move still goes through the lifecycle (alignment, speed limits) and the
receiver's limits; the operator's Hold, Return, Disable and the foot switch stay in charge.
"""
import math

ROLLING = ('TELEOP',)
# On the arms a policy session runs a trained checkpoint, or hold_still to try the mode. The harness's other
# reference policies check the simulation: one drives a gripper down through the table on purpose.
LIVE_REFERENCE = ('hold_still',)
CHECKPOINTS = ('checkpoint:', 'lerobot:')


def validate_request(payload, *, live):
    """Helm's policy request: {'policy': a spec for evaluation.policies.load_policy}."""
    if (not isinstance(payload, dict) or set(payload) - {'kind', 'policy'} or payload.get('kind', 'policy') != 'policy'
            or not isinstance(payload.get('policy'), str)):
        raise ValueError('A policy session names its policy')
    spec = payload['policy'].strip()
    if not spec or len(spec) > 1000:
        raise ValueError('A policy session names its policy')
    if live and not (spec in LIVE_REFERENCE or spec.startswith(CHECKPOINTS)):
        raise ValueError('On the arms a policy session runs a trained checkpoint (checkpoint:<path> or '
                         "lerobot:<path>) or hold_still; the harness's other policies are for the simulation")
    return {'kind': 'policy', 'policy': spec}


class PolicySource:
    def __init__(self, names, *, action_dt, max_age_s=1., blend_s=.2, policy=None):
        if not names:
            raise ValueError('A policy drives at least one joint')
        for value, label in ((action_dt, 'action_dt'), (max_age_s, 'max_age_s')):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{label} must be a positive number of seconds')
        if type(blend_s) not in (int, float) or not math.isfinite(blend_s) or blend_s < 0:
            raise ValueError('blend_s must be zero or a positive number of seconds')
        self.names, self.policy = tuple(names), policy
        self.action_dt, self.max_age_s, self.blend_s = float(action_dt), float(max_age_s), float(blend_s)
        self.chunk = None        # The newest chunk: {'obs_time', 'rows', 'received_at', 'request'}.
        self.previous = None     # The chunk it replaced, faded out over blend_s from the new one's arrival.
        self.pending = None      # The request awaiting its chunk: {'request', 'obs_time', 'purpose'}.
        self.aligned = None      # HOLD/ENGAGING: the row the arms align to, from the preview chunk.
        self.requests = self.chunks = self.stale_cycles = 0
        self.sequence = 0
        self.last_error = None
        self.inference_ms = None
        self._state = None
        self._rolling_since = None

    # -- the worker's side ------------------------------------------------------------------

    def wants(self, state):
        """'preview' or 'rollout' when the source needs a chunk now (none pending), else None."""
        if self.pending is not None:
            return None
        if state == 'HOLD' and self.aligned is None:
            return 'preview'
        if state in ROLLING:
            return 'rollout'
        return None

    def asked(self, request, obs_time, purpose):
        """The loop sent observation `request`, taken at `obs_time` (its clock), for `purpose`."""
        self.pending = {'request': request, 'obs_time': float(obs_time), 'purpose': purpose}
        self.requests += 1

    def receive(self, request, chunk, now, *, inference_ms=None):
        """A chunk answering `request`. One answering anything but the pending request is dropped (the session
        paused or the request was replaced); a malformed one raises."""
        import numpy as np
        pending = self.pending
        if pending is None or request != pending['request']:
            return False
        self.pending = None
        rows = np.asarray(chunk, dtype=np.float64)
        if rows.ndim == 1:
            rows = rows[None, :]
        if rows.ndim != 2 or rows.shape[1] != len(self.names) or len(rows) < 1 or not np.all(np.isfinite(rows)):
            raise ValueError(f'A policy answers with a finite (rows, {len(self.names)}) chunk, not {rows.shape}')
        self.chunks += 1
        self.inference_ms = inference_ms
        received = {'obs_time': pending['obs_time'], 'rows': rows, 'received_at': float(now), 'request': request}
        if pending['purpose'] == 'preview':
            self.aligned = rows[0].copy()
            return True
        if self._fresh(self.chunk, now):
            self.previous = self.chunk
        self.chunk = received
        return True

    def failed(self, request, error):
        """The worker could not answer `request`; the next cycle asks again."""
        if self.pending is not None and request == self.pending['request']:
            self.pending = None
        self.last_error = str(error)

    # -- the loop's side ---------------------------------------------------------------------

    def _fresh(self, chunk, now):
        """Observed within max_age_s. Past its last row a chunk holds that row, as the evaluation runner does: a
        one-row chunk is a pose to hold until the next."""
        return chunk is not None and 0 <= now - chunk['obs_time'] <= self.max_age_s

    def _row(self, chunk, now):
        rows = chunk['rows']
        f = max(0., (now - chunk['obs_time']) / self.action_dt)
        k = int(math.floor(f + 1e-9))
        if k >= len(rows) - 1:
            return rows[-1]
        w = f - k
        return rows[k] * (1 - w) + rows[k + 1] * w

    def position(self, now):
        """The rollout's position now: the newest fresh chunk, blended in from the one it replaced; None if stale."""
        if not self._fresh(self.chunk, now):
            return None
        q = self._row(self.chunk, now)
        if self.previous is not None and self.blend_s > 0:
            w = (now - self.chunk['received_at']) / self.blend_s
            if w < 1 and self._fresh(self.previous, now):
                q = self._row(self.previous, now) * (1 - max(w, 0.)) + q * max(w, 0.)
            else:
                self.previous = None
        return q

    def pause(self):
        """Hold or a fault ended the rollout: its chunks are not followed again, and Engage aligns anew."""
        self.chunk = self.previous = self.aligned = None
        self.pending = None

    def expire(self, now):
        """A request unanswered for max_age_s is given up: its chunk would be stale, and the next cycle asks again."""
        if self.pending is not None and now - self.pending['obs_time'] > self.max_age_s:
            self.last_error = f'The policy did not answer within {self.max_age_s:g} s'
            self.pending = None

    def target(self, state, now):
        """This cycle's target rows: the aligned preview row before following, the rollout while following."""
        self.expire(now)
        if self._state in ROLLING and state not in ROLLING:
            self.pause()  # Hold (or a fault) ended the rollout: Engage aligns anew from where the arms are.
        if state in ROLLING and self._state not in ROLLING:
            self._rolling_since = now
        self._state = state
        if state in ROLLING:
            q = self.position(now)
            if (q is None and self.chunk is None and self.aligned is not None
                    and now - self._rolling_since <= self.max_age_s):
                q = self.aligned  # Following has just begun: hold the aligned row until the first chunk.
            if q is None:
                self.stale_cycles += 1
                return None
        elif state in ('HOLD', 'ENGAGING'):
            q = self.aligned
            if q is None:
                return None
        else:
            self.pause()
            return None
        self.sequence += 1
        return [dict(name=name, position=float(value), sampled_at=now, sequence=self.sequence)
                for name, value in zip(self.names, q)]

    def snapshot(self, now=None):
        chunk = self.chunk
        return dict(policy=getattr(self.policy, 'name', None) if self.policy is not None else None,
                    requests=self.requests, chunks=self.chunks, stale_cycles=self.stale_cycles,
                    pending=self.pending['purpose'] if self.pending else None,
                    aligned=self.aligned is not None, inference_ms=self.inference_ms,
                    chunk_age_s=None if chunk is None or now is None else round(now - chunk['obs_time'], 3),
                    error=self.last_error)
