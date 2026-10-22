"""The policy in its own process for Helm's policy mode: observations in, action chunks out.

The teleop loop never waits on it. Each cycle the loop may hand it one observation (`ask`: the arms' measured
positions, the receiver's commands and efforts, and the loop clock they were taken at) and collects whatever
answers have arrived (`poll`). The worker adds the cameras' latest frames, calls the policy and sends back its
chunk with the request it answers (teleop.policy_source matches them).

A policy is the evaluation harness's (evaluation.policies, evaluation.runner): `reset(context)`, then
`policy(observation) -> chunk`, rows of receiver motor positions, row k meant for k * `action_dt` after the
observation. The worker reports the policy's name, `action_dt` and camera roles once it has loaded.

On the robot the worker keeps off the control CPUs (scheduling.prefer_background); inference may use the GPU.
"""
import math
import multiprocessing
import threading
import time
from types import SimpleNamespace

LOAD_TIMEOUT_S = 180.  # Loading a checkpoint onto the Jetson's GPU takes tens of seconds the first time.


def _context(context):
    import numpy as np
    values = dict(context or {})
    for key in ('lower', 'upper', 'home'):
        if values.get(key) is not None:
            values[key] = np.asarray(values[key], dtype=float)
    return SimpleNamespace(index=0, cube_start=None, **values)


def serve(conn, policy, names, context=None, cameras=None):
    """The worker's loop: load `policy` (a spec for evaluation.policies.load_policy, or a policy object), then
    answer ('observe', request, obs_time, state) messages until ('close',) or the pipe closes."""
    import numpy as np
    from robot_platform.control.scheduling import prefer_background
    prefer_background()
    try:
        if isinstance(policy, str):
            from robot_platform.evaluation.policies import load_policy
            policy = load_policy(policy)
        if hasattr(policy, 'reset'):
            policy.reset(_context(dict(context or {}, joint_names=tuple(names))))
        roles = tuple(getattr(policy, 'camera_roles', None) or ())
        camera = cameras.open(roles) if roles and cameras is not None else None
        if roles and camera is None:
            raise ValueError(f'The policy needs the cameras {", ".join(roles)}, and this session has none')
        # Seconds between chunk rows; by default the loop's period, as the evaluation runner defaults to its own.
        action_dt = float(getattr(policy, 'action_dt', None) or (context or {}).get('period_s') or 0)
        if not math.isfinite(action_dt) or action_dt <= 0:
            raise ValueError('The policy does not say how far apart its chunk rows are (action_dt)')
    except Exception as exc:  # noqa: BLE001 - the loop reports why the policy could not start.
        conn.send({'error': f'{type(exc).__name__}: {exc}'})
        return
    name = getattr(policy, 'name', None) or type(policy).__name__
    conn.send({'hello': 1, 'name': name, 'action_dt': action_dt, 'camera_roles': list(roles),
               'describe': policy.describe() if hasattr(policy, 'describe') else None})
    try:
        while True:
            try:
                message = conn.recv()
            except (EOFError, OSError):
                return
            if message[0] == 'close':
                return
            _, request, obs_time, state = message
            started = time.perf_counter()
            try:
                observation = dict(state, t=obs_time, joint_names=tuple(names))
                if camera is not None:
                    observation['images'], observation['image_ages_s'] = camera.latest()
                chunk = np.asarray(policy(observation), dtype=np.float64)
                conn.send({'request': request, 'chunk': chunk,
                           'inference_ms': (time.perf_counter() - started) * 1000})
            except Exception as exc:  # noqa: BLE001 - one failed answer; the loop asks again or holds.
                conn.send({'request': request, 'error': f'{type(exc).__name__}: {exc}'})
    finally:
        if camera is not None:
            camera.close()


class PolicyWorker:
    """The loop's handle on the policy process (or, for tests, a thread with the same pipe). It starts loading at
    once; `ready` waits for it, so the loop can open its receiver meanwhile."""

    def __init__(self, policy, names, *, context=None, cameras=None, in_process=False):
        ctx = multiprocessing.get_context('spawn')
        self._conn, child = ctx.Pipe()
        args = (child, policy, list(names), context, cameras)
        if in_process:
            self.process = threading.Thread(target=serve, args=args, daemon=True, name='helm-policy')
        else:
            self.process = ctx.Process(target=serve, args=args, daemon=True, name='helm-policy')
        self.process.start()
        if not in_process:
            child.close()  # The child owns its end; EOF then means it exited.
        self.info = None

    def ready(self, timeout_s=LOAD_TIMEOUT_S):
        """The policy's name, action_dt and camera roles, once it has loaded; raises if it could not."""
        if self.info is None:
            if not self._conn.poll(timeout_s):
                raise RuntimeError(f'The policy did not load within {timeout_s:g} s')
            hello = self._conn.recv()
            if 'error' in hello:
                raise RuntimeError(f"The policy could not start: {hello['error']}")
            self.info = hello
        return self.info

    def ask(self, request, obs_time, state):
        self._conn.send(('observe', request, obs_time, state))

    def poll(self):
        """The answers that have arrived, without waiting."""
        replies = []
        try:
            while self._conn.poll():
                replies.append(self._conn.recv())
        except (EOFError, OSError):
            replies.append({'request': None, 'error': 'The policy process ended'})
        return replies

    def close(self, timeout=5.):
        try:
            self._conn.send(('close',))
        except (OSError, ValueError):
            pass
        self.process.join(timeout)
        if self.process.is_alive() and hasattr(self.process, 'terminate'):
            self.process.terminate()  # Still loading, or stuck in inference: it holds nothing the arms need.
            self.process.join(timeout)
        self._conn.close()
