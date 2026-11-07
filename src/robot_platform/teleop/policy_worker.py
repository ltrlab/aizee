"""The policy in its own process for Helm's policy mode: observations in, action chunks out.

The teleop loop never waits on it. Each cycle the loop may hand it one observation (`ask`: the arms' measured
positions, the receiver's commands and efforts, and the loop clock they were taken at) and collects whatever
answers have arrived (`poll`). The worker adds the cameras' latest frames, calls the policy and sends back its
chunk with the request it answers (teleop.policy_source matches them).

A policy is the evaluation harness's (evaluation.policies, evaluation.runner): `reset(context)`, then
`policy(observation) -> chunk`, rows of receiver motor positions, row k meant for k * `action_dt` after the
observation. The worker reports the policy's name, `action_dt` and camera roles once it has loaded.

On the robot the worker keeps off the control CPUs (scheduling.prefer_background); inference may use the GPU.

It is an ordinary subprocess (`python -m robot_platform.teleop.policy_worker`) speaking length-prefixed pickles on
its stdin and stdout: Helm's teleop loop on a PC runs in a daemonic process, which multiprocessing forbids children.
Everything else the policy's libraries print goes to a log file, never to the frames or to an SSH stream nobody
reads until the session ends.
"""

import math
import multiprocessing
import os
from pathlib import Path
import pickle
import queue
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

LOAD_TIMEOUT_S = 180.0  # Loading a checkpoint onto the Jetson's GPU takes tens of seconds the first time.


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
    conn.send(
        {
            'hello': 1,
            'name': name,
            'action_dt': action_dt,
            'camera_roles': list(roles),
            'describe': policy.describe() if hasattr(policy, 'describe') else None,
        }
    )
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
                conn.send({'request': request, 'chunk': chunk, 'inference_ms': (time.perf_counter() - started) * 1000})
            except Exception as exc:  # noqa: BLE001 - one failed answer; the loop asks again or holds.
                conn.send({'request': request, 'error': f'{type(exc).__name__}: {exc}'})
    finally:
        if camera is not None:
            camera.close()


class CaptureCameras:
    """The robot's cameras for a policy, read as the task capture reads them (helm.task_capture): the capture
    profile's loopback endpoints, each JPEG decoded at the recording size by a CameraCache, the newest coherent set
    within the profile's age and skew bounds, transformed as `recording_images` stores it. A policy thus sees the
    pixels its training episodes stored. It travels to the worker process as its profile; the sockets open there.
    No fresh coherent set: the observation fails, the policy's chunks go stale and the arms hold."""

    def __init__(self, config, *, context_factory=None, clock=time.monotonic, wall_clock=time.time):
        from robot_platform.helm.task_capture import validate_config

        self.config = validate_config(config)
        self.context_factory, self.clock, self.wall_clock = context_factory, clock, wall_clock

    def open(self, roles):
        missing = [role for role in roles if role not in self.config['camera_roles']]
        if missing:
            raise ValueError(f'The policy needs the {", ".join(missing)} camera, which the capture profile lacks')
        return _CameraReader(self, tuple(roles))


class _CameraReader:
    def __init__(self, cameras, roles):
        import zmq
        from robot_platform.helm.task_capture import CameraCache

        self.cameras, self.roles = cameras, roles
        self._cache, self._lock, self._stop = CameraCache(cameras.config), threading.Lock(), threading.Event()
        self._context = (cameras.context_factory or zmq.Context)()
        self._sockets = {}
        for name in cameras.config['camera_roles']:  # Every camera: a set is coherent only across all of them.
            socket = self._context.socket(zmq.SUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 2)
            socket.setsockopt(zmq.MAXMSGSIZE, 16 * 1024 * 1024)
            socket.setsockopt(zmq.SUBSCRIBE, b'')
            socket.connect(cameras.config['cameras'][name]['endpoint'])
            self._sockets[name] = socket
        self._thread = threading.Thread(target=self._run, daemon=True, name='policy-cameras')
        self._thread.start()

    def _run(self):
        import zmq

        poller = zmq.Poller()
        for socket in self._sockets.values():
            poller.register(socket, zmq.POLLIN)
        while not self._stop.is_set():
            events = dict(poller.poll(10))
            for name, socket in self._sockets.items():
                if socket not in events:
                    continue
                for _ in range(8):
                    try:
                        parts = socket.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    received = self.cameras.clock()
                    with self._lock:
                        self._cache.ingest(
                            name,
                            parts,
                            received_at=received,
                            wall_time=self.cameras.wall_clock(),
                            mapped_at=self.cameras.clock(),
                        )

    def latest(self):
        """({role: (H, W, 3) uint8 as recorded}, {role: age in s}); a ValueError without a fresh coherent set."""
        from robot_platform.helm.task_capture import recording_images

        now = self.cameras.clock()
        with self._lock:
            frames = self._cache.select(now)
        images, _ = recording_images(frames, self.cameras.config.get('recording_image_size'))
        return (
            {role: images[role] for role in self.roles},
            {role: round(now - frames[role]['sampled_at'], 4) for role in self.roles},
        )

    def close(self):
        self._stop.set()
        self._thread.join(2.0)
        for socket in self._sockets.values():
            socket.close(0)
        self._context.term()


def _read_frame(stream):
    header = stream.read(4)
    if len(header) < 4:
        raise EOFError('The policy pipe closed')
    size = int.from_bytes(header, 'big')
    body = b''
    while len(body) < size:
        chunk = stream.read(size - len(body))
        if not chunk:
            raise EOFError('The policy pipe closed')
        body += chunk
    return pickle.loads(body)


def _write_frame(stream, message):
    body = pickle.dumps(message, protocol=4)
    stream.write(len(body).to_bytes(4, 'big') + body)
    stream.flush()


class _ChildEnd:
    """The worker's end of its stdin/stdout: blocking receive, send."""

    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    def recv(self):
        return _read_frame(self.reader)

    def send(self, message):
        _write_frame(self.writer, message)


class _ProcessEnd:
    """The loop's end of the worker's pipes: send, and a reader thread so poll never blocks."""

    def __init__(self, process):
        self.process, self._lock, self._inbox = process, threading.Lock(), queue.Queue()
        threading.Thread(target=self._read, daemon=True, name='helm-policy-reader').start()

    def _read(self):
        try:
            while True:
                self._inbox.put(_read_frame(self.process.stdout))
        except (EOFError, OSError, ValueError, pickle.UnpicklingError):
            self._inbox.put(EOFError)

    def send(self, message):
        with self._lock:
            _write_frame(self.process.stdin, message)

    def poll(self, timeout=0.0):
        if not self._inbox.empty():
            return True
        if timeout <= 0:
            return False
        try:
            item = self._inbox.get(timeout=timeout)
        except queue.Empty:
            return False
        self._inbox.queue.appendleft(item)
        return True

    def recv(self):
        item = self._inbox.get()
        if item is EOFError:
            self._inbox.put(EOFError)  # Every later read ends too.
            raise EOFError('The policy process ended')
        return item

    def close(self):
        for stream in (self.process.stdin, self.process.stdout):
            try:
                stream.close()
            except OSError:
                pass


class PolicyWorker:
    """The loop's handle on the policy process (or, for tests, a thread on a pipe). It starts loading at once;
    `ready` waits for it, so the loop can open its receiver meanwhile. `python`: the interpreter to run it with
    (default this one)."""

    def __init__(self, policy, names, *, context=None, cameras=None, in_process=False, python=None):
        args = (policy, list(names), context, cameras)
        self.info, self.log_path, self._log = None, None, None
        if in_process:
            self._conn, child = multiprocessing.Pipe()
            self.process = threading.Thread(target=serve, args=(child, *args), daemon=True, name='helm-policy')
            self.process.start()
            return
        self.log_path = Path(tempfile.gettempdir()) / f'minerva-policy-{os.getpid()}-{time.time_ns()}.log'
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            [python or sys.executable, '-u', '-m', 'robot_platform.teleop.policy_worker'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self._log,
        )
        self._conn = _ProcessEnd(self.process)
        self._conn.send(args)

    @property
    def alive(self):
        return self.process.is_alive() if isinstance(self.process, threading.Thread) else self.process.poll() is None

    def ready(self, timeout_s=LOAD_TIMEOUT_S):
        """The policy's name, action_dt and camera roles, once it has loaded; raises if it could not."""
        if self.info is None:
            if not self._conn.poll(timeout_s):
                raise RuntimeError(f'The policy did not load within {timeout_s:g} s')
            try:
                hello = self._conn.recv()
            except EOFError:
                raise RuntimeError(f'The policy process ended while loading{self._log_tail()}') from None
            if 'error' in hello:
                raise RuntimeError(f'The policy could not start: {hello["error"]}')
            self.info = hello
        return self.info

    def _log_tail(self):
        if self.log_path is None:
            return ''
        try:
            tail = self.log_path.read_bytes()[-400:].decode(errors='replace').strip()
        except OSError:
            return ''
        return f': {tail}' if tail else ''

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

    def close(self, timeout=5.0):
        try:
            self._conn.send(('close',))
        except (OSError, ValueError, BrokenPipeError):
            pass
        if isinstance(self.process, threading.Thread):
            self.process.join(timeout)
            self._conn.close()
            return
        try:
            self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()  # Still loading, or stuck in inference: it holds nothing the arms need.
            try:
                self.process.wait(timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout)
        self._conn.close()
        if self._log is not None:
            self._log.close()


def main():
    """The worker process: its arguments arrive as the first frame on stdin, frames leave on the original stdout,
    and anything else printed goes to stderr (the loop's log file)."""
    frames = os.fdopen(os.dup(sys.stdout.fileno()), 'wb')
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    conn = _ChildEnd(sys.stdin.buffer, frames)
    try:
        policy, names, context, cameras = conn.recv()
    except EOFError:
        return 0
    serve(conn, policy, names, context, cameras)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
