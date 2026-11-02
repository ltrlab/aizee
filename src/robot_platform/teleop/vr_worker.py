"""The headset's grab and drag in its own process, for Helm's Headset input: the controllers in, each arm's commanded
CAD angles and gripper closure out (teleop.vr_arms), on a grid of the arms' loop period (VR arm control,
docs/development/VR_ARM_TELEOP_PLAN.md, phase 4).

The teleop loop never waits on it. Hand samples and seeds go to a sender thread that sends only the newest of each;
the worker's answers come back on a reader thread that keeps only the newest (teleop.headset_source takes it). The
worker stamps each hand sample on arrival with its own clock and updates on its own grid, so no two computers' clocks
are ever compared. Each answer names the seed it was computed from: one computed before the loop's latest seed is
not followed. A B/Y press is counted, not flagged, so one is never lost between two answers.

Timed on Minerva's Jetson (2026-11-02, background CPUs at nice 10, numpy single-threaded): both arms take 3.4 ms an
update (median), 6.6 ms (99th percentile) and 12.4 ms at most at 50 Hz, none past the 20 ms period; a warm IK solve
0.7 ms. On the robot the worker keeps off the control CPUs (scheduling.prefer_background) at nice 10.

It is an ordinary subprocess (`python -m robot_platform.teleop.vr_worker`) speaking policy_worker's length-prefixed
pickles on its stdin and stdout: Helm's teleop loop on a PC runs in a daemonic process, which multiprocessing forbids
children. Everything it prints goes to a log file.
"""
import math
import multiprocessing
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

from .policy_worker import _ChildEnd, _read_frame, _write_frame

START_TIMEOUT_S = 30.  # Building both arms' chains takes well under a second; importing numpy cold, a few.


def _sample(value, at):
    """A controller as Helm checked it (helm.xr.hand_sample: position, orientation, grip, trigger, hold, ui)."""
    from .vr_arms import HandSample
    return HandSample(tuple(value['position']), tuple(value['orientation']), float(value['grip']),
                      float(value['trigger']), bool(value['hold']), at, bool(value['ui']))


def _answer(commands, holds, triggers):
    """Each arm's command, its count of B/Y presses, and its trigger (None without a fresh hand off the screens): an
    arm not yet seeded still says where its trigger is, for the gripper Engage aligns to."""
    return {arm: {'positions': c.positions, 'closure': c.closure, 'gripped': c.gripped, 'tracking': c.tracking,
                  'clamped': c.clamped, 'converged': c.converged, 'at_limit': list(c.at_limit),
                  'sigma': None if not math.isfinite(c.sigma) else c.sigma, 'near_singular': c.near_singular,
                  'holds': holds[arm], 'trigger': triggers.get(arm)}
            for arm, c in commands.items()}


def build(setup):
    """The worker's VRArms from `setup`: {'model': the model profile's path, 'verified': {mesh: sha256} (the release's,
    on the robot), 'limits': {arm: {cad joint: (low, high)}}, 'settings': DragSettings fields}."""
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.models.urdf import register_verified_assets
    from .arm_ik import ArmKinematics
    from .vr_arms import DragSettings, VRArms
    if setup.get('verified'):
        register_verified_assets(setup['verified'])  # No CAD files on the robot: its release vouches for them.
    model = UrdfModel(ModelProfile.load(setup['model']))
    kinematics = {arm: ArmKinematics(model, arm, limits) for arm, limits in setup['limits'].items()}
    return VRArms(kinematics, DragSettings(**(setup.get('settings') or {})))


def serve(conn, setup, *, clock=time.monotonic, vr=None, own_process=True):
    """The worker's loop: build (or take `vr`), say hello, then update every `setup['period_s']` from the newest
    messages until ('close',) or the pipe closes. Messages: ('hands', {arm: sample}, epoch, forward_yaw) and ('seed',
    number, {arm: CAD positions}). In a process of its own it moves off the control CPUs first."""
    if own_process:
        from robot_platform.control.scheduling import prefer_background
        prefer_background()
        try:
            os.nice(10)
        except (AttributeError, OSError):
            pass
    try:
        vr = vr or build(setup)
        period = float(setup['period_s'])
        if not math.isfinite(period) or period <= 0:
            raise ValueError('The update period is a positive number of seconds')
    except Exception as exc:  # noqa: BLE001 - the loop reports why the headset could not start.
        conn.send({'error': f'{type(exc).__name__}: {exc}'})
        return
    conn.send({'hello': 1, 'arms': sorted(vr.arms)})
    lock, inbox = threading.Lock(), {'hands': {}, 'epoch': None, 'forward': None, 'seed': None, 'closed': False}

    def read():
        try:
            while True:
                message = conn.recv()
                with lock:
                    if message[0] == 'close':
                        inbox['closed'] = True
                        return
                    if message[0] == 'hands':
                        _, hands, epoch, forward = message
                        at = clock()  # Stamped on arrival: this process's clock alone.
                        for arm, value in hands.items():
                            inbox['hands'][arm] = (value, at)
                        inbox['epoch'], inbox['forward'] = epoch, forward
                    elif message[0] == 'seed':
                        inbox['seed'] = message[1:]
        except (EOFError, OSError):
            with lock:
                inbox['closed'] = True

    threading.Thread(target=read, daemon=True, name='vr-worker-read').start()
    seeded, epoch, forward = 0, None, None
    holds = {arm: 0 for arm in vr.arms}
    tick = clock()
    while True:
        tick += period
        delay = tick - clock()
        if delay > 0:
            time.sleep(delay)
        else:
            tick = clock()  # Behind: start the grid again from now rather than run late updates back to back.
        with lock:
            if inbox['closed']:
                return
            seed, inbox['seed'] = inbox['seed'], None
            hands = dict(inbox['hands'])
            new_epoch, new_forward = inbox['epoch'], inbox['forward']
        started = time.perf_counter()
        try:
            if seed is not None:
                seeded, positions = seed
                for arm, q in positions.items():
                    if arm in vr.arms:
                        vr.seed(arm, q)
            if new_epoch is not None and new_epoch != epoch:
                if epoch is not None:
                    vr.recenter()  # The headset's space started again: hands attach afresh where they are.
                epoch = new_epoch
            if new_forward is not None and new_forward != forward:
                vr.set_forward(new_forward)
                forward = new_forward
            now = clock()
            samples = {arm: _sample(value, at) for arm, (value, at) in hands.items() if arm in vr.arms}
            commands = vr.update(now, samples)
            for arm, command in commands.items():
                holds[arm] += bool(command.hold)
            triggers = {arm: min(1., max(0., sample.trigger)) for arm, sample in samples.items()
                        if now - sample.at <= vr.settings.stale_s and not sample.ui}
            conn.send({'seed': seeded, 'commands': _answer(commands, holds, triggers), 'at': now,
                       'compute_ms': (time.perf_counter() - started) * 1000})
        except (EOFError, OSError, BrokenPipeError):
            return
        except Exception as exc:  # noqa: BLE001 - one bad update; the loop holds while answers are missing.
            try:
                conn.send({'seed': seeded, 'error': f'{type(exc).__name__}: {exc}'})
            except (EOFError, OSError):
                return


class VRWorker:
    """The loop's handle on the worker process (or, for tests, a thread on a pipe). It starts building at once; `ready`
    waits for it. `hands` and `seed` never block: a sender thread sends the newest of each. `latest` is the newest
    answer with when it arrived (`clock`, the loop's)."""

    def __init__(self, setup, *, in_process=False, python=None, clock=time.monotonic, vr=None):
        self.clock = clock
        self.info, self.error, self.log_path, self._log = None, None, None, None
        self._latest = None
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._hands = self._seed = None
        self._closing = False
        self._hello = threading.Event()
        if in_process:
            self._conn, child = multiprocessing.Pipe()
            self.process = threading.Thread(target=serve, args=(child, setup), daemon=True, name='vr-worker',
                                            kwargs={'vr': vr, 'own_process': False})
            self.process.start()
        else:
            self.log_path = Path(tempfile.gettempdir()) / f'minerva-vr-{os.getpid()}-{time.time_ns()}.log'
            self._log = open(self.log_path, 'wb')
            # Small matrices: BLAS threads would only contend with the cameras on the background CPUs.
            env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
            self.process = subprocess.Popen([python or sys.executable, '-u', '-m', 'robot_platform.teleop.vr_worker'],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log, env=env)
            self._conn = _Pipes(self.process)
            self._conn.send(setup)
        self._threads = [threading.Thread(target=self._send, daemon=True, name='vr-worker-send'),
                         threading.Thread(target=self._read, daemon=True, name='vr-worker-reader')]
        for thread in self._threads:
            thread.start()

    @property
    def alive(self):
        return self.process.is_alive() if isinstance(self.process, threading.Thread) else self.process.poll() is None

    def ready(self, timeout_s=START_TIMEOUT_S):
        """The worker's hello ({'arms': [...]}) once it has built its arms; raises if it could not."""
        if not self._hello.wait(timeout_s):
            raise RuntimeError(f'The headset worker did not start within {timeout_s:g} s{self._log_tail()}')
        with self._lock:
            if self.info is None:
                raise RuntimeError(f'The headset worker could not start: {self.error}{self._log_tail()}')
            return self.info

    def hands(self, hands, epoch, forward):
        """The controllers' newest samples as Helm checked them ({arm: {position, orientation, grip, trigger, hold,
        ui}}), the headset's epoch and forward yaw."""
        with self._wake:
            self._hands = ('hands', hands, epoch, forward)
            self._wake.notify()

    def seed(self, number, positions):
        """Seed `number`: each arm's commanded CAD positions are now `positions` ({arm: six angles})."""
        with self._wake:
            self._seed = ('seed', number, positions)
            self._wake.notify()

    def latest(self):
        """(the newest answer, when it arrived on the loop's clock), or None before the first."""
        with self._lock:
            return self._latest

    def _send(self):
        while True:
            with self._wake:
                self._wake.wait_for(lambda: self._hands is not None or self._seed is not None or self._closing)
                seed, hands, closing = self._seed, self._hands, self._closing
                self._seed = self._hands = None
            try:
                for message in (seed, hands):  # A seed first: the hands that follow it count from it.
                    if message is not None:
                        self._conn.send(message)
                if closing:
                    self._conn.send(('close',))
                    return
            except (OSError, EOFError, ValueError, BrokenPipeError):
                return

    def _read(self):
        try:
            while True:
                message = self._conn.recv()
                with self._lock:
                    if 'hello' in message:
                        self.info = message
                        self._hello.set()
                    elif 'commands' in message:
                        self._latest = (message, self.clock())
                    elif 'error' in message:
                        self.error = message['error']
                        self._hello.set()
        except (EOFError, OSError):
            with self._lock:
                if not self._closing:
                    self.error = self.error or 'The headset worker ended'
                self._hello.set()

    def _log_tail(self):
        if self.log_path is None:
            return ''
        try:
            tail = self.log_path.read_bytes()[-400:].decode(errors='replace').strip()
        except OSError:
            return ''
        return f': {tail}' if tail else ''

    def close(self, timeout=5.):
        with self._wake:
            self._closing = True
            self._wake.notify_all()
        self._threads[0].join(timeout)
        if isinstance(self.process, threading.Thread):
            self.process.join(timeout)
            return
        try:
            self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()  # It holds nothing the arms need: they follow the loop's own targets.
            try:
                self.process.wait(timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout)
        self._conn.close()
        if self._log is not None:
            self._log.close()


class _Pipes:
    """The loop's end of the worker's stdin and stdout."""

    def __init__(self, process):
        self.process, self._lock = process, threading.Lock()

    def send(self, message):
        with self._lock:
            _write_frame(self.process.stdin, message)

    def recv(self):
        return _read_frame(self.process.stdout)

    def close(self):
        for stream in (self.process.stdin, self.process.stdout):
            try:
                stream.close()
            except OSError:
                pass


def main():
    """The worker process: its setup arrives as the first frame on stdin, frames leave on the original stdout, and
    anything else printed goes to stderr (the loop's log file)."""
    frames = os.fdopen(os.dup(sys.stdout.fileno()), 'wb')
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    conn = _ChildEnd(sys.stdin.buffer, frames)
    try:
        setup = conn.recv()
    except EOFError:
        return 0
    serve(conn, setup)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
