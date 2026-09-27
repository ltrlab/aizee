"""Helm's teleop loop on the robot computer, driven from the operator PC over one SSH channel.

On the robot, from the active release:
    python3 -m robot_platform.helm.edge
reads a start frame (the session bundle, a one-time leader-link key and the
tether address), activates the release, checks that its calibration files are
the ones Helm sent, binds the authenticated leader link and reports its port,
then runs the same loop as on the PC (run_loop + TeleopCore). Keys, route edits
and the zero arrive as frames on stdin; views and replies leave on stdout.

On the PC, EdgeWorker stands in for the loop process in TeleopSession: it opens
the pinned SSH session, serves the leaders over the link from their own
responsive process (LeaderLinkProcess) and relays frames. A servo sweep
(bundle kind 'tune', helm.tune) runs the same way with no leader link.
"""
import json
from pathlib import Path
import queue
import secrets
import sys
import threading
import time

from robot_platform.transport.wire import pack_msg, unpack_msg

ROOT = Path(__file__).resolve().parents[3]
MAX_FRAME = 1 << 22


def write_frame(stream, message):
    body = pack_msg(message)
    stream.write(len(body).to_bytes(4, 'big') + body)
    stream.flush()


def _read_exactly(stream, size):
    data = b''
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise EOFError('The channel closed')
        data += chunk
    return data


def read_frame(stream):
    size = int.from_bytes(_read_exactly(stream, 4), 'big')
    if size > MAX_FRAME:
        raise ValueError(f'A {size}-byte frame exceeds the limit')
    return unpack_msg(_read_exactly(stream, size))


class _End:
    def __init__(self, reason):
        self.reason = reason


class Inbox:
    """Receiving end with a pipe Connection's poll/recv: frames are queued, poll() never blocks."""

    def __init__(self):
        self._queue = queue.Queue()

    def put(self, message):
        self._queue.put(message)

    def end(self, reason=''):
        self._queue.put(_End(reason))

    def poll(self):
        return not self._queue.empty()

    def recv(self):
        message = self._queue.get()
        if isinstance(message, _End):
            self._queue.put(message)  # Every later recv ends too.
            raise EOFError(message.reason)
        return message

    def close(self):
        pass


class ArrivalInbox(Inbox):
    """Views from the robot, stamped when they reach this PC: the robot's clock is not this PC's.

    The viewer ages a guide by this PC's clock, so a robot clock a little ahead
    would make every live pose look stale. Transit over the tether is ~1 ms.
    """

    def put(self, message):
        if isinstance(message, list) and len(message) == 3 and message[0] == 'view' and message[2]:
            message[2]['updated_at'] = time.time()
        super().put(message)


def pump(stream, inbox):
    """Queue frames from `stream` until it ends (EOF, a closed channel or a corrupt frame); returns why."""
    try:
        while True:
            inbox.put(read_frame(stream))
    except Exception as exc:
        return str(exc) or type(exc).__name__


class Outbox:
    """Sending end with a pipe Connection's send/close."""

    def __init__(self, stream):
        self._stream, self._lock = stream, threading.Lock()

    def send(self, message):
        with self._lock:
            write_frame(self._stream, list(message))

    def close(self):
        with self._lock:
            try:
                self._stream.close()
            except OSError:
                pass


# -- Robot side ----------------------------------------------------------------------

def prepare(start, root=ROOT, *, leaders=True):
    """Activate the release, match Helm's calibration and bind the leader link: (bundle, link or None)."""
    from robot_platform.control.joint_calibration import load_plan
    from robot_platform.ops.release import activate
    from robot_platform.teleop.leader_link import LeaderLinkInput
    from .teleop import load_profile
    root = Path(root).resolve()
    activate(root)
    bundle = dict(start['bundle'])
    for key in ('calibration_profile', 'teleop_profile'):
        bundle[key] = str(root / bundle[key])
    if load_plan(bundle['calibration_profile'])[-1] != bundle['bindings']:
        raise ValueError("The robot's calibration files differ from this PC's. "
                         'Deploy (scripts/deploy_minerva_arms.sh) and start again.')
    teleop = load_profile(bundle['teleop_profile'])
    if not leaders:
        return bundle, None
    leaders = json.loads((Path(bundle['teleop_profile']).parent / teleop['leader_profile']).read_text(encoding='utf-8'))
    link = LeaderLinkInput(bytes.fromhex(start['key']), leaders, bind=(start['bind'], 0), timeout_s=.02)
    return bundle, link


def serve(start, commands, views, *, root=ROOT, core_factory=None):
    """One session after its start frame; returns when the loop ends."""
    from .teleop import TeleopCore, run_loop
    tune = (start.get('bundle') or {}).get('kind') == 'tune'
    try:
        bundle, link = prepare(start, root, leaders=False) if tune else prepare(start, root)
    except Exception as exc:
        views.send(['error', str(exc) or type(exc).__name__])
        views.close()
        return 1
    if tune:
        from .tune import TuneCore, run_tune_loop
        views.send(['link', 0])  # No leaders: the sweep plans its own motion.
        run_tune_loop(commands, views, (core_factory or TuneCore)(bundle))
        return 0
    views.send(['link', link.address[1]])

    def open_leaders(_profile):
        link.wait_for_pc(10.)
        return link

    core = (core_factory or TeleopCore)(bundle, open_leaders=open_leaders)
    run_loop(commands, views, core)
    return 0


def main():
    import signal
    # A dropped channel must still let the loop close the receiver cleanly.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    sys.stdout = sys.stderr  # Nothing but frames may reach the channel.
    try:
        start = read_frame(stdin)
    except Exception:
        return 1
    commands = Inbox()
    threading.Thread(target=lambda: commands.end(pump(stdin, commands)), daemon=True, name='edge-commands').start()
    return serve(start, commands, Outbox(stdout))


# -- PC side ---------------------------------------------------------------------------

class _Commands:
    """TeleopSession's sending end; it waits for the robot's loop to accept frames."""

    def __init__(self, worker):
        self._worker = worker

    def send(self, message):
        worker = self._worker
        with worker._lock:
            writer = worker._writer
            if writer is None:
                if message[0] == 'stop':
                    worker.terminate()
                    return
                raise OSError("Minerva's Jetson is still starting the loop")
            write_frame(writer, list(message))

    def close(self):
        with self._worker._lock:
            writer, self._worker._writer = self._worker._writer, None
        if writer is not None:
            try:
                writer.channel.shutdown_write()
            except (OSError, AttributeError):
                pass


class EdgeWorker:
    """Stands in for the local loop process: TeleopSession sends and receives the same messages.

    Start-up failures (unreachable robot, no release, calibration mismatch, no
    leaders) arrive as a fault view naming the reason, then the end of the stream.
    """

    def __init__(self, bundle, leader_profile, *, base_view, address=None, profile=None, open_ssh=None,
                 serve_link=None, real_leaders=True, name='Teleop'):
        from robot_platform.ops.ssh import DEFAULT_PROFILE, TETHER
        self.bundle, self.leader_profile, self.base_view = bundle, leader_profile, base_view
        self.address, self.profile = address or TETHER, profile or DEFAULT_PROFILE
        self.open_ssh, self.serve_link, self.real_leaders = open_ssh, serve_link, real_leaders
        self.name = name
        self.views, self.commands = ArrivalInbox(), _Commands(self)
        self._lock = threading.Lock()
        self._writer = self._channel = None
        self._aborted = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='helm-edge')

    def start(self):
        self._thread.start()

    def is_alive(self):
        return self._thread.is_alive()

    def join(self, timeout=None):
        self._thread.join(timeout)

    def terminate(self):
        self._aborted.set()
        channel = self._channel
        if channel is not None:
            channel.close()

    def _run(self):
        from robot_platform.ops.check import CURRENT
        from robot_platform.ops.ssh import connect, route_source
        client = server = None
        try:
            client = (self.open_ssh or connect)(self.profile, address=self.address)
            if self._aborted.is_set():
                raise RuntimeError('Stopped before the loop started')
            stdin, stdout, stderr = client.exec_command(
                f'cd {CURRENT} && PYTHONPATH=src exec python3 -u -m robot_platform.helm.edge')
            self._channel = stdout.channel
            key = secrets.token_bytes(32)
            write_frame(stdin, {'bundle': self.bundle, 'key': key.hex(), 'bind': self.address})
            try:
                first = read_frame(stdout)
            except EOFError:
                raise RuntimeError(stderr.read().decode(errors='replace').strip()[-400:]
                                   or 'the robot-side loop exited while starting') from None
            if first[0] == 'error':
                raise RuntimeError(first[1])
            if first[0] != 'link':
                raise RuntimeError(f'unexpected first message {first[0]!r} from the robot')
            if self._aborted.is_set():
                raise RuntimeError('Stopped before the loop started')
            if int(first[1]):  # Port 0: this loop takes no leaders.
                if self.serve_link is None:
                    from robot_platform.ops.check import serve_link
                    self.serve_link = serve_link
                server = self.serve_link(self.leader_profile, key, (self.address, int(first[1])),
                                         local=(route_source(self.address) or '0.0.0.0', 0), real=self.real_leaders)
            with self._lock:
                self._writer = stdin
            reason = pump(stdout, self.views)
            status = stdout.channel.recv_exit_status()
            if status not in (0, -1) and not self._aborted.is_set():  # -1: we closed the channel.
                tail = stderr.read().decode(errors='replace').strip()[-400:]
                raise RuntimeError(f'the robot-side loop exited with status {status}: {tail or reason}')
            self.views.end(reason)
        except Exception as exc:
            reason = str(exc) or type(exc).__name__
            self.views.put(['view', dict(self.base_view, phase='fault', fault=reason, joints=[],
                                         message=f"{self.name} on Minerva's Jetson stopped: {reason}"), None])
            self.views.end(reason)
        finally:
            with self._lock:
                self._writer = None
            if server is not None:
                server.close()
            if client is not None:
                client.close()


if __name__ == '__main__':
    raise SystemExit(main())
