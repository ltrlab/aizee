"""Leader samples over a direct network link, for a control loop on the robot.

The robot asks for the latest leader pair with a fresh random challenge and the
operator PC answers from its latest USB sample (`sample_reply`). Source age plus
the whole round trip bounds each sample's age without comparing clocks, as in
the loopback bridge. UDP keeps a late or lost reply to one skipped sample, never
a wedged socket.

Every datagram carries an HMAC-SHA256 tag under a per-session key handed over
the SSH session that starts the robot's loop, so no other host can inject
samples or draw replies. The PC opens the flow with a signed hello (repeated
each second) and the robot learns the PC's address from it, so the PC never
has to accept unsolicited inbound traffic. On the PC, LeaderLinkProcess runs the
server in its own process that Windows does not throttle, next to the leader USB.
"""
import hashlib
import hmac
import socket
import threading
import time
import uuid

from robot_platform.transport.wire import pack_msg, unpack_msg
from .leader_bridge import LeaderSnapshotClient, sample_reply
from .openrb import LeaderSampleSkipped

TAG = 32
MAX_DATAGRAM = 4096
HELLO_S = 1.


def seal(key, body):
    return hmac.new(key, body, hashlib.sha256).digest() + body


def unseal(key, packet):
    """The authenticated message, or None for anything else."""
    if len(packet) <= TAG:
        return None
    tag, body = packet[:TAG], packet[TAG:]
    if not hmac.compare_digest(tag, hmac.new(key, body, hashlib.sha256).digest()):
        return None
    try:
        message = unpack_msg(body)
    except Exception:
        return None
    return message if isinstance(message, dict) else None


def _checked_key(key):
    if not isinstance(key, bytes) or len(key) < 16:
        raise ValueError('A session key of at least 16 bytes is required')
    return key


class LeaderLinkServer:
    """Operator PC: answer authenticated sample requests from the latest USB pair."""

    def __init__(self, inputs, key, robot, *, local=('0.0.0.0', 0), clock=time.perf_counter):
        self.inputs, self.key, self.clock = inputs, _checked_key(key), clock
        self.epoch = str(uuid.uuid4())
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(local)
        self.socket.connect(robot)  # Only the robot's address can reach this socket.
        self.socket.settimeout(.1)
        self.served = self.skipped = self.rejected = 0
        self.failure = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='leader-link-server')

    def start(self):
        self._thread.start()
        return self

    def _send(self, message):
        try:
            self.socket.send(seal(self.key, pack_msg(message)))
        except OSError:
            pass  # The robot is not listening yet; the next hello retries.

    def _run(self):
        hello_at = 0.
        while not self._stop.is_set():
            if self.clock() >= hello_at:
                self._send({'type': 'hello', 'protocol_version': 1, 'epoch': self.epoch})
                hello_at = self.clock() + HELLO_S
            try:
                packet = self.socket.recv(MAX_DATAGRAM)
            except socket.timeout:
                continue
            except OSError:
                continue  # ICMP unreachable from an earlier hello; keep serving.
            request = unseal(self.key, packet)
            if request is None:
                self.rejected += 1
                continue
            nonce = request.get('nonce')
            if self.failure:
                reply = {'protocol_version': 1, 'epoch': self.epoch, 'nonce': nonce, 'error': self.failure}
            else:
                try:
                    reply = sample_reply(request, self.inputs, self.epoch, clock=self.clock)
                    self.served += 1
                except LeaderSampleSkipped as exc:  # Tell the robot at once instead of letting it time out.
                    self.skipped += 1
                    reply = {'protocol_version': 1, 'epoch': self.epoch, 'nonce': nonce, 'skipped': str(exc)}
                except ValueError as exc:
                    self.rejected += 1
                    reply = {'protocol_version': 1, 'epoch': self.epoch, 'nonce': nonce, 'error': str(exc)}
                except Exception as exc:  # The USB sampler failed: every later request gets this error.
                    self.failure = str(exc) or type(exc).__name__
                    reply = {'protocol_version': 1, 'epoch': self.epoch, 'nonce': nonce, 'error': self.failure}
            self._send(reply)

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(1.)
        self.socket.close()


class LeaderLinkInput(LeaderSnapshotClient):
    """Robot side: one fresh authenticated leader pair per poll, or LeaderSampleSkipped when late.

    Refused, lost or late replies are timing only (the lifecycle holds after a few);
    a wrong epoch, roster, sequence or an error from the PC faults the input.
    """

    def __init__(self, key, profile, *, bind=('0.0.0.0', 0), timeout_s=.02, clock=time.perf_counter):
        super().__init__(self._exchange, profile, clock=clock)
        self.inputs = self  # max_age_s / max_skew_s, like LatestLeaderInputs
        self.key, self.timeout_s = _checked_key(key), timeout_s
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(bind)
        self.address = self.socket.getsockname()
        self.peer = None
        self.late = 0

    def wait_for_pc(self, timeout_s=10.):
        """Learn the PC's address from its signed hello."""
        deadline = self.clock() + timeout_s
        while self.peer is None:
            remaining = deadline - self.clock()
            if remaining <= 0:
                raise TimeoutError('No signed hello from the leader PC')
            self.socket.settimeout(remaining)
            try:
                packet, sender = self.socket.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                continue
            message = unseal(self.key, packet)
            if message and message.get('type') == 'hello' and message.get('protocol_version') == 1:
                self.peer = sender
                self.socket.connect(sender)
        return self.peer

    def _exchange(self, request):
        if self.peer is None:
            raise LeaderSampleSkipped('Waiting for the leader PC')
        try:
            self.socket.send(seal(self.key, pack_msg(request)))
        except OSError as exc:
            raise LeaderSampleSkipped(f'Leader PC unreachable: {exc}') from exc
        deadline = self.clock() + self.timeout_s
        while True:
            remaining = deadline - self.clock()
            if remaining <= 0:
                self.late += 1
                raise LeaderSampleSkipped(f'No leader reply within {self.timeout_s * 1000:.0f} ms')
            self.socket.settimeout(remaining)
            try:
                packet = self.socket.recv(MAX_DATAGRAM)
            except socket.timeout:
                continue
            except OSError as exc:  # For example ICMP port unreachable: the PC side stopped.
                raise LeaderSampleSkipped(f'Leader PC unreachable: {exc}') from exc
            reply = unseal(self.key, packet)
            if reply is None or reply.get('type') == 'hello' or reply.get('nonce') != request['nonce']:
                continue  # A hello, a stranger, or a late reply to an earlier request.
            if 'skipped' in reply:
                raise LeaderSampleSkipped(f"Leader PC: {reply['skipped']}")
            if 'error' in reply:
                raise RuntimeError(f"Leader PC: {reply['error']}")
            return reply

    def close(self):
        self.socket.close()


class SyntheticLeaders:
    """A steady, always-fresh leader pair for testing the link without the leader arms."""

    def __init__(self, profile, clock=time.perf_counter):
        self.pins = {name: item['usb_serial'] for name, item in profile['leaders'].items()}
        self.clock, self.sequence, self.skipped = clock, 0, 0

    def poll(self):
        from .openrb import LeaderSample
        self.sequence += 1
        now = self.clock()
        return {name: LeaderSample((0.,) * 7, (0,) * 7, self.sequence, now, now, (0, 0, 0, 0),
                                   10 * self.sequence, 10 * self.sequence + 1) for name in self.pins}

    def close(self):
        pass


def _serve(connection, profile, key, robot, local, real):
    """Child process: open the leaders, answer the robot until told to stop, then report counts."""
    from robot_platform.control.scheduling import prefer_responsive
    scheduling = prefer_responsive()
    source = server = None
    try:
        if real:
            from .leader_bridge import SAMPLER_HZ, LatestLeaderInputs
            from .openrb import OpenRBInputs
            source = LatestLeaderInputs(OpenRBInputs(profile), rate_hz=SAMPLER_HZ)
            source.connect()
        else:
            source = SyntheticLeaders(profile)
        server = LeaderLinkServer(source, key, robot, local=local).start()
        connection.send({'ready': True, 'scheduling': scheduling})
        try:
            connection.recv()  # Any message, or the parent going away, stops the server.
        except EOFError:
            pass
        report = {'served': server.served, 'skipped': server.skipped, 'rejected': server.rejected,
                  'failure': server.failure, 'usb_skipped': source.skipped}
    except Exception as exc:
        report = {'error': f'{type(exc).__name__}: {exc}'}
    finally:
        if server is not None:
            server.close()
        if source is not None:
            source.close()
    try:
        connection.send(report)
    except OSError:
        pass


class LeaderLinkProcess:
    """The operator PC's end of the link in its own process, opted out of Windows throttling.

    A reply later than the robot's timeout is a skipped sample, so the server must
    not share a busy or background-throttled process (see control.scheduling). It
    owns the leader USB ports (real=True) or serves a synthetic pair.
    """

    def __init__(self, profile, key, robot, *, local=('0.0.0.0', 0), real=True):
        import multiprocessing
        context = multiprocessing.get_context('spawn')
        self._connection, self._child = context.Pipe()
        self._process = context.Process(target=_serve, args=(self._child, profile, _checked_key(key), robot,
                                                             local, real), daemon=True, name='leader-link')
        self.scheduling, self.report = None, None

    def start(self, timeout=15.):
        self._process.start()
        self._child.close()
        if not self._connection.poll(timeout):
            self.close()
            raise TimeoutError('The leader link process did not start')
        message = self._connection.recv()
        if 'error' in message:
            self._process.join(2.)
            raise RuntimeError(message['error'])
        self.scheduling = message['scheduling']
        return self

    def close(self, timeout=5.):
        """Stop serving; returns the counts (served, skipped, rejected, failure, usb_skipped)."""
        if self.report is None:
            self.report = {}
            try:
                self._connection.send('stop')
                if self._connection.poll(timeout):
                    self.report = self._connection.recv()
            except (OSError, EOFError):
                pass
            self._process.join(timeout)
            if self._process.is_alive():
                self._process.terminate()
            self._connection.close()
        return self.report


def serve_link(profile, key, robot, *, local, real):
    """The PC end of the leader link, as a session runs it: its own responsive process."""
    return LeaderLinkProcess(profile, key, robot, local=local, real=real).start()
