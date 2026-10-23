"""The GELLO bridge's Helm end, for Helm running on Minerva's Jetson (`helm --on-robot`).

The leader arms stay on the operator PC. Its bridge (teleop.gello_bridge) keeps one pinned SSH session to the Jetson
over the tether, whose key may run nothing but this module's relay (`python3 -m robot_platform.helm.bridge relay`, a
forced command). The relay joins Helm's BridgeHub on a loopback port, proving itself with a token only this user can
read, and passes frames both ways. So:

- a session's leader-link key reaches the PC encrypted (over SSH, as when Helm ran on the PC), and only the PC whose
  key is authorized can be the bridge;
- the PC opens every connection: it accepts nothing inbound;
- the leader link itself is unchanged. The bridge runs the PC end (teleop.leader_link.LeaderLinkProcess) toward the
  robot's loop over the tether, in its own responsive process: the same code, process and path, so the same latency.

`BridgeHub.serve_link` stands in for leader_link.serve_link in a robot-side session (helm.edge.EdgeWorker), and
`BridgeHub.open_leaders` for the leaders the Leaders workspace reads (helm.leaders), as samples the bridge streams with
their age: the PC's clock never meets the Jetson's.

Frames, each a 4-byte length and a msgpack body (helm.edge), from Helm to the bridge:
    ['beat'] | ['link', id, {profile, key, robot: [address, port], real}] | ['link_stop', id]
    | ['leaders_open', id, {profile, rate_hz}] | ['leaders_close', id]
and from the bridge:
    ['hello', {version, host, ...}] (answered by ['welcome', {version, machine_id, host}]) | ['beat'] | ['link_up', id] | ['link_report', id, report]
    | ['leaders_up', id] | ['leaders', id, {leader: sample}] | ['error', id, message]
"""
import os
from pathlib import Path
import secrets
import socket
import sys
import threading
import time

VERSION = 1
PORT = 8761  # Loopback only: the relay on this computer.
SILENCE_S = 5.  # A bridge that sends nothing (not even a beat) for this long is gone.
BEAT_S = 1.
LINK_TIMEOUT_S = 20.  # Opening the leader USB and the link process takes a few seconds on Windows.
LEADER_STALE_S = .5
TOKEN_FILE = Path.home() / 'minerva' / 'run' / 'helm-bridge-token'


class BridgeError(RuntimeError):
    pass


def _send(sock, lock, message):
    from robot_platform.transport.wire import pack_msg
    body = pack_msg(list(message))
    with lock:
        sock.sendall(len(body).to_bytes(4, 'big') + body)


def _recv(stream):
    from .edge import read_frame
    return read_frame(stream)


class _Link:
    """A running PC end of the leader link, as serve_link returns one: close() stops it and returns its report."""

    def __init__(self, hub, request):
        self.hub, self.request, self.report = hub, request, None

    def close(self, timeout=5.):
        if self.report is None:
            try:
                self.report = self.hub._call(['link_stop', self.request], 'link_report', timeout) or {}
            except BridgeError as exc:
                self.report = {'error': str(exc)}
        return self.report


class BridgeLeaders:
    """The leaders through the bridge, as the Leaders workspace polls them: the latest pair, restamped on arrival."""

    def __init__(self, hub, request):
        self.hub, self.request = hub, request

    def poll(self):
        from robot_platform.teleop.openrb import LeaderSampleSkipped
        samples, received = self.hub._leaders.get(self.request, (None, None))
        if not self.hub.connected:
            raise BridgeError('The GELLO bridge went away')
        if samples is None or self.hub.clock() - received > LEADER_STALE_S:
            raise LeaderSampleSkipped('No fresh leader sample from the bridge')
        return samples

    def close(self):
        self.hub._leaders.pop(self.request, None)
        try:
            self.hub._post(['leaders_close', self.request])
        except BridgeError:
            pass


class BridgeHub:
    """Accepts the bridge's relay on a loopback port; one bridge at a time, the newest replacing the last."""

    def __init__(self, *, port=PORT, token_file=TOKEN_FILE, machine_id=None, clock=time.monotonic):
        self.port, self.token_file, self.clock = port, Path(token_file), clock
        if machine_id is None:
            try:
                machine_id = Path('/etc/machine-id').read_text().strip()
            except OSError:
                machine_id = None
        self.machine_id = machine_id
        self.token = secrets.token_hex(32)
        self.token_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.token_file.with_suffix('.tmp')
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(descriptor, 'w') as handle:
            handle.write(self.token)
        os.replace(temporary, self.token_file)
        self._server = socket.create_server(('127.0.0.1', port))
        self.port = self._server.getsockname()[1]
        self._lock, self._send_lock = threading.Condition(), threading.Lock()
        self._sock = self.info = self.peer = None
        self._heard = None
        self._replies, self._leaders = {}, {}
        self._ids = iter(range(1, 1 << 62))
        self._closed = False
        threading.Thread(target=self._accept, daemon=True, name='helm-bridge-accept').start()
        threading.Thread(target=self._beat, daemon=True, name='helm-bridge-beat').start()

    # -- connection ----------------------------------------------------------------------

    @property
    def connected(self):
        with self._lock:
            return self._sock is not None and self._heard is not None and self.clock() - self._heard <= SILENCE_S

    def _accept(self):
        while not self._closed:
            try:
                sock, _ = self._server.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(sock,), daemon=True, name='helm-bridge').start()

    def _serve(self, sock):
        stream = sock.makefile('rb')
        try:
            sock.settimeout(SILENCE_S * 2)
            first = _recv(stream)
            if (not isinstance(first, dict) or first.get('relay') != VERSION
                    or not secrets.compare_digest(str(first.get('token', '')), self.token)):
                return
            hello = _recv(stream)
            if not (isinstance(hello, list) and hello[:1] == ['hello'] and isinstance(hello[1], dict)
                    and hello[1].get('version') == VERSION):
                _send(sock, self._send_lock, ['error', None, f'This Helm speaks bridge version {VERSION}'])
                return
            # The bridge pins this computer's host key; its key may only run the relay, so it checks the machine
            # id here instead of over SFTP (ops.ssh.pinned).
            _send(sock, self._send_lock, ['welcome', {'version': VERSION, 'machine_id': self.machine_id,
                                                      'host': socket.gethostname()}])
            with self._lock:
                previous, self._sock = self._sock, sock
                self.info, self.peer = dict(hello[1]), first.get('peer')
                self._heard = self.clock()
                self._replies.clear()
                self._lock.notify_all()
            if previous is not None:
                previous.close()  # The newest bridge wins; the old relay ends.
            sock.settimeout(None)
            while True:
                message = _recv(stream)
                with self._lock:
                    if self._sock is not sock:
                        return
                    self._heard = self.clock()
                    kind = message[0] if isinstance(message, list) and message else None
                    if kind == 'leaders' and len(message) == 3:
                        self._leaders[message[1]] = (self._restamp(message[2]), self.clock())
                    elif kind in ('link_up', 'link_report', 'leaders_up', 'error') and len(message) >= 2:
                        self._replies[message[1]] = message
                        self._lock.notify_all()
        except (EOFError, OSError, ValueError):
            pass
        finally:
            with self._lock:
                if self._sock is sock:
                    self._sock = None
                    self._lock.notify_all()
            sock.close()

    def _restamp(self, raw):
        """The bridge's samples on this computer's clock: the PC sends each one's age, never its clock."""
        from robot_platform.teleop.openrb import LeaderSample
        now, samples = self.clock(), {}
        for name, row in raw.items():
            age = float(row['age_s'])
            samples[name] = LeaderSample(positions_rad=tuple(row['positions_rad']),
                                         raw_positions=tuple(row['raw_positions']), sequence=int(row['sequence']),
                                         sampled_at=now - max(age, 0.), received_at=now,
                                         joystick=tuple(row['joystick']), board_sample_ms=int(row['board_sample_ms']),
                                         board_reply_ms=int(row['board_reply_ms']))
        return samples

    def _beat(self):
        while not self._closed:
            time.sleep(BEAT_S)
            try:
                self._post(['beat'])
            except BridgeError:
                pass

    def _post(self, message):
        with self._lock:
            sock = self._sock
        if sock is None:
            raise BridgeError('The GELLO bridge is not connected: start it on the PC with the leader arms')
        try:
            _send(sock, self._send_lock, message)
        except OSError as exc:
            raise BridgeError(f'The GELLO bridge went away: {exc}') from None

    def _call(self, message, expect, timeout):
        self._post(message)
        request = message[1]
        with self._lock:
            if not self._lock.wait_for(lambda: request in self._replies or self._sock is None, timeout):
                raise BridgeError(f'The GELLO bridge did not answer within {timeout:g} s')
            reply = self._replies.pop(request, None)
        if reply is None:
            raise BridgeError('The GELLO bridge went away')
        if reply[0] == 'error':
            raise BridgeError(f'The GELLO bridge: {reply[2]}')
        if reply[0] != expect:
            raise BridgeError(f'The GELLO bridge answered {reply[0]!r}, not {expect!r}')
        return reply[2] if len(reply) > 2 else None

    # -- what Helm asks of it -----------------------------------------------------------

    def serve_link(self, profile, key, robot, *, local=None, real=True):
        """The PC end of a session's leader link, served by the bridge toward `robot` (the loop's link address
        on the tether). `local` is the bridge's own business."""
        request = next(self._ids)
        self._call(['link', request, {'profile': profile, 'key': key.hex() if isinstance(key, bytes) else key,
                                      'robot': list(robot), 'real': bool(real)}], 'link_up', LINK_TIMEOUT_S)
        return _Link(self, request)

    def open_leaders(self, profile, rate_hz=50.):
        """The leaders for the Leaders workspace: the bridge opens their USB and streams samples."""
        request = next(self._ids)
        self._call(['leaders_open', request, {'profile': profile, 'rate_hz': float(rate_hz)}], 'leaders_up',
                   LINK_TIMEOUT_S)
        return BridgeLeaders(self, request)

    def snapshot(self):
        with self._lock:
            heard = None if self._heard is None else round(self.clock() - self._heard, 2)
            return {'connected': self._sock is not None and heard is not None and heard <= SILENCE_S,
                    'host': (self.info or {}).get('host'), 'version': (self.info or {}).get('version'),
                    'peer': self.peer, 'heard_s': heard}

    def close(self):
        self._closed = True
        self._server.close()
        with self._lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()


# -- the relay: the forced command of the bridge's SSH key ---------------------------------------------------------

def relay(stdin=None, stdout=None, *, port=PORT, token_file=TOKEN_FILE):
    """Join the hub on loopback, prove it with the token, then pass bytes both ways until either side ends."""
    from .edge import write_frame
    stdin = stdin or sys.stdin.buffer
    stdout = stdout or sys.stdout.buffer
    try:
        token = Path(token_file).read_text().strip()
        sock = socket.create_connection(('127.0.0.1', port), timeout=5.)
    except OSError as exc:
        sys.stderr.write(f'Helm is not running on Minerva ({exc})\n')
        return 1
    sock.settimeout(None)
    peer = (os.environ.get('SSH_CLIENT') or '').split(' ')[0] or None
    out = sock.makefile('wb')
    write_frame(out, {'relay': VERSION, 'token': token, 'peer': peer})

    def upstream():
        try:
            while True:
                chunk = stdin.read1(65536) if hasattr(stdin, 'read1') else stdin.read(4096)
                if not chunk:
                    break
                sock.sendall(chunk)
        except OSError:
            pass
        finally:
            try:
                sock.shutdown(socket.SHUT_WR)
            except OSError:
                pass
    threading.Thread(target=upstream, daemon=True, name='relay-up').start()
    try:
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            stdout.write(chunk)
            stdout.flush()
    except OSError:
        pass
    finally:
        sock.close()
    return 0


if __name__ == '__main__':
    if sys.argv[1:] != ['relay']:
        sys.stderr.write('usage: python3 -m robot_platform.helm.bridge relay\n')
        raise SystemExit(2)
    raise SystemExit(relay())
