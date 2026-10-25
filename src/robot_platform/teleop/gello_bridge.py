"""The GELLO bridge: the operator PC's half of Helm running on Minerva's Jetson (helm.bridge).

Run on the PC the leader arms are plugged into:

    python -m robot_platform.teleop.gello_bridge --key <private key> [--robot auto]

It keeps one SSH session to the Jetson, pinned to the robot's host key, whose key may only run Helm's relay. It finds
Minerva wherever it is (candidates): the tether or access point's 10.42.0.1 first, then the addresses Helm reported
last time (wired first), the robot's name on the network (aizee-rover.local), then the profile's address; the first
that answers and proves to be Minerva wins. It says hello, checks the robot's machine id in Helm's welcome, and then
does what Helm asks:

- link: serve a teleop session's leader link toward the robot's loop. This is the PC end that ran beside Helm on the
  operator PC before (teleop.leader_link.serve_link): the 200 Hz USB sampler and the signed UDP answers, in their own
  responsive process, over the tether. Same code, process and path: the same latency. Stopping it sends back its
  counts and timing.
- leaders: open the leaders and stream their latest samples for the Leaders workspace, each with its age on this PC,
  never this PC's clock.

It answers Helm's beats and ends a connection that goes silent, stopping what it served; then it connects again.
Nothing here commands the robot: the loop on the Jetson decides what every sample may do.
"""
import argparse
import logging
import socket
import sys
import threading
import time
import traceback

from robot_platform.helm.bridge import BEAT_S, RELAY_COMMAND, SILENCE_S, VERSION

RECONNECT_S = (1., 2., 5., 10.)
# main's --log: the bridge task's log file. Windows PowerShell turns each redirected stderr line into an error record,
# so the task writes its own file instead.
LOG_FILE = None


def log(message):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}\n"
    if sys.stderr is not None:
        sys.stderr.write(line)
        sys.stderr.flush()
    if LOG_FILE is not None:
        with open(LOG_FILE, 'a', encoding='utf-8') as handle:
            handle.write(line)


def sample_row(sample, now):
    """One leader's sample for the wire: its age on this PC instead of this PC's clock."""
    return {'positions_rad': [float(q) for q in sample.positions_rad],
            'raw_positions': [int(v) for v in sample.raw_positions], 'sequence': int(sample.sequence),
            'age_s': max(0., now - sample.sampled_at), 'joystick': [int(v) for v in sample.joystick],
            'board_sample_ms': int(sample.board_sample_ms), 'board_reply_ms': int(sample.board_reply_ms)}


def open_usb_leaders(profile, rate_hz):
    from .leader_bridge import LatestLeaderInputs
    from .openrb import OpenRBInputs
    leaders = LatestLeaderInputs(OpenRBInputs(profile), rate_hz=rate_hz)
    leaders.connect()
    return leaders


class BridgeSession:
    """One connection: Helm's frames in, their work done, the answers out."""

    def __init__(self, reader, writer, *, machine_id, serve_link=None, open_leaders=None, route=None,
                 synthetic=False, clock=time.perf_counter, host=None, hangup=None):
        from .leader_link import serve_link as usb_serve_link
        from robot_platform.ops.ssh import route_source
        self.reader, self.writer, self.machine_id = reader, writer, machine_id
        self.serve_link = serve_link or usb_serve_link
        self.open_leaders = open_leaders or open_usb_leaders
        self.route = route or route_source
        self.synthetic, self.clock = synthetic, clock
        self.host = host or socket.gethostname()
        # Ends the connection from another thread (a blocked read only returns when its transport closes).
        self.hangup = hangup or reader.close
        self._lock = threading.Lock()
        self._links, self._streams = {}, {}
        self._heard = clock()
        self._ended = threading.Event()
        self.addresses = []  # Minerva's addresses as its welcome reported them, the best path first.

    def send(self, message):
        from robot_platform.helm.edge import write_frame
        with self._lock:
            write_frame(self.writer, list(message))

    def run(self):
        """Serve until the connection ends or Helm goes silent; everything served stops with it."""
        from robot_platform.helm.edge import read_frame
        try:
            self.send(['hello', {'version': VERSION, 'host': self.host, 'synthetic': self.synthetic}])
            welcome = read_frame(self.reader)
            if not (isinstance(welcome, list) and welcome[:1] == ['welcome'] and isinstance(welcome[1], dict)):
                raise RuntimeError(f'Unexpected first answer from Helm: {welcome!r:.200}')
            if welcome[1].get('version') != VERSION:
                raise RuntimeError(f"Helm speaks bridge version {welcome[1].get('version')}, this bridge {VERSION}")
            if welcome[1].get('machine_id') != self.machine_id:
                raise RuntimeError('Wrong robot computer: its machine id differs from the device profile')
            self.addresses = [a for a in welcome[1].get('addresses') or [] if isinstance(a, str)][:8]
            log(f"connected to Helm on {welcome[1].get('host')}")
            threading.Thread(target=self._beat, daemon=True, name='bridge-beat').start()
            while not self._ended.is_set():
                message = read_frame(self.reader)
                self._heard = self.clock()
                self._handle(message)
        except (EOFError, OSError) as exc:
            log(f'connection ended: {exc or type(exc).__name__}')
        except (RuntimeError, ValueError) as exc:  # A wrong robot or version: say so, hang up, try again later.
            log(f'refused: {exc}')
        finally:
            self._ended.set()
            self.stop_all()
            try:
                self.hangup()
            except OSError:
                pass

    def _beat(self):
        while not self._ended.wait(BEAT_S):
            if self.clock() - self._heard > SILENCE_S:
                log(f'Helm silent for {SILENCE_S:g} s: ending the connection')
                self._ended.set()
                try:
                    self.hangup()
                except OSError:
                    pass
                return
            try:
                self.send(['beat'])
            except (OSError, ValueError):
                return

    def _handle(self, message):
        if not isinstance(message, list) or not message:
            return
        kind = message[0]
        if kind == 'beat':
            return
        if kind in ('link', 'link_stop', 'leaders_open', 'leaders_close') and len(message) >= 2:
            threading.Thread(target=getattr(self, '_' + kind), args=tuple(message[1:]), daemon=True,
                             name=f'bridge-{kind}').start()

    def _fail(self, request, exc):
        log(f'request {request} failed: {exc}')
        try:
            self.send(['error', request, f'{type(exc).__name__}: {exc}'])
        except (OSError, ValueError):
            pass

    # -- a session's leader link ---------------------------------------------------------

    def _link(self, request, args):
        try:
            if self._streams:
                raise RuntimeError('The leaders are open for the Leaders tab; disconnect them there first')
            robot = (str(args['robot'][0]), int(args['robot'][1]))
            local = (self.route(robot[0]) or '0.0.0.0', 0)
            link = self.serve_link(args['profile'], bytes.fromhex(args['key']), robot, local=local,
                                   real=bool(args.get('real', True)) and not self.synthetic)
            self._links[request] = link
            log(f'serving the leader link to {robot[0]}:{robot[1]} from {local[0]}')
            self.send(['link_up', request])
        except Exception as exc:  # noqa: BLE001 - Helm shows the reason; the session does not start.
            self._fail(request, exc)

    def _link_stop(self, request):
        link = self._links.pop(request, None)
        report = link.close() if link is not None else {}
        log(f'leader link stopped: {report.get("served")} served, {report.get("skipped")} skipped')
        try:
            self.send(['link_report', request, report])
        except (OSError, ValueError):
            pass

    # -- the Leaders workspace's samples --------------------------------------------------

    def _leaders_open(self, request, args):
        from .leader_link import SyntheticLeaders
        try:
            if self._links:
                raise RuntimeError('A teleop session is using the leaders')
            rate = min(max(float(args.get('rate_hz', 50.)), 1.), 100.)
            leaders = (SyntheticLeaders(args['profile']) if self.synthetic
                       else self.open_leaders(args['profile'], 100.))
        except Exception as exc:  # noqa: BLE001
            self._fail(request, exc)
            return
        stop = threading.Event()
        self._streams[request] = (leaders, stop)
        self.send(['leaders_up', request])
        threading.Thread(target=self._stream, args=(request, leaders, stop, 1. / rate), daemon=True,
                         name='bridge-leaders').start()

    def _stream(self, request, leaders, stop, period):
        from .openrb import LeaderSampleSkipped
        try:
            while not stop.wait(period) and not self._ended.is_set():
                try:
                    samples = leaders.poll()
                except LeaderSampleSkipped:
                    continue
                now = self.clock()
                self.send(['leaders', request, {name: sample_row(sample, now) for name, sample in samples.items()}])
        except Exception as exc:  # noqa: BLE001 - USB unplugged, the connection gone: Helm shows the reason.
            self._fail(request, exc)
        finally:
            leaders.close()

    def _leaders_close(self, request):
        stream = self._streams.pop(request, None)
        if stream is not None:
            stream[1].set()

    def stop_all(self):
        for request in list(self._links):
            try:
                self._links.pop(request).close()
            except Exception:  # noqa: BLE001 - closing on the way out
                pass
        for request in list(self._streams):
            self._streams.pop(request)[1].set()


def ssh_relay(profile, robot, key):
    """(reader, writer, close) of the relay on the Jetson: the bridge key's only command."""
    from robot_platform.ops.ssh import pinned
    client = pinned(profile, address=robot, key=key, timeout=8.)
    try:
        transport = client.get_transport()
        transport.set_keepalive(2)
        stdin, stdout, _stderr = client.exec_command(RELAY_COMMAND)
        stdout.channel.settimeout(None)
    except BaseException:
        client.close()
        raise
    return stdout, stdin, client.close


def candidates(profile, robot='auto', remembered=()):
    """Minerva's addresses to try, in order. 'auto': the tether or access point's 10.42.0.1, the addresses Helm
    reported last time (the wired first), the robot's name on the network (mDNS), then the profile's address.
    Otherwise the addresses given, comma-separated."""
    from robot_platform.ops.ssh import TETHER, robot_identity
    if robot not in (None, '', 'auto'):
        return [address.strip() for address in str(robot).split(',') if address.strip()]
    device = robot_identity(profile)[0]
    found = [TETHER, *remembered]
    if device.get('hostname'):
        found.append(f"{device['hostname']}.local")
    if device.get('address'):
        found.append(device['address'])
    return list(dict.fromkeys(found))


def reachable(address, port=22, timeout=1.5):
    """Whether something answers at `address`: a quick look before the pinned SSH connection proves it is Minerva."""
    try:
        socket.create_connection((address, port), timeout=timeout).close()
        return True
    except OSError:  # Unreachable, refused, or a name that does not resolve.
        return False


def ipv4(address):
    """`address` as an IPv4 address: a name resolves to IPv4 only, or None. Windows answers aizee-rover.local with
    Minerva's IPv6 link-local address first, which the bridge key's from= (IPv4 networks) refuses (2026-10-25)."""
    try:
        return socket.getaddrinfo(address, 22, socket.AF_INET, socket.SOCK_STREAM)[0][4][0]
    except (OSError, IndexError):
        return None


def run(*, profile, robot='auto', key, synthetic=False, connect=None, stop=None, session_options=None,
        probe=reachable, resolve=ipv4):
    """Connect to the first of Minerva's addresses (candidates) that answers and proves to be Minerva, serve, and
    connect again after a pause, until `stop` is set."""
    from robot_platform.ops.ssh import robot_identity
    machine_id = robot_identity(profile)[0]['machine_id']
    stop = stop or threading.Event()
    connect = connect or (lambda address: ssh_relay(profile, address, key))
    remembered, attempt = [], 0
    while not stop.is_set():
        started = time.monotonic()
        tried = candidates(profile, robot, remembered)
        for name in tried:
            address = resolve(name)
            if stop.is_set() or address is None or not probe(address):
                continue
            try:
                reader, writer, close = connect(address)
            except Exception as exc:  # noqa: BLE001 - not Minerva, or Helm not running there: the next one.
                log(f'cannot reach Helm on {name}: {exc or type(exc).__name__}')
                continue
            log(f'reached Minerva at {name}' + (f' ({address})' if address != name else ''))
            session = BridgeSession(reader, writer, machine_id=machine_id, synthetic=synthetic, hangup=close,
                                    **(session_options or {}))
            try:
                session.run()
            finally:
                close()
            remembered = session.addresses or remembered
            break
        else:
            log(f"cannot reach Minerva at {', '.join(tried)}")
        attempt = 0 if time.monotonic() - started > 30. else attempt + 1
        stop.wait(RECONNECT_S[min(attempt, len(RECONNECT_S) - 1)])


def main(argv=None):
    from robot_platform.ops.ssh import DEFAULT_PROFILE
    parser = argparse.ArgumentParser(prog='python -m robot_platform.teleop.gello_bridge', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--key', required=True, help="The bridge's private key, authorized on the Jetson for the relay")
    parser.add_argument('--robot', default='auto',
                        help="Minerva's address, or several comma-separated (default auto: the tether or access "
                             'point, then its network addresses)')
    parser.add_argument('--profile', default=str(DEFAULT_PROFILE), help='The calibration profile (robot identity)')
    parser.add_argument('--synthetic', action='store_true', help='Serve synthetic leaders instead of the USB arms')
    parser.add_argument('--log', help='Also append the log, and library warnings, to this file (the bridge task does)')
    args = parser.parse_args(argv)
    global LOG_FILE
    LOG_FILE = args.log
    if args.log:
        logging.basicConfig(filename=args.log, level=logging.WARNING, format='%(asctime)s %(name)s: %(message)s')
    try:
        run(profile=args.profile, robot=args.robot, key=args.key, synthetic=args.synthetic)
    except KeyboardInterrupt:
        pass
    except Exception:
        log('stopped: ' + traceback.format_exc().rstrip())
        raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
