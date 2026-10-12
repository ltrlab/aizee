"""Finite, stopped-only ROBSTRIDE captures for operator calibration.

Runs on the identified Linux CAN host. No enable, zero, save or parameter-write
operation is permitted by this transport, including when called incorrectly.
The one exception is explicit: run_capture's `maintain` caller (the power-on
range) gets the session's exchange after the stopped preflight, and the socket
boundary still limits it to the fixed maintenance frames.
"""
import itertools
import json
import math
import statistics
import select
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .mechanical_zero import SocketCanMaintenance, StoppedMaintenance
else:  # The SSH client stages these two source files together, by content hash.
    from mechanical_zero import SocketCanMaintenance, StoppedMaintenance


POWER_ON_RANGE = 0x7029  # zero_sta: the window a drive puts its angle in at power-on (0: 0…2π, 1: −π…π).


def stopped_exchange(exchange):
    def call(bus, frame, kind, parameter):
        if kind not in {"identity", "stop", "version", "read"}:
            raise ValueError("Calibration permits stopped reads only")
        if kind == "read" and parameter not in {0x7005, 0x7019, 0x701B, POWER_ON_RANGE}:
            raise ValueError("Calibration parameter is not allowlisted")
        return exchange(bus, frame, kind, parameter)
    return call


def capture_roster(motors, exchange, *, sleep=time.sleep):
    """Fail the whole snapshot on missing identity, motion or a non-Reset drive."""
    names = [m['name'] for m in motors]
    addresses = [(m['bus'], m['id']) for m in motors]
    if not names or len(names) > 32 or len(set(names)) != len(names) or len(set(addresses)) != len(names):
        raise ValueError("Invalid motor roster")
    reader = StoppedMaintenance(stopped_exchange(exchange), sleep=sleep)
    rows, cleanup_errors = [], []
    try:
        for motor in motors:
            if reader.request(motor, 'identity') != motor['uid']:
                raise RuntimeError(f"{motor['name']}: UID changed")
            reader.request(motor, 'stop')
            if reader.request(motor, 'version') != motor['firmware']:
                raise RuntimeError(f"{motor['name']}: firmware changed")
            if reader.request(motor, 'read', 0x7005) != 0:
                raise RuntimeError(f"{motor['name']}: wrong run mode")
        for motor in motors:
            samples = reader.stationary(motor)
            if not all(math.isfinite(q) for q in samples):
                raise RuntimeError('Nonfinite position')
            rows.append({k: motor[k] for k in ('name', 'bus', 'id', 'uid', 'firmware')} |
                        {'position_rad': statistics.median(samples), 'samples_rad': samples,
                         'reset_confirmed': True})
    finally:
        for motor in motors:
            try:
                reader.request(motor, 'stop')
            except Exception as exc:
                cleanup_errors.append(f"{motor['name']}: {exc}")
        if cleanup_errors:
            raise RuntimeError('Final stopped replies failed: ' + '; '.join(cleanup_errors))
    return rows


class MotorSilent(TimeoutError):
    """A motor stopped answering; `motor` names it for the operator."""
    def __init__(self, message, motor):
        super().__init__(message)
        self.motor = motor


def live_preflight(reader, motors, result):
    """Identity, Reset, firmware and run mode for every motor before streaming.

    Every motor is asked, so one silent drive (loose plug, power loss) cannot hide
    others; all silent motors are named. A wrong UID/firmware/run mode is a hard
    failure: that is a different or misconfigured motor, not a dropout.
    """
    silent = []
    for motor in motors:
        try:
            if reader.request(motor, 'identity') != motor['uid']:
                raise RuntimeError('Motor UID mismatch: ' + motor['name'])
        except TimeoutError:
            silent.append(motor['name'])
    if not silent:
        for motor in motors:
            try:
                reader.request(motor, 'stop')
                if reader.request(motor, 'version') != motor['firmware'] or reader.request(motor, 'read', 0x7005) != 0:
                    raise RuntimeError('Motor firmware or run-mode mismatch: ' + motor['name'])
            except TimeoutError:
                silent.append(motor['name'])
    if silent:
        result['error_code'] = 'motors_unresponsive'
        result['unresponsive'] = silent
        raise RuntimeError('Not answering: ' + ', '.join(silent))
    result['identity_verified'] = True


def paced(io, gap=.01, *, clock=time.perf_counter, sleep=time.sleep):
    """`io` with at least `gap` seconds between requests on each bus; every bus is its own USB adapter."""
    sent = {}

    def exchange(bus, *args):
        sleep(max(0., sent.get(bus, -math.inf) + gap - clock()))
        try:
            return io(bus, *args)
        finally:
            sent[bus] = io.last_tx
    return exchange


def alternating(motors):
    """The roster in ranks of one motor per bus, so consecutive requests alternate adapters."""
    buses = {}
    for motor in motors:
        buses.setdefault(motor['bus'], []).append(motor)
    return [[motor for motor in rank if motor is not None] for rank in itertools.zip_longest(*buses.values())]


def live_scan(reader, motors, *, power_on=True):
    """Stop and read every motor, rows in roster order, each with the temperature its stop reply reports.
    Requests alternate buses, so one adapter answers while the other waits out its spacing. `power_on` also reads each drive's zero_sta, for Check's scan; the live
    stream never uses it, and skipping it cuts each frame by a third."""
    def request(motor, *args):
        try:
            return reader.request(motor, *args)
        except TimeoutError as exc:
            raise MotorSilent('Missing stopped position: ' + motor['name'], motor['name']) from exc
    rows, temperatures = {}, {}
    for rank in alternating(motors):
        for motor in rank:
            # The stop's own reply carries the drive's temperature: no extra request.
            temperatures[motor['name']] = (request(motor, 'stop') or {}).get('temperature_c')
        for motor in rank:
            rows[motor['name']] = ({k: motor[k] for k in ('name', 'bus', 'id', 'uid', 'firmware')} |
                                   {'position_rad': request(motor, 'read', 0x7019), 'reset_confirmed': True,
                                    'temperature_c': temperatures[motor['name']]})
        for motor in rank if power_on else ():
            try:
                rows[motor['name']]['zero_sta'] = reader.request(motor, 'read', POWER_ON_RANGE)
            except (TimeoutError, RuntimeError):
                rows[motor['name']]['zero_sta'] = None  # Firmware without the parameter: unknown, not silent.
    return [rows[motor['name']] for motor in motors]


def run_capture(device, reference, *, live=None, maintain=None):
    """The stopped session: host, adapter and service checks, CAN up, then `live(reader, motors, result)` after
    the preflight (or the roster capture without it), CAN down. `maintain(exchange, motors, result)` instead
    gets the session's exchange after the preflight, for an explicit maintenance write."""
    import fcntl

    def command(args):
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=5)

    if reference['asset_id'] != device['asset_id']:
        raise ValueError('Reference belongs to another device')
    if Path('/etc/machine-id').read_text().strip() != device['machine_id']:
        raise RuntimeError('Wrong host machine identity')
    if device.get('module_serial') and Path('/proc/device-tree/serial-number').read_bytes().rstrip(b'\0\n').decode() != device['module_serial']:
        raise RuntimeError('Wrong module identity')
    interfaces = [b['interface'] for b in device['buses']]
    if len(set(interfaces)) != len(interfaces) or any(m['bus'] not in interfaces for m in reference['motors']):
        raise ValueError('Invalid bus roster')
    for bus in device['buses']:
        target = (Path('/sys/class/net') / bus['interface'] / 'device').resolve()
        serial = next((p / 'serial').read_text().strip() for p in target.parents if (p / 'serial').is_file())
        if serial != bus['usb_serial']:
            raise RuntimeError('Wrong CAN adapter identity')
    for service in device['conflicting_services']:
        if command(['systemctl', 'show', service, '--property=MainPID', '--value']).strip() != '0':
            raise RuntimeError('Conflicting motor service is running: ' + service)
    if command(['ss', '-H', '-ltn', 'sport', '=', ':5590']).strip():
        raise RuntimeError('Guarded receiver already running')

    def links():
        return {name: json.loads(command(['ip', '-details', '-statistics', '-json', 'link', 'show', name]))[0] for name in interfaces}

    with open('/tmp/robot-platform-mechanical-reference.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        before = links()
        if any('UP' in row['flags'] for row in before.values()):
            raise RuntimeError('CAN must start down; another process may own it')
        result = {'schema': 1, 'asset_id': device['asset_id'], 'success': False,
                  'started_at': datetime.now(timezone.utc).isoformat(),
                  'enable_requested': False, 'parameter_writes_requested': False,
                  'before': before, 'events': []}
        io = None
        try:
            for name in interfaces:
                command(['sudo', '-n', device['can_up_helper'], name])
            io = SocketCanMaintenance(interfaces, result['events'])
            # Calibration tolerates modest display latency. Space requests
            # on each installed USB adapter instead of saturating its
            # firmware with alternating stop/parameter transactions.
            exchange = paced(io)
            if live is None and maintain is None:
                result['motors'] = capture_roster(reference['motors'], exchange)
            else:
                reader = StoppedMaintenance(stopped_exchange(exchange))
                motors = reference['motors']
                live_preflight(reader, motors, result)
                if maintain is None:
                    live(reader, motors, result)
                else:
                    result['parameter_writes_requested'] = True
                    maintain(exchange, motors, result)
            result['success'] = True
        except Exception as exc:
            result['error'] = str(exc)
        finally:
            try:
                result['before_cleanup'] = links()
            except Exception as exc:
                result['diagnostic_error'] = str(exc)
            if io:
                io.close()
            cleanup = []
            for name in interfaces:
                try:
                    command(['sudo', '-n', '/usr/sbin/ip', 'link', 'set', name, 'down'])
                except Exception as exc:
                    cleanup.append(str(exc))
            result['after'] = links()
            if cleanup or any('UP' in row['flags'] for row in result['after'].values()):
                result['success'] = False
                result['cleanup_errors'] = cleanup or ['CAN still up']
            result['finished_at'] = datetime.now(timezone.utc).isoformat()
        return result


def main():
    streaming = '--live' in sys.argv[1:]
    # Unbuffered reads keep select() and line consumption consistent. There is
    # exactly one client request per scan; queued old frames cannot look fresh.
    input_stream = sys.stdin.buffer.raw if streaming else None
    payload = json.loads(input_stream.readline()) if streaming else json.load(sys.stdin)
    directory = Path(__file__).resolve().parent / 'captures'
    directory.mkdir(exist_ok=True)
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    def stream(reader, motors, result):
        # On disconnect, retain Python cleanup instead of dying with CAN up.
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        started = time.perf_counter()
        sequence = rejected = consecutive = 0
        result['raw_frames'] = str(directory / (name + '-frames.jsonl'))
        with (directory / (name + '-frames.jsonl')).open('x') as audit:
            try:
                while time.perf_counter() - started < 900:
                    # Read-only preview: tolerate Wi-Fi/desktop scheduling
                    # stalls, while the UI independently rejects >750 ms data.
                    if not select.select([input_stream], [], [], 5)[0]:
                        result['error_code'] = 'heartbeat_expired'
                        raise RuntimeError('Calibration client heartbeat expired')
                    line = input_stream.readline().decode().strip()
                    if not line.startswith('heartbeat '):
                        result['completion_reason'] = 'client_stop'
                        break
                    nonce = line.split(' ', 1)[1]
                    if len(nonce) != 32 or any(c not in '0123456789abcdef' for c in nonce):
                        raise RuntimeError('Invalid live sample request')
                    begin = time.perf_counter()
                    try:
                        rows = live_scan(reader, motors, power_on=False)
                        span = time.perf_counter() - begin
                        if span > .5:
                            raise TimeoutError('Live scan exceeded 500 ms')
                    except TimeoutError as exc:
                        rejected += 1; consecutive += 1
                        audit.write(json.dumps({'rejected_scan': str(exc), 'events': result['events']}) + '\n')
                        audit.flush(); result['events'].clear()
                        result['rejected_scans'] = rejected
                        if consecutive >= 3:
                            result['error_code'] = 'incomplete_scans'
                            if isinstance(exc, MotorSilent):
                                result['unresponsive'] = [exc.motor]
                            raise RuntimeError('Three incomplete scans: ' + str(exc)) from exc
                        print(json.dumps({'kind': 'scan_rejected', 'request_id': payload['request_id'],
                                          'request_nonce': nonce, 'error': str(exc)}), flush=True)
                        continue  # Never emit or freshen a partial scan.
                    consecutive = 0
                    sequence += 1
                    frame = {'kind': 'positions', 'request_id': payload['request_id'], 'sequence': sequence,
                             'request_nonce': nonce,
                             'asset_id': device['asset_id'], 'scan_seconds': span, 'motors': rows,
                             'enable_requested': False, 'parameter_writes_requested': False}
                    # The CAN trace is kept for the preflight, the first frame and every rejected scan; later
                    # healthy frames log their positions alone, so hours of live view stay small.
                    audit.write(json.dumps({'frame': frame, 'events': result['events']} if sequence == 1
                                           else {'frame': frame}) + '\n')
                    audit.flush(); result['events'].clear()
                    print(json.dumps(frame), flush=True)
                else:
                    result['completion_reason'] = 'duration_limit'
            finally:
                result['frames'] = sequence
                result['rejected_scans'] = rejected
                if result['events']:
                    audit.write(json.dumps({'final_events': result['events']}) + '\n')

    device = payload['device']
    try:
        report = run_capture(device, payload['reference'], live=stream if streaming else None)
    except Exception as exc:
        report = {'success': False, 'error': str(exc), 'enable_requested': False}
    report['request_id'] = payload.get('request_id')
    # Evidence persists remotely even if the client disconnects during a read.
    report['kind'] = 'finished'
    report['remote_capture'] = str(directory / (name + '.json'))
    with (directory / (name + '.json')).open('x') as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps(report, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
