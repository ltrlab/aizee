"""Robot-side probes for Minerva Check, run from the active release over SSH.

    python3 -m robot_platform.ops.probe health   system, CAN setup, adapters and services (no CAN traffic)
    python3 -m robot_platform.ops.probe link     leader-link round trips (stdin: key, leader profile, seconds, address)
    python3 -m robot_platform.ops.probe motors   stopped-only scan of every motor

Each prints one JSON object (link prints "ready <port>" first). Nothing here
enables a motor or writes a parameter. The motor scan is the calibration tool's
stopped reader: host, adapter, service and CAN-down checks, then identity,
firmware and fault-free Reset for every motor and one position read each; CAN
links go down again afterwards.
"""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
UDEV_RULE = '/etc/udev/rules.d/80-minerva-can.rules'
RECEIVER_PORT = 5590
EVIDENCE = Path.home() / 'minerva' / 'evidence'


def identity(root=ROOT):
    """(device, reference) named by the release's calibration profile."""
    from .release import CALIBRATION
    plan_path = Path(root) / CALIBRATION
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    load = lambda key: json.loads((plan_path.parent / plan[key]).read_text(encoding='utf-8'))
    return load('device'), load('reference')


def _run(args, timeout=5.):
    """(exit status or None, stripped stdout or the error)."""
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    return result.returncode, result.stdout.strip()


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def _percentiles(values):
    values = sorted(values)
    pick = lambda p: round(values[min(len(values) - 1, int(p * (len(values) - 1)))], 2) if values else None
    return {'p50': pick(.5), 'p99': pick(.99), 'max': pick(1.)}


def health(root=ROOT):
    device, _reference = identity(root)
    report = {'time': time.time(), 'hostname': socket.gethostname(), 'release': Path(root).resolve().name}
    usage = shutil.disk_usage(Path.home())
    report['disk_free_gb'] = round(usage.free / 1e9, 1)
    meminfo = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines() if ':' in line)
    report['memory_available_gb'] = round(int(meminfo['MemAvailable'].split()[0]) / 1e6, 1)
    temperatures = {}
    for zone in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
        name, value = _read(zone / 'type'), _read(zone / 'temp')
        if name and value and value.lstrip('-').isdigit() and -40 < int(value) / 1000 < 150:
            temperatures[name] = round(int(value) / 1000, 1)
    report['temperatures_c'] = temperatures
    status, out = _run(['timedatectl', 'show', '--property=NTPSynchronized', '--value'])
    report['clock_synchronized'] = out == 'yes' if status == 0 else None
    report['governor'] = _read('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')
    status, out = _run(['nvpmodel', '-q'])
    report['power_mode'] = out.splitlines()[0].split(':', 1)[-1].strip() if status == 0 and out else None
    report['load'] = [round(value, 2) for value in os.getloadavg()]
    report['cpus'] = os.cpu_count()
    report['uptime_s'] = round(float(_read('/proc/uptime').split()[0]))

    helper = device['can_up_helper']
    buses = [bus['interface'] for bus in device['buses']]
    report['setup'] = {'helper': os.access(helper, os.X_OK),
                       # A listing query only: -n never prompts, nothing runs as root.
                       'sudo': _run(['sudo', '-n', '-l', helper, buses[0]])[0] == 0,
                       'udev_rule': Path(UDEV_RULE).is_file()}
    adapters = {}
    for bus in device['buses']:
        path = Path('/sys/class/net') / bus['interface']
        row = {'present': path.exists(), 'expected_serial': bus['usb_serial']}
        if row['present']:
            target = (path / 'device').resolve()
            row['serial'] = next((_read(parent / 'serial') for parent in target.parents
                                  if (parent / 'serial').is_file()), None)
            status, out = _run(['ip', '-details', '-json', 'link', 'show', bus['interface']])
            link = json.loads(out)[0] if status == 0 else {}
            row['up'] = 'UP' in link.get('flags', [])
        adapters[bus['interface']] = row
    report['adapters'] = adapters
    services = {}
    for service in device['conflicting_services']:
        status, out = _run(['systemctl', 'show', service, '--property=MainPID', '--value'])
        services[service] = int(out) if status == 0 and out.isdigit() else None
    report['services'] = services
    status, out = _run(['ss', '-H', '-ltn', 'sport', '=', f':{RECEIVER_PORT}'])
    report['receiver_listening'] = bool(out) if status == 0 else None
    return report


def link(stream=None, out=None):
    """25 Hz authenticated leader-sample requests to the PC, like the robot's control loop."""
    from robot_platform.teleop.leader_link import LeaderLinkInput
    from robot_platform.teleop.openrb import LeaderSampleSkipped
    stream, out = stream or sys.stdin, out or sys.stdout
    key = bytes.fromhex(stream.readline().strip())
    profile = json.loads(stream.readline())
    seconds, address = float(stream.readline()), stream.readline().strip()
    if not 0 < seconds <= 60:
        raise ValueError('Link test duration must be within 60 s')
    robot = LeaderLinkInput(key, profile, bind=(address, 0), timeout_s=.02)
    try:
        out.write(f'ready {robot.address[1]}\n')
        out.flush()
        robot.wait_for_pc(10.)
        ages, polls, skipped, run, longest, last_skip = [], [], 0, 0, 0, None
        due = start = time.perf_counter()
        while time.perf_counter() - start < seconds:
            began = time.perf_counter()
            try:
                pair = robot.poll()
                ages.append((time.perf_counter() - min(s.sampled_at for s in pair.values())) * 1000)
                run = 0
            except LeaderSampleSkipped as exc:
                skipped, run, last_skip = skipped + 1, run + 1, str(exc)
                longest = max(longest, run)
            polls.append((time.perf_counter() - began) * 1000)
            due += .04
            time.sleep(max(0., due - time.perf_counter()))
        return {'cycles': len(polls), 'fresh': len(ages), 'skipped': skipped, 'longest_skip_run': longest,
                'last_skip': last_skip, 'age_ms': _percentiles(ages), 'poll_ms': _percentiles(polls),
                'max_age_ms': round(robot.max_age_s * 1000)}
    finally:
        robot.close()


def motors(root=ROOT):
    """The stopped reader's identity preflight, then one position read per motor."""
    import signal
    from robot_platform.control.joint_capture import live_scan, run_capture
    # A dropped SSH session must not kill the scan between CAN up and CAN down.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    device, reference = identity(root)

    def scan(reader, roster, result):
        result['motors'] = live_scan(reader, roster)

    try:
        report = run_capture(device, reference, live=scan)
    except Exception as exc:  # Raised before CAN came up: lock held, CAN already up, wrong host.
        report = {'success': False, 'error': str(exc), 'enable_requested': False, 'parameter_writes_requested': False}
    if report.get('success'):
        report.pop('events', None)  # Every frame is only worth keeping when something failed.
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    try:
        EVIDENCE.mkdir(parents=True, exist_ok=True)
        path = EVIDENCE / f'check-motors-{stamp}.json'
        path.write_text(json.dumps(report, indent=1, allow_nan=False) + '\n')
        report['evidence'] = str(path)
    except OSError as exc:
        report['evidence_error'] = str(exc)
    return report


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    probes = {'health': health, 'link': link, 'motors': motors}
    if len(argv) != 1 or argv[0] not in probes:
        print(json.dumps({'ok': False, 'error': 'Use health, link or motors'}))
        return 2
    try:
        report = {'ok': True, **probes[argv[0]]()}
    except Exception as exc:
        report = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
    print(json.dumps(report, allow_nan=False), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
