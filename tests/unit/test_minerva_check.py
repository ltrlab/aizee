"""Minerva Check: a full run against a fake robot (real link code over loopback), and its findings."""
import json
import queue
import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.control.joint_calibration import load_plan
from robot_platform.helm.session import HelmSession
from robot_platform.ops import check, probe
from robot_platform.ops.release import ROOT, _sha, collect, release_drift
from robot_platform.teleop.leader_link import LeaderLinkServer, SyntheticLeaders
from robot_platform.teleop.leader_ranges import range_table
from robot_platform.teleop.openrb import LeaderSample

PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
TELEOP = ROOT / 'config/devices/minerva-teleop.json'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
LEADERS = json.loads((ROOT / 'config/devices/gello-dual-viewer.json').read_text(encoding='utf-8'))
RANGES = json.loads((ROOT / 'config/devices/gello-dual-leader-ranges.json').read_text(encoding='utf-8'))
PLAN = load_plan(PROFILE)
REFERENCE = PLAN[3]
LIMITS = json.loads((ROOT / 'config/devices/minerva-production-01-joint-limits.json').read_text(encoding='utf-8'))


class Stream:
    """A paramiko channel file stand-in: readline gives str, read gives bytes."""

    def __init__(self):
        self.lines = queue.Queue()
        self.channel = SimpleNamespace(shutdown_write=lambda: None, recv_exit_status=lambda: 0)

    def write(self, text):
        for line in text.splitlines(keepends=True):
            self.lines.put(line)

    def flush(self):
        pass

    def readline(self):
        try:
            return self.lines.get(timeout=15)
        except queue.Empty:
            return ''

    def read(self):
        chunks = []
        while not self.lines.empty():
            chunks.append(self.lines.get())
        return ''.join(chunks).encode()


def health(**change):
    report = {'time': time.time(), 'hostname': 'aizee-rover', 'release': 'x', 'disk_free_gb': 40.2,
              'memory_available_gb': 5.1, 'temperatures_c': {'cpu-thermal': 45.1, 'tj-thermal': 47.},
              'clock_synchronized': False, 'governor': 'schedutil', 'power_mode': 'MAXN', 'load': [.3, .2, .1],
              'cpus': 8, 'uptime_s': 3600, 'setup': {'helper': True, 'sudo': True, 'udev_rule': True},
              'adapters': {'canL': {'present': True, 'expected_serial': 'A', 'serial': 'A', 'up': False},
                           'canR': {'present': True, 'expected_serial': 'B', 'serial': 'B', 'up': False}},
              'services': {'aizee-minerva-left': 0, 'aizee-minerva-right': 0, 'aizee-motor-control-rover': 0},
              'receiver_listening': False}
    report.update(change)
    return report


def scan(**change):
    report = {'success': True, 'asset_id': PLAN[2]['asset_id'], 'identity_verified': True,
              'enable_requested': False, 'parameter_writes_requested': False,
              'motors': [{**{k: m[k] for k in ('name', 'bus', 'id', 'uid', 'firmware')},
                          'position_rad': (LIMITS['joints'][m['name']]['min_rad']
                                           + LIMITS['joints'][m['name']]['max_rad']) / 2, 'reset_confirmed': True}
                         for m in REFERENCE['motors']],
              'after': {'canL': {'flags': ['NOARP']}, 'canR': {'flags': ['NOARP']}}}
    report.update(change)
    return report


class FakeRobot:
    """Answers the check's commands; the link probe really runs, over loopback."""

    def __init__(self, *, release=True, health_report=None, motors_report=None):
        manifest = {'id': '20260926T000000Z-abcdef0', 'source': {'commit': 'abcdef0123', 'dirty': False},
                    'files': {key: {'sha256': _sha(path.read_bytes()), 'size': path.stat().st_size}
                              for key, path in collect().items() if path.is_file()}}
        self.release = {'root': check.CURRENT, 'problems': [], 'manifest': manifest} if release else {
            'error': 'FileNotFoundError: no release.json'}
        self.health, self.motors = health_report or health(), motors_report or scan()
        self.commands, self.closed = [], False

    def exec_command(self, command, timeout=None):
        self.commands.append(command)
        stdin, stdout, stderr = Stream(), Stream(), Stream()
        if command.endswith('probe link'):
            def serve():
                stdout.write(json.dumps({'ok': True, **probe.link(stdin, out=stdout)}) + '\n')
            threading.Thread(target=serve, daemon=True).start()
        elif command.startswith(f'python3 - {check.CURRENT}'):
            stdout.write(json.dumps(self.release) + '\n')
        elif 'ops.smoke' in command:
            stdout.write(json.dumps({'ok': True, 'load_ms': 1281, 'keepout_query_ms': {'p50': 1.52, 'max': 2.1},
                                     'receiver_harness': 'disarmed'}) + '\n')
        elif command.endswith('probe health'):
            stdout.write(json.dumps({'ok': True, **self.health, 'time': time.time()}) + '\n')
        elif command.endswith('probe motors'):
            stdout.write(json.dumps({'ok': True, **self.motors}) + '\n')
        else:
            raise AssertionError(f'Unexpected command {command}')
        return stdin, stdout, stderr

    def close(self):
        self.closed = True


class Leaders:
    """Both leaders at their saved sweep centres (or offset by `shift` radians)."""

    def __init__(self, shift=0.):
        table = range_table(RANGES, LEADERS)
        self.positions = {name: tuple((row[0] if row else 0.) + shift for row in rows) for name, rows in table.items()}
        self.sequence, self.closed = 0, False

    def poll(self):
        self.sequence += 1
        now = time.perf_counter()
        return {name: LeaderSample(q, (0,) * 7, self.sequence, now, now, (0, 0, 0, 0), self.sequence, self.sequence)
                for name, q in self.positions.items()}

    def close(self):
        self.closed = True


@pytest.fixture
def runs(tmp_path):
    """Helm's direction session (every arm joint confirmed) and confirmed routes, as Helm writes them."""
    runs = tmp_path / 'runs'
    session = HelmSession(PROFILE, runs=runs, limits=tmp_path / 'unused-limits.json', seed=SEED)
    session.close()
    path = session.state_path
    state = json.loads(path.read_text())
    for row in state['preview_mapping'].values():
        row['direction_verified'] = True
    path.write_text(json.dumps(state))
    teleop = json.loads(TELEOP.read_text(encoding='utf-8'))
    edits = runs / f"{session.asset_id}-teleop/routes.json"
    edits.parent.mkdir(parents=True)
    edits.write_text(json.dumps({'schema': 1, 'profile': teleop['id'], 'asset_id': session.asset_id,
                                 'routes': {m: {'sign': r['sign'], 'verified': True}
                                            for m, r in teleop['routes'].items()}}))
    return runs


@pytest.fixture
def loopback(monkeypatch):
    """Treat loopback as the tether, stub the ARM64 build check and present both leaders by USB."""
    monkeypatch.setattr(check, 'TETHER_NETWORK', '127.0.0.')
    builds = next(spec for spec in check.CHECKS if spec['id'] == 'builds')
    monkeypatch.setitem(builds, 'run', lambda ctx: (check.PASS, 'stubbed'))
    ports = [SimpleNamespace(vid=0x2F5D, pid=0x2202, serial_number=item['usb_serial'], device=f'COM{i}')
             for i, item in enumerate(LEADERS['leaders'].values(), 7)]
    monkeypatch.setattr('serial.tools.list_ports.comports', lambda: ports)


def serve_in_process(profile, key, robot, *, local, real):
    """The PC end of the link in this process (LeaderLinkProcess is tested on its own)."""
    server = LeaderLinkServer(SyntheticLeaders(profile), key, robot, local=local).start()

    def close():
        server.close()
        return {'served': server.served, 'skipped': server.skipped, 'rejected': server.rejected,
                'failure': server.failure}
    return SimpleNamespace(close=close)


def run(runs, robot, *, leaders=None, **options):
    leaders = leaders or Leaders()
    published = []
    report = check.run_check(runs=runs, address='127.0.0.1', link_seconds=1., open_ssh=lambda *a, **k: robot,
                             open_leaders=lambda profile: leaders, serve_link=serve_in_process,
                             publish=published.append, **options)
    return report, {entry['id']: entry for stage in report['stages'] for entry in stage['checks']}, published


def test_a_complete_check_of_a_healthy_robot_is_a_go_and_is_saved(runs, loopback):
    robot = FakeRobot()
    report, checks, published = run(runs, robot)
    assert report['verdict'] == 'go', report['summary']
    assert {entry['status'] for key, entry in checks.items() if key != 'sweep'} <= {'pass', 'warn'}
    assert checks['sweep']['status'] == 'pending'
    assert checks['link']['status'] != 'fail' and checks['link']['data']['fresh'] > 0
    assert checks['link']['data']['pc']['served'] >= checks['link']['data']['fresh']
    assert 'All 14 motors answered' in checks['scan']['detail'] and 'inside their swept ranges' in checks['pose']['detail']
    assert any(entry['status'] == 'running' for stage in published[0]['stages'] for entry in stage['checks'])
    assert robot.closed and sum(command.endswith('probe motors') for command in robot.commands) == 1
    saved = check.latest_report(runs)
    assert saved['verdict'] == 'go' and saved['evidence'] == report['evidence']


def test_a_failure_skips_what_depends_on_it_and_says_how_to_fix_it(runs, loopback):
    report, checks, _published = run(runs, FakeRobot(release=False))
    assert report['verdict'] == 'no_go' and 'Deployed release' in report['summary']
    assert 'scripts/deploy_minerva_arms.sh' in checks['release']['detail']
    for key in ('current', 'smoke', 'health', 'link', 'setup', 'adapters', 'services', 'scan', 'pose'):
        assert checks[key]['status'] == 'skip', key
    assert checks['ssh']['status'] == 'pass' and checks['leaders']['status'] == 'pass'


def test_skipping_the_motors_is_never_a_go(runs, loopback):
    report, checks, _published = run(runs, FakeRobot(), motors=False)
    assert checks['scan']['status'] == 'skip' and checks['pose']['status'] == 'skip'
    assert report['verdict'] == 'incomplete'


def test_robot_problems_are_named(runs, loopback):
    bad_health = health(setup={'helper': True, 'sudo': False, 'udev_rule': True},
                        services={'aizee-minerva-left': 812, 'aizee-minerva-right': 0, 'aizee-motor-control-rover': 0})
    _report, checks, _ = run(runs, FakeRobot(health_report=bad_health))
    assert 'setup_minerva_system.sh' in checks['setup']['detail'] and 'aizee-minerva-left' in checks['services']['detail']
    assert checks['scan']['status'] == 'skip'
    moved = scan()
    moved['motors'][2]['position_rad'] = LIMITS['joints'][moved['motors'][2]['name']]['max_rad'] + .4
    _report, checks, _ = run(runs, FakeRobot(motors_report=moved), leaders=Leaders(shift=3.))
    assert checks['pose']['status'] == 'fail' and 'beyond its swept' in checks['pose']['detail']
    assert checks['leaders']['status'] == 'warn' and 'outside the saved sweep' in checks['leaders']['detail']


def test_unconfirmed_calibration_fails_before_anything_is_derived(runs, loopback):
    path = next(runs.glob('*-directions/session.json'))
    state = json.loads(path.read_text())
    state['preview_mapping']['left_arm_j3']['direction_verified'] = False
    path.write_text(json.dumps(state))
    _report, checks, _ = run(runs, FakeRobot())
    assert checks['directions']['status'] == 'fail' and '11/12' in checks['directions']['detail']
    assert checks['receiver']['status'] == 'skip'


def findings(report):
    labels = {m['name']: m['name'].replace('_', ' ') for m in REFERENCE['motors']}
    ctx = SimpleNamespace(facts={'reference': REFERENCE, 'labels': labels}, label=lambda motor: labels[motor])
    return check.scan_findings(ctx, report)


def test_scan_findings_name_silent_motors_arms_and_latched_faults():
    names = [m['name'] for m in REFERENCE['motors']]
    left = [m['name'] for m in REFERENCE['motors'] if m['bus'] == 'canL']
    assert findings(scan())[0] == 'pass'
    status, detail = findings(scan(success=False, error_code='motors_unresponsive', unresponsive=names))
    assert status == 'fail' and 'foot switch' in detail
    status, detail = findings(scan(success=False, error_code='motors_unresponsive', unresponsive=left))
    assert 'Every motor on canL' in detail
    status, detail = findings(scan(success=False, error_code='motors_unresponsive', unresponsive=left[-2:]))
    assert detail.startswith('Not answering: ') and left[-1].replace('_', ' ') in detail
    status, detail = findings(scan(success=False, error='Drive did not confirm fault-free Reset: left_arm_j5 '
                                                        'reports mode Reset; latched faults: overcurrent'))
    assert 'power cycle' in detail
    status, detail = findings(scan(after={'canL': {'flags': ['UP']}, 'canR': {'flags': []}}))
    assert status == 'fail' and 'not confirmed down' in detail
    assert findings(scan(enable_requested=True))[0] == 'fail'


def test_health_findings_grade_disk_memory_heat_and_clock():
    assert {status for status, _ in check.health_findings(health(), .2)} == {'pass'}
    graded = dict((text.split(' ')[0], status) for status, text in check.health_findings(
        health(disk_free_gb=.5, memory_available_gb=1., temperatures_c={'tj-thermal': 80.}), 7.))
    assert graded['0.5'] == 'fail' and graded['1'] == 'warn' and graded['hottest'] == 'warn' and graded['clock'] == 'warn'


def test_release_drift_lists_changed_new_and_removed_files():
    files = {key: path for key, path in collect().items() if path.is_file()}
    manifest = {'files': {key: {'sha256': _sha(path.read_bytes())} for key, path in files.items()}}
    assert release_drift(manifest) == []
    changed, new = sorted(files)[:2]
    manifest['files'][changed]['sha256'] = '0' * 64
    del manifest['files'][new]
    manifest['files']['src/robot_platform/gone.py'] = {'sha256': '1' * 64}
    assert set(release_drift(manifest)) == {f'changed {changed}', f'new {new}', 'removed src/robot_platform/gone.py'}
