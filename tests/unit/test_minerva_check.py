"""Minerva Check: a full run against a fake robot, and its findings."""
import json
import math
import queue
import time
from types import SimpleNamespace

import pytest

from robot_platform.control.joint_calibration import load_plan
from robot_platform.helm.session import HelmSession
from robot_platform.ops import check
from robot_platform.ops.release import ROOT, _sha, collect, release_drift

PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
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
                                           + LIMITS['joints'][m['name']]['max_rad']) / 2, 'zero_sta': 1,
                          'reset_confirmed': True}
                         for m in REFERENCE['motors']],
              'after': {'canL': {'flags': ['NOARP']}, 'canR': {'flags': ['NOARP']}}}
    report.update(change)
    return report


class FakeRobot:
    """Answers the check's commands."""

    def __init__(self, *, release=True, health_report=None, motors_report=None):
        manifest = {'id': '20260926T000000Z-abcdef0', 'source': {'commit': 'abcdef0123', 'dirty': False},
                    'files': {key: {'sha256': _sha(path.read_bytes()), 'size': path.stat().st_size}
                              for key, path in collect().items() if path.is_file()}}
        self.release = {'root': check.CURRENT, 'problems': [], 'manifest': manifest} if release else {
            'error': 'FileNotFoundError: no release.json'}
        self.health, self.motors = health_report or health(), motors_report or scan()
        self.commands, self.closed = [], False
        self.staged, self.validated, self.cleaned = {}, [], False

    def exec_command(self, command, timeout=None):
        self.commands.append(command)
        stdin, stdout, stderr = Stream(), Stream(), Stream()
        if command.startswith(f'python3 - {check.CURRENT}'):
            stdout.write(json.dumps(self.release) + '\n')
        elif 'ops.smoke' in command:
            stdout.write(json.dumps({'ok': True, 'load_ms': 1281, 'keepout_query_ms': {'p50': 1.52, 'max': 2.1},
                                     'receiver_harness': 'disarmed'}) + '\n')
        elif command.endswith('probe health'):
            stdout.write(json.dumps({'ok': True, **self.health, 'time': time.time()}) + '\n')
        elif command.endswith('probe motors'):
            stdout.write(json.dumps({'ok': True, **self.motors}) + '\n')
        elif ' && cat > ' in command:
            self.staged[command.rsplit(' ', 1)[-1]] = stdin
        elif command.endswith('--validate'):
            config = json.loads(''.join(self.staged[command.split()[1]].lines.queue))
            self.validated.append(config)
            stdout.write('configuration valid; no sockets opened\n' if config['mode'] == 'teleop' else '')
        elif command.startswith('rm -rf /tmp/minerva-check-'):
            self.cleaned = True
        else:
            raise AssertionError(f'Unexpected command {command}')
        return stdin, stdout, stderr

    def close(self):
        self.closed = True


@pytest.fixture
def runs(tmp_path):
    """Helm's direction session (every arm joint confirmed), as Helm writes it."""
    runs = tmp_path / 'runs'
    session = HelmSession(PROFILE, runs=runs, limits=tmp_path / 'unused-limits.json', seed=SEED)
    session.close()
    path = session.state_path
    state = json.loads(path.read_text())
    for row in state['preview_mapping'].values():
        row['direction_verified'] = True
    path.write_text(json.dumps(state))
    return runs


@pytest.fixture
def loopback(monkeypatch):
    """Treat loopback as the tether and stub the ARM64 build check."""
    monkeypatch.setattr(check, 'TETHER_NETWORK', '127.0.0.')
    builds = next(spec for spec in check.CHECKS if spec['id'] == 'builds')
    monkeypatch.setitem(builds, 'run', lambda ctx: (check.PASS, 'stubbed'))


def run(runs, robot, **options):
    published = []
    report = check.run_check(runs=runs, address='127.0.0.1', open_ssh=lambda *a, **k: robot,
                             publish=published.append, **options)
    return report, {entry['id']: entry for stage in report['stages'] for entry in stage['checks']}, published


def test_a_complete_check_of_a_healthy_robot_is_a_go_and_is_saved(runs, loopback):
    robot = FakeRobot()
    report, checks, published = run(runs, robot)
    # The stopped checks pass; GO needs the live joint sweeps, which run attended from the Check tab.
    assert report['verdict'] == 'incomplete' and 'live joint sweep' in report['summary'], report['summary']
    live = ('sweep_left', 'sweep_right')
    assert {entry['status'] for key, entry in checks.items() if key not in live} <= {'pass', 'warn'}
    assert checks['sweep_left']['status'] == checks['sweep_right']['status'] == 'pending'
    # The robot's systems only: nothing about the leader arms or how the robot is controlled.
    assert not {'leaders', 'leader_ranges', 'routes', 'link'} & set(checks)
    assert 'All 14 motors answered' in checks['scan']['detail'] and 'inside their swept ranges' in checks['pose']['detail']
    assert any(entry['status'] == 'running' for stage in published[0]['stages'] for entry in stage['checks'])
    assert robot.closed and sum(command.endswith('probe motors') for command in robot.commands) == 1
    assert [len(config['motors']) for config in robot.validated] == [7, 7, 14] and robot.cleaned
    assert [[bus['interface'] for bus in config['buses']] for config in robot.validated] == [
        ['canL'], ['canR'], ['canL', 'canR']]
    assert robot.validated[2]['controller']['collision']['bound'] == 'motion'
    assert 'Both arms' in checks['live_config']['detail']
    saved = check.latest_report(runs)
    assert saved['verdict'] == 'incomplete' and saved['evidence'] == report['evidence']
    # A live sweep's outcome goes into the same report, which then becomes a GO.
    passing = {'left_arm_j1': {'label': 'Left J1', 'check': {'passed': True, 'reasons': [], 'lag': .02,
                                                            'friction': .4, 'calibrated': True}}}
    check.record_live(saved, 'left', passing)
    assert saved['verdict'] == 'incomplete' and 'right arm' in saved['summary'] and 'left arm' not in saved['summary']
    # A gripper's check is its travel, from its swept min to its swept max and back: no lag, never servo-swept.
    gripper = {'label': 'Right gripper', 'check': {
        'passed': True, 'reasons': [], 'lag': None, 'friction': None, 'calibrated': None,
        'travel': {'open': {'reached': True, 'seconds': 3.1}, 'close': {'reached': True, 'seconds': 3.4}}}}
    check.record_live(saved, 'right', {'right_arm_j1': passing['left_arm_j1'], 'right_gripper': gripper})
    assert saved['verdict'] == 'go', saved['summary']
    entry = next(c for stage in saved['stages'] for c in stage['checks'] if c['id'] == 'sweep_right')
    assert entry['status'] == 'pass' and 'Right gripper opened and closed its whole travel (3.1 s, 3.4 s)' in \
        entry['detail'] and 'followed within half their tracking bound' in entry['detail']
    assert check.latest_report(runs)['verdict'] == 'go'  # Saved over the same file.
    failing = {'right_arm_j5': {'label': 'Right wrist J5', 'check': {
        'passed': False, 'reasons': ['friction 0.41 Nm, up from 0.12 when swept'], 'lag': .02, 'friction': .41,
        'calibrated': True}}}
    check.record_live(saved, 'right', failing)
    entry = next(c for stage in saved['stages'] for c in stage['checks'] if c['id'] == 'sweep_right')
    assert saved['verdict'] == 'no_go' and 'Right wrist J5: friction 0.41 Nm' in entry['detail']
    check.record_live(saved, 'right', None, fault='Receiver faulted: limit_violation')
    assert 'stopped: Receiver faulted' in next(c for stage in saved['stages'] for c in stage['checks']
                                               if c['id'] == 'sweep_right')['detail']


def test_a_failure_skips_what_depends_on_it_and_says_how_to_fix_it(runs, loopback):
    report, checks, _published = run(runs, FakeRobot(release=False))
    assert report['verdict'] == 'no_go' and 'Deployed release' in report['summary']
    assert 'scripts/deploy_minerva_arms.sh' in checks['release']['detail']
    for key in ('current', 'smoke', 'health', 'setup', 'adapters', 'services', 'scan', 'pose'):
        assert checks[key]['status'] == 'skip', key
    assert checks['ssh']['status'] == 'pass' and checks['receiver']['status'] == 'pass'


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
    _report, checks, _ = run(runs, FakeRobot(motors_report=moved))
    assert checks['pose']['status'] == 'fail' and 'beyond its swept' in checks['pose']['detail']


def hanging(**positions):
    report = scan()
    for row in report['motors']:
        name = row['name']
        # Arms hang in the reference pose; closed grippers rest on their swept ends.
        row['position_rad'] = positions.get(name, LIMITS['joints'][name]['min_rad'] if 'gripper' in name
                                            else next(m['raw_reference_rad'] for m in REFERENCE['motors']
                                                      if m['name'] == name))
    return report


def test_what_would_stop_a_live_session_at_once_is_caught_before_one_starts(runs, loopback):
    _report, checks, _ = run(runs, FakeRobot(motors_report=hanging()))
    assert checks['live_pose']['status'] == 'pass', checks['live_pose']['detail']
    grip = LIMITS['joints']['right_gripper']['min_rad']
    _report, checks, _ = run(runs, FakeRobot(motors_report=hanging(right_gripper=grip - .0495)))
    assert checks['live_pose']['status'] == 'fail'
    assert "live receiver's limits" in checks['live_pose']['detail'] and 'Right gripper' in checks['live_pose']['detail']
    _report, checks, _ = run(runs, FakeRobot(motors_report=hanging(left_arm_j2=.3)))
    assert checks['live_pose']['status'] == 'warn' and 'left arm: Left shoulder hinge (J2) +0.30 rad' in checks['live_pose']['detail']


def test_a_drive_that_powered_up_a_whole_turn_high_is_explained_not_blamed_on_its_zero(runs, loopback):
    report = hanging(left_arm_j1=2 * math.pi - .11)
    next(row for row in report['motors'] if row['name'] == 'left_arm_j1')['zero_sta'] = 0
    _report, checks, _ = run(runs, FakeRobot(motors_report=report))
    for key in ('pose', 'live_pose'):
        detail = checks[key]['detail']
        assert checks[key]['status'] == 'fail', key
        assert detail.startswith('Left shoulder swivel (J1) reads +6.17 rad, -0.11 rad plus a whole turn'), detail
        assert 'power-cycle the motors' in detail and 'zero shifted' not in detail and 'Re-sweep' not in detail
    assert checks['power_on']['status'] == 'warn'
    assert checks['power_on']['detail'].startswith('Left shoulder swivel (J1) starts within 0…2π (zero_sta 0)')
    # A joint far outside its range by anything but a whole turn is still an unexplained reading.
    _report, checks, _ = run(runs, FakeRobot(motors_report=hanging(left_arm_j1=4.)))
    assert 'zero shifted' in checks['pose']['detail'] and 'whole turn' not in checks['pose']['detail']
    assert checks['power_on']['status'] == 'pass' and 'All 14 drives start within' in checks['power_on']['detail']


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
    assert status == 'fail' and 'foot switch' in detail and 'acknowledged' not in detail
    passive = {bus: {'flags': ['UP'], 'linkinfo': {'info_data': {'state': 'ERROR-PASSIVE'}}} for bus in ('canL', 'canR')}
    status, detail = findings(scan(success=False, error_code='motors_unresponsive', unresponsive=names,
                                   before_cleanup=passive))
    assert 'nothing on canL or canR acknowledged a frame. Motor power is off' in detail
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


def test_the_servo_calibration_is_checked_against_the_motors_it_was_swept_with(runs, loopback, monkeypatch, tmp_path):
    from robot_platform.control import servo_calibration
    path = tmp_path / 'servo.json'
    monkeypatch.setattr(servo_calibration, 'servo_path', lambda profile, asset: path)
    _report, checks, _ = run(runs, FakeRobot(), motors=False)
    assert checks['servo']['status'] == 'warn' and 'Tune tab' in checks['servo']['detail']
    # Grippers are not swept (they squeeze what they hold): the live joint sweep opens and closes them instead.
    joints = {m['name']: {'uid': m['uid'], 'firmware': m['firmware'], 'hold_kp': 10., 'hold_kd': 1.}
              for m in REFERENCE['motors'] if not m['name'].endswith('_gripper')}
    path.write_text(json.dumps({'schema': 1, 'kind': 'servo_calibration', 'updated_at': '2026-09-27T00:00:00',
                                'joints': joints}))
    _report, checks, _ = run(runs, FakeRobot(), motors=False)
    assert checks['servo']['status'] == 'pass' and 'All 12 arm joints' in checks['servo']['detail']
    assert 'grippers are checked by their travel' in checks['servo']['detail']
    assert checks['receiver']['status'] == 'pass'  # The receiver derives from it.
    joints['left_arm_j4'] = dict(joints['left_arm_j4'], uid='0000000000000000')
    path.write_text(json.dumps({'schema': 1, 'kind': 'servo_calibration', 'joints': joints}))
    _report, checks, _ = run(runs, FakeRobot(), motors=False)
    assert checks['servo']['status'] == 'fail' and 'sweep them again' in checks['servo']['detail'].lower()
