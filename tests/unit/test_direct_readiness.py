"""Selected direct-mode preflight and checkout isolation, without physical devices."""
import builtins
import json
import os
import re
import shlex
import subprocess
import sys

import pytest

from robot_platform.ops import check, cli, release, smoke
from tests.unit.test_minerva_check import FakeRobot, Stream, loopback, run, runs  # noqa: F401


class DirectRobot(FakeRobot):
    """Accept a direct candidate only when its controller and physical mode agree."""

    def exec_command(self, command, timeout=None):
        if not command.endswith('--validate'):
            return super().exec_command(command, timeout)
        self.commands.append(command)
        config = json.loads(''.join(self.staged[shlex.split(command)[1]].lines.queue))
        self.validated.append(config)
        assert config['mode'] == 'teleop_direct'
        assert config['controller']['identity']['collision_policy'] == 'disabled-v1'
        assert config['controller']['direct']['command_ms'] == 30
        assert config['controller'].get('collision') is None
        stdin, stdout, stderr = Stream(), Stream(), Stream()
        stdout.write('configuration valid; no sockets opened\n')
        return stdin, stdout, stderr


def test_direct_preflight_validates_each_arm_and_both_without_collision(runs, loopback, monkeypatch):
    monkeypatch.setattr(check, 'route_source', lambda _: '127.0.0.1')
    robot = DirectRobot()
    profile = release.ROOT / 'config/devices/minerva-teleop-direct.json'
    report, checks, _ = run(runs, robot, teleop=profile)
    assert checks['receiver']['status'] == check.PASS, checks['receiver']['detail']
    assert checks['live_config']['status'] == check.PASS, checks['live_config']['detail']
    assert 'collision-disabled direct mode' in checks['live_config']['detail']
    assert [len(config['motors']) for config in robot.validated] == [7, 7, 14]
    assert [[bus['interface'] for bus in config['buses']] for config in robot.validated] == [
        ['canL'], ['canR'], ['canL', 'canR']]
    assert robot.cleaned
    # Parsing configurations does not claim attended physical qualification.
    assert report['verdict'] == 'incomplete'
    assert checks['sweep_left']['status'] == checks['sweep_right']['status'] == check.PENDING


def test_check_cli_forwards_selected_profiles_and_never_enables_motors(tmp_path, monkeypatch):
    captured = {}

    def run_check(**options):
        captured.update(options)
        return dict(summary='Stopped preflight only', evidence='test.json', verdict='incomplete')

    monkeypatch.setattr(check, 'run_check', run_check)
    profile, teleop, limits = (tmp_path / name for name in ('calibration.json', 'direct.json', 'limits.json'))
    candidate = '/home/ltr/minerva/validation/direct-candidate'
    assert cli.check_main(['--profile', str(profile), '--teleop', str(teleop), '--limits', str(limits),
                           '--runs', str(tmp_path), '--remote-root', candidate, '--no-motors']) == 1
    assert captured['profile'] == profile and captured['teleop'] == teleop
    assert captured['limits'] == limits and captured['runs'] == tmp_path
    assert captured['motors'] is False
    assert captured['remote_root'] == candidate


def test_isolated_candidate_check_uses_candidate_code_for_every_remote_probe(runs, loopback, monkeypatch):
    monkeypatch.setattr(check, 'route_source', lambda _: '127.0.0.1')
    # Spaces, shell operators and quotes remain one literal path argument.
    candidate = "/home/ltr/minerva/validation/direct candidate' ; $(never_execute)"

    class CandidateRobot(DirectRobot):
        def __init__(self):
            super().__init__()
            self.sent = []
            self.release['root'] = candidate

        def exec_command(self, command, timeout=None):
            self.sent.append(command)
            arguments = shlex.split(command)
            if arguments[:2] == ['python3', '-']:
                assert arguments == ['python3', '-', candidate]
                # Reuse the fake release reply; no remote command is executed.
                return super().exec_command(f'python3 - {check.CURRENT}', timeout)
            return super().exec_command(command, timeout)

    robot = CandidateRobot()
    profile = release.ROOT / 'config/devices/minerva-teleop-direct.json'
    report, checks, _ = run(runs, robot, teleop=profile, remote_root=candidate)
    assert report['remote_root'] == candidate and report['verdict'] == 'incomplete'
    assert all(checks[name]['status'] == check.PASS for name in ('release', 'current', 'smoke', 'live_config', 'scan'))
    assert checks['health']['status'] in (check.PASS, check.WARN)
    assert all(check.CURRENT not in command for command in robot.sent)
    probes = [shlex.split(command) for command in robot.sent if 'robot_platform.ops.probe' in command]
    assert {arguments[-1] for arguments in probes} == {'health', 'motors'}
    assert all(arguments[:2] == ['cd', candidate] for arguments in probes)
    smoked = next(shlex.split(command) for command in robot.sent if 'robot_platform.ops.smoke' in command)
    assert smoked[:2] == ['cd', candidate] and smoked[-1] == candidate
    binaries = [shlex.split(command)[0] for command in robot.sent if command.endswith('--validate')]
    assert binaries == [candidate + '/bin/guarded_controller'] * 3


@pytest.mark.parametrize('remote_root', ['', 'relative/candidate', '/', '//', '/tmp/../candidate',
                                       'C:\\candidate', '/tmp/candidate\nnext', '/tmp/candidate\x00'])
def test_invalid_candidate_paths_fail_before_ssh_or_evidence(tmp_path, remote_root):
    with pytest.raises(ValueError, match='Remote release root'):
        check.run_check(remote_root=remote_root, runs=tmp_path,
                        open_ssh=lambda *args, **kwargs: pytest.fail('invalid root must not connect'))
    assert not (tmp_path / 'minerva-check').exists()


def test_check_cli_rejects_relative_candidate_before_running(monkeypatch):
    monkeypatch.setattr(check, 'run_check', lambda **kwargs: pytest.fail('invalid root must not run'))
    with pytest.raises(SystemExit) as error:
        cli.check_main(['--remote-root', 'relative/candidate'])
    assert error.value.code == 2


def test_release_declares_zmq_and_smoke_fails_before_motion_if_it_is_missing(tmp_path, monkeypatch):
    payload = tmp_path / 'payload.py'
    payload.write_text('# candidate\n')
    monkeypatch.setattr(release, 'collect', lambda root: {'src/robot_platform/payload.py': payload})
    monkeypatch.setattr(release, 'check_binaries', lambda files, root: None)
    monkeypatch.setattr(release, 'verify_model', lambda root: ({}, {}))
    monkeypatch.setattr(release, '_git', lambda *args, **kwargs: 'abcdef0' if args[0] == 'rev-parse' else '')
    manifest, _archive = release.build(tmp_path / 'out', root=tmp_path)
    assert 'zmq' in manifest['requires']['modules']
    monkeypatch.setattr(release, 'activate', lambda root: manifest)
    original_import = builtins.__import__

    def without_zmq(name, *args, **kwargs):
        if name == 'zmq':
            raise ModuleNotFoundError('No module named zmq')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', without_zmq)
    with pytest.raises(ModuleNotFoundError, match='zmq'):
        smoke.run(tmp_path)


@pytest.mark.parametrize('foreign', [False, True])
def test_helm_import_guard_rejects_another_checkout_before_start(tmp_path, foreign):
    launcher = (release.ROOT / 'scripts/helm.ps1').read_text(encoding='utf-8')
    guard = re.search(r"\$helmImportCheck = @'\r?\n(.*?)\r?\n'@", launcher, re.DOTALL)
    assert guard is not None
    expected = tmp_path / 'candidate'
    imported = tmp_path / 'other' if foreign else expected
    package = imported / 'src/robot_platform'
    package.mkdir(parents=True)
    (package / '__init__.py').write_text('')
    environment = dict(os.environ, PYTHONPATH=str(imported / 'src'))
    # -S excludes the test environment's editable installation; only this fixture
    # package is importable. The launcher guard runs unchanged under the same Python.
    result = subprocess.run([sys.executable, '-S', '-c', guard[1], str(expected)], env=environment,
                            capture_output=True, text=True, timeout=10)
    assert (result.returncode != 0) is foreign
    if foreign:
        assert 'Refusing Helm from another checkout' in result.stderr
