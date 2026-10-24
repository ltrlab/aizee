"""A successful pytest process cannot turn missing required evidence into a pass."""
import importlib.util
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('aizee_dev_gate', ROOT / 'scripts/dev.py')
dev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dev)


@pytest.mark.parametrize('marker', ['skip', 'xfail'])
def test_successful_pytest_with_missing_evidence_fails_required_gate(tmp_path, marker):
    case = '<testcase name="release"><skipped type="pytest.' + marker + '"/></testcase>'
    path = tmp_path / 'tests.xml'
    path.write_text('<testsuites><testsuite>' + case + '</testsuite></testsuites>')
    with pytest.raises(RuntimeError, match='must execute every test'):
        dev.required_test_results(path)


def test_empty_required_suite_is_not_evidence(tmp_path):
    path = tmp_path / 'tests.xml'
    path.write_text('<testsuites/>')
    with pytest.raises(RuntimeError, match='must execute every test'):
        dev.required_test_results(path)


def test_required_suite_counts_executed_cases(tmp_path):
    path = tmp_path / 'tests.xml'
    path.write_text('<testsuites><testsuite><testcase name="release"/></testsuite></testsuites>')
    assert dev.required_test_results(path) == {'collected': 1, 'skipped': []}


def test_release_preflight_refuses_an_empty_checkout_before_native_build(tmp_path):
    # Exercise the real release validator in a subprocess, with an isolated empty target.
    script = ('from robot_platform.ops import release; '
              f'release.ROOT = __import__("pathlib").Path({str(tmp_path)!r}); '
              'release.check_binaries({"bin/guarded_controller": release.ROOT / "missing"}, release.ROOT)')
    for folder in ('rust/control_core', 'rust/collision'):
        source = tmp_path / folder / 'lib.rs'
        source.parent.mkdir(parents=True)
        source.write_text('// source')
    import subprocess
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'missing or not an AArch64 build' in result.stderr


def test_qualified_reference_bytes_survive_both_git_checkout_policies(tmp_path):
    """A fresh checkout must still satisfy the recorded calibration checksum."""
    relative = Path('config/devices/minerva-reference-20261009.json')
    expected = json.loads((ROOT / 'config/devices/minerva-production-01-joint-limits.json')
                          .read_text(encoding='utf-8'))['reference_sha256']
    repository = tmp_path / 'repository'
    target = repository / relative
    target.parent.mkdir(parents=True)
    target.write_bytes((ROOT / relative).read_bytes())
    (repository / '.gitattributes').write_bytes((ROOT / '.gitattributes').read_bytes())

    def git(*args):
        subprocess.run(['git', '-C', str(repository), *args], check=True, capture_output=True)

    git('init', '--quiet')
    git('-c', 'core.autocrlf=true', 'add', '.gitattributes', relative.as_posix())
    for policy in ('true', 'false'):
        checkout = tmp_path / ('checkout-' + policy)
        checkout.mkdir()
        git('-c', 'core.autocrlf=' + policy, 'checkout-index', '--all',
            '--prefix=' + checkout.as_posix() + '/')
        assert hashlib.sha256((checkout / relative).read_bytes()).hexdigest() == expected


# -- the quick lane ----------------------------------------------------------------------------------------------

def test_the_quick_lane_refuses_the_motion_contract_and_the_locks():
    for path in ('rust/control_core/src/lib.rs', 'src/robot_platform/control/approach.py',
                 'src/robot_platform/control/collision_envelope.py', 'src/robot_platform/control/guarded_config.py',
                 'src/robot_platform/teleop/guarded_lifecycle.py', 'requirements/dev.lock', 'pyproject.toml'):
        assert path.startswith(dev.MOTION_CONTRACT), path
    for path in ('src/robot_platform/teleop/grip_feedback.py', 'src/robot_platform/helm/static/helm.js',
                 'src/robot_platform/helm/teleop.py', 'docs/DEVELOPMENT.md'):
        assert not path.startswith(dev.MOTION_CONTRACT), path


def test_the_quick_lane_runs_the_tests_a_change_touches_or_the_ones_named():
    assert dev.module_name('src/robot_platform/teleop/grip_feedback.py') == 'robot_platform.teleop.grip_feedback'
    assert dev.module_name('src/robot_platform/helm/__init__.py') == 'robot_platform.helm'
    found = dev.select_tests(['src/robot_platform/teleop/grip_feedback.py'])
    assert 'tests/unit/test_grip_feedback.py' in found and 'tests/contract/test_collision_envelope.py' not in found
    assert set(found) <= set(dev.CORE)
    assert dev.select_tests(['docs/DEVELOPMENT.md']) == []
    assert dev.select_tests(['src/robot_platform/helm/static/grip.js']) == ['tests/unit/test_browser_scripts.py']
    assert 'tests/unit/test_gate_time_study.py' in dev.select_tests(['scripts/gate_time_study.py'])
    # Named: just those, with every changed test and, for a changed page, the page checks.
    assert dev.select_tests(['src/robot_platform/helm/teleop.py', 'src/robot_platform/helm/static/helm.js',
                             'tests/unit/test_grip_feedback.py'], ['tests/unit/test_helm_teleop.py']) == [
        'tests/unit/test_browser_scripts.py', 'tests/unit/test_grip_feedback.py', 'tests/unit/test_helm_teleop.py']


def test_the_quick_lane_compares_with_the_last_passing_check(tmp_path, monkeypatch):
    monkeypatch.setattr(dev, 'ROOT', tmp_path)
    assert dev.last_passed_commit() is None
    for name, gate, passed, commit in (('20261024T010000Z-a', 'offline-control', True, 'older'),
                                       ('20261024T020000Z-b', 'offline-control', True, 'latest'),
                                       ('20261024T030000Z-c', 'offline-control', False, 'failed')):
        folder = tmp_path / 'build/dev-check' / name
        folder.mkdir(parents=True)
        (folder / 'report.json').write_text(json.dumps({'gate': gate, 'passed': passed,
                                                        'source_before': {'commit': commit}}))
    assert dev.last_passed_commit() == 'latest'
