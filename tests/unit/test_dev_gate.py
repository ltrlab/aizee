"""A successful pytest process cannot turn missing required evidence into a pass."""
import importlib.util
from pathlib import Path
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
