"""A gate run's time, split by stage, test file and test (scripts/gate_time_study.py)."""
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('aizee_gate_time_study', ROOT / 'scripts/gate_time_study.py')
study_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study_module)

STARTED = datetime(2026, 10, 24, 1, 0, 0, tzinfo=timezone.utc)


def run(tmp_path, name, cases, *, pytest_at=10., pytest_s=85., wall_s=100.):
    """A finished run folder: tests.xml with `cases` ((classname, test, seconds)) and a report written at the end."""
    folder = tmp_path / name
    folder.mkdir()
    started = datetime.fromtimestamp(STARTED.timestamp() + pytest_at, timezone.utc).isoformat()
    rows = ''.join(f'<testcase classname="{c}" name="{n}" time="{s}" />' for c, n, s in cases)
    (folder / 'tests.xml').write_text(f'<?xml version="1.0"?><testsuites><testsuite timestamp="{started}" '
                                      f'time="{pytest_s}">{rows}</testsuite></testsuites>', encoding='utf-8')
    (folder / 'report.json').write_text(json.dumps({'passed': True}), encoding='utf-8')
    end = STARTED.timestamp() + wall_s
    os.utime(folder / 'report.json', (end, end))
    return folder


def test_a_run_splits_into_stages_files_and_its_slowest_tests(tmp_path):
    folder = run(tmp_path, '20261024T010000Z-abc12345', [
        ('tests.unit.test_a', 'test_one', 50.), ('tests.unit.test_a.TestCase', 'test_two', 5.),
        ('tests.contract.test_b', 'test_three[left]', 30.)])
    result = study_module.study(folder)
    assert result['stages_s'] == {'before_pytest': 10., 'pytest': 85., 'after_pytest': 5.} and result['wall_s'] == 100.
    assert result['files_s'] == {'tests/unit/test_a.py': 55., 'tests/contract/test_b.py': 30.}  # A class's tests too.
    assert [row['test'] for row in result['slowest']] == [
        'tests/unit/test_a.py::test_one', 'tests/contract/test_b.py::test_three[left]', 'tests/unit/test_a.py::test_two']


def test_two_runs_show_the_files_whose_time_changed_most(tmp_path, capsys):
    first = run(tmp_path, '20261024T010000Z-first000', [('tests.unit.test_a', 'one', 10.), ('tests.unit.test_b', 'two', 20.)])
    last = run(tmp_path, '20261024T010000Z-last0000', [('tests.unit.test_a', 'one', 3000.), ('tests.unit.test_b', 'two', 18.)],
               pytest_s=3018., wall_s=3040.)
    rows = study_module.changes(study_module.study(first), study_module.study(last))
    assert rows[0] == (2990., 10., 3000., 'tests/unit/test_a.py')  # The test that ran while the computer slept.
    assert study_module.main(['--json', str(first), str(last)]) == 0
    assert [s['run'] for s in json.loads(capsys.readouterr().out)] == [first.name, last.name]
