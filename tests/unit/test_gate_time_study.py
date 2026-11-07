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


def junit(path, cases, started, seconds):
    rows = ''.join(f'<testcase classname="{c}" name="{n}" time="{s}" />' for c, n, s in cases)
    path.write_text(
        f'<?xml version="1.0"?><testsuites><testsuite timestamp="{started}" '
        f'time="{seconds}">{rows}</testsuite></testsuites>',
        encoding='utf-8',
    )


def run(tmp_path, name, cases, *, pytest_at=10.0, pytest_s=85.0, wall_s=100.0, serial=None, serial_s=0.0):
    """A finished run folder: tests.xml with `cases` ((classname, test, seconds)), the serial phase's
    tests-serial.xml when `serial` is given, and a report written at the end."""
    folder = tmp_path / name
    folder.mkdir()
    started = datetime.fromtimestamp(STARTED.timestamp() + pytest_at, timezone.utc).isoformat()
    junit(folder / 'tests.xml', cases, started, pytest_s)
    if serial is not None:
        junit(folder / 'tests-serial.xml', serial, started, serial_s)
    (folder / 'report.json').write_text(json.dumps({'passed': True}), encoding='utf-8')
    end = STARTED.timestamp() + wall_s
    os.utime(folder / 'report.json', (end, end))
    return folder


def test_a_run_splits_into_stages_files_and_its_slowest_tests(tmp_path):
    folder = run(
        tmp_path,
        '20261024T010000Z-abc12345',
        [
            ('tests.unit.test_a', 'test_one', 50.0),
            ('tests.unit.test_a.TestCase', 'test_two', 5.0),
            ('tests.contract.test_b', 'test_three[left]', 30.0),
        ],
    )
    result = study_module.study(folder)
    assert (
        result['stages_s'] == {'before_pytest': 10.0, 'pytest': 85.0, 'after_pytest': 5.0} and result['wall_s'] == 100.0
    )
    assert result['files_s'] == {'tests/unit/test_a.py': 55.0, 'tests/contract/test_b.py': 30.0}  # A class's tests too.
    assert [row['test'] for row in result['slowest']] == [
        'tests/unit/test_a.py::test_one',
        'tests/contract/test_b.py::test_three[left]',
        'tests/unit/test_a.py::test_two',
    ]


def test_a_parallel_run_counts_its_serial_phase_and_its_summed_test_time(tmp_path, capsys):
    # Three workers ran 90 s of tests in 30 s; the wall-clock tests then ran alone for 20 s.
    folder = run(
        tmp_path,
        '20261024T010000Z-par12345',
        [('tests.unit.test_a', 'one', 30.0), ('tests.unit.test_b', 'two', 30.0), ('tests.unit.test_c', 'three', 30.0)],
        pytest_s=30.0,
        wall_s=65.0,
        serial=[('tests.contract.test_d', 'clock', 20.0)],
        serial_s=20.0,
    )
    result = study_module.study(folder)
    assert result['stages_s'] == {'before_pytest': 10.0, 'pytest': 30.0, 'serial_pytest': 20.0, 'after_pytest': 5.0}
    assert result['tests'] == 4 and result['test_s'] == 110.0 and result['files_s']['tests/contract/test_d.py'] == 20.0
    assert study_module.main([str(folder)]) == 0
    assert 'serial 20 s' in capsys.readouterr().out


def test_two_runs_show_the_files_whose_time_changed_most(tmp_path, capsys):
    first = run(
        tmp_path, '20261024T010000Z-first000', [('tests.unit.test_a', 'one', 10.0), ('tests.unit.test_b', 'two', 20.0)]
    )
    last = run(
        tmp_path,
        '20261024T010000Z-last0000',
        [('tests.unit.test_a', 'one', 3000.0), ('tests.unit.test_b', 'two', 18.0)],
        pytest_s=3018.0,
        wall_s=3040.0,
    )
    rows = study_module.changes(study_module.study(first), study_module.study(last))
    assert rows[0] == (2990.0, 10.0, 3000.0, 'tests/unit/test_a.py')  # The test that ran while the computer slept.
    assert study_module.main(['--json', str(first), str(last)]) == 0
    assert [s['run'] for s in json.loads(capsys.readouterr().out)] == [first.name, last.name]
