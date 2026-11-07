"""Where an offline gate run's time went (scripts/dev.py check): its stages, its test files and its slowest tests.

    python scripts/gate_time_study.py                  the latest run under build/dev-check
    python scripts/gate_time_study.py RUN [RUN...]     runs by folder name or path; with two or more, also the files
                                                       whose time changed most from the first to the last
    python scripts/gate_time_study.py --json RUN       the same as JSON

Stages come from the run's start (its folder name), pytest's start (tests.xml), the serial phase's length
(tests-serial.xml, after the parallel phase) and the report's write; tests and files from both reports. With the
parallel phase a file's seconds add up across workers, so the percentages are shares of the summed test time.
A run that spans sleep counts the sleep: an outlier test usually means the computer slept.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'build' / 'dev-check'


def module_path(classname):
    """tests/unit/test_x.py from pytest's classname (tests.unit.test_x, or tests.unit.test_x.TestCase)."""
    parts = classname.split('.')
    end = next((i for i, part in enumerate(parts) if part.startswith('test_')), len(parts) - 1)
    return '/'.join(parts[: end + 1]) + '.py'


def suite_of(path):
    root = ET.parse(path).getroot()
    return root if root.tag == 'testsuite' else root.find('testsuite')


def study(folder):
    """The time study of one run folder (report.json, tests.xml and the serial phase's tests-serial.xml, if any)."""
    folder = Path(folder)
    started = datetime.strptime(folder.name.split('-')[0], '%Y%m%dT%H%M%SZ').replace(tzinfo=timezone.utc)
    report = json.loads((folder / 'report.json').read_text(encoding='utf-8'))
    ended = datetime.fromtimestamp((folder / 'report.json').stat().st_mtime, timezone.utc)
    suites = [suite_of(folder / 'tests.xml')]
    if (folder / 'tests-serial.xml').is_file():
        suites.append(suite_of(folder / 'tests-serial.xml'))
    pytest_started = datetime.fromisoformat(suites[0].get('timestamp'))
    pytest_s = float(suites[0].get('time'))
    files, tests = defaultdict(float), []
    for case in (case for suite in suites for case in suite.iter('testcase')):
        seconds = float(case.get('time') or 0.0)
        name = module_path(case.get('classname'))
        files[name] += seconds
        tests.append({'test': f'{name}::{case.get("name")}', 'seconds': round(seconds, 3)})
    before = (pytest_started - started).total_seconds()
    wall = (ended - started).total_seconds()
    stages = {'before_pytest': round(before, 1), 'pytest': round(pytest_s, 1)}
    serial_s = float(suites[1].get('time')) if len(suites) > 1 else 0.0
    if len(suites) > 1:
        stages['serial_pytest'] = round(serial_s, 1)
    stages['after_pytest'] = round(wall - before - pytest_s - serial_s, 1)
    return {
        'run': folder.name,
        'passed': report.get('passed'),
        'tests': len(tests),
        'wall_s': round(wall, 1),
        'stages_s': stages,
        'test_s': round(sum(files.values()), 1),
        'files_s': {name: round(seconds, 1) for name, seconds in sorted(files.items(), key=lambda item: -item[1])},
        'slowest': sorted(tests, key=lambda item: -item['seconds'])[:15],
    }


def changes(first, last, count=10):
    """The files whose time changed most between two studies: (change, first, last, file)."""
    names = set(first['files_s']) | set(last['files_s'])
    rows = [
        (
            round(last['files_s'].get(n, 0.0) - first['files_s'].get(n, 0.0), 1),
            first['files_s'].get(n, 0.0),
            last['files_s'].get(n, 0.0),
            n,
        )
        for n in names
    ]
    return sorted(rows, key=lambda row: -abs(row[0]))[:count]


def resolve(run):
    path = Path(run)
    return path if path.is_dir() else RUNS / run


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='python scripts/gate_time_study.py',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('runs', nargs='*', help='run folders (names under build/dev-check, or paths)')
    parser.add_argument('--json', action='store_true', help='print the studies as JSON')
    args = parser.parse_args(argv)
    folders = [resolve(run) for run in args.runs]
    if not folders:
        finished = sorted((path for path in RUNS.glob('*') if (path / 'tests.xml').is_file()), key=lambda p: p.name)
        if not finished:
            parser.error(f'No finished gate run under {RUNS}')
        folders = finished[-1:]
    studies = [study(folder) for folder in folders]
    if args.json:
        print(json.dumps(studies if len(studies) > 1 else studies[0], indent=1))
        return 0
    for s in studies:
        stages = s['stages_s']
        serial = f', serial {stages["serial_pytest"]:.0f} s' if 'serial_pytest' in stages else ''
        print(
            f'\n{s["run"]}: {"passed" if s["passed"] else "FAILED"}, {s["tests"]} tests, {s["wall_s"] / 60:.1f} min '
            f'(before pytest {stages["before_pytest"]:.0f} s, pytest {stages["pytest"]:.0f} s{serial}, '
            f'after {stages["after_pytest"]:.0f} s; {s["test_s"]:.0f} s of tests)'
        )
        print('  slowest files:')
        for name, seconds in list(s['files_s'].items())[:12]:
            print(f'    {seconds:8.1f} s  {100 * seconds / max(s["test_s"], 1e-9):5.1f} %  {name}')
        print('  slowest tests:')
        for row in s['slowest'][:10]:
            print(f'    {row["seconds"]:8.1f} s  {row["test"]}')
    if len(studies) > 1:
        print(f'\nfiles whose time changed most, {studies[0]["run"]} -> {studies[-1]["run"]}:')
        for change, first, last, name in changes(studies[0], studies[-1]):
            print(f'    {change:+8.1f} s  {first:7.1f} -> {last:7.1f}  {name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
