"""One trial record for simulated and hand-logged real-robot trials alike.

A trial reaches up to three stages in order: pickup, handoff, bucket. Times are seconds from
the trial's start (the scene reset in simulation; the operator's go signal on the robot). A
stage counts only when every earlier stage was reached: a cube pushed or dropped into the bucket
without a handoff is not a bucket placement. A trial ends at the first of:

- success          every stage reached, eligible, no failure;
- dropped_cube     the cube returned to the table or floor after pickup;
- forbidden_penetration  the robot went into the table or bucket, or one arm into the other (ineligible);
- incomplete_sequence    the cube settled in the bucket without the stages before it;
- timeout          the trial ran out of time;
- source_gap_or_abort    the run stopped: receiver fault, command source gap, policy error,
                   numerical quarantine, or an operator stop on the robot;
- invalid_start    the initial condition was invalid (the cube placed inside the robot, staging
                   failed). Not counted: it says nothing about the policy.

Simulated outcomes are read from MinervaTaskScene's own events, never re-derived.
"""
from __future__ import annotations

import csv
import io
import json
import math
from pathlib import Path

SCHEMA = 'minerva_eval_trial/1'
STAGES = ('pickup', 'handoff', 'bucket')
FAILURES = ('dropped_cube', 'forbidden_penetration', 'incomplete_sequence', 'timeout', 'source_gap_or_abort')
# The scene's events that make a trial ineligible: the robot into the table or bucket, or one arm into the other.
FORBIDDEN_EVENTS = ('forbidden_robot_environment_penetration', 'forbidden_robot_robot_penetration')
EXCLUDED = ('invalid_start',)
CATEGORIES = ('success',) + FAILURES + EXCLUDED
SOURCES = ('simulation', 'real')
# Hand-logging columns (CSV). Blank stage times mean the stage was not reached.
CSV_COLUMNS = ('trial', 'cube_x', 'cube_y', 'cube_yaw', 'pickup_s', 'handoff_s', 'bucket_s', 'outcome',
               'eligible', 'duration_s', 'detail', 'notes')


def _time(value, name):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError(f'{name} must be a finite non-negative time in seconds, or empty')
    return float(value)


def reached(stages):
    """{stage: bool}: reached, and every stage before it reached."""
    result, previous = {}, True
    for stage in STAGES:
        previous = previous and stages.get(stage) is not None
        result[stage] = previous
    return result


def validate(record):
    """A checked copy of one trial record, or ValueError naming what is wrong."""
    if not isinstance(record, dict):
        raise ValueError('a trial record is a JSON object')
    trial = dict(record)
    trial.setdefault('schema', SCHEMA)
    if trial['schema'] != SCHEMA:
        raise ValueError(f"unknown trial schema {trial['schema']!r}")
    if trial.get('source') not in SOURCES:
        raise ValueError(f"source must be one of {SOURCES}")
    if type(trial.get('trial')) is not int or trial['trial'] < 0:
        raise ValueError('trial must be a non-negative integer index')
    if trial.get('outcome') not in CATEGORIES:
        raise ValueError(f"trial {trial['trial']}: outcome must be one of {CATEGORIES}")
    stages = trial.get('stages') or {}
    if set(stages) - set(STAGES):
        raise ValueError(f"trial {trial['trial']}: unknown stages {sorted(set(stages) - set(STAGES))}")
    stages = {stage: _time(stages.get(stage), f"trial {trial['trial']} {stage}") for stage in STAGES}
    times = [stages[s] for s in STAGES if reached(stages)[s]]
    if times != sorted(times):
        raise ValueError(f"trial {trial['trial']}: stage times must not decrease (pickup <= handoff <= bucket)")
    trial['stages'] = stages
    if type(trial.get('eligible')) is not bool:
        raise ValueError(f"trial {trial['trial']}: eligible must be true or false")
    outcome = trial['outcome']
    if outcome == 'success' and not (all(reached(stages).values()) and trial['eligible']):
        raise ValueError(f"trial {trial['trial']}: success needs all three stages and an eligible trial")
    if outcome == 'forbidden_penetration' and trial['eligible']:
        raise ValueError(f"trial {trial['trial']}: forbidden penetration makes a trial ineligible")
    if outcome != 'forbidden_penetration' and not trial['eligible'] and outcome not in EXCLUDED:
        raise ValueError(f"trial {trial['trial']}: only forbidden penetration (or an invalid start) is ineligible")
    trial['duration_s'] = _time(trial.get('duration_s'), f"trial {trial['trial']} duration_s")
    start = trial.get('cube_start')
    if start is not None:
        if not isinstance(start, dict) or not all(isinstance(start.get(k), (int, float)) and math.isfinite(start[k])
                                                  for k in ('x', 'y')):
            raise ValueError(f"trial {trial['trial']}: cube_start needs finite x and y (m)")
        trial['cube_start'] = {'x': float(start['x']), 'y': float(start['y']),
                               'yaw': float(start.get('yaw') or 0.), **{k: v for k, v in start.items()
                                                                        if k not in ('x', 'y', 'yaw')}}
    trial['reached'] = reached(stages)
    trial['counted'] = outcome not in EXCLUDED
    return trial


def outcome_from_events(events, *, timed_out=False, aborted=None):
    """Stages, outcome and eligibility from MinervaTaskScene events (scene_time since reset).

    The scene latches its events; this only reads them. `aborted` (a reason) and `timed_out` say
    why the runner stopped when no scene event ended the trial.
    """
    stages = {stage: None for stage in STAGES}
    outcome, detail, eligible = None, None, True
    for event in events:
        kind, at = event.get('kind'), event.get('scene_time')
        if kind == 'pickup' and stages['pickup'] is None:
            stages['pickup'] = at
        elif kind == 'handoff' and stages['handoff'] is None:
            stages['handoff'] = at
        elif kind == 'bucket_settled' and stages['bucket'] is None:
            stages['bucket'] = at
            if outcome is None:
                outcome = 'success' if event.get('task_success') else 'incomplete_sequence'
        elif kind == 'dropped_outside_bucket' and outcome is None:
            outcome, detail = 'dropped_cube', f'dropped at {at:.3f} s'
        elif kind in FORBIDDEN_EVENTS and eligible:
            # Ineligibility outranks every other outcome, a latched success included (the scene revokes it).
            eligible, outcome = False, 'forbidden_penetration'
            detail = f"{' / '.join(event.get('geoms') or ())} {event.get('depth_m', 0) * 1000:.1f} mm at {at:.3f} s"
    if outcome is None:
        if aborted:
            outcome, detail = 'source_gap_or_abort', str(aborted)
        elif timed_out:
            outcome = 'timeout'
    return stages, outcome, eligible, detail


def load(path):
    """Trial records from a .jsonl (one JSON object per line) or .csv (CSV_COLUMNS) file.

    CSV rows are the hand-logging format: `source` and `policy` come from the caller."""
    path = Path(path)
    text = path.read_text(encoding='utf-8-sig')
    if path.suffix.lower() == '.csv':
        return from_csv(text)
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def _cell(row, key):
    value = (row.get(key) or '').strip()
    return value or None


def from_csv(text):
    reader = csv.DictReader(io.StringIO(text))
    missing = {'trial', 'outcome'} - set(reader.fieldnames or ())
    if missing:
        raise ValueError(f'CSV needs columns {sorted(missing)}; the full set is {", ".join(CSV_COLUMNS)}')
    records = []
    for number, row in enumerate(reader, start=2):
        try:
            number_or_none = lambda key: None if _cell(row, key) is None else float(_cell(row, key))
            eligible = (_cell(row, 'eligible') or '').lower()
            if eligible not in ('', 'true', 'false', 'yes', 'no', '1', '0'):
                raise ValueError('eligible must be true/false')
            record = {'trial': int(_cell(row, 'trial')), 'outcome': _cell(row, 'outcome'),
                      'stages': {stage: number_or_none(f'{stage}_s') for stage in STAGES},
                      'duration_s': number_or_none('duration_s'),
                      # Unlogged eligibility follows the outcome: only forbidden penetration is ineligible.
                      'eligible': (eligible in ('true', 'yes', '1')) if eligible else
                                  _cell(row, 'outcome') != 'forbidden_penetration',
                      'detail': _cell(row, 'detail'), 'notes': _cell(row, 'notes')}
            if _cell(row, 'cube_x') is not None or _cell(row, 'cube_y') is not None:
                record['cube_start'] = {'x': number_or_none('cube_x'), 'y': number_or_none('cube_y'),
                                        'yaw': number_or_none('cube_yaw') or 0.}
        except (TypeError, ValueError) as exc:
            raise ValueError(f'CSV line {number}: {exc}') from exc
        records.append(record)
    return records


def write_jsonl(path, trials):
    with Path(path).open('w', encoding='utf-8') as handle:
        for trial in trials:
            handle.write(json.dumps(trial, allow_nan=False) + '\n')
