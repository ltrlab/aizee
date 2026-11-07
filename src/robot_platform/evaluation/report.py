"""The evaluation report, identical for simulated and hand-logged real-robot trials.

`build_report(trials, meta)` validates every trial (evaluation.trials) and computes the metrics;
`markdown(report)` renders it. Simulation and real reports differ only in `source` and in the
provenance the caller supplies, so two reports compare field by field.
"""

from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics

from . import stats, trials as trial_records

SCHEMA = 'minerva_eval_report/1'


def build_report(records, meta=None):
    meta = dict(meta or {})
    checked = [trial_records.validate(r) for r in records]
    if not checked:
        raise ValueError('a report needs at least one trial')
    indices = [t['trial'] for t in checked]
    if len(set(indices)) != len(indices):
        raise ValueError('trial indices must be unique')
    sources = sorted({t['source'] for t in checked})
    counted = [t for t in checked if t['counted']]
    excluded = [t for t in checked if not t['counted']]
    n = len(counted)
    successes = [t for t in counted if t['outcome'] == 'success']
    histogram = Counter(t['outcome'] for t in counted)
    times = [t['stages']['bucket'] for t in successes]
    report = {
        'schema': SCHEMA,
        'generated_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'source': sources[0] if len(sources) == 1 else 'mixed',
        'policy': meta.pop('policy', None),
        'trials': {
            'counted': n,
            'excluded_invalid_start': len(excluded),
            'logged': len(checked),
            'eligible': sum(t['eligible'] for t in counted),
        },
        'success': {
            'count': len(successes),
            'rate': len(successes) / n if n else None,
            'wilson95': list(stats.wilson_interval(len(successes), n)),
        },
        'stage_completion': {
            'score': statistics.fmean(sum(t['reached'].values()) / len(trial_records.STAGES) for t in counted)
            if n
            else None,
            'stages': {
                stage: {
                    'count': sum(t['reached'][stage] for t in counted),
                    'rate': sum(t['reached'][stage] for t in counted) / n if n else None,
                    'wilson95': list(stats.wilson_interval(sum(t['reached'][stage] for t in counted), n)),
                }
                for stage in trial_records.STAGES
            },
        },
        'time_to_complete_s': {
            'n': len(times),
            'median': stats.percentile(times, 50),
            'p90': stats.percentile(times, 90),
            'min': min(times) if times else None,
            'max': max(times) if times else None,
            'definition': 'trial start to the bucket_settled event, successful trials only; '
            'percentiles by linear interpolation (Hyndman-Fan type 7)',
        },
        'outcomes': {category: histogram.get(category, 0) for category in ('success',) + trial_records.FAILURES},
        'excluded': [{'trial': t['trial'], 'outcome': t['outcome'], 'detail': t.get('detail')} for t in excluded],
        'protocol': meta.pop('protocol', {}),
        'provenance': dict(meta.pop('provenance', {}), sim2real_validated=False, training_promoted=False),
        'trials_detail': checked,
    }
    report.update(meta)
    return report


def _fmt(value, digits=2, unit=''):
    return '—' if value is None else f'{value:.{digits}f}{unit}'


def _pct(value):
    return '—' if value is None else f'{100 * value:.1f}%'


def markdown(report):
    success, stages, ttc = report['success'], report['stage_completion'], report['time_to_complete_s']
    trials, protocol, provenance = report['trials'], report.get('protocol') or {}, report.get('provenance') or {}
    lines = [
        f'# Minerva evaluation: {report.get("policy") or "unnamed policy"}',
        '',
        f'- Source: **{report["source"]}**' + (f' (plant: `{provenance["plant"]}`)' if provenance.get('plant') else ''),
        f'- Generated: {report["generated_at"]}',
        f'- Trials: {trials["counted"]} counted, {trials["excluded_invalid_start"]} excluded (invalid start), '
        f'{trials["eligible"]} eligible',
        '- `sim2real_validated`: false · `training_promoted`: false',
        '',
        '## Metrics',
        '',
        '| Metric | Value |',
        '|---|---|',
        f'| Full-task success | {success["count"]}/{trials["counted"]} = {_pct(success["rate"])} '
        f'(Wilson 95% {_pct(success["wilson95"][0])}–{_pct(success["wilson95"][1])}) |',
        f'| Stage-completion score | {_fmt(stages["score"], 3)} (mean fraction of pickup, handoff, bucket) |',
    ]
    for stage, value in stages['stages'].items():
        lines.append(f'| Reached {stage} | {value["count"]}/{trials["counted"]} = {_pct(value["rate"])} |')
    lines += [
        f'| Time to complete (successes, n={ttc["n"]}) | median {_fmt(ttc["median"], 2, " s")}, '
        f'p90 {_fmt(ttc["p90"], 2, " s")} |',
        '',
        '## Outcomes',
        '',
        '| Outcome | Trials |',
        '|---|---|',
    ]
    lines += [f'| {category} | {count} |' for category, count in report['outcomes'].items()]
    if report['excluded']:
        lines += ['', 'Excluded (invalid start, not counted):', '']
        lines += [f'- trial {e["trial"]}: {e["detail"] or e["outcome"]}' for e in report['excluded']]
    lines += [
        '',
        '## Trials',
        '',
        '| # | Cube start (x, y, yaw) | Pickup | Handoff | Bucket | Outcome | Detail |',
        '|---|---|---|---|---|---|---|',
    ]
    for t in report['trials_detail']:
        start = t.get('cube_start')
        where = (
            (
                f'{start["x"]:.3f}, {start["y"]:.3f}, {start["yaw"]:.2f}'
                + (f' ({start["cell"]})' if start.get('cell') == 'nominal' else '')
            )
            if start
            else '—'
        )
        lines.append(
            f'| {t["trial"]} | {where} | '
            + ' | '.join(_fmt(t['stages'][s], 2, ' s') for s in ('pickup', 'handoff', 'bucket'))
            + f' | {t["outcome"]} | {(t.get("detail") or "").replace("|", "/")} |'
        )
    if protocol:
        lines += ['', '## Protocol', '', '```json', json.dumps(protocol, indent=1), '```']
    if provenance:
        lines += ['', '## Provenance', '', '```json', json.dumps(provenance, indent=1, default=str), '```']
    return '\n'.join(lines) + '\n'


def write(report, directory, stem='report'):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f'{stem}.json').write_text(
        json.dumps(report, indent=1, allow_nan=False, default=str), encoding='utf-8'
    )
    (directory / f'{stem}.md').write_text(markdown(report), encoding='utf-8')
    return directory / f'{stem}.json', directory / f'{stem}.md'
