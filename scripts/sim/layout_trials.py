"""Run the handoff demonstrator on drawn training layouts through the receiver, without cameras, and count what
happens: how many draws the planner rejected and why, and how many planned layouts the task completed in.

    .venv-dev/Scripts/python.exe scripts/sim/layout_trials.py --out <dir> --attempts 24 --workers 4 [--seed 2000]
        [--workcell <file>] [--randomize <file>] [--timeout 90]

Attempt k draws its layout as `generate --randomize` does (evaluation.generate.draw_layout, seed [SEED, k]): a layout
the demonstrator cannot plan is drawn again, up to the file's plan_draws. Each planned layout then runs as an
evaluation trial (the native receiver, the armplant joints, driven grippers, the contact scene) with nothing
rendered. Worker w takes attempts w, w + W, ...; each has its own receiver. Writes attempts.jsonl (one line per
attempt: its layout, its rejected draws with their reasons, the trial's outcome and events) and summary.json.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
RANDOMIZATION = ROOT / 'config/devices/simulation/minerva-production-01-randomization.json'


def _worker(job):
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.generate import draw_layout
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.evaluation.runner import RunConfig, TrialRunner
    from robot_platform.simulation.layouts import LayoutSampler
    worker, attempts, args = job
    out = Path(args['out'])
    sampler = LayoutSampler(args['workcell'], args['randomize'])
    runner = TrialRunner(RunConfig(grippers='driven', timeout_s=args['timeout'], workcell=args['workcell'],
                                   plant_seed=worker + 1), stack=derive_stack(), directory=out / f'receiver-w{worker}')
    demo = HandoffDemonstrator()
    lines = []
    try:
        for attempt in attempts:
            started = time.perf_counter()
            layout, rejections = draw_layout(runner, demo, sampler, [args['seed'], attempt])
            entry = {'attempt': attempt, 'worker': worker, 'layout_seed': [args['seed'], attempt],
                     'rejected_draws': len(rejections), 'rejections': rejections}
            if layout is None:
                entry.update(outcome='layout_rejected', planning_s=time.perf_counter() - started)
            else:
                planned = time.perf_counter()
                runner.use_workcell(layout.workcell)
                record = runner.run_trial(demo, dict(layout.cube_start, cell='layout', index=attempt), attempt)
                entry.update(layout=layout.describe(), drop=demo.drop['name'] if demo.drop else None,
                             outcome=record['outcome'], eligible=record['eligible'], detail=record.get('detail'),
                             stages=record['stages'], duration_s=record['duration_s'],
                             clipped=record['commands_clipped_cycles'], planning_s=planned - started,
                             wall_s=time.perf_counter() - planned,
                             max_tracking_rad=max(record['max_abs_pose_minus_command_rad'].values()))
            lines.append(json.dumps(entry))
            with (out / f'attempts-w{worker}.jsonl').open('a', encoding='utf-8') as handle:
                handle.write(lines[-1] + '\n')
    finally:
        runner.close()
        demo.close()
    return lines


def _wilson(successes, n, z=1.959963984540054):
    if not n:
        return [0., 1.]
    p = successes / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [max(0., centre - half), min(1., centre + half)]


def summarize(entries):
    planned = [e for e in entries if e['outcome'] != 'layout_rejected']
    draws = sum(e['rejected_draws'] for e in entries) + len(planned)
    successes = sum(1 for e in planned if e['outcome'] == 'success')
    reasons = {}
    for entry in entries:
        for rejection in entry['rejections']:
            reason = rejection['reason']
            kind = ('layout: no bucket place' if reason.startswith('layout:') else
                    'no plan reaches the cube' if 'no plan reaches' in reason else
                    'no plan clears' if 'no plan clears' in reason else reason[:60])
            reasons[kind] = reasons.get(kind, 0) + 1
    outcomes = {}
    for entry in planned:
        outcomes[entry['outcome']] = outcomes.get(entry['outcome'], 0) + 1
    return {'attempts': len(entries), 'layouts_planned': len(planned), 'draws': draws,
            'rejected_draws': draws - len(planned), 'rejection_rate': (draws - len(planned)) / draws if draws else None,
            'rejections_by_reason': reasons, 'attempts_without_a_layout': len(entries) - len(planned),
            'trial_outcomes': outcomes, 'successes': successes,
            'success_rate_of_planned': successes / len(planned) if planned else None,
            'success_wilson95': _wilson(successes, len(planned)),
            'drops_used': {name: sum(1 for e in planned if e.get('drop') == name)
                           for name in sorted({e.get('drop') for e in planned if e.get('drop')})}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--attempts', type=int, default=24)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=2000)
    parser.add_argument('--workcell', default=str(WORKCELL))
    parser.add_argument('--randomize', default=str(RANDOMIZATION))
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    settings = dict(vars(args))
    started = time.time()
    jobs = [(w, list(range(1 + w, args.attempts + 1, args.workers)), settings) for w in range(args.workers)]
    entries = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for lines in pool.map(_worker, jobs):
            entries += [json.loads(line) for line in lines]
    entries.sort(key=lambda entry: entry['attempt'])
    (out / 'attempts.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in entries), encoding='utf-8')
    summary = dict(summarize(entries), settings=settings, wall_s=round(time.time() - started, 1))
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in summary.items() if k != 'settings'}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
