"""python -m robot_platform.evaluation {run,report,grid,compare,power}. See docs/development/MINERVA_EVAL.md."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

from .grid import GridSpec, cube_poses, parse_counts
from . import stats

ROOT = Path(__file__).resolve().parents[3]


def _grid_arguments(parser, default='3x3'):
    parser.add_argument('--grid', default=default, help=f'cells NXxNY over the default reachable area (default {default})')
    parser.add_argument('--seed', type=int, default=0, help='grid seed (default 0)')
    parser.add_argument('--jitter', type=float, default=.5, help='offset within a cell, fraction of half a cell')
    parser.add_argument('--yaw', type=float, default=0., help='cube yaw drawn from [-yaw, yaw] rad (default 0)')
    parser.add_argument('--no-nominal', action='store_true', help="leave out trial 0 at the scene's own cube start")


def _grid(args):
    nx, ny = parse_counts(args.grid)
    return GridSpec(nx=nx, ny=ny, seed=args.seed, jitter=args.jitter, yaw=args.yaw, nominal=not args.no_nominal)


def _fraction(text):
    successes, sep, trials = text.partition('/')
    if not sep:
        raise argparse.ArgumentTypeError('give a success count as SUCCESSES/TRIALS, e.g. 7/20')
    return int(successes), int(trials)


def _summary(report):
    success, stages, ttc = report['success'], report['stage_completion'], report['time_to_complete_s']
    low, high = success['wilson95']
    lines = [f"{report['policy']} ({report['source']}): {success['count']}/{report['trials']['counted']} succeeded "
             f"[Wilson 95% {low:.2f}-{high:.2f}], stage score {stages['score']:.3f}",
             'outcomes: ' + ', '.join(f'{k} {v}' for k, v in report['outcomes'].items() if v)]
    if ttc['n']:
        lines.append(f"time to complete: median {ttc['median']:.2f} s, p90 {ttc['p90']:.2f} s")
    if report['trials']['excluded_invalid_start']:
        lines.append(f"excluded (invalid start): {report['trials']['excluded_invalid_start']}")
    return '\n'.join(lines)


def run(args):
    from .policies import load_policy
    from .plants import derive_stack
    from .runner import RunConfig, evaluate
    policy = load_policy(args.policy)
    config = RunConfig(plant=args.plant, timeout_s=args.timeout, grid=_grid(args),
                       trials=None if args.trials is None else tuple(int(i) for i in args.trials.split(',')),
                       plant_seed=args.plant_seed, scene_pose=args.scene_pose)
    stack = derive_stack(profile=args.profile, directions=args.directions)
    out = Path(args.out) if args.out else ROOT / 'build/eval' / (
        f"{getattr(policy, 'name', 'policy')}-{args.plant}-" + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
    report = evaluate(policy, config, out, stack=stack, progress=lambda line: print(line, flush=True))
    print(_summary(report))
    print(f'report: {out / "report.md"}')
    return 0


def report(args):
    from . import report as reports, trials as trial_records
    records = trial_records.load(args.trials)
    for record in records:
        record.setdefault('source', args.source)
        record.setdefault('policy', args.policy)
    meta = {'policy': args.policy, 'provenance': {'trials_file': str(Path(args.trials).resolve())}}
    if args.meta:
        meta['provenance'].update(json.loads(Path(args.meta).read_text(encoding='utf-8')))
    if args.grid:
        meta['protocol'] = {'grid': _grid(args).describe(), 'cube_poses': cube_poses(_grid(args))}
    built = reports.build_report(records, meta)
    out = Path(args.out) if args.out else Path(args.trials).with_suffix('')
    reports.write(built, out)
    print(_summary(built))
    print(f'report: {out / "report.md"}')
    return 0


def grid(args):
    poses = cube_poses(_grid(args))
    writer = csv.writer(sys.stdout, lineterminator='\n')
    if args.template:
        # The hand-logging sheet: cube positions filled in, results left for the operator.
        from .trials import CSV_COLUMNS
        writer.writerow(CSV_COLUMNS)
        for pose in poses:
            writer.writerow([pose['index'], f"{pose['x']:.4f}", f"{pose['y']:.4f}", f"{pose['yaw']:.4f}"]
                            + [''] * (len(CSV_COLUMNS) - 4))
    else:
        writer.writerow(('trial', 'cell', 'cube_x_m', 'cube_y_m', 'cube_yaw_rad'))
        for pose in poses:
            cell = pose['cell'] if isinstance(pose['cell'], str) else 'x'.join(str(c) for c in pose['cell'])
            writer.writerow([pose['index'], cell, f"{pose['x']:.4f}", f"{pose['y']:.4f}", f"{pose['yaw']:.4f}"])
    return 0


def compare(args):
    (s1, n1), (s2, n2) = args.a, args.b
    result = stats.two_proportion_test(s1, n1, s2, n2, alpha=args.alpha)
    print(json.dumps(result, indent=1))
    print(f"{'DIFFER' if result['differ'] else 'no detectable difference'} at alpha {args.alpha} "
          f"({result['method']}, p = {result['p_value']:.4g})")
    if not result['differ'] and result['a']['rate'] != result['b']['rate']:
        need = stats.trials_per_arm(result['a']['rate'], result['b']['rate'], alpha=args.alpha)
        print(f'to detect {result["a"]["rate"]:.2f} vs {result["b"]["rate"]:.2f} at 80% power: {need} trials per arm')
    return 0


def power(args):
    need = stats.trials_per_arm(args.p1, args.p2, alpha=args.alpha, power=args.power)
    print(f'{need} trials per arm detect {args.p1} vs {args.p2} (two-sided alpha {args.alpha}, power {args.power})')
    return 0


def parser():
    p = argparse.ArgumentParser(prog='python -m robot_platform.evaluation',
                                description='Closed-loop Minerva task evaluation (simulation and hand-logged robot trials)')
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('run', help='run a policy against the simulated task scene')
    r.add_argument('--policy', required=True, help="fixture_replay, hold_still, penetrating, or 'module:factory'")
    r.add_argument('--plant', choices=('armplant', 'ideal'), default='armplant',
                   help='armplant: the helm --sim receiver and simulated joints (default); ideal: exact tracking')
    r.add_argument('--timeout', type=float, default=30., help='seconds per trial (default 30)')
    r.add_argument('--trials', help='comma-separated trial indices (default: all grid poses)')
    r.add_argument('--out', help='output directory (default build/eval/<policy>-<plant>-<time>)')
    r.add_argument('--profile', default='minerva-teleop-direct.json', help='collision-disabled teleop profile')
    r.add_argument('--directions', help="a Helm direction session's session.json (default: documented directions)")
    r.add_argument('--plant-seed', type=int, default=1, help='simulated feedback noise seed (default 1)')
    r.add_argument('--scene-pose', choices=('plant', 'measured'), default='plant',
                   help='what moves the object scene: the simulated joints (default) or receiver feedback')
    _grid_arguments(r)
    r.set_defaults(handler=run)
    m = sub.add_parser('report', help='report trials from a .jsonl or hand-logged .csv file')
    m.add_argument('trials')
    m.add_argument('--source', choices=('real', 'simulation'), default='real')
    m.add_argument('--policy', required=True, help='policy name for the report')
    m.add_argument('--meta', help='JSON with provenance (robot, operator, checkpoint, ...)')
    m.add_argument('--out', help='output directory (default: beside the trials file)')
    _grid_arguments(m, default=None)  # Recorded as the protocol only when given.
    m.set_defaults(handler=report)
    g = sub.add_parser('grid', help='print the cube start poses (place the cube there on the robot)')
    g.add_argument('--template', action='store_true', help='print a hand-logging CSV with the poses filled in')
    _grid_arguments(g)
    g.set_defaults(handler=grid)
    c = sub.add_parser('compare', help='do two success counts differ? (two-proportion test)')
    c.add_argument('a', type=_fraction, help='SUCCESSES/TRIALS of policy A')
    c.add_argument('b', type=_fraction, help='SUCCESSES/TRIALS of policy B')
    c.add_argument('--alpha', type=float, default=.05)
    c.set_defaults(handler=compare)
    w = sub.add_parser('power', help='trials per arm to detect success rates p1 vs p2')
    w.add_argument('p1', type=float)
    w.add_argument('p2', type=float)
    w.add_argument('--alpha', type=float, default=.05)
    w.add_argument('--power', type=float, default=.8)
    w.set_defaults(handler=power)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    return args.handler(args)


if __name__ == '__main__':
    raise SystemExit(main())
