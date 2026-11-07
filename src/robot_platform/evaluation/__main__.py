"""python -m robot_platform.evaluation {run,report,grid,compare,power}. See docs/development/MINERVA_EVAL.md."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

from .generate import APPEARANCE
from .grid import GridSpec, cube_poses, parse_counts
from . import stats

ROOT = Path(__file__).resolve().parents[3]


def _grid_arguments(parser, default='3x3'):
    parser.add_argument('--grid', default=default, help=f'cells NXxNY over the default reachable area (default {default})')
    parser.add_argument('--seed', type=int, default=0, help='grid seed (default 0)')
    parser.add_argument('--jitter', type=float, default=.5, help='offset within a cell, fraction of half a cell')
    parser.add_argument('--yaw', type=float, default=0., help='cube yaw drawn from [-yaw, yaw] rad (default 0)')
    parser.add_argument('--no-nominal', action='store_true', help="leave out trial 0 at the scene's own cube start")
    parser.add_argument('--workcell', help="a workcell file (simulation.workcell), or 'synthetic' (default): the "
                                           "table, bucket and cube, and the grid's cube region")


def _workcell(args):
    from robot_platform.simulation.workcell import resolve
    return resolve(getattr(args, 'workcell', None))


def _grid(args):
    nx, ny = parse_counts(args.grid)
    options = dict(nx=nx, ny=ny, seed=args.seed, jitter=args.jitter, yaw=args.yaw, nominal=not args.no_nominal)
    if getattr(args, 'workcell', None) in (None, 'synthetic'):
        return GridSpec(**options)
    return GridSpec.for_workcell(_workcell(args), **options)


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
    width, height = parse_counts(args.camera_size)
    config = RunConfig(plant=args.plant, timeout_s=args.timeout, grid=_grid(args),
                       trials=None if args.trials is None else tuple(int(i) for i in args.trials.split(',')),
                       camera_size=(width, height), visual_mode=args.visual_mode,
                       plant_seed=args.plant_seed, scene_pose=args.scene_pose, workcell=args.workcell,
                       grippers=args.grippers, latency_s=args.latency, actuator_spread=args.actuator_spread,
                       appearance=APPEARANCE if args.vary_look else None, disturbance_seed=args.disturbance_seed)
    stack = derive_stack(profile=args.profile, directions=args.directions)
    out = Path(args.out) if args.out else ROOT / 'build/eval' / (
        f"{getattr(policy, 'name', 'policy')}-{args.plant}-" + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
    report = evaluate(policy, config, out, stack=stack, progress=lambda line: print(line, flush=True))
    print(_summary(report))
    print(f'report: {out / "report.md"}')
    return 0


def generate(args):
    from .generate import generate as run_generation
    from .policies import load_policy
    from .plants import derive_stack
    from .runner import RunConfig
    width, height = parse_counts(args.camera_size)
    grid = _grid(args)
    from dataclasses import replace
    grid = replace(grid, nominal=False)  # The evaluation's own trial 0 is never generated.
    config = RunConfig(plant='armplant', timeout_s=args.timeout, grid=grid, plant_seed=args.plant_seed,
                       camera_size=(width, height), visual_mode=args.visual_mode, workcell=args.workcell,
                       grippers='driven')
    out = Path(args.out) if args.out else ROOT / 'build/generated' / time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    appearance = None if args.same_look else {'light': args.light, 'color': args.color,
                                                'camera_shift': args.camera_shift, 'camera_turn_deg': args.camera_turn}
    layouts = None
    if args.randomize:
        from robot_platform.simulation.layouts import LayoutSampler
        if args.workcell in (None, 'synthetic'):
            raise SystemExit('--randomize draws layouts around a workcell file: give --workcell <file>')
        layouts = LayoutSampler(args.workcell, args.randomize)
    made, attempts = run_generation(load_policy(args.policy), config, out, episodes=args.episodes,
                                    stack=derive_stack(profile=args.profile, directions=args.directions), hz=args.hz,
                                    appearance=appearance, progress=lambda line: print(line, flush=True),
                                    layouts=layouts)
    print(f'{len(made)} episodes from {attempts} attempts: {out / "episodes"} (manifest {out / "manifest.jsonl"})')
    return 0 if made else 1


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
    r.add_argument('--policy', required=True,
                   help="fixture_replay, hold_still, penetrating, handoff_demonstrator, 'checkpoint:<path>' or 'module:factory'")
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
    r.add_argument('--latency', type=float, default=0., help='seconds before a chunk takes effect (default 0)')
    r.add_argument('--actuator-spread', type=float, default=0.,
                   help="each joint's inertia, friction and damping x U[1-s, 1+s] per trial, unknown to the receiver")
    r.add_argument('--vary-look', action='store_true', help="vary each trial's lights, colours and camera places")
    r.add_argument('--disturbance-seed', type=int, default=0, help='seed for the per-trial disturbances (default 0)')
    r.add_argument('--grippers', choices=('kinematic', 'driven'), default='kinematic',
                   help='kinematic: posed like the arms (default); driven: the grippers move under their drives in '
                        'the scene and stall on the cube, and the receiver reads that back (armplant only)')
    r.add_argument('--camera-size', default='160x120',
                   help="the policy's rendered camera size WxH (default 160x120; generated episodes record 320x240)")
    r.add_argument('--visual-mode', choices=('cad', 'collision_proxy'), default='collision_proxy',
                   help="what the policy's cameras see (default collision_proxy; generated episodes record cad)")
    _grid_arguments(r)
    r.set_defaults(handler=run)
    n = sub.add_parser('generate', help='record synthetic demonstrations: the episodes a demonstrator completes')
    n.add_argument('--policy', default='handoff_demonstrator', help='the demonstrator (default handoff_demonstrator)')
    n.add_argument('--episodes', type=int, required=True, help='successful episodes to record')
    n.add_argument('--out', help='output directory (default build/generated/<time>)')
    n.add_argument('--hz', type=float, default=15., help='record rate in scene time (default 15, near real capture)')
    n.add_argument('--camera-size', default='320x240', help='rendered camera size WxH (default 320x240, as recorded)')
    n.add_argument('--visual-mode', choices=('cad', 'collision_proxy'), default='cad',
                   help='camera appearance (default cad: the full CAD visuals)')
    n.add_argument('--timeout', type=float, default=70., help='seconds per attempt (default 70)')
    n.add_argument('--light', type=float, default=.3, help='light strength varies by up to this fraction (default .3)')
    n.add_argument('--color', type=float, default=.1, help='workcell and cube colours vary by this fraction (default .1)')
    n.add_argument('--camera-shift', type=float, default=.01, help='cameras move up to this far per axis, m (default .01)')
    n.add_argument('--camera-turn', type=float, default=2., help='cameras turn up to this far, degrees (default 2)')
    n.add_argument('--same-look', action='store_true', help="keep the scene's own lights, colours and camera places")
    n.add_argument('--randomize', metavar='FILE',
                   help='a layout randomization file (config/devices/simulation/<asset>-randomization.json): each '
                        "attempt draws its own cube, bucket, table placement and head camera pose around --workcell's, "
                        "redrawing a layout the demonstrator cannot plan; without it, the workcell's fixed layout")
    n.add_argument('--profile', default='minerva-teleop-direct.json', help='collision-disabled teleop profile')
    n.add_argument('--directions', help="a Helm direction session's session.json (default: documented directions)")
    n.add_argument('--plant-seed', type=int, default=1, help='simulated feedback noise seed (default 1)')
    _grid_arguments(n)
    n.set_defaults(handler=generate, seed=1000)
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
