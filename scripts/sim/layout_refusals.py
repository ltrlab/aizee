"""Where drawn training layouts are refused, and what that does to a generated set.

    python scripts/sim/layout_refusals.py map OUT.jsonl [--draws 240] [--workers 6] [--seed 5000]
    python scripts/sim/layout_refusals.py report OUT.jsonl
    python scripts/sim/layout_refusals.py view OUT.jsonl

`map` draws layouts from the production workcell and randomization files the way `generate --randomize` does (draw k
from seed [S, k]) and puts each through the head-camera check and the demonstrator's planning, without a trial: one
JSON line per draw with its values, verdict and reason. `report` counts the verdicts and what the refused plans met,
and gives the acceptance by quartile of each drawn value. `view` says how much of the bucket and the block the head
camera sees, for the workcell file's own layout and every accepted draw.

An attempt keeps its first accepted draw, so a generated set's layouts are the drawn layouts weighted by their chance of
acceptance: a quartile accepted at half the overall rate is half as dense in the episodes as in the draws.
"""
import argparse
import collections
from concurrent.futures import ProcessPoolExecutor
from itertools import product
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
RANDOMIZATION = ROOT / 'config/devices/simulation/minerva-production-01-randomization.json'
PLANNING = ('no plan clears', 'no plan reaches')
_state = {}


def _init():
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.generate import HeadCameraClearance
    from robot_platform.evaluation.plants import _Stack, derive_stack
    from robot_platform.simulation.layouts import LayoutSampler
    stack = derive_stack()
    _state.update(stack=stack, limits=_Stack(stack), demo=HandoffDemonstrator(), camera=HeadCameraClearance(stack),
                  sampler=LayoutSampler(WORKCELL, RANDOMIZATION))


def _one(key):
    from robot_platform.evaluation.demonstrator import PlanningError
    try:
        layout = _state['sampler'].draw(np.random.default_rng(key), seed=key)
    except ValueError as exc:
        return {'seed': key, 'verdict': 'no bucket place', 'reason': str(exc)}
    entry = {'seed': key, 'values': layout.describe()}
    problem = _state['camera'](layout.workcell)
    if problem:
        return dict(entry, verdict='head camera', reason=problem)
    stack, limits = _state['stack'], _state['limits']
    # The context TrialRunner.plan gives the demonstrator, without opening a plant.
    context = SimpleNamespace(joint_names=stack['names'], joint_map=stack['derived']['joint_map'], lower=limits.lower,
                              upper=limits.upper, home=limits.home, workcell=layout.workcell,
                              period_s=limits.period_s, lead=limits.lead, cube_start=dict(layout.cube_start))
    try:
        _state['demo'].reset(context)
    except PlanningError as exc:
        text = str(exc)
        return dict(entry, verdict=PLANNING[0] if PLANNING[0] in text else PLANNING[1], reason=text)
    return dict(entry, verdict='planned', drop=_state['demo'].drop['name'], grasp=_state['demo'].grasp)


def run_map(args):
    keys = [[args.seed, k] for k in range(args.draws)]
    with ProcessPoolExecutor(max_workers=args.workers, initializer=_init) as pool, \
            open(args.out, 'w', encoding='utf-8') as out:
        for result in pool.map(_one, keys, chunksize=1):
            out.write(json.dumps(result) + '\n')
            out.flush()


def wilson(k, n, z=1.959963984540054):
    if not n:
        return 0., 1.
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return centre - half, centre + half


ENTRY = re.compile(r'(drop [^:]+|pick and handoff): ([^:]+): (.*)')


def met(detail):
    """What a failed move met: 'reach', or the kind of geom it came within the clearance of."""
    if 'cannot reach' in detail:
        return 'reach'
    found = re.search(r'comes (-?[\d.]+) mm from (\S+)', detail)
    if not found:
        return 'other'
    other = found.group(2)
    if other.startswith('task/bucket'):
        return 'bucket'
    if other.startswith('task/table'):
        return 'table'
    if other.startswith('task/'):
        return other.split('/')[1]
    return 'other arm'


VALUES = {
    'block height mm': lambda v: v['cube']['size_m'][2] * 1000,
    'block shorter side mm': lambda v: min(v['cube']['size_m'][:2]) * 1000,
    'block longer side mm': lambda v: max(v['cube']['size_m'][:2]) * 1000,
    'block x m': lambda v: v['cube']['drawn_in_group_frame_m'][0],
    'block y m': lambda v: v['cube']['drawn_in_group_frame_m'][1],
    'bucket height mm': lambda v: v['bucket']['height_m'] * 1000,
    'bucket width mm': lambda v: v['bucket']['inner_width_m'] * 1000,
    'bucket length mm': lambda v: v['bucket']['inner_length_m'] * 1000,
    'bucket yaw deg': lambda v: v['bucket']['drawn_in_group_frame']['yaw_deg'],
    'bucket x m': lambda v: v['bucket']['drawn_in_group_frame']['center_m'][0],
    'bucket y m': lambda v: v['bucket']['drawn_in_group_frame']['center_m'][1],
    'table x cm': lambda v: v['table_offset']['x_m'] * 100,
    'table toward robot cm': lambda v: v['table_offset']['y_m'] * 100,
    'table yaw deg': lambda v: v['table_offset']['yaw_deg'],
    # The head camera is given in the rectangle frame, whose y points away from the robot.
    'camera toward robot cm': lambda v: (v['table_offset']['y_m'] - v['head_camera']['shift_m'][1]) * 100,
}


def report(path):
    rows = [json.loads(line) for line in open(path, encoding='utf-8') if line.strip()]
    n_all = len(rows)
    verdicts = collections.Counter(r['verdict'] for r in rows)
    print(f'{n_all} draws: ' + ', '.join(f'{k} {v} ({v / n_all:.0%})' for k, v in verdicts.most_common()))
    planned = verdicts['planned']
    low, high = wilson(planned, n_all)
    print(f'accepted {planned}/{n_all} = {planned / n_all:.0%} (Wilson 95% {low:.2f}-{high:.2f})')
    tried = [r for r in rows if r.get('values') and r['verdict'] != 'head camera']
    low, high = wilson(planned, len(tried))
    print(f'planned of those past the head-camera check: {planned}/{len(tried)} = {planned / len(tried):.0%} '
          f'(Wilson 95% {low:.2f}-{high:.2f})')

    per_refusal, moves = collections.Counter(), collections.Counter()
    for r in rows:
        if r['verdict'] not in PLANNING:
            continue
        text = r['reason']
        kinds = set()
        for part in text[text.find('(') + 1:text.rfind(')')].split('; '):
            entry = ENTRY.match(part)
            if entry:
                kind = met(entry.group(3))
                kinds.add(f'reach ({entry.group(2)})' if kind == 'reach' else kind)
                moves[f'{entry.group(2)}: {kind}'] += 1
        per_refusal[' + '.join(sorted(kinds)) or 'other'] += 1
    print('\nplanning refusals, by what their drop options met:')
    for key, count in per_refusal.most_common():
        print(f'  {count:3d}  {key}')
    print('failed moves over every drop option:')
    for key, count in moves.most_common(12):
        print(f'  {count:3d}  {key}')

    valued = [r for r in rows if r.get('values')]
    overall = planned / n_all
    print('\nby quartile: accepted (density in the episodes against the draws), head-camera refusals of all, '
          'planning refusals of those planned')
    for name, value in VALUES.items():
        values = sorted(value(r['values']) for r in valued)
        edges = [values[int(len(values) * q / 4)] for q in range(4)] + [values[-1] + 1e-9]
        cells = []
        for low, high in zip(edges, edges[1:]):
            group = [r for r in valued if low <= value(r['values']) < high]
            counts = collections.Counter(r['verdict'] for r in group)
            n, past = len(group), len(group) - counts['head camera']
            refused = sum(counts[k] for k in PLANNING)
            cells.append(f"{low:6.2f}..{high:6.2f}: {counts['planned'] / n:4.0%} "
                         f"({counts['planned'] / n / overall:3.1f}x) cam {counts['head camera'] / n:3.0%} "
                         f"plan {refused / past if past else 0:3.0%}")
        print(f'{name:22s} | ' + ' | '.join(cells))

    xs = sorted(VALUES['bucket x m'](r['values']) for r in tried)
    ys = sorted(VALUES['bucket y m'](r['values']) for r in tried)
    xe = [xs[int(len(xs) * q / 3)] for q in range(3)] + [xs[-1] + 1e-9]
    ye = [ys[int(len(ys) * q / 3)] for q in range(3)] + [ys[-1] + 1e-9]
    print('\nbucket centre as drawn (base frame, before the table offset): planned / past the head-camera check')
    print(' ' * 17 + ''.join(f'x {xe[i]:+.2f}..{xe[i + 1]:+.2f}  ' for i in range(3)))
    for j in reversed(range(3)):
        line = f'y {ye[j]:+.2f}..{ye[j + 1]:+.2f}  '
        for i in range(3):
            cell = [r for r in tried if xe[i] <= VALUES['bucket x m'](r['values']) < xe[i + 1]
                    and ye[j] <= VALUES['bucket y m'](r['values']) < ye[j + 1]]
            ok = sum(r['verdict'] == 'planned' for r in cell)
            line += f'  {ok:2d}/{len(cell):2d} ({ok / len(cell) if cell else 0:4.0%})  '
        print(line)
    print('\nblock height (1 mm bins, past the head-camera check): planned / tried')
    heights = collections.defaultdict(lambda: [0, 0])
    for r in tried:
        cell = heights[int(VALUES['block height mm'](r['values']))]
        cell[0] += r['verdict'] == 'planned'
        cell[1] += 1
    print('  ' + ', '.join(f'{h}: {ok}/{n}' for h, (ok, n) in sorted(heights.items())))
    drops = collections.Counter(r.get('drop') for r in rows if r['verdict'] == 'planned')
    print('\ndrops planned:', dict(drops.most_common()))


def view(path, width=320, height=240):
    """What the head camera sees of the bucket and the block at width x height, with the arms hanging: the share of
    the bucket walls' box corners in its image, whether the opening's centre (the middle of the walls' tops) is, and
    whether the block is. The scene is built and its points projected through workcell.head_pinhole, not rendered."""
    import mujoco
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.simulation.layouts import LayoutSampler
    from robot_platform.simulation.task_scene import MinervaTaskScene
    from robot_platform.simulation.workcell import head_pinhole, load_workcell
    stack = derive_stack()
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)

    def seen(workcell):
        scene = MinervaTaskScene(stack['derived']['joint_map'], stack['names'], workcell=workcell)
        model, data = scene.model, scene.data
        mujoco.mj_forward(model, data)
        corners, tops = [], []
        for geom in range(model.ngeom):
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or '').startswith('task/bucket/wall'):
                centre, turn, size = data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3), model.geom_size[geom]
                corners += [centre + turn @ (np.array(signs) * size[:3]) for signs in product((-1, 1), repeat=3)]
                tops.append(centre + turn @ np.array([0., 0., size[2]]))
        block = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'task/cube/geom')]
        camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'head')
        origin, axes = data.cam_xpos[camera], data.cam_xmat[camera].reshape(3, 3)
        fx, fy, cx, cy = head_pinhole(workcell.head_camera, width, height)
        inside = []
        for point in [*corners, np.mean(tops, axis=0), block]:
            x, y, z = axes.T @ (point - origin)  # MuJoCo cameras look along their -z, y up
            inside.append(bool(z < 0 and -.5 <= cx + fx * x / -z <= width - .5 and
                               -.5 <= cy - fy * y / -z <= height - .5))
        return sum(inside[:-2]) / len(corners), inside[-2], inside[-1]

    own = seen(load_workcell(WORKCELL))
    print(f'the workcell file: bucket corners in view {own[0]:.0%}, its opening {own[1]}, the block {own[2]}')
    rows = [json.loads(line) for line in open(path, encoding='utf-8') if line.strip()]
    results = [seen(sampler.draw(np.random.default_rng(r['seed']), seed=r['seed']).workcell)
               for r in rows if r['verdict'] == 'planned']
    shares = np.array([share for share, _, _ in results])
    n = len(results)
    print(f'{n} accepted draws at {width} x {height}: the whole bucket in view {np.mean(shares == 1):.0%}, at least '
          f'half its corners {np.mean(shares >= .5):.0%}, none {np.mean(shares == 0):.0%} '
          f'(median {np.median(shares):.0%}); its opening {sum(r[1] for r in results)} of {n}; '
          f'the block {sum(r[2] for r in results)} of {n}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    commands = parser.add_subparsers(dest='command', required=True)
    mapping = commands.add_parser('map')
    mapping.add_argument('out')
    mapping.add_argument('--draws', type=int, default=240)
    mapping.add_argument('--workers', type=int, default=6)
    mapping.add_argument('--seed', type=int, default=5000)
    for name in ('report', 'view'):
        commands.add_parser(name).add_argument('out')
    args = parser.parse_args()
    if args.command == 'map':
        run_map(args)
    elif args.command == 'report':
        report(args.out)
    else:
        view(args.out)


if __name__ == '__main__':
    main()
