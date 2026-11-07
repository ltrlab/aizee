"""Where on a workcell's table can the handoff demonstrator plan the whole task? Plan only: no receiver, no motion.

    .venv-dev/Scripts/python.exe scripts/sim/demonstrator_reach_map.py [--workcell <file>] [--step 0.04] [--workers 6]
        [--yaws -40 -20 0 20 40]

For each cube position on a grid it plans the demonstration (evaluation.demonstrator) and checks every planned move's
clearance to the table and bucket. '#': plans and clears by more than 1 mm; 'c': plans but closer; '.': no plan.
Use it to choose a workcell's cube region (docs/development/SIM_DEMONSTRATOR.md). With --yaws, one map per cube yaw
(degrees). Each worker samples the arms' poses once (about 10 s); keep the workers within memory (8 ran out on 16 GB).
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import math
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
_state = {}


def _init(workcell):
    from types import SimpleNamespace
    from robot_platform.evaluation import demonstrator
    from robot_platform.evaluation.plants import _Stack, derive_stack
    from robot_platform.simulation.workcell import resolve
    stack = derive_stack()
    _state.update(demonstrator=demonstrator, workcell=resolve(workcell), stack=stack, limits=_Stack(stack),
                  demo=demonstrator.HandoffDemonstrator(), context=SimpleNamespace)


def _plan(item):
    (x, y), yaw = item
    stack, limits = _state['stack'], _state['limits']
    context = _state['context'](joint_names=stack['names'], joint_map=stack['derived']['joint_map'],
                                lower=limits.lower, upper=limits.upper, home=limits.home, workcell=_state['workcell'],
                                period_s=limits.period_s, lead=limits.lead,
                                cube_start={'x': x, 'y': y, 'yaw': math.radians(yaw)})
    try:
        _state['demo'].reset(context)
    except _state['demonstrator'].PlanningError:
        return item, '.'
    worst = min(distance for _, distance, _, _ in _state['demo'].plan_clearance())
    return item, '#' if worst > .001 else 'c'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--workcell', default=str(DEFAULT), help="a workcell file or 'synthetic'")
    parser.add_argument('--x', type=float, nargs=2, default=(0., .28), help='x range, m (default 0 0.28)')
    parser.add_argument('--y', type=float, nargs=2, default=(-.52, -.20), help='y range, m (default -0.52 -0.20)')
    parser.add_argument('--step', type=float, default=.04, help='grid step, m (default 0.04)')
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--yaws', type=float, nargs='+', default=[0.], help='cube yaws, degrees (default 0)')
    args = parser.parse_args(argv)
    xs = np.round(np.arange(args.x[0], args.x[1] + 1e-9, args.step), 3)
    ys = np.round(np.arange(args.y[1], args.y[0] - 1e-9, -args.step), 3)
    points = [(float(x), float(y)) for y in ys for x in xs]
    started = time.time()
    items = [(point, yaw) for yaw in args.yaws for point in points]
    with ProcessPoolExecutor(max_workers=args.workers, initializer=_init, initargs=(args.workcell,)) as pool:
        marks = dict(pool.map(_plan, items))
    for yaw in args.yaws:
        if args.yaws != [0.]:
            print(f'yaw {yaw:+.0f} degrees')
        print('x:     ' + ' '.join(f'{x:+.2f}' for x in xs))
        for y in ys:
            print(f'y={y:+.2f} ' + '     '.join(marks[((float(x), float(y)), yaw)] for x in xs))
    print(f"'#' plans and clears by over 1 mm, 'c' plans closer, '.' no plan ({time.time() - started:.0f} s)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
