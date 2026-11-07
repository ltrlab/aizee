"""Record the nominal handoff trial step by step, or compare two recordings bit for bit.

    .venv-dev/Scripts/python.exe scripts/sim/trial_trace.py record <out.npz> [--workcell <file>] [--timeout 70]
    .venv-dev/Scripts/python.exe scripts/sim/trial_trace.py compare <a.npz> <b.npz>

record runs the nominal trial: the handoff demonstrator through the actual receiver with driven grippers, trial 0 at
the workcell's cube start. After every scene step (each 2.5 ms receiver tick) it keeps the scene's time, qpos and
qvel, and a digest of every geom's surface velocity; then the trial's events and outcome. compare says whether two
recordings took the same steps with the same values, bit for bit, and the same events, and where they first differ.

A scene change that should leave the physics alone (colours, visual-only geoms) is checked this way: record on the
commit before it and on the commit after it (put the older source first on PYTHONPATH; both runs can share one
control_harness through ROBOT_CONTROL_HARNESS when the Rust code is the same) and compare.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'


def record(out, workcell=WORKCELL, timeout=70.):
    import robot_platform
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.grid import GridSpec, cube_poses
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.evaluation.runner import RunConfig, TrialRunner
    from robot_platform.simulation import task_scene
    from robot_platform.simulation.control_harness import default_binary
    from robot_platform.simulation.workcell import load_workcell
    steps = {'time': [], 'qpos': [], 'qvel': [], 'surface': []}
    original = task_scene.MinervaTaskScene.step_driven

    def traced(self, *args, **kwargs):
        state = original(self, *args, **kwargs)
        steps['time'].append(float(self.data.time))
        steps['qpos'].append(self.data.qpos.copy())
        steps['qvel'].append(self.data.qvel.copy())
        steps['surface'].append(hashlib.sha256(self.model.geom_surfacevel.tobytes()).digest())
        return state
    task_scene.MinervaTaskScene.step_driven = traced
    grid = GridSpec.for_workcell(load_workcell(workcell))
    with tempfile.TemporaryDirectory() as directory:
        runner = TrialRunner(RunConfig(grippers='driven', timeout_s=timeout, workcell=str(workcell), grid=grid),
                             stack=derive_stack(), directory=directory)
        demo = HandoffDemonstrator()
        try:
            result = runner.run_trial(demo, cube_poses(grid)[0], 0)
            ngeom = int(runner.scene.model.ngeom)
        finally:
            runner.close()
            demo.close()
    identity = {'source': str(Path(robot_platform.__file__).resolve().parents[1]), 'harness': str(default_binary()),
                'harness_sha256': hashlib.sha256(default_binary().read_bytes()).hexdigest(), 'workcell': str(workcell),
                'outcome': result['outcome'], 'events': result['events'], 'ngeom': ngeom,
                'clipped': result['commands_clipped_cycles']}
    np.savez_compressed(out, time=np.array(steps['time']), qpos=np.array(steps['qpos']), qvel=np.array(steps['qvel']),
                        surface=np.frombuffer(b''.join(steps['surface']), dtype=np.uint8).reshape(-1, 32),
                        identity=json.dumps(identity))
    print(f"{len(steps['time'])} scene steps, {result['outcome']}: "
          + ', '.join(f"{e['kind']} {e['scene_time']:.4f} s" for e in result['events'] if e['kind'] != 'reset'))
    return 0


def compare(a, b):
    first, second = np.load(a), np.load(b)
    ids = [json.loads(str(f['identity'])) for f in (first, second)]
    for f, identity in zip((a, b), ids):
        print(f"{f}: {len(np.load(f)['time'])} steps, {identity['outcome']}, source {identity['source']}, "
              f"harness {identity['harness_sha256'][:12]}")
    same = True
    if len(first['time']) != len(second['time']):
        print(f"different step counts: {len(first['time'])} and {len(second['time'])}")
        same = False
    count = min(len(first['time']), len(second['time']))
    for key in ('time', 'qpos', 'qvel', 'surface'):
        x, y = first[key][:count], second[key][:count]
        if x.shape[1:] != y.shape[1:]:
            print(f'{key}: different shapes {x.shape} and {y.shape}')
            same = False
            continue
        differ = np.flatnonzero(np.any((x != y).reshape(count, -1), axis=1))
        if len(differ):
            print(f'{key}: first differs at step {differ[0]} ({len(differ)} steps differ)')
            same = False
        else:
            print(f'{key}: identical at all {count} steps')
    if ids[0]['events'] != ids[1]['events']:
        print('events differ:', ids[0]['events'], ids[1]['events'], sep='\n')
        same = False
    else:
        print('events identical: ' + ', '.join(f"{e['kind']} {e['scene_time']:.4f} s" for e in ids[0]['events']))
    print('BIT FOR BIT THE SAME' if same else 'DIFFERENT')
    return 0 if same else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    r = sub.add_parser('record')
    r.add_argument('out')
    r.add_argument('--workcell', default=str(WORKCELL))
    r.add_argument('--timeout', type=float, default=70.)
    c = sub.add_parser('compare')
    c.add_argument('a')
    c.add_argument('b')
    args = parser.parse_args(argv)
    if args.command == 'record':
        return record(args.out, Path(args.workcell), args.timeout)
    return compare(args.a, args.b)


if __name__ == '__main__':
    sys.exit(main())
