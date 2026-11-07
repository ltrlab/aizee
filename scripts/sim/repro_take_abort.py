"""Reproduce the right's take aborting on a large held block: layout trial attempt 23 of seed 2000 at 0789f74, saved as
a fixture (2026-10-18; docs/development/SIM_DEMONSTRATOR.md, "Follow-ups").

    .venv-dev/Scripts/python.exe scripts/sim/repro_take_abort.py [--fixture <file>] [--plant-seed 5]

The handoff demonstrator runs through the receiver with driven grippers on the saved layout: a 50 x 47 x 48 mm block at
(0.331, -0.481) m, the bucket at (0.052, -0.565) m. Prints the trial's outcome, the receiver's last trip, the last moves
the demonstrator started, and when the right's fingers first pressed on the block. Expected while the follow-up is
open: the receiver stops on the right gripper's position_above_upper about 1 s into 'right turns', a right finger on
the held block from about 0.64 s into it.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / 'tests/fixtures/minerva_layout_take_abort_20261018.json'
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--fixture', default=str(FIXTURE))
    parser.add_argument('--plant-seed', type=int, default=5)
    args = parser.parse_args()
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.evaluation.runner import RunConfig, TrialRunner
    from robot_platform.simulation.task_scene import JAW_BODIES
    from robot_platform.simulation.workcell import Workcell
    saved = json.loads(Path(args.fixture).read_text(encoding='utf-8'))
    runner = TrialRunner(RunConfig(grippers='driven', timeout_s=90., workcell=str(WORKCELL), plant_seed=args.plant_seed),
                         stack=derive_stack(), directory=Path(tempfile.mkdtemp()))
    demo, touches = HandoffDemonstrator(), {}
    try:
        runner.use_workcell(Workcell.from_description(saved['workcell']))
        plant, scene = runner._open_plant(), runner._open_scene()  # The runner lets go of an aborted plant.
        snapshot = scene.snapshot

        def watched():
            # The trial reads the coupled scene through its snapshot after every control cycle.
            state = snapshot()
            for name in state['cube_contacts']:
                if name.split('/')[0] in JAW_BODIES['right']:
                    touches.setdefault(name, state['scene_time'])
            return state
        scene.snapshot = watched
        index = saved['seed'][1]
        record = runner.run_trial(demo, dict(saved['cube_start'], cell='layout', index=index), index)
        trip = (plant.controller.status or {}).get('last_trip')
        moves = list(demo.log[-3:])
    finally:
        runner.close()
        demo.close()
    print('outcome:', record['outcome'], '-', record.get('detail'))
    print('stages:', record['stages'])
    print('last trip:', json.dumps(trip))
    print('last moves started (name, s):', moves)
    for name, at in sorted(touches.items(), key=lambda item: item[1]):
        print(f'right finger on the block from {at:.3f} s: {name}')
    expected = saved['expected']
    reproduced = record['outcome'] == expected['outcome'] and trip is not None and all(
        trip.get(key) == value for key, value in expected['trip'].items()) and moves and \
        moves[-1][0] == expected['last_move']
    print('reproduced' if reproduced else 'NOT reproduced (the follow-up may be fixed)')


if __name__ == '__main__':
    main()
