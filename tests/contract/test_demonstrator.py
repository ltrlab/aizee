"""The scripted handoff demonstration through the actual receiver, driven grippers and the measured workcell."""
from pathlib import Path

from robot_platform.evaluation.demonstrator import HandoffDemonstrator
from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.runner import RunConfig, TrialRunner
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.workcell import load_workcell

MEASURED = Path(__file__).resolve().parents[2] / 'config/devices/simulation/minerva-production-01-workcell.json'


def test_the_demonstration_picks_hands_over_and_drops_the_cube_in_the_bucket(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    workcell = load_workcell(MEASURED)
    grid = GridSpec.for_workcell(workcell)
    runner = TrialRunner(RunConfig(grippers='driven', timeout_s=70., workcell=MEASURED, grid=grid),
                         stack=derive_stack(), directory=tmp_path)
    demo = HandoffDemonstrator()
    try:
        record = runner.run_trial(demo, cube_poses(grid)[0], 0)
    finally:
        runner.close()
        demo.close()
    assert record['outcome'] == 'success' and record['eligible'], record.get('detail')
    kinds = [event['kind'] for event in record['events']]
    assert kinds.index('pickup') < kinds.index('handoff') < kinds.index('bucket_settled')
    assert record['commands_clipped_cycles'] == 0
