"""The evaluation harness against its reference policies, on the actual scene, receiver and simulated joints.

No USB, CAN, SSH or motors. The `armplant` runs need the native control harness
(cargo build -p robot_control --bin control_harness), as the other receiver contracts do.
"""
import numpy as np
import pytest

from robot_platform.evaluation import trials as trial_records
from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.policies import (FixtureReplayPolicy, HoldStillPolicy, PenetratingPolicy,
                                                fixture_trajectory)
from robot_platform.evaluation.reach import GraspIK
from robot_platform.evaluation.runner import RunConfig, TrialRunner, evaluate
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.task_scene import PHYSICS_DT, MinervaTaskScene

pytestmark = pytest.mark.integration
mujoco = pytest.importorskip('mujoco')


@pytest.fixture(scope='module')
def stack():
    return derive_stack()


def armplant():
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    return 'armplant'


def test_fixture_replay_succeeds_from_its_own_start_with_exact_tracking(stack):
    report = evaluate(FixtureReplayPolicy(), RunConfig(plant='ideal', timeout_s=20., trials=(0,)), stack=stack)
    trial = report['trials_detail'][0]
    assert trial['cube_start']['cell'] == 'nominal'
    assert trial['outcome'] == 'success' and trial['eligible']
    # The scene's own events, in order, carried into the record unchanged.
    assert [e['kind'] for e in trial['events']] == ['reset', 'pickup', 'handoff', 'bucket_settled']
    assert trial['stages'] == {stage: next(e['scene_time'] for e in trial['events'] if e['kind'] == kind)
                               for stage, kind in (('pickup', 'pickup'), ('handoff', 'handoff'),
                                                   ('bucket', 'bucket_settled'))}
    assert report['success']['count'] == 1 and report['stage_completion']['score'] == 1.
    assert report['time_to_complete_s']['median'] == trial['stages']['bucket'] < 12.5
    assert report['provenance']['plant'] == 'ideal'
    assert not report['provenance']['sim2real_validated'] and not report['provenance']['training_promoted']


def test_fixture_replay_through_the_receiver_reports_what_the_scene_says(stack):
    """No success is asserted: an open-loop millimetre-margin replay through tracking error is the question.
    The record must be exactly the scene's verdict (MINERVA_EVAL.md records the measured result)."""
    runner = TrialRunner(RunConfig(plant=armplant(), timeout_s=12.), stack=stack)
    try:
        trial = runner.run_trial(FixtureReplayPolicy(), cube_poses(GridSpec())[0], 0)
    finally:
        runner.close()
    checked = trial_records.validate(trial)
    assert checked['counted'] and trial['staging']['duration_s'] > 0
    assert (trial['stages'], trial['outcome'], trial['eligible']) == trial_records.outcome_from_events(
        trial['events'], timed_out=trial['outcome'] == 'timeout')[:3]
    assert trial['eligible'] == (not any(e['kind'] == 'forbidden_robot_environment_penetration'
                                         for e in trial['events']))


def test_hold_still_scores_zero_with_timeouts(stack):
    report = evaluate(HoldStillPolicy(), RunConfig(plant=armplant(), timeout_s=2., trials=(0, 1, 9)), stack=stack)
    assert [t['outcome'] for t in report['trials_detail']] == ['timeout'] * 3
    assert all(t['duration_s'] == pytest.approx(2., abs=.02) for t in report['trials_detail'])
    assert report['success'] == {'count': 0, 'rate': 0., 'wilson95': [0., pytest.approx(.5615, abs=1e-4)]}
    assert report['stage_completion']['score'] == 0. and report['outcomes']['timeout'] == 3
    assert report['trials']['eligible'] == 3


def test_penetrating_policy_is_ineligible(stack):
    report = evaluate(PenetratingPolicy(), RunConfig(plant=armplant(), timeout_s=5., trials=(0,)), stack=stack)
    trial = report['trials_detail'][0]
    assert trial['outcome'] == 'forbidden_penetration' and trial['eligible'] is False
    assert 'task/table' in trial['detail']
    assert report['trials']['eligible'] == 0 and report['success']['count'] == 0
    assert report['outcomes']['forbidden_penetration'] == 1


def test_policy_errors_and_bad_chunks_abort_one_trial_and_the_run_goes_on(stack):
    class Broken:
        name = 'broken'

        def __init__(self):
            self.trial = -1

        def reset(self, context):
            self.trial += 1

        def __call__(self, observation):
            if self.trial == 0 and observation['t'] > .2:
                raise RuntimeError('model exploded')
            if self.trial == 1:
                return np.zeros((2, 13))
            return observation['q'][None, :]
    report = evaluate(Broken(), RunConfig(plant='ideal', timeout_s=.5, trials=(0, 1, 2)), stack=stack)
    outcomes = [(t['outcome'], t['detail']) for t in report['trials_detail']]
    assert outcomes[0][0] == outcomes[1][0] == 'source_gap_or_abort'
    assert 'model exploded' in outcomes[0][1] and '(H, 14)' in outcomes[1][1]
    assert outcomes[2] == ('timeout', None)


def test_default_grid_poses_are_reachable_without_penetration(stack):
    """Every default cube start: a left-gripper top grasp within receiver limits, posed without penetration."""
    names = stack['names']
    rows = {row['name']: row for row in stack['derived']['lifecycle']}
    scene = MinervaTaskScene(stack['derived']['joint_map'], names, width=64, height=48)
    try:
        ik = GraspIK(scene, [rows[n]['lower'] for n in names], [rows[n]['upper'] for n in names])
        seed = fixture_trajectory(names, stack['derived']['joint_map'])[0][0]
        for pose in cube_poses(GridSpec()):
            solved = ik.solve('left', pose['x'], pose['y'], pose['yaw'], seed=seed)
            assert solved['ok'], (pose, solved)
            # Public scene API: move there, reset (clearing the move's audit), then audit the pose itself.
            scene.update(solved['q'], .05)
            scene.reset()
            state = scene.update(solved['q'], PHYSICS_DT)
            assert not state['forbidden_contacts'], (pose, state['forbidden_contacts'])
    finally:
        scene.close()
