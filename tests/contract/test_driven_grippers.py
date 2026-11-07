"""Driven grippers through the actual receiver: the drive stalls on the cube, and the receiver reads that back."""
import numpy as np
import pytest

from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.policies import fixture_trajectory
from robot_platform.evaluation.runner import RunConfig, TrialRunner
from robot_platform.simulation.control_harness import default_binary


class CloseLeftGripper:
    """From the frozen fixture's start (open left jaws around the cube), hold the arms and close the left
    gripper past the cube while the right one closes on nothing. Keeps every observation."""
    name = 'close_left_gripper'

    def reset(self, context):
        rows, _ = fixture_trajectory(context.joint_names, context.joint_map)
        self.names = list(context.joint_names)
        self.start = rows[0].copy()
        self.command = self.start.copy()
        self.left, self.right = self.names.index('left_gripper'), self.names.index('right_gripper')
        # The jaws reach the 36 mm cube well before 0.5 rad; the free right gripper closes most of the way.
        self.command[self.left] = .5
        self.command[self.right] = 1.
        self.seen = []

    def start_pose(self, context):
        return self.start

    def __call__(self, observation):
        self.seen.append(observation)
        return self.command[None, :]


def test_the_receiver_reads_a_driven_gripper_stalled_on_the_cube(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    with pytest.raises(ValueError, match='armplant'):
        RunConfig(plant='ideal', grippers='driven')
    stack = derive_stack()
    runner = TrialRunner(RunConfig(grippers='driven', timeout_s=3.), stack=stack, directory=tmp_path)
    policy = CloseLeftGripper()
    try:
        record = runner.run_trial(policy, cube_poses(GridSpec())[0], 0)
        drives = runner.scene.capabilities['gripper_drives']
        snapshot = runner.scene.snapshot()
    finally:
        runner.close()
    assert record['outcome'] == 'timeout' and record['eligible'], record.get('detail')
    last = policy.seen[-1]
    left, right = policy.left, policy.right
    # The receiver's own feedback: the left gripper stopped well short of its command, pressing with its drive...
    assert last['q'][left] > policy.command[left] + .8
    assert last['effort'][left] < -.5 and abs(last['effort'][left]) <= drives['left_gripper']['max_effort'] + 1e-9
    # ...the right one, holding nothing, reached its command.
    assert last['q'][right] == pytest.approx(policy.command[right], abs=.05)
    assert snapshot['jaw_contacts']['left'] == [0, 1] and snapshot['grippers']['left_gripper']['q'] > .5 + .8
    # Readings are the drive's own: steady once stalled, and the same in the scene and the receiver's feedback.
    tail = np.array([o['q'][left] for o in policy.seen[-20:]])
    assert np.ptp(tail) < .01
    assert last['q'][left] == pytest.approx(snapshot['grippers']['left_gripper']['q'], abs=.01)
