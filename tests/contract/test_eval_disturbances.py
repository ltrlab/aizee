"""Evaluation disturbances through the actual receiver: inference latency, actuator spread and the scene's look."""
import numpy as np
import pytest

from robot_platform.evaluation.generate import APPEARANCE
from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.runner import RunConfig, TrialRunner
from robot_platform.simulation.control_harness import default_binary


class NudgeLeftShoulder:
    """Holds the staged pose with left J1 nudged 0.02 rad; keeps the time and receiver command of each call."""
    name = 'nudge_left_shoulder'
    execute_steps = 1

    def reset(self, context):
        self.joint = list(context.joint_names).index('left_arm_j1')
        self.target = None
        self.seen = []

    def __call__(self, observation):
        if self.target is None:
            self.target = np.asarray(observation['qcmd'], dtype=float).copy()
            self.target[self.joint] += .02
        self.seen.append((observation['t'], float(observation['qcmd'][self.joint])))
        return self.target[None, :]


def test_latency_delays_each_chunk_and_the_other_disturbances_are_recorded_and_undone(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    with pytest.raises(ValueError, match='latency'):
        RunConfig(latency_s=-.1)
    config = RunConfig(timeout_s=1., latency_s=.2, actuator_spread=.3, appearance=APPEARANCE, disturbance_seed=5,
                       camera_size=(64, 48))
    runner = TrialRunner(config, stack=derive_stack(), directory=tmp_path)
    policy = NudgeLeftShoulder()
    try:
        record = runner.run_trial(policy, cube_poses(GridSpec())[0], 0)
        specs = [dict(spec) for spec in runner.plant.plant.specs]
        nominal = runner.plant._nominal_specs
        cameras = runner.scene.model.cam_pos.copy()
        original = runner.scene._appearance['cam_pos']
    finally:
        runner.close()
    # One inference in flight: a call about every 0.2 s. The receiver's command does not move before the first chunk
    # takes effect at 0.2 s (without latency it ramps within 20 ms), and has reached the nudge by the third call.
    times = [t for t, _ in policy.seen]
    assert len(times) >= 4 and np.allclose(np.diff(times), .2, atol=.011)
    start = policy.seen[0][1]
    assert policy.seen[1][1] == pytest.approx(start, abs=1e-6)
    assert policy.seen[2][1] == pytest.approx(start + .02, abs=1e-4)
    disturbances = record['disturbances']
    factors = np.array(list(disturbances['actuator_factors'].values()))
    assert factors.shape == (14, 3) and np.all(factors >= .7) and np.all(factors <= 1.3)
    assert disturbances['latency_s'] == .2 and disturbances['appearance']['cameras']
    # The plant and the scene are back to their own values after the trial.
    assert specs == nominal and np.array_equal(cameras, original)
