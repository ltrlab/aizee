"""Synthetic demonstrations through the actual receiver: what the recorder publishes, and what it throws away."""

import json

import h5py
import numpy as np

from robot_platform.evaluation.generate import EpisodeRecorder
from robot_platform.simulation.task_scene import SIMULATED_HEAD
from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.policies import HoldStillPolicy
from robot_platform.evaluation.runner import RunConfig, TrialRunner
from robot_platform.simulation.control_harness import default_binary


class _Unplannable(HoldStillPolicy):
    def reset(self, context):
        raise ValueError('no plan clears the table and bucket')


def test_a_failed_trial_leaves_no_episode_and_a_success_publishes_one(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    runner = TrialRunner(
        RunConfig(grippers='driven', timeout_s=1.0, camera_size=(64, 48)),
        stack=derive_stack(),
        directory=tmp_path / 'receiver',
    )
    recorder = EpisodeRecorder(tmp_path / 'episodes', hz=15.0, metadata={'test': True})
    try:
        # Holding still never succeeds: the recording is thrown away, partial file and all.
        record = runner.run_trial(HoldStillPolicy(), cube_poses(GridSpec())[0], 0, recorder=recorder)
        assert record['outcome'] == 'timeout' and recorder.last_path is None and recorder.last_frames >= 14
        assert not list((tmp_path / 'episodes').glob('*.hdf5'))
        # A policy that cannot plan never starts a recording: no frames are counted, not the last trial's.
        record = runner.run_trial(_Unplannable(), cube_poses(GridSpec())[0], 1, recorder=recorder)
        assert record['outcome'] == 'source_gap_or_abort' and recorder.last_path is None and recorder.last_frames == 0
        # The same recorder on the live scene and plant publishes the frames of a trial that succeeded.
        plant, scene = runner.plant, runner.scene
        context = type(
            'Context', (), dict(index=7, cube_start={'x': 0.18, 'y': -0.34, 'yaw': 0.0}, joint_names=tuple(plant.names))
        )()
        recorder.start(scene, plant, context, HoldStillPolicy())
        for _ in range(20):
            plant.cycle()
            scene.update(plant.scene_pose(), plant.period_s)
            recorder.sample(scene, plant)
        path = recorder.finish({'outcome': 'success', 'eligible': True})
    finally:
        runner.close()
    assert path is not None and path.is_file() and not list((tmp_path / 'episodes').glob('*.partial.hdf5'))
    with h5py.File(path, 'r') as episode:
        attrs = dict(episode.attrs)
        metadata = json.loads(attrs['metadata_json'])
        assert attrs['format_version'] == 9 and attrs['complete'] and not attrs['training_promoted']
        assert metadata['collection_mode'] == 'scripted_demonstrator' and metadata['test'] is True
        assert metadata['head'] == SIMULATED_HEAD
        assert metadata['scene']['gripper_drives'] and metadata['cube_start']['x'] == 0.18
        frames = int(attrs['frame_count'])
        assert frames == recorder.last_frames and 2 <= frames <= 4  # 20 cycles of 10 ms at 15 Hz.
        assert set(json.loads(attrs['camera_roles'])) == {'left_wrist', 'right_wrist', 'head'}
        assert episode['observations/images/head'].shape[1:] == (48, 64, 3)
        assert np.all(np.diff(episode['timestamps/telem'][:]) > 0)
        torques, valid = episode['observations/torques'][:], episode['observations/torque_valid'][:]
        assert np.all(np.isfinite(torques[:, :14])) and np.all(valid[:, :14])  # The arms' efforts, as the receiver's.
        assert np.all(np.isnan(torques[:, 14:])) and not np.any(valid[:, 14:])  # The simulated head has none.
        assert np.all(episode['observations/qpos'][:, 14:] == 0)  # It stays at the neck's zero.
        assert set(episode['authority'].asstr()[:]) == {'policy'}
