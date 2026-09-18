"""Historical HDF5 contracts and caller-selected recording definitions."""

import json
import subprocess
import sys

import h5py
import numpy as np
import pytest

from robot_platform.data.formats import aizee, minerva


def positions(joints):
    return [np.arange(joints, dtype=np.float32) + frame for frame in range(3)]


def frames():
    return [np.full((4, 6, 3), frame, dtype=np.uint8) for frame in range(3)]


def expected_actions(samples):
    return np.stack([samples[1], samples[2], samples[2]])


@pytest.mark.parametrize("scene", [False, True])
@pytest.mark.parametrize("commanded", [False, True])
def test_aizee_v4_v5_contract(tmp_path, scene, commanded):
    qpos = positions(7)
    qcmd = [sample + 10 for sample in qpos] if commanded else None
    path, count = aizee.save_episode(
        tmp_path, qpos, frames(),
        telem_ts_buf=[1., 2., 3.], gripper_ts_buf=[0.9, 1.9, 2.9],
        qcmd_buf=qcmd, torque_buf=positions(7),
        scene_buf=frames() if scene else None,
        scene_ts_buf=[0.8, 1.8, 2.8] if scene else None,
        task_tag="pick", notes="contract test",
    )
    assert path.name == "episode_0000.hdf5"
    assert count == 3
    with h5py.File(path, "r") as episode:
        assert episode.attrs["format_version"] == (5 if scene else 4)
        assert episode.attrs["hz"] == 20
        assert episode.attrs["arm_joints"] == ",".join(aizee.DEFAULT_JOINT_NAMES)
        assert episode.attrs["action_space"] == "absolute"
        assert episode.attrs["task_tag"] == "pick"
        assert episode.attrs["notes"] == "contract test"
        assert episode.attrs["collected_at"] > 0
        assert ("observations/qcmd" in episode) is commanded
        np.testing.assert_array_equal(episode["observations/qpos"], np.stack(qpos))
        np.testing.assert_array_equal(episode["actions"], expected_actions(qcmd or qpos))
        assert episode["actions"].dtype == np.dtype("float32")
        assert episode["observations/images/gripper"].chunks == (1, 4, 6, 3)
        assert episode["observations/images/gripper"].dtype == np.dtype("uint8")
        assert episode["observations/qpos"].compression == "gzip"
        assert episode["observations/qpos"].compression_opts == 4
        assert ("observations/images/scene" in episode) is scene
        assert ("timestamps/camera_scene" in episode) is scene
        assert episode["timestamps/telem"].dtype == np.dtype("float64")
        np.testing.assert_array_equal(episode["timestamps/camera_gripper"], [0.9, 1.9, 2.9])
    second, _ = aizee.save_episode(tmp_path, qpos, frames())
    assert second.name == "episode_0001.hdf5"


@pytest.mark.parametrize("segmented", [False, True])
@pytest.mark.parametrize("commanded", [False, True])
def test_minerva_v6_v7_contract(tmp_path, segmented, commanded):
    qpos = positions(17)
    qcmd = [sample + 10 for sample in qpos] if commanded else None
    segments = [{"start": 0, "end": 3, "label": "reach"}] if segmented else None
    cameras = {name: frames() for name in minerva.DEFAULT_CAMERA_NAMES}
    path, count = minerva.save_minerva_episode(
        tmp_path, qpos, cameras, qcmd_buf=qcmd, torque_buf=positions(17),
        telem_ts_buf=[1., 2., 3.],
        camera_ts_bufs={name: [0.9, 1.9, 2.9] for name in cameras},
        language_instruction="pick up the block", task_id=42, notes="contract test",
        segments=segments,
    )
    assert count == 3
    with h5py.File(path, "r") as episode:
        assert episode.attrs["format_version"] == (7 if segmented else 6)
        assert episode.attrs["hz"] == 20
        assert episode.attrs["minerva_joints"] == ",".join(minerva.DEFAULT_JOINT_NAMES)
        assert episode.attrs["language_instruction"] == "pick up the block"
        assert episode.attrs["task_id"] == 42
        assert episode.attrs["action_space"] == "absolute"
        assert ("segments" in episode.attrs) is segmented
        if segmented:
            assert json.loads(episode.attrs["segments"]) == segments
        np.testing.assert_array_equal(episode["actions"], expected_actions(qcmd or qpos))
        assert episode["observations/torques"].shape == (3, 17)
        assert ("observations/qcmd" in episode) is commanded
        assert set(episode["observations/images"]) == set(cameras)
        for camera in cameras:
            image_data = episode[f"observations/images/{camera}"]
            np.testing.assert_array_equal(image_data, np.stack(cameras[camera]))
            assert image_data.chunks == (1, 4, 6, 3)
            assert image_data.compression == "gzip"
            assert image_data.compression_opts == 4
            assert episode[f"timestamps/camera_{camera}"].dtype == np.dtype("float64")
    second, _ = minerva.save_minerva_episode(tmp_path, qpos, cameras)
    assert second.name == "episode_0001.hdf5"


def test_selected_joint_camera_and_rate_settings(tmp_path):
    joint_names = ("shoulder", "finger")
    path, _ = minerva.save_minerva_episode(
        tmp_path, positions(2), {"overview": frames()},
        joint_names=joint_names, camera_names=("overview",), hz=15,
    )
    with h5py.File(path, "r") as episode:
        assert episode.attrs["minerva_joints"] == "shoulder,finger"
        assert episode.attrs["hz"] == 15
        assert episode["actions"].shape == (3, 2)
        assert list(episode["observations/images"]) == ["overview"]
    path, _ = aizee.save_episode(tmp_path, positions(2), frames(), joint_names=joint_names, hz=12)
    with h5py.File(path, "r") as episode:
        assert episode.attrs["arm_joints"] == "shoulder,finger"
        assert episode.attrs["hz"] == 12
        assert episode["actions"].shape == (3, 2)


def test_minerva_rejects_missing_cameras_and_wrong_joint_count(tmp_path):
    with pytest.raises(ValueError, match="left_wrist"):
        minerva.save_minerva_episode(tmp_path, positions(17), {})
    with pytest.raises(ValueError, match="17"):
        minerva.save_minerva_episode(tmp_path, positions(7), {})


def test_legacy_writers_reexport_shared_functions():
    from python.scripts.collect_demo_app import recording
    from python.scripts.collect_minerva_app import recording as minerva_session

    assert recording.save_episode is aizee.save_episode
    assert minerva_session.save_minerva_episode is minerva.save_minerva_episode


def test_shared_writers_do_not_import_collector_or_control_modules():
    result = subprocess.run(
        [sys.executable, "-c", (
            "import sys; original = sys.getswitchinterval(); "
            "from robot_platform.data.formats import aizee, minerva; "
            "assert sys.getswitchinterval() == original; "
            "assert not any('collect_demo_app' in m or 'collect_minerva_app' in m "
            "or m.startswith('common.') or m.startswith('python.') for m in sys.modules)"
        )],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, result.stderr
