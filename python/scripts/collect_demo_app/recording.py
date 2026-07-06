"""HDF5 episode writer (from collect_demo.py)."""
from __future__ import annotations

import time
from pathlib import Path

import h5py
import numpy as np

from common.arm_constants import ARM_JOINTS

from .runtime import REC_HZ

# ---------------------------------------------------------------------------
# HDF5 episode writer
# ---------------------------------------------------------------------------

def save_episode(
    output_dir, qpos_buf, gripper_buf,
    telem_ts_buf=None, gripper_ts_buf=None,
    qcmd_buf=None, torque_buf=None,
    scene_buf=None, scene_ts_buf=None,
    task_tag: str = "", notes: str = "",
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(output_dir.glob("episode_*.hdf5"))
    ep_num   = (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0
    path     = output_dir / f"episode_{ep_num:04d}.hdf5"

    # Buffers are already 7-DOF (swivel-first) — qpos_buf comes straight from
    # _qpos which iterates ARM_JOINTS, and ARM_JOINTS now includes swivel.
    qpos_arr    = np.stack(qpos_buf,    axis=0).astype(np.float32)   # [T, 7]
    gripper_arr = np.stack(gripper_buf, axis=0)                       # [T, H, W, 3]
    qcmd_arr    = (np.stack(qcmd_buf, axis=0).astype(np.float32)
                   if qcmd_buf and len(qcmd_buf) == len(qpos_buf) else None)
    torque_arr  = (np.stack(torque_buf, axis=0).astype(np.float32)
                   if torque_buf and len(torque_buf) == len(qpos_buf) else None)
    # Scene cam is optional — a session without --scene-cam (or with the
    # service down) produces v4 episodes; a populated scene_buf upgrades
    # to v5 with `observations/images/scene` + `timestamps/camera_scene`.
    scene_arr   = (np.stack(scene_buf, axis=0)
                   if scene_buf and len(scene_buf) == len(qpos_buf) else None)

    # Actions derived from commanded positions (no sag) when available
    act_src  = qcmd_arr if qcmd_arr is not None else qpos_arr
    actions  = np.concatenate([act_src[1:], act_src[-1:]], axis=0).astype(np.float32)  # [T, 7]
    H, W     = gripper_arr.shape[1], gripper_arr.shape[2]

    with h5py.File(path, "w") as f:
        f.attrs["hz"]           = REC_HZ
        f.attrs["arm_joints"]   = ",".join(ARM_JOINTS)
        f.attrs["action_space"] = "absolute"
        # v5 = v4 + `observations/images/scene` (RealSense color) and
        #      `timestamps/camera_scene`. Written only when scene_arr is
        #      populated.
        # v4 = 7-DOF unified arm + single gripper camera (replacing the
        # previous stereo D435 pair). v3 = 7-DOF + stereo left/right images.
        # v2 was also 7-DOF stereo but post-prepend, with a sidecar swivel
        # field; v1 was 6-DOF arm + sidecar swivel. load_recording in
        # record_replay.py handles all four (older versions are dropped on
        # the new single-camera training path).
        f.attrs["format_version"] = 5 if scene_arr is not None else 4
        f.attrs["task_tag"]     = task_tag
        f.attrs["notes"]        = notes
        f.attrs["collected_at"] = float(time.time())
        obs  = f.create_group("observations")
        obs.create_dataset("qpos",   data=qpos_arr,  compression="gzip", compression_opts=4)
        if qcmd_arr is not None:
            obs.create_dataset("qcmd", data=qcmd_arr, compression="gzip", compression_opts=4)
        if torque_arr is not None:
            obs.create_dataset("torques", data=torque_arr, compression="gzip", compression_opts=4)
        imgs = obs.create_group("images")
        imgs.create_dataset("gripper", data=gripper_arr,
                            compression="gzip", compression_opts=4,
                            chunks=(1, H, W, 3))
        if scene_arr is not None:
            sH, sW = scene_arr.shape[1], scene_arr.shape[2]
            imgs.create_dataset("scene", data=scene_arr,
                                compression="gzip", compression_opts=4,
                                chunks=(1, sH, sW, 3))
        f.create_dataset("actions",  data=actions,   compression="gzip", compression_opts=4)
        if telem_ts_buf is not None:
            ts = f.create_group("timestamps")
            ts.create_dataset("telem",          data=np.array(telem_ts_buf,   dtype=np.float64))
            ts.create_dataset("camera_gripper", data=np.array(gripper_ts_buf, dtype=np.float64))
            if scene_ts_buf is not None and len(scene_ts_buf) == len(qpos_buf):
                ts.create_dataset("camera_scene",
                                  data=np.array(scene_ts_buf, dtype=np.float64))

    return path, len(qpos_buf)
