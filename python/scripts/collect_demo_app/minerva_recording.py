"""
minerva_recording.py — HDF5 episode writer for the Minerva bimanual robot.

Extends the AIZEE single-camera recorder (recording.py) to Minerva's 3-camera,
17-DoF, language-conditioned schema. Kept as a sibling module so the working
AIZEE ACT collection path is untouched.

Episode HDF5 schema (format_version = 6), read by
python/training/minerva_dataset.py:

    /observations/qpos                 float32 [T, 17]
    /observations/qcmd                 float32 [T, 17]   (optional)
    /observations/torques              float32 [T, 17]   (optional)
    /observations/images/left_wrist    uint8   [T, H, W, 3]
    /observations/images/right_wrist   uint8   [T, H, W, 3]
    /observations/images/head          uint8   [T, Hs, Ws, 3]
    /actions                           float32 [T, 17]
    /timestamps/{telem, camera_<name>} float64 [T]        (optional)
    attrs: hz, minerva_joints, action_space="absolute", format_version=6,
           language_instruction="<task string>", task_id=<int>, notes, collected_at

Wiring into the live collection loop still requires subscribing to the three
camera ZMQ endpoints (config/minerva.yaml endpoints.cameras) and accumulating
per-camera frame buffers alongside qpos/qcmd/torque — that part depends on your
teleop/publisher setup. This module owns only the on-disk contract, which is
what the dataset + smoke test validate.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import h5py
import numpy as np

try:  # collect_demo_app runs with python/ on sys.path (`common.*`);
    from common.minerva_constants import CAMERAS, MINERVA_JOINTS, RECORD_HZ
except ImportError:  # training/tests run with aizee/ on sys.path (`python.*`).
    from python.common.minerva_constants import CAMERAS, MINERVA_JOINTS, RECORD_HZ

_NUM_JOINTS = len(MINERVA_JOINTS)


def save_minerva_episode(
    output_dir,
    qpos_buf: Sequence[np.ndarray],
    camera_bufs: Dict[str, Sequence[np.ndarray]],
    *,
    qcmd_buf: Optional[Sequence[np.ndarray]] = None,
    torque_buf: Optional[Sequence[np.ndarray]] = None,
    telem_ts_buf: Optional[Sequence[float]] = None,
    camera_ts_bufs: Optional[Dict[str, Sequence[float]]] = None,
    language_instruction: str = "",
    task_id: Optional[int] = None,
    notes: str = "",
    action_space: str = "absolute",
    segments: Optional[Sequence[dict]] = None,
):
    """Write one Minerva episode (schema v6). Returns (path, num_frames).

    Args:
        qpos_buf: list of length T of [17] float arrays (canonical joint order).
        camera_bufs: {"left_wrist": [T frames], "right_wrist": [...], "head": [...]},
            each frame a uint8 [H, W, 3] RGB array. All three cameras are required.
        qcmd_buf / torque_buf: optional [17] per-frame arrays. Actions are derived
            from qcmd when present (no gravity sag), else from qpos.
        language_instruction: the task string for language conditioning.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    existing = sorted(output_dir.glob("episode_*.hdf5"))
    ep_num = (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0
    path = output_dir / f"episode_{ep_num:04d}.hdf5"

    T = len(qpos_buf)
    if T < 2:
        raise ValueError("episode must have >= 2 frames")
    qpos_arr = np.stack(qpos_buf, axis=0).astype(np.float32)      # [T, 17]
    if qpos_arr.shape[1] != _NUM_JOINTS:
        raise ValueError(f"qpos has {qpos_arr.shape[1]} joints, expected {_NUM_JOINTS}")

    qcmd_arr = (np.stack(qcmd_buf, 0).astype(np.float32)
                if qcmd_buf is not None and len(qcmd_buf) == T else None)
    torque_arr = (np.stack(torque_buf, 0).astype(np.float32)
                  if torque_buf is not None and len(torque_buf) == T else None)

    cam_arrays: Dict[str, np.ndarray] = {}
    for name in CAMERAS:
        buf = camera_bufs.get(name)
        if buf is None or len(buf) != T:
            raise ValueError(
                f"camera '{name}' missing or wrong length "
                f"({0 if buf is None else len(buf)} vs {T} frames)")
        cam_arrays[name] = np.stack(buf, axis=0)                  # [T, H, W, 3] uint8

    # Actions = next commanded position (falls back to qpos), last repeated.
    act_src = qcmd_arr if qcmd_arr is not None else qpos_arr
    actions = np.concatenate([act_src[1:], act_src[-1:]], axis=0).astype(np.float32)

    with h5py.File(path, "w") as f:
        f.attrs["hz"] = RECORD_HZ
        f.attrs["minerva_joints"] = ",".join(MINERVA_JOINTS)
        f.attrs["action_space"] = action_space
        # v7 = v6 + temporal action segments (per-phase language labels), stored
        # as a JSON list of {start, end, label} with half-open [start, end) frame
        # ranges. Absent -> plain v6 (single episode-level instruction).
        f.attrs["format_version"] = 7 if segments else 6
        f.attrs["language_instruction"] = str(language_instruction)
        if segments:
            f.attrs["segments"] = json.dumps(list(segments))
        if task_id is not None:
            f.attrs["task_id"] = int(task_id)
        f.attrs["notes"] = notes
        f.attrs["collected_at"] = float(time.time())

        obs = f.create_group("observations")
        obs.create_dataset("qpos", data=qpos_arr, compression="gzip", compression_opts=4)
        if qcmd_arr is not None:
            obs.create_dataset("qcmd", data=qcmd_arr, compression="gzip", compression_opts=4)
        if torque_arr is not None:
            obs.create_dataset("torques", data=torque_arr, compression="gzip", compression_opts=4)

        imgs = obs.create_group("images")
        for name, arr in cam_arrays.items():
            H, W = arr.shape[1], arr.shape[2]
            imgs.create_dataset(name, data=arr, compression="gzip", compression_opts=4,
                                chunks=(1, H, W, 3))

        f.create_dataset("actions", data=actions, compression="gzip", compression_opts=4)

        if telem_ts_buf is not None or camera_ts_bufs:
            ts = f.create_group("timestamps")
            if telem_ts_buf is not None:
                ts.create_dataset("telem", data=np.array(telem_ts_buf, dtype=np.float64))
            for name, tsbuf in (camera_ts_bufs or {}).items():
                if tsbuf is not None and len(tsbuf) == T:
                    ts.create_dataset(f"camera_{name}", data=np.array(tsbuf, dtype=np.float64))

    return path, T


__all__ = ["save_minerva_episode"]
