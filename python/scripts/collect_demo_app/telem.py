"""Telemetry extraction helpers (from collect_demo.py)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from common.arm_constants import ARM_JOINTS

from .alignment import _motor_to_urdf_pos

# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

def _qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract 7-element arm joint positions in URDF frame.

    Motor encoders report in motor frame; we apply the per-joint sign +
    offset from joint_align.json so the rest of collect_demo (IK, engage
    threshold, recording, replay) all work in a single consistent frame.
    Returns None if no arm telemetry is present."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    q_motor = np.array(
        [float(motors[j].get("position", 0.0)) if j in motors else 0.0
         for j in ARM_JOINTS],
        dtype=np.float32,
    )
    return _motor_to_urdf_pos(q_motor)

def _qpos_motor(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Same as _qpos but returns the RAW motor-frame positions.

    Used only for the telem broadcast field `qpos_motor` so the /preview
    calibration UI can compute live offsets/signs against the raw encoder
    reading."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    return np.array(
        [float(motors[j].get("position", 0.0)) if j in motors else 0.0
         for j in ARM_JOINTS],
        dtype=np.float32,
    )


def _qtorque(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("torque", _nan)) if j in motors else _nan
         for j in ARM_JOINTS],
        dtype=np.float32,
    )


def _qtemp(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("temperature", _nan)) if j in motors else _nan
         for j in ARM_JOINTS],
        dtype=np.float32,
    )
