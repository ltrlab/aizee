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
    q_motor = _qpos_motor(telem)
    return None if q_motor is None else _motor_to_urdf_pos(q_motor)

def _qpos_motor(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Same as _qpos but returns the RAW motor-frame positions.

    Used only for the telem broadcast field `qpos_motor` so the /preview
    calibration UI can compute live offsets/signs against the raw encoder
    reading."""
    if not isinstance(telem, dict):
        return None
    motors = telem.get("motors")
    if not isinstance(motors, dict):
        return None
    try:
        values = [motors[j]["position"] for j in ARM_JOINTS]
        if any(isinstance(value, (str, bytes, bool)) for value in values):
            return None
        result = np.asarray(values, dtype=np.float32)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    # Missing joints must never masquerade as measured zero positions during
    # engagement or replay. Match the shared session's position validity.
    return result if result.shape == (len(ARM_JOINTS),) and np.isfinite(result).all() else None


def _numeric_motor_field(telem: Optional[dict], field: str) -> Optional[np.ndarray]:
    """Legacy display vectors retain NaNs for invalid/missing measurements."""
    if not isinstance(telem, dict) or not isinstance(telem.get("motors"), dict):
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    result = np.full(len(ARM_JOINTS), np.nan, dtype=np.float32)
    for index, name in enumerate(ARM_JOINTS):
        motor = motors.get(name)
        value = motor.get(field) if isinstance(motor, dict) else None
        if isinstance(value, (str, bytes, bool)):
            continue
        try:
            parsed = float(value)
            if np.isfinite(parsed) and abs(parsed) <= np.finfo(np.float32).max:
                result[index] = parsed
        except (TypeError, ValueError, OverflowError):
            pass
    return result


def _qtorque(telem: Optional[dict]) -> Optional[np.ndarray]:
    return _numeric_motor_field(telem, "torque")


def _qtemp(telem: Optional[dict]) -> Optional[np.ndarray]:
    return _numeric_motor_field(telem, "temperature")
