"""telem.py — extract Minerva joint state from a telemetry message.

Mirrors AIZEE's collect_demo_app/telem.py (`_qpos`/`_qtorque`) but over the
17-DoF MINERVA_JOINTS vocabulary. Accepts either the AIZEE-style
`telem["motors"][joint] = {position, torque, temperature, ...}` schema, a flat
`telem["joints"] = {joint: value}` dict, or a `telem["positions"]` list already
in canonical order. Returns None when the required joints aren't all present so
the caller skips the tick (never zero-fills a safety-critical joint).

Adapt the exact field names to Minerva's real motor_control telemetry.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from common.minerva_constants import MINERVA_JOINTS, NUM_MINERVA_JOINTS, build_qpos

_N = NUM_MINERVA_JOINTS


def _from_motors(telem: dict, field: str) -> Optional[np.ndarray]:
    motors = telem.get("motors")
    if not isinstance(motors, dict):
        return None
    if not all(j in motors and isinstance(motors[j], dict) for j in MINERVA_JOINTS):
        return None
    try:
        return np.array(
            [float(motors[j].get(field, 0.0)) for j in MINERVA_JOINTS], dtype=np.float32)
    except (TypeError, ValueError):
        return None


def extract_qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem:
        return None
    v = _from_motors(telem, "position")
    if v is not None:
        return v
    joints = telem.get("joints")
    if isinstance(joints, dict) and all(j in joints for j in MINERVA_JOINTS):
        return build_qpos(joints)
    pos = telem.get("positions")
    if isinstance(pos, (list, tuple)) and len(pos) == _N:
        return np.asarray(pos, dtype=np.float32)
    return None


def extract_torques(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem:
        return None
    v = _from_motors(telem, "torque")
    if v is not None:
        return v
    for key in ("torques", "efforts"):
        arr = telem.get(key)
        if isinstance(arr, (list, tuple)) and len(arr) == _N:
            return np.asarray(arr, dtype=np.float32)
    tq = telem.get("torques")
    if isinstance(tq, dict) and all(j in tq for j in MINERVA_JOINTS):
        return build_qpos(tq)
    return None


__all__ = ["extract_qpos", "extract_torques"]
