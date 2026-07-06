"""Motor <-> URDF frame alignment (from collect_demo.py)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .runtime import NUM_JOINTS

# ---------------------------------------------------------------------------
# Motor ↔ URDF frame alignment
# ---------------------------------------------------------------------------
# `signs` is a real frame-direction correction — some motor encoders rotate
# opposite to the URDF joint axis (mounting / cabling artefact), and the
# control loop MUST account for that or commands move the wrong way.
#
# `offsets` is a PURELY VISUAL knob for the URDF mirror — used when the
# motor's mechanical zero doesn't visually match the URDF neutral pose,
# but the IK / control loop is otherwise correct.  Applying it in the
# boundary would actively offset the motor command and physically shove
# the joint — which is what just happened to gantry_end.  Offsets are
# forwarded to the browser via telemetry; scene.js / preview.html add
# them only when rendering the mesh.
#
# Boundary math (in collect_demo) is sign-only:
#     q_urdf_ctrl  = q_motor * sign        # both in URDF *control* frame
#     q_motor      = q_urdf_ctrl * sign    # sign² = 1
#     v_urdf       = v_motor * sign
#     τ_urdf       = τ_motor * sign
#
# Loaded from config/joint_align.json at startup; hot-reloaded via mtime.
_ALIGN_OFFSETS = np.zeros(NUM_JOINTS, dtype=np.float32)
_ALIGN_SIGNS   = np.ones(NUM_JOINTS,  dtype=np.float32)
_ALIGN_PATH    = Path(__file__).resolve().parents[3] / "config" / "joint_align.json"
_ALIGN_MTIME   = 0.0   # last mtime we loaded; reload-if-changed gate

def _load_joint_align() -> None:
    """Populate the module-level _ALIGN_* arrays from joint_align.json."""
    global _ALIGN_OFFSETS, _ALIGN_SIGNS, _ALIGN_MTIME
    if not _ALIGN_PATH.exists():
        return
    try:
        _ALIGN_MTIME = _ALIGN_PATH.stat().st_mtime
        data = json.loads(_ALIGN_PATH.read_text())
    except Exception as e:
        print(f"WARNING: joint_align.json parse failed: {e}", flush=True)
        return
    o = data.get("offsets")
    if isinstance(o, list) and len(o) >= NUM_JOINTS:
        _ALIGN_OFFSETS = np.asarray(o[:NUM_JOINTS], dtype=np.float32)
    s = data.get("signs")
    if isinstance(s, list) and len(s) >= NUM_JOINTS:
        _ALIGN_SIGNS = np.asarray(
            [1.0 if float(x) >= 0 else -1.0 for x in s[:NUM_JOINTS]],
            dtype=np.float32,
        )

def _maybe_reload_joint_align() -> bool:
    """Reload joint_align.json if its mtime has changed since last load.

    Cheap to call (one stat per invocation); intended to be hit from the
    main loop a few times per second so /preview Save edits take effect
    without restarting collect_demo.  Returns True if reloaded."""
    try:
        m = _ALIGN_PATH.stat().st_mtime if _ALIGN_PATH.exists() else 0.0
    except OSError:
        return False
    if m == _ALIGN_MTIME:
        return False
    _load_joint_align()
    print(f"[joint_align] reloaded: signs={_ALIGN_SIGNS.tolist()} "
          f"offsets={[round(float(x), 3) for x in _ALIGN_OFFSETS]}", flush=True)
    return True

def _push_visual_offsets_to_leader(leader) -> None:
    """If the active leader exposes set_visual_offsets (QuestLeader does),
    hand it the current _ALIGN_OFFSETS so its IK/FK operate in visual
    frame (= mesh frame).  Physical leaders ignore this."""
    fn = getattr(leader, "set_visual_offsets", None)
    if fn is not None:
        try:
            fn(_ALIGN_OFFSETS)
        except Exception as e:
            print(f"[joint_align] leader.set_visual_offsets failed: {e}", flush=True)

def _motor_to_urdf_pos(q_motor: np.ndarray) -> np.ndarray:
    return np.asarray(q_motor, dtype=np.float32) * _ALIGN_SIGNS

def _urdf_to_motor_pos(q_urdf: np.ndarray) -> np.ndarray:
    # signs are ±1 → 1/sign == sign
    return np.asarray(q_urdf, dtype=np.float32) * _ALIGN_SIGNS

def _flip_sign_vec(v: np.ndarray) -> np.ndarray:
    return np.asarray(v, dtype=np.float32) * _ALIGN_SIGNS

def _urdf_to_motor_arm_payload(arm: dict) -> dict:
    """Transform an arm_joints dict from URDF frame to motor frame.

    Returns a new dict — the input is not mutated (other callers may
    still hold a reference to it, e.g. holder['last_q_cmd'])."""
    if not arm or "positions" not in arm:
        return arm
    out = dict(arm)
    pos = np.asarray(arm["positions"], dtype=np.float32)
    out["positions"] = _urdf_to_motor_pos(pos).tolist()
    if "velocities" in arm and arm["velocities"] is not None:
        vel = np.asarray(arm["velocities"], dtype=np.float32)
        out["velocities"] = _flip_sign_vec(vel).tolist()
    if "torques" in arm and arm["torques"] is not None:
        tq = np.asarray(arm["torques"], dtype=np.float32)
        out["torques"] = _flip_sign_vec(tq).tolist()
    return out

_SAT_TORQUE = {
    "swivel":      12.0,   # RS03 nominal
    "gantry_base": 24.0,   # RS04 nominal
    "gantry_mid":  12.0,   # RS03 nominal
    "gantry_end":   5.0,   # RS02 nominal
    "wrist_pitch":  5.0,   # RS02 nominal
    "wrist_roll":   0.5,   # RS00 nominal
    "gripper":      0.5,   # RS00 nominal
}
