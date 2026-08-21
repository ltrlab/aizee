"""
minerva_constants.py — canonical joint layout, gains, and limits for the
**Minerva** bimanual humanoid torso.

Mirrors ``aizee/python/common/arm_constants.py`` (the single-arm AIZEE robot)
but for Minerva's 17-DoF bimanual morphology. Minerva reuses AIZEE actuators
in a humanoid-torso form: two 6-DoF arms + 2 grippers + a 2-DoF head/neck
(pan/tilt) carrying an Intel RealSense + a 1-DoF linear lift (desk-leg), on the
AIZEE rover. The rover base is positioned manually and is **not** part of the
policy action space.

Canonical action / qpos ordering (grippers adjacent to their arm, so a single
shared decoder can attend across the whole vector and capture cross-arm
coordination — the ALOHA/RDT bimanual convention):

    index  joint            group          notes
    0..5   left_arm_j1..j6  left arm       6-DoF
    6      left_gripper     left gripper   1-DoF, continuous [closed..open]
    7..12  right_arm_j1..j6 right arm      6-DoF
    13     right_gripper    right gripper  1-DoF
    14     head_pan         head/neck      2-DoF
    15     head_tilt        head/neck
    16     lift             linear lift    1-DoF prismatic (metres)

NOTE: the per-joint limits/gains below are seeded from AIZEE's single arm
(``config/joint_limits.yaml`` + ``config/teleop.yaml``) duplicated per arm, with
placeholder head/lift values. Re-derive them from Minerva's own URDF before
trusting them on hardware — they are marked PLACEHOLDER where guessed.
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Joint layout
# ---------------------------------------------------------------------------

LEFT_ARM_JOINTS: List[str] = [
    "left_arm_j1", "left_arm_j2", "left_arm_j3",
    "left_arm_j4", "left_arm_j5", "left_arm_j6",
]
RIGHT_ARM_JOINTS: List[str] = [
    "right_arm_j1", "right_arm_j2", "right_arm_j3",
    "right_arm_j4", "right_arm_j5", "right_arm_j6",
]
HEAD_JOINTS: List[str] = ["head_pan", "head_tilt"]
LIFT_JOINTS: List[str] = ["lift"]

# Full canonical order — this IS the policy action/qpos ordering.
MINERVA_JOINTS: List[str] = (
    LEFT_ARM_JOINTS + ["left_gripper"]
    + RIGHT_ARM_JOINTS + ["right_gripper"]
    + HEAD_JOINTS + LIFT_JOINTS
)
NUM_MINERVA_JOINTS: int = len(MINERVA_JOINTS)          # 17
assert NUM_MINERVA_JOINTS == 17, MINERVA_JOINTS

# Index slices into the 17-vector, by kinematic group. Handy for per-group
# safety limits, gains, gripper handling, and logging.
IDX: Dict[str, slice] = {
    "left_arm":     slice(0, 6),
    "left_gripper": slice(6, 7),
    "right_arm":    slice(7, 13),
    "right_gripper": slice(13, 14),
    "head":         slice(14, 16),
    "lift":         slice(16, 17),
}
GRIPPER_INDICES: Tuple[int, int] = (6, 13)
ARM_INDICES: List[int] = list(range(0, 6)) + list(range(7, 13))
HEAD_INDICES: List[int] = [14, 15]
LIFT_INDEX: int = 16

# ---------------------------------------------------------------------------
# Effective joint limits  [lower, upper]  (radians; lift in metres)
# Per-arm 6-DoF values reuse AIZEE's effective limits from joint_limits.yaml
# (swivel, gantry_base, gantry_mid, gantry_end, wrist_pitch, wrist_swivel).
# ---------------------------------------------------------------------------

_ARM_LIMITS_6DOF: List[Tuple[float, float]] = [
    (-3.141593,  3.141593),   # j1  (was: swivel)
    (-1.750000,  0.000000),   # j2  (was: gantry_base)
    ( 0.000000,  3.141593),   # j3  (was: gantry_mid)
    (-1.000000,  2.199115),   # j4  (was: gantry_end)
    (-0.785398,  0.785398),   # j5  (was: wrist_pitch)
    (-1.000000,  1.000000),   # j6  (was: wrist_swivel)
]
_GRIPPER_LIMITS: Tuple[float, float] = (0.0, 1.0)          # normalized closed..open
_HEAD_LIMITS: List[Tuple[float, float]] = [
    (-1.570796, 1.570796),    # head_pan   PLACEHOLDER
    (-0.785398, 0.785398),    # head_tilt  PLACEHOLDER
]
_LIFT_LIMITS: Tuple[float, float] = (0.0, 0.30)           # metres  PLACEHOLDER

JOINT_LIMITS: np.ndarray = np.array(
    _ARM_LIMITS_6DOF + [_GRIPPER_LIMITS]
    + _ARM_LIMITS_6DOF + [_GRIPPER_LIMITS]
    + _HEAD_LIMITS + [_LIFT_LIMITS],
    dtype=np.float32,
)                                                          # [17, 2]
assert JOINT_LIMITS.shape == (NUM_MINERVA_JOINTS, 2)

# ---------------------------------------------------------------------------
# Default position-control gains (MIT-mode kp/kd), swivel-analogue first.
# Per-arm gains reuse AIZEE arm.kp/kd (config/teleop.yaml) minus the gripper.
# Head/lift gains are PLACEHOLDER — tune on hardware.
# ---------------------------------------------------------------------------

_ARM_KP_6DOF = [250.0, 220.0, 215.0, 50.0, 8.0, 3.0]
_ARM_KD_6DOF = [22.0,  15.0,  17.0,  4.0,  1.0, 1.0]
_GRIPPER_KP, _GRIPPER_KD = 2.0, 1.0
_HEAD_KP, _HEAD_KD = [8.0, 8.0], [1.0, 1.0]                # PLACEHOLDER
_LIFT_KP, _LIFT_KD = 80.0, 5.0                            # PLACEHOLDER (holds torso weight)

KP: List[float] = (
    _ARM_KP_6DOF + [_GRIPPER_KP]
    + _ARM_KP_6DOF + [_GRIPPER_KP]
    + _HEAD_KP + [_LIFT_KP]
)
KD: List[float] = (
    _ARM_KD_6DOF + [_GRIPPER_KD]
    + _ARM_KD_6DOF + [_GRIPPER_KD]
    + _HEAD_KD + [_LIFT_KD]
)
assert len(KP) == len(KD) == NUM_MINERVA_JOINTS

# Nominal per-joint saturation torque (Nm) — the conservative "safe" torque per
# actuator model (NOT the hard max). Used to bound the engage/tracking LEAD so PD
# torque (kp·lead) stays within a joint's safe range. Mirrors collect_demo's
# _SAT_TORQUE table, duplicated per arm:
#   j1 RS03=12 · j2 RS04=24 · j3 RS03=12 · j4 RS02=5 · j5 RS02=5 · j6 RS00=0.5 · grip RS00=0.5
_ARM_SAT_6DOF = [12.0, 24.0, 12.0, 5.0, 5.0, 0.5]
_GRIPPER_SAT = 0.5
_HEAD_SAT = [5.0, 5.0]     # PLACEHOLDER
_LIFT_SAT = 40.0           # PLACEHOLDER (holds torso weight)
SAT_TORQUE: np.ndarray = np.array(
    _ARM_SAT_6DOF + [_GRIPPER_SAT]
    + _ARM_SAT_6DOF + [_GRIPPER_SAT]
    + _HEAD_SAT + [_LIFT_SAT],
    dtype=np.float32,
)
assert SAT_TORQUE.shape == (NUM_MINERVA_JOINTS,)

# Per-step velocity guard (rad/step or m/step) for the deploy-time safety clamp.
# Arms/grippers move fast; head + lift are deliberately slower.
MAX_DELTA_ARM: float = 0.30       # rad/step  (~6 rad/s at 20 Hz)
MAX_DELTA_GRIPPER: float = 0.50   # normalized/step
MAX_DELTA_HEAD: float = 0.15      # rad/step
MAX_DELTA_LIFT: float = 0.01      # m/step    (slow prismatic)

RECORD_HZ: int = 20               # control / record rate

# Camera stream identities feeding the policy (order matters — used for the
# learned camera-identity embedding in minerva_model.py).
CAMERAS: List[str] = ["left_wrist", "right_wrist", "head"]
WRIST_CAMERAS: List[str] = ["left_wrist", "right_wrist"]
SCENE_CAMERA: str = "head"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def max_delta_vector(arm: float = MAX_DELTA_ARM, gripper: float = MAX_DELTA_GRIPPER,
                     head: float = MAX_DELTA_HEAD, lift: float = MAX_DELTA_LIFT) -> np.ndarray:
    """Per-joint per-step delta cap [17] for the velocity guard. Defaults to the
    module constants; pass overrides (e.g. from config/minerva.yaml `safety:`)."""
    v = np.full(NUM_MINERVA_JOINTS, arm, dtype=np.float32)
    for i in GRIPPER_INDICES:
        v[i] = gripper
    for i in HEAD_INDICES:
        v[i] = head
    v[LIFT_INDEX] = lift
    return v


def lead_cap_vector(kp: Sequence[float], max_delta: np.ndarray) -> np.ndarray:
    """Per-joint LEAD cap [17] = min(max_delta, SAT_TORQUE / kp): the largest
    command-minus-actual error whose PD torque (kp·lead) stays within the joint's
    nominal saturation. Bounds engage + tracking torque — much tighter than the flat
    velocity guard on high-kp joints (0.30 rad on an RS03 at kp≈250 would demand
    ~22 Nm vs a safe ~12) — and auto-tightens as kp rises (e.g. kp_scale up)."""
    kp = np.asarray(kp, dtype=np.float32)
    md = np.asarray(max_delta, dtype=np.float32)
    torque_lead = np.where(kp > 1e-6, SAT_TORQUE / np.maximum(kp, 1e-6), md)
    return np.minimum(md, torque_lead).astype(np.float32)


def build_qpos(joint_values: Dict[str, float]) -> np.ndarray:
    """Assemble a 17-vector in canonical order from a {joint_name: value} dict.

    Missing joints default to 0.0 (with the expectation that the caller has
    validated telemetry freshness). Mirror the live telemetry field names onto
    ``MINERVA_JOINTS`` in the inference node.
    """
    return np.array(
        [float(joint_values.get(name, 0.0)) for name in MINERVA_JOINTS],
        dtype=np.float32,
    )


def clamp_positions(q: np.ndarray) -> np.ndarray:
    """Clamp a 17-vector to the effective joint limits."""
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]).astype(np.float32)


def apply_safety_limits(
    action: np.ndarray,
    qpos_raw: np.ndarray,
    *,
    dataset_action_min: Sequence[float] | None = None,
    dataset_action_max: Sequence[float] | None = None,
    max_delta: np.ndarray | None = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Two-layer deploy-time safety clamp (mirrors act_policy_node.apply_safety_limits).

    Layer 1 — clamp each joint to the training-data range (if provided) AND the
              physical joint limits.
    Layer 2 — per-joint per-step delta guard (velocity limit).

    Returns (clamped_action [17], delta_clamped [17] bool).
    """
    action = np.asarray(action, dtype=np.float32).copy()

    lo = JOINT_LIMITS[:, 0].copy()
    hi = JOINT_LIMITS[:, 1].copy()
    if dataset_action_min is not None:
        lo = np.maximum(lo, np.asarray(dataset_action_min, dtype=np.float32))
    if dataset_action_max is not None:
        hi = np.minimum(hi, np.asarray(dataset_action_max, dtype=np.float32))
    action = np.clip(action, lo, hi)

    md = max_delta if max_delta is not None else max_delta_vector()
    delta = action - qpos_raw
    delta_clamped = np.abs(delta) > md
    delta = np.clip(delta, -md, md)
    action = (qpos_raw + delta).astype(np.float32)
    return action, delta_clamped


__all__ = [
    "MINERVA_JOINTS", "NUM_MINERVA_JOINTS", "IDX", "GRIPPER_INDICES",
    "ARM_INDICES", "HEAD_INDICES", "LIFT_INDEX",
    "JOINT_LIMITS", "KP", "KD", "SAT_TORQUE", "RECORD_HZ",
    "CAMERAS", "WRIST_CAMERAS", "SCENE_CAMERA",
    "MAX_DELTA_ARM", "MAX_DELTA_GRIPPER", "MAX_DELTA_HEAD", "MAX_DELTA_LIFT",
    "max_delta_vector", "lead_cap_vector", "build_qpos", "clamp_positions",
    "apply_safety_limits",
]
