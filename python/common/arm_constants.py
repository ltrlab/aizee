"""Canonical arm constants + small shared helpers for the AIZEE 7-DoF arm.

Single source of truth for the joint vocabulary, PD gains, link lengths,
calibration-limit loading, telemetry field extraction, and the non-blocking
keyboard reader. Everything here used to live at the top of
python/scripts/record_replay.py and was imported from there by ~10 sibling
scripts; record_replay re-exports these names so its old import surface
still works.

Import as:  from common.arm_constants import ARM_JOINTS, KP, KD, ...
(scripts under python/scripts/ put python/ on sys.path already).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Joint vocabulary
# ---------------------------------------------------------------------------
# Swivel is the first arm joint.  Historically the firmware spoke a separate
# `Swivel` command and ARM_JOINTS was 6-DOF; that split has been removed and
# ARM_JOINTS now matches the 7-DOF representation the policy pipeline always
# used.  POLICY_JOINTS is kept as an alias to ease the migration of training
# code that still imports it; new code should prefer ARM_JOINTS.
ARM_JOINTS = [
    "swivel",
    "gantry_base",
    "gantry_mid",
    "gantry_end",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
]
NUM_ARM_JOINTS = len(ARM_JOINTS)            # 7

POLICY_JOINTS = ARM_JOINTS
NUM_POLICY_JOINTS = NUM_ARM_JOINTS

# Subset useful for code that still wants to address only the gantry joints
# (e.g. arm-only kinematics that doesn't include the swivel base).  Always
# the trailing slice — swivel is index 0.
GANTRY_JOINTS = ARM_JOINTS[1:]              # 6 — exclude swivel
NUM_GANTRY_JOINTS = len(GANTRY_JOINTS)

# Arm link lengths (metres) — from rerun_bridge.py
L0 = 0.5906   # base → mid
L1 = 0.5649   # mid → end
L2 = 0.100    # end → wrist_pitch pivot
L3 = 0.1063   # wrist_pitch pivot → wrist_roll pivot
L5 = 0.132    # wrist_roll pivot → gripper tip
ARM_MOUNT_Z = 0.200  # arm mount height above rover base frame

# Gains — config/teleop.yaml `arm` section, 7-element (swivel-first) lists.
KP = [150.0, 100.0, 100.0, 40.0, 7.0, 3.0, 3.0]
KD = [5.0,   7.0,   5.5,   4.0,  0.2, 1.0, 1.0]

# Backwards-compat aliases for callers that still treat swivel separately
# (e.g. live-replay HUD).  Prefer indexing KP/KD by joint going forward.
SWIVEL_KP = KP[0]
SWIVEL_KD = KD[0]

RECORD_HZ = 20

# This file lives at <repo>/python/common/; three parents up is the repo root.
ROBSTRIDE_CALIB_PATH = Path(__file__).parent.parent.parent / "config" / "robstride_calibration.json"


def load_arm_limits(path: Optional[Path] = None) -> Optional[dict[str, tuple[float, float]]]:
    """Load arm joint limits from robstride_calibration.json.

    Returns {joint_name: (min_rad, max_rad)} or None if the file is absent.
    """
    calib_path = Path(path) if path is not None else ROBSTRIDE_CALIB_PATH
    try:
        with open(calib_path) as f:
            calib = json.load(f)
        limits = {}
        for joint, data in calib.get("joints", {}).items():
            a, b = float(data["min_rad"]), float(data["max_rad"])
            limits[joint] = (min(a, b), max(a, b))
        return limits or None
    except FileNotFoundError:
        return None
    except Exception as exc:
        print(f"Warning: could not load arm limits from {calib_path}: {exc}", file=sys.stderr)
        return None


def clamp_arm_positions(
    positions: list,
    limits: dict[str, tuple[float, float]],
    joints: list[str] = ARM_JOINTS,
) -> list:
    """Clamp each arm joint position to its calibration [min_rad, max_rad]."""
    out = list(positions)
    for i, joint in enumerate(joints):
        if joint in limits and i < len(out):
            lo, hi = limits[joint]
            out[i] = max(lo, min(hi, float(out[i])))
    return out


# ---------------------------------------------------------------------------
# Telemetry extraction (originally from act_policy_node.py)
# ---------------------------------------------------------------------------

def drain_sub(sock) -> Optional[dict]:
    """Drain a ZMQ SUB socket (msgpack), return latest message or None."""
    import zmq

    from common.wire import unpack_msg

    latest = None
    while True:
        try:
            latest = unpack_msg(sock.recv(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def _extract_field(
    telem: Optional[dict], field: str, joints: list = ARM_JOINTS,
) -> Optional[np.ndarray]:
    if telem is None or "motors" not in telem:
        return None
    motors = telem["motors"]
    out = []
    for joint in joints:
        m = motors.get(joint)
        if m is None:
            return None
        out.append(float(m.get(field, 0.0)))
    return np.array(out, dtype=np.float32)


def extract_qpos(telem: dict) -> Optional[np.ndarray]:
    """Extract [7] float32 positions in ARM_JOINTS order (swivel first)."""
    return _extract_field(telem, "position")


def extract_velocities(telem: dict) -> Optional[np.ndarray]:
    """Extract [7] float32 velocities in ARM_JOINTS order."""
    return _extract_field(telem, "velocity")


# POLICY_JOINTS == ARM_JOINTS now; these are kept as aliases for callers in
# the training pipeline that still import the policy-prefixed names.
extract_policy_qpos = extract_qpos


def extract_policy_torques(telem: dict) -> Optional[np.ndarray]:
    """Extract [7] float32 torques in ARM_JOINTS order."""
    return _extract_field(telem, "torque")


def apply_safety_limits(
    action: np.ndarray,
    qpos_raw: np.ndarray,
    max_delta: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Delta-clamp action around current qpos.

    Returns:
        clamped_action: [J] float32, safe to send  (J = len(ARM_JOINTS))
        delta_clamped:  [J] bool,    True for joints where delta was binding
    """
    delta = action - qpos_raw
    delta_clamped = np.abs(delta) > max_delta
    delta_clipped = np.clip(delta, -max_delta, max_delta)
    action = qpos_raw + delta_clipped
    return action.astype(np.float32), delta_clamped


# ---------------------------------------------------------------------------
# Keyboard
# ---------------------------------------------------------------------------

def setup_keyboard():
    """Return a function that reads a single key without blocking (or None)."""
    if sys.platform == "win32":
        import msvcrt

        def _get_key():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch.upper() if hasattr(ch, "upper") else None
            return None
    else:
        import select
        import tty
        import termios

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        def _get_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                return ch.upper()
            return None

        import atexit
        atexit.register(lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old_settings))

    return _get_key
