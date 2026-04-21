#!/usr/bin/env python3
"""gravity_calibrate.py — Empirical gravity compensation calibration.

Determines actual link masses and center-of-mass positions from
motor torque measurements at multiple static poses.

Modes:
    --record-poses   Manually position the arm and record poses to JSON
    (default)        Run calibration using recorded or built-in poses

Calibration phases:
    Phase 1: Geometry validation with 1 kg load test (optional)
    Phase 2: Static torque data collection at multiple poses
    Phase 3: Least-squares solve for link parameters

Usage:
    # Step 1: Record safe poses (use teleop to position arm)
    python gravity_calibrate.py --record-poses

    # Step 2: Run calibration with recorded poses
    python gravity_calibrate.py --poses config/calibration_poses.json

    # Or run with built-in poses (skip pose recording)
    python gravity_calibrate.py --skip-load-test

Keys during operation:
    ENTER / SPACE  — advance to next step
    Q              — abort (disables motors)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
import zmq

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from record_replay import (
    ARM_JOINTS,
    KP,
    KD,
    setup_keyboard,
    load_arm_limits,
    clamp_arm_positions,
)
from control.gravity_comp import (
    ArmGravityModel,
    LinkParams,
    JointDef,
    _rotation_matrix,
    _axis_vector,
    _DEFAULT_CHAIN,
)

NUM_JOINTS = len(ARM_JOINTS)  # 6
LOOP_HZ = 30
SETTLE_TIME = 2.0        # seconds to wait for arm to settle
SETTLE_VEL = 0.01        # rad/s threshold for "settled"
COLLECT_SAMPLES = 60     # telemetry samples per pose (~1.2s at 50 Hz)
RAMP_DELTA = 0.02        # rad/step delta clamp for movements (~0.6 rad/s at 30Hz)
RAMP_HZ = 30             # movement command rate
GRAVITY = 9.81

# Built-in calibration poses (used when --poses is not specified).
# All with gantry_base=0, wrist_roll=0, gripper=0.
# [gantry_base, gantry_mid, gantry_end, wrist_pitch, wrist_roll, gripper]
DEFAULT_POSES = [
    {"name": "Horizontal reach (max torque)",
     "q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {"name": "29 deg upward lift",
     "q": [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]},
    {"name": "Elbow bent down",
     "q": [0.0, 0.0, -1.5, 0.0, 0.0, 0.0]},
    {"name": "Wrist pitched",
     "q": [0.0, 0.3, -1.0, -0.8, 0.0, 0.0]},
    {"name": "Wrist heavily pitched",
     "q": [0.0, 0.0, -1.0, -1.5, 0.0, 0.0]},
    {"name": "Deep elbow + slight wrist",
     "q": [0.0, 0.5, -2.0, 0.5, 0.0, 0.0]},
]


# ---------------------------------------------------------------------------
# Geometry matrix builder
# ---------------------------------------------------------------------------

def build_geometry_rows(chain: list[JointDef], q: np.ndarray, g: float = GRAVITY) -> np.ndarray:
    """Build geometry coefficient matrix for one pose.

    Returns [5 x 12] matrix (5 useful joints x 12 parameters).
    Columns are [m_0, P_0, m_1, P_1, ..., m_5, P_5] where P_j = m_j * com_x_j.

    Joint 0 (gantry_base, Z-axis) always has zero gravity torque and is skipped,
    giving 5 rows for joints 1..5.
    """
    n = len(chain)

    # Build cumulative rotation matrices
    rotations: list[np.ndarray] = []
    R = np.eye(3)
    for i, jd in enumerate(chain):
        R = R @ _rotation_matrix(jd.axis, float(q[i]))
        rotations.append(R.copy())

    g_world = np.array([0.0, 0.0, -g])

    # Useful joints: 1..5 (skip gantry_base at index 0)
    useful = list(range(1, n))
    A = np.zeros((len(useful), 2 * n))

    for row, i in enumerate(useful):
        axis_local = _axis_vector(chain[i].axis)
        axis_world = rotations[i] @ axis_local

        for j in range(i, n):
            # Chain traversal: position from joint i to start of link j
            r_chain = np.zeros(3)
            for k in range(i, j):
                link_vec = np.array([chain[k].link.length, 0.0, 0.0])
                r_chain += rotations[k] @ link_vec

            # Direction of com_x offset in world frame
            d_j = rotations[j] @ np.array([1.0, 0.0, 0.0])

            # Geometry coefficients
            C_ij = np.dot(axis_world, np.cross(r_chain, g_world))
            D_ij = np.dot(axis_world, np.cross(d_j, g_world))

            # tau_measured = -sum [m_j * C_ij + P_j * D_ij]
            A[row, 2 * j] = -C_ij
            A[row, 2 * j + 1] = -D_ij

    return A


def build_full_system(
    chain: list[JointDef],
    poses: list[np.ndarray],
    torques: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Build the full overdetermined linear system A @ theta = tau.

    Returns:
        A: [5K x 11] geometry matrix (P_5 column removed).
        b: [5K] measured torque vector (joints 1..5 only).
    """
    A_blocks = []
    b_blocks = []

    for q, tau in zip(poses, torques):
        A_pose = build_geometry_rows(chain, q)   # [5, 12]
        A_blocks.append(A_pose)
        b_blocks.append(tau[1:])                 # skip gantry_base (joint 0)

    A_full = np.vstack(A_blocks)                 # [5K, 12]
    b_full = np.concatenate(b_blocks)            # [5K]

    # Remove column 11 (P_5 = m_5 * com_x_5, always 0 since gripper com_x = 0)
    A_reduced = np.delete(A_full, 11, axis=1)    # [5K, 11]

    return A_reduced, b_full


def solve_parameters(
    A: np.ndarray, b: np.ndarray, chain: list[JointDef]
) -> dict:
    """Solve for link parameters via least squares.

    Parameters theta = [m_0, P_0, m_1, P_1, m_2, P_2, m_3, P_3, m_4, P_4, m_5]
    where P_j = m_j * com_x_j.

    Note on identifiability:
    - Link 0 (gantry_base, Z-axis): completely unobservable from joints 1..5.
      Uses default values.
    - Link 1 (gantry_mid): only P_1 = m_1 * com_x_1 is identifiable.
      We set m_1 to default and compute com_x_1 = P_1 / m_1.
    - Links 2..5: both m_j and P_j are identifiable.
    """
    theta, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

    # Unpack: theta = [m_0, P_0, m_1, P_1, m_2, P_2, m_3, P_3, m_4, P_4, m_5]
    masses = []
    com_xs = []
    for j in range(5):
        m_j = theta[2 * j]
        P_j = theta[2 * j + 1]

        if abs(m_j) < 0.01:
            # Mass unidentifiable -- use default mass, derive com_x from P_j
            m_default = chain[j].link.mass
            if m_default > 0.01:
                m_j = m_default
                com_x_j = P_j / m_j
            elif abs(P_j) > 1e-6:
                m_j = 1.0
                com_x_j = P_j
            else:
                m_j = chain[j].link.mass
                com_x_j = chain[j].link.com_x
        else:
            com_x_j = P_j / m_j

        masses.append(m_j)
        com_xs.append(com_x_j)

    # Link 5 (gripper): mass only, com_x = 0
    m_5 = theta[10]
    if abs(m_5) < 0.01:
        m_5 = chain[5].link.mass
    masses.append(m_5)
    com_xs.append(0.0)

    # Residuals and R-squared
    predicted = A @ theta
    ss_res = np.sum((b - predicted) ** 2)
    ss_tot = np.sum((b - np.mean(b)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0

    cond = sv[0] / sv[-1] if len(sv) > 0 and sv[-1] > 1e-12 else float("inf")

    # Physical validation (skip links 0-1 since they use defaults)
    all_valid = True
    warnings = []
    for j in range(2, 6):
        L_j = chain[j].link.length
        if masses[j] < 0:
            warnings.append(f"link {j} ({chain[j].name}): negative mass {masses[j]:.3f} kg")
            all_valid = False
        if j < 5 and L_j > 0 and (com_xs[j] < 0 or com_xs[j] > L_j):
            warnings.append(
                f"link {j} ({chain[j].name}): com_x={com_xs[j]:.4f} outside [0, {L_j:.4f}]"
            )
    warnings.append("link 0 (gantry_base): unobservable, using default values")
    warnings.append("link 1 (gantry_mid): only m*com_x product identifiable, m set to default")

    links = []
    for j in range(6):
        links.append({
            "name": chain[j].name,
            "mass": float(masses[j]),
            "com_x": float(com_xs[j]),
            "length": chain[j].link.length,
        })

    return {
        "links": links,
        "total_mass": float(sum(masses)),
        "r_squared": float(r_squared),
        "condition_number": float(cond),
        "rank": int(rank),
        "rms_residual_Nm": float(np.sqrt(ss_res / len(b))),
        "physically_valid": all_valid,
        "warnings": warnings,
    }


def solve_masses_only(
    chain: list[JointDef],
    poses: list[np.ndarray],
    torques: list[np.ndarray],
) -> dict:
    """Fallback: fix com_x = L/2 and solve for masses only (6 unknowns)."""
    A_blocks = []
    b_blocks = []

    for q, tau in zip(poses, torques):
        A_full = build_geometry_rows(chain, q)  # [5, 12]
        A_mass = np.zeros((5, 6))
        for j in range(6):
            half_L = chain[j].link.length / 2.0
            A_mass[:, j] = A_full[:, 2 * j] + A_full[:, 2 * j + 1] * half_L
        A_blocks.append(A_mass)
        b_blocks.append(tau[1:])

    A = np.vstack(A_blocks)
    b = np.concatenate(b_blocks)

    masses, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

    predicted = A @ masses
    ss_res = np.sum((b - predicted) ** 2)
    ss_tot = np.sum((b - np.mean(b)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0

    links = []
    for j in range(6):
        links.append({
            "name": chain[j].name,
            "mass": float(masses[j]),
            "com_x": chain[j].link.length / 2.0,
            "length": chain[j].link.length,
        })

    return {
        "links": links,
        "total_mass": float(sum(masses)),
        "r_squared": float(r_squared),
        "rms_residual_Nm": float(np.sqrt(ss_res / len(b))),
        "rank": int(rank),
        "physically_valid": all(m > 0 for m in masses),
        "fallback": True,
        "warnings": ["Fallback mode: com_x fixed at L/2"],
    }


# ---------------------------------------------------------------------------
# 1 kg load geometry validation
# ---------------------------------------------------------------------------

def predict_1kg_delta_torques(chain: list[JointDef]) -> np.ndarray:
    """Predict delta-tau from attaching 1 kg at gripper tip, arm horizontal (q=0)."""
    delta = np.zeros(6)
    for i in range(6):
        if chain[i].axis in ("Z", "X"):
            delta[i] = 0.0
        else:
            arm = sum(chain[k].link.length for k in range(i, 6))
            delta[i] = 1.0 * GRAVITY * arm
    return delta


def validate_geometry(
    delta_measured: np.ndarray, delta_predicted: np.ndarray
) -> tuple[bool, list[str]]:
    """Check if measured 1 kg delta-tau matches predicted within 15%."""
    ok = True
    msgs = []
    for i in [1, 2, 3]:  # gantry_mid, gantry_end, wrist_pitch
        pred = delta_predicted[i]
        meas = delta_measured[i]
        if abs(pred) < 0.1:
            continue
        rel_err = abs(meas - pred) / abs(pred)
        status = "OK" if rel_err < 0.15 else "FAIL"
        if rel_err >= 0.15:
            ok = False
        msgs.append(
            f"  {ARM_JOINTS[i]:<16} predicted={pred:+6.2f} Nm  "
            f"measured={meas:+6.2f} Nm  err={rel_err*100:.1f}%  [{status}]"
        )
    return ok, msgs


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def _drain(sock) -> Optional[dict]:
    latest = None
    while True:
        try:
            latest = json.loads(sock.recv_string(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def _send(sock, msg: dict) -> None:
    try:
        sock.send_string(json.dumps(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


def _extract_arm(telem: dict, field: str) -> Optional[np.ndarray]:
    """Extract [6] array from telemetry for given field."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    vals = []
    for j in ARM_JOINTS:
        m = motors.get(j)
        if m is None:
            return None
        vals.append(float(m.get(field, 0.0)))
    return np.array(vals, dtype=np.float64)


def _read_current_pos(telem_sock, retries: int = 20) -> Optional[np.ndarray]:
    """Read current arm position from telemetry."""
    for _ in range(retries):
        telem = _drain(telem_sock)
        if telem:
            pos = _extract_arm(telem, "position")
            if pos is not None:
                return pos
        time.sleep(0.05)
    return None


# ---------------------------------------------------------------------------
# Movement helpers
# ---------------------------------------------------------------------------

def _hold_position(cmd_sock, telem_sock, kp, kd, duration: float = 1.0) -> Optional[np.ndarray]:
    """Send hold-at-current-position commands for the given duration.

    Returns the current position after holding.
    """
    current = _read_current_pos(telem_sock)
    if current is None:
        return None

    t_end = time.time() + duration
    period = 1.0 / RAMP_HZ
    while time.time() < t_end:
        t0 = time.time()
        _send(cmd_sock, {
            "type": "arm_joints",
            "positions": current.tolist(),
            "velocities": [0.0] * NUM_JOINTS,
            "kp": kp,
            "kd": kd,
            "torques": [0.0] * NUM_JOINTS,
        })
        telem = _drain(telem_sock)
        if telem:
            pos = _extract_arm(telem, "position")
            if pos is not None:
                current = pos
        sleep_t = period - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)
    return current


def ramp_to_pose(
    cmd_sock,
    telem_sock,
    target: np.ndarray,
    arm_limits: Optional[dict],
    get_key,
    kp: list,
    kd: list,
    timeout: float = 30.0,
    hold_hint: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """Ramp arm to target pose with delta clamping.

    The commanded position ramps toward the target.  Once the command has
    reached the target, gravity sag may keep the actual position offset --
    that is expected (we measure actual positions, not commanded).  We
    detect steady-state by watching for the error to stop decreasing.

    Args:
        hold_hint: If provided, send hold commands at this position while
                   waiting for telemetry.  Prevents motor watchdog timeout
                   between poses.

    Returns final actual position, or None if aborted (Q key).
    """
    period = 1.0 / RAMP_HZ

    # Read current position while sending hold commands to keep watchdog alive
    current = None
    for _ in range(90):  # ~3 seconds
        # Send hold command (prevents watchdog from killing telemetry)
        hold_pos = hold_hint if current is None else current
        if hold_pos is not None:
            _send(cmd_sock, {
                "type": "arm_joints",
                "positions": hold_pos.tolist(),
                "velocities": [0.0] * NUM_JOINTS,
                "kp": kp, "kd": kd,
                "torques": [0.0] * NUM_JOINTS,
            })
        telem = _drain(telem_sock)
        if telem:
            pos = _extract_arm(telem, "position")
            if pos is not None:
                current = pos
                break
        time.sleep(0.03)

    if current is None:
        print("  ERROR: No telemetry — refusing to move (would jump)")
        return None

    cmd_pos = current.copy()
    t_start = time.time()
    prev_err = float("inf")
    stall_count = 0
    _ramp_lines = 0

    while time.time() - t_start < timeout:
        t0 = time.time()

        key = get_key()
        if key == "Q":
            return None

        # Ramp the COMMAND toward target
        delta = target - cmd_pos
        cmd_pos = cmd_pos + np.clip(delta, -RAMP_DELTA, RAMP_DELTA)

        if arm_limits:
            cmd_pos = np.array(clamp_arm_positions(cmd_pos.tolist(), arm_limits), dtype=np.float64)

        _send(cmd_sock, {
            "type": "arm_joints",
            "positions": cmd_pos.tolist(),
            "velocities": [0.0] * NUM_JOINTS,
            "kp": kp,
            "kd": kd,
            "torques": [0.0] * NUM_JOINTS,
        })

        telem = _drain(telem_sock)
        if telem:
            pos = _extract_arm(telem, "position")
            if pos is not None:
                current = pos

        err = float(np.max(np.abs(current - target)))

        # Command has reached target -- check if actual has settled
        cmd_reached = np.all(np.abs(cmd_pos - target) < 0.001)
        if cmd_reached:
            if err < 0.05:
                if _ramp_lines > 0:
                    sys.stdout.write(f"\033[{_ramp_lines}A")
                print(f"  Reached target: err={err:.4f} rad                        ")
                for _ in range(_ramp_lines - 1):
                    print(" " * 70)
                return current
            # Steady-state: error stopped improving (gravity sag)
            if abs(err - prev_err) < 0.002:
                stall_count += 1
            else:
                stall_count = 0
            if stall_count >= 15:  # ~0.5s of no improvement
                if _ramp_lines > 0:
                    sys.stdout.write(f"\033[{_ramp_lines}A")
                print(f"  Steady-state at err={err:.4f} rad (gravity sag)          ")
                for _ in range(_ramp_lines - 1):
                    print(" " * 70)
                return current

        prev_err = err

        # Live display with torques
        if _ramp_lines > 0:
            sys.stdout.write(f"\033[{_ramp_lines}A")
        _ramp_lines = 1 + NUM_JOINTS
        print(f"  Moving: max_err={err:.4f} rad                                ")
        if telem:
            motors = telem.get("motors", {})
            for j in ARM_JOINTS:
                m = motors.get(j, {})
                p = float(m.get("position", 0))
                t = float(m.get("torque", 0))
                print(f"    {j:<16} pos={p:+.4f}  torque={t:+7.2f} Nm")
        else:
            for j in ARM_JOINTS:
                print(f"    {j:<16} pos={'---':>7}  torque={'---':>7}")
        sys.stdout.flush()

        sleep_t = period - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)

    err = float(np.max(np.abs(current - target)))
    if _ramp_lines > 0:
        sys.stdout.write(f"\033[{_ramp_lines}A")
    print(f"  Timeout at err={err:.4f} rad -- proceeding                      ")
    for _ in range(max(0, _ramp_lines - 1)):
        print(" " * 70)
    return current


def wait_settled(
    cmd_sock,
    telem_sock,
    target: np.ndarray,
    get_key,
    kp: list,
    kd: list,
    timeout: float = SETTLE_TIME,
) -> bool:
    """Wait for arm to settle (velocity < threshold).

    Sends hold commands to prevent watchdog timeout.
    Displays live torque telemetry.
    Returns False if Q pressed.
    """
    t_start = time.time()
    settled_since = None
    _tlines = 0

    while time.time() - t_start < timeout + 2.0:
        key = get_key()
        if key == "Q":
            if _tlines:
                print()
            return False

        # Keep sending hold commands (prevents motor watchdog timeout)
        _send(cmd_sock, {
            "type": "arm_joints",
            "positions": target.tolist(),
            "velocities": [0.0] * NUM_JOINTS,
            "kp": kp, "kd": kd,
            "torques": [0.0] * NUM_JOINTS,
        })

        telem = _drain(telem_sock)
        if telem:
            vel = _extract_arm(telem, "velocity")
            if vel is not None:
                max_vel = float(np.max(np.abs(vel)))
                elapsed = time.time() - t_start
                tag = "settled" if max_vel < SETTLE_VEL else "SETTLING"

                # Live display (overwrite previous)
                if _tlines > 0:
                    sys.stdout.write(f"\033[{_tlines}A")
                _tlines = 1 + NUM_JOINTS
                print(f"  [{elapsed:.1f}s] max_vel={max_vel:.4f} — {tag}         ")
                motors = telem.get("motors", {})
                for j in ARM_JOINTS:
                    m = motors.get(j, {})
                    p = float(m.get("position", 0))
                    t = float(m.get("torque", 0))
                    v = float(m.get("velocity", 0))
                    tp = float(m.get("temperature", 0))
                    print(f"    {j:<16} pos={p:+.4f}  torque={t:+7.2f} Nm  vel={v:+.4f}  {tp:.0f}C")
                sys.stdout.flush()

                if max_vel < SETTLE_VEL:
                    if settled_since is None:
                        settled_since = time.time()
                    elif time.time() - settled_since >= timeout:
                        print()
                        return True
                else:
                    settled_since = None

        time.sleep(0.02)

    if _tlines:
        print()
    return True  # timeout -- proceed anyway


def collect_samples(
    cmd_sock,
    telem_sock,
    target: np.ndarray,
    get_key,
    kp: list,
    kd: list,
    n_samples: int = COLLECT_SAMPLES,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Collect n_samples of (position, torque) while holding position.

    Sends zero-feedforward arm_joints commands to maintain PD hold.
    Shows live torque telemetry during collection.
    Returns (mean_q, mean_tau) or (None, None) if aborted.
    """
    q_samples = []
    tau_samples = []
    _tlines = 0

    for _ in range(n_samples * 3):
        key = get_key()
        if key == "Q":
            if _tlines:
                print()
            return None, None

        _send(cmd_sock, {
            "type": "arm_joints",
            "positions": target.tolist(),
            "velocities": [0.0] * NUM_JOINTS,
            "kp": kp,
            "kd": kd,
            "torques": [0.0] * NUM_JOINTS,
        })

        telem = _drain(telem_sock)
        if telem:
            pos = _extract_arm(telem, "position")
            tau = _extract_arm(telem, "torque")
            if pos is not None and tau is not None:
                q_samples.append(pos)
                tau_samples.append(tau)

                # Live torque display (overwrite previous)
                if _tlines > 0:
                    sys.stdout.write(f"\033[{_tlines}A")
                _tlines = 1 + NUM_JOINTS
                print(f"  Collecting: {len(q_samples)}/{n_samples}                    ")
                motors = telem.get("motors", {})
                for j in ARM_JOINTS:
                    m = motors.get(j, {})
                    p = float(m.get("position", 0))
                    t = float(m.get("torque", 0))
                    tp = float(m.get("temperature", 0))
                    print(f"    {j:<16} pos={p:+.4f}  torque={t:+7.2f} Nm  temp={tp:.0f}C")
                sys.stdout.flush()

                if len(q_samples) >= n_samples:
                    break

        time.sleep(0.02)

    print()

    if len(q_samples) < 10:
        print(f"  WARNING: Only collected {len(q_samples)} samples")
        if len(q_samples) == 0:
            return None, None

    mean_q = np.mean(q_samples, axis=0)
    mean_tau = np.mean(tau_samples, axis=0)
    std_tau = np.std(tau_samples, axis=0)

    print(f"  ── Mean readings ──")
    for ji, jn in enumerate(ARM_JOINTS):
        print(f"    {jn:<16} q={mean_q[ji]:+.4f}  τ={mean_tau[ji]:+7.3f} ± {std_tau[ji]:.3f} Nm")

    return mean_q, mean_tau


# ---------------------------------------------------------------------------
# Pose I/O
# ---------------------------------------------------------------------------

def load_poses(path: Path) -> list[dict]:
    """Load calibration poses from JSON file."""
    data = json.loads(path.read_text())
    poses = data.get("poses", data)
    if isinstance(poses, list):
        return poses
    raise ValueError(f"Expected 'poses' list in {path}")


def save_poses(path: Path, poses: list[dict]) -> None:
    """Save calibration poses to JSON file."""
    out = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "poses": poses,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2))


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _load_teleop_yaml() -> dict:
    here = Path(__file__).parent
    for candidate in [
        here / ".." / ".." / "config" / "teleop.yaml",
        Path("config") / "teleop.yaml",
    ]:
        p = candidate.resolve()
        if p.exists():
            return yaml.safe_load(p.read_text()) or {}
    return {}


# ---------------------------------------------------------------------------
# Mode: record poses
# ---------------------------------------------------------------------------

def record_poses_mode(args) -> None:
    """Interactively record arm poses from telemetry.

    Shows live-updating joint positions and torques.  The user positions the
    arm (e.g. via teleop in another terminal, or by hand with motors disabled)
    and presses SPACE or ENTER to snapshot the current pose.
    """
    output_path = Path(args.output) if args.output else (
        Path(__file__).resolve().parent.parent.parent / "config" / "calibration_poses.json"
    )

    ctx = zmq.Context()
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.setsockopt(zmq.LINGER, 0)
    telem_sock.setsockopt(zmq.CONFLATE, 1)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sock.connect(args.telem)

    get_key = setup_keyboard()

    print("=" * 60)
    print("  AIZEE Gravity Calibration -- Record Poses")
    print("=" * 60)
    print()
    print("Position the arm, then press SPACE/ENTER to record.")
    print("Press Q to finish and save.  Minimum 6 poses recommended.")
    print()

    # Wait for telemetry
    print("Waiting for telemetry...", end="", flush=True)
    for _ in range(60):
        telem = _drain(telem_sock)
        if telem and _extract_arm(telem, "position") is not None:
            print(" OK")
            break
        time.sleep(0.1)
    else:
        print(" TIMEOUT")
        ctx.term()
        sys.exit(1)

    poses = []
    current_pos = None
    current_tau = None
    _tlines = 0

    try:
        while True:
            key = get_key()

            if key == "Q":
                # Clear live display
                if _tlines > 0:
                    sys.stdout.write(f"\033[{_tlines}A")
                    for _ in range(_tlines):
                        print(" " * 75)
                    sys.stdout.write(f"\033[{_tlines}A")
                break

            if key in (" ", "\r", "\n"):
                if current_pos is None:
                    continue
                # Snapshot pose
                idx = len(poses) + 1
                # Clear live display before prompting
                if _tlines > 0:
                    sys.stdout.write(f"\033[{_tlines}A")
                    for _ in range(_tlines):
                        print(" " * 75)
                    sys.stdout.write(f"\033[{_tlines}A")
                _tlines = 0

                print(f"  Pose {idx} captured: [{', '.join(f'{v:+.4f}' for v in current_pos)}]")
                if current_tau is not None:
                    print(f"  Torques:          [{', '.join(f'{v:+.2f}' for v in current_tau)}]")
                name = input(f"  Description (optional, ENTER to skip): ").strip()
                if not name:
                    name = f"Pose {idx}"
                poses.append({
                    "name": name,
                    "q": current_pos.tolist(),
                })
                print(f"  Saved as: {name}")
                print()
                continue

            # Poll telemetry and update live display
            telem = _drain(telem_sock)
            if telem:
                pos = _extract_arm(telem, "position")
                tau = _extract_arm(telem, "torque")
                if pos is not None:
                    current_pos = pos
                if tau is not None:
                    current_tau = tau

                # Live display (overwrite previous)
                if _tlines > 0:
                    sys.stdout.write(f"\033[{_tlines}A")
                _tlines = 2 + NUM_JOINTS
                idx = len(poses) + 1
                print(f"  ── Pose {idx} ── Move arm, then SPACE to record / Q to finish ──")
                print(f"  {'Joint':<16} {'Position':>9}  {'Torque':>9}  {'Temp':>5}")
                motors = telem.get("motors", {})
                for j in ARM_JOINTS:
                    m = motors.get(j, {})
                    p = float(m.get("position", 0))
                    t = float(m.get("torque", 0))
                    tp = float(m.get("temperature", 0))
                    print(f"  {j:<16} {p:>+9.4f}  {t:>+9.2f} Nm  {tp:>4.0f}C")
                sys.stdout.flush()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupted.")

    telem_sock.close()
    ctx.term()

    if not poses:
        print("No poses recorded.")
        return

    # Print summary
    print(f"\n{'=' * 60}")
    print(f"  Recorded {len(poses)} poses:")
    for i, p in enumerate(poses):
        q = p["q"]
        print(f"  {i+1}. {p['name']}: [{', '.join(f'{v:+.3f}' for v in q)}]")

    save_poses(output_path, poses)
    print(f"\nSaved to: {output_path}")

    print(f"\nTo run calibration with these poses:")
    print(f"  python {Path(__file__).name} --poses {output_path} --skip-load-test")


# ---------------------------------------------------------------------------
# Main calibration procedure
# ---------------------------------------------------------------------------

def calibrate_mode(args) -> None:
    """Run the full calibration procedure."""
    _yaml = _load_teleop_yaml()
    _tcfg = _yaml.get("gantry", {})
    _kp: list = _tcfg.get("kp", KP)[:NUM_JOINTS]
    _kd: list = _tcfg.get("kd", KD)[:NUM_JOINTS]

    output_path = Path(args.output) if args.output else (
        Path(__file__).resolve().parent.parent.parent / "config" / "gravity_calibration.json"
    )
    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)

    # Load poses
    if args.poses:
        pose_list = load_poses(Path(args.poses))
        print(f"Loaded {len(pose_list)} poses from {args.poses}")
    else:
        pose_list = DEFAULT_POSES
        print(f"Using {len(pose_list)} built-in poses")

    chain = list(_DEFAULT_CHAIN)

    print("=" * 60)
    print("  AIZEE Gravity Compensation Calibration")
    print("=" * 60)
    print()
    print("Arm model:")
    for jd in chain:
        print(f"  {jd.name:<16} axis={jd.axis}  L={jd.link.length:.4f} m")
    print()
    print("Calibration poses:")
    for i, p in enumerate(pose_list):
        q = p["q"]
        print(f"  {i+1}. {p['name']}: [{', '.join(f'{v:+.3f}' for v in q)}]")
    print()

    # ZMQ setup
    ctx = zmq.Context()
    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 2)
    cmd_sock.setsockopt(zmq.LINGER, 0)

    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.setsockopt(zmq.LINGER, 0)
    telem_sock.setsockopt(zmq.CONFLATE, 1)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")

    if not args.dry_run:
        cmd_sock.connect(args.cmd)
        telem_sock.connect(args.telem)

    get_key = setup_keyboard()

    # Wait for telemetry
    if not args.dry_run:
        print("Waiting for telemetry...", end="", flush=True)
        for _ in range(60):
            telem = _drain(telem_sock)
            if telem and _extract_arm(telem, "position") is not None:
                print(" OK")
                break
            time.sleep(0.1)
        else:
            print(" TIMEOUT -- no telemetry received")
            ctx.term()
            sys.exit(1)

    # Temperature check
    telem = _drain(telem_sock)
    if telem:
        temps = _extract_arm(telem, "temperature")
        if temps is not None:
            max_temp = float(np.max(temps))
            if max_temp > 65:
                print(f"WARNING: Max motor temperature is {max_temp:.0f} C")

    print("\nEnsure the workspace around the arm is CLEAR.")
    print("The arm will move through multiple poses.")
    input("Press ENTER to continue (or Ctrl+C to abort)...")

    aborted = False

    try:
        if not args.dry_run:
            print("\nEnabling arm motors...")
            _send(cmd_sock, {"type": "enable", "motor_ids": ARM_JOINTS})
            time.sleep(1.0)

            # Send hold-at-current commands to let motors stabilise
            print("Holding current position...")
            _hold_position(cmd_sock, telem_sock, _kp, _kd, duration=1.5)

        # -------------------------------------------------------------------
        # Phase 1: 1 kg load geometry validation
        # -------------------------------------------------------------------
        load_test_data = None
        if not args.skip_load_test:
            print("\n" + "=" * 60)
            print("  Phase 1: Geometry Validation (1 kg load test)")
            print("=" * 60)

            print("\n[1/5] Moving to horizontal pose (q = 0)...")
            q_horizontal = np.zeros(NUM_JOINTS)
            actual = ramp_to_pose(cmd_sock, telem_sock, q_horizontal, arm_limits, get_key, _kp, _kd)
            if actual is None:
                aborted = True
                return

            print("\n[2/5] Settling and collecting UNLOADED torques...")
            wait_settled(cmd_sock, telem_sock, q_horizontal, get_key, _kp, _kd)
            q_unloaded, tau_unloaded = collect_samples(
                cmd_sock, telem_sock, q_horizontal, get_key, _kp, _kd
            )
            if q_unloaded is None:
                aborted = True
                return

            print("\n[3/5] Attach 1 kg weight at gripper tip.")
            input("Press ENTER when weight is attached...")

            print("\n[4/5] Settling and collecting LOADED torques...")
            wait_settled(cmd_sock, telem_sock, q_horizontal, get_key, _kp, _kd)
            q_loaded, tau_loaded = collect_samples(
                cmd_sock, telem_sock, q_horizontal, get_key, _kp, _kd
            )
            if q_loaded is None:
                aborted = True
                return

            delta_measured = tau_loaded - tau_unloaded
            delta_predicted = predict_1kg_delta_torques(chain)

            print("\n[5/5] Geometry validation results:")
            print(f"  {'Joint':<16} {'Predicted':>10} {'Measured':>10} {'Error':>8}")
            print(f"  {'-'*16} {'-'*10} {'-'*10} {'-'*8}")

            geom_ok, geom_msgs = validate_geometry(delta_measured, delta_predicted)
            for msg in geom_msgs:
                print(msg)

            if geom_ok:
                print("\n  Geometry VALIDATED")
            else:
                print("\n  WARNING: Geometry validation FAILED")
                resp = input("  Continue anyway? [y/N] ")
                if resp.lower() != "y":
                    aborted = True
                    return

            print("\nRemove the 1 kg weight from the gripper.")
            input("Press ENTER when weight is removed...")

            load_test_data = {
                "q_unloaded": q_unloaded.tolist(),
                "tau_unloaded": tau_unloaded.tolist(),
                "q_loaded": q_loaded.tolist(),
                "tau_loaded": tau_loaded.tolist(),
                "delta_measured": delta_measured.tolist(),
                "delta_predicted": delta_predicted.tolist(),
                "geometry_valid": geom_ok,
            }

        # -------------------------------------------------------------------
        # Phase 2: Static pose data collection
        # -------------------------------------------------------------------
        print("\n" + "=" * 60)
        print("  Phase 2: Static Pose Data Collection")
        print("=" * 60)

        collected_poses: list[np.ndarray] = []
        collected_torques: list[np.ndarray] = []
        raw_pose_data = []
        prev_target: Optional[np.ndarray] = None

        for idx, pose_def in enumerate(pose_list):
            target = np.array(pose_def["q"], dtype=np.float64)
            name = pose_def["name"]

            print(f"\n--- Pose {idx + 1}/{len(pose_list)}: {name} ---")
            print(f"  Target: [{', '.join(f'{v:+.3f}' for v in target)}]")

            actual = ramp_to_pose(
                cmd_sock, telem_sock, target, arm_limits, get_key, _kp, _kd,
                hold_hint=prev_target,
            )
            if actual is None:
                aborted = True
                return
            print()

            print("  Settling...")
            wait_settled(cmd_sock, telem_sock, target, get_key, _kp, _kd)

            mean_q, mean_tau = collect_samples(
                cmd_sock, telem_sock, target, get_key, _kp, _kd
            )
            if mean_q is None:
                aborted = True
                return

            prev_target = target
            collected_poses.append(mean_q)
            collected_torques.append(mean_tau)
            raw_pose_data.append({
                "name": name,
                "target": target.tolist(),
                "q_actual": mean_q.tolist(),
                "tau_mean": mean_tau.tolist(),
            })

        # -------------------------------------------------------------------
        # Phase 3: Solve
        # -------------------------------------------------------------------
        print("\n" + "=" * 60)
        print("  Phase 3: Least-Squares Solve")
        print("=" * 60)

        A, b = build_full_system(chain, collected_poses, collected_torques)
        result = solve_parameters(A, b, chain)

        print(f"\n  R-squared = {result['r_squared']:.6f}")
        print(f"  RMS residual = {result['rms_residual_Nm']:.4f} Nm")
        print(f"  Condition number = {result['condition_number']:.1f}")
        print(f"  Rank = {result['rank']}")
        print(f"  Total mass = {result['total_mass']:.3f} kg")
        print(f"  Physically valid: {result['physically_valid']}")

        if result["warnings"]:
            print("\n  Warnings:")
            for w in result["warnings"]:
                print(f"    - {w}")

        if not result["physically_valid"]:
            print("\n  Trying fallback model (com_x = L/2, masses only)...")
            result_fb = solve_masses_only(chain, collected_poses, collected_torques)
            print(f"  Fallback R-squared = {result_fb['r_squared']:.6f}")
            print(f"  Fallback total mass = {result_fb['total_mass']:.3f} kg")
            if result_fb["physically_valid"]:
                print("  Fallback is physically valid -- using fallback")
                result = result_fb
            else:
                print("  WARNING: Fallback also invalid. Using full model anyway.")

        print(f"\n  {'Link':<16} {'Mass (kg)':>10} {'CoM_x (m)':>10} {'Length (m)':>10}")
        print(f"  {'-'*16} {'-'*10} {'-'*10} {'-'*10}")
        for lk in result["links"]:
            print(f"  {lk['name']:<16} {lk['mass']:10.3f} {lk['com_x']:10.4f} {lk['length']:10.4f}")

        # -------------------------------------------------------------------
        # Return arm to zero and disable
        # -------------------------------------------------------------------
        print("\nReturning arm to zero...")
        ramp_to_pose(cmd_sock, telem_sock, np.zeros(NUM_JOINTS), arm_limits, get_key, _kp, _kd)

        if not args.dry_run:
            print("Disabling motors...")
            _send(cmd_sock, {"type": "disable", "motor_ids": ARM_JOINTS})
            time.sleep(0.2)

        # -------------------------------------------------------------------
        # Save results
        # -------------------------------------------------------------------
        output = {
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "result": result,
            "raw_data": {
                "poses": raw_pose_data,
                "load_test": load_test_data,
            },
            "config": {
                "kp": _kp,
                "kd": _kd,
                "settle_time": SETTLE_TIME,
                "collect_samples": COLLECT_SAMPLES,
                "gravity": GRAVITY,
            },
        }

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(output, indent=2))
        print(f"\nResults saved to: {output_path}")

        # Python snippet
        print("\n" + "=" * 60)
        print("  Python snippet for gravity_comp.py _DEFAULT_CHAIN:")
        print("=" * 60)
        print("_DEFAULT_CHAIN = [")
        for lk in result["links"]:
            axis = next(jd.axis for jd in chain if jd.name == lk["name"])
            print(
                f'    JointDef("{lk["name"]:<16}", "{axis}", '
                f'LinkParams(length={lk["length"]:.4f}, mass={lk["mass"]:.3f}, com_x={lk["com_x"]:.4f})),'
            )
        print("]")

        print("\nOr load automatically:")
        print(f'  model = ArmGravityModel.from_calibration("{output_path.name}")')

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        aborted = True
    finally:
        if not args.dry_run:
            if aborted:
                print("Disabling motors...")
                _send(cmd_sock, {"type": "disable", "motor_ids": ARM_JOINTS})
                time.sleep(0.2)
            cmd_sock.close()
            telem_sock.close()
        ctx.term()
        if aborted:
            print("Calibration aborted.")
        else:
            print("Done.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    _ep = _load_teleop_yaml().get("endpoints", {})
    ap = argparse.ArgumentParser(
        description="Gravity compensation calibration for AIZEE arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--cmd", default=_ep.get("command", "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem", default=_ep.get("telemetry", "tcp://192.168.0.27:5556"))
    ap.add_argument("--output", default=None,
                    help="Output JSON path")
    ap.add_argument("--poses", default=None,
                    help="Load calibration poses from JSON (from --record-poses)")
    ap.add_argument("--record-poses", action="store_true", dest="record_poses",
                    help="Interactive mode: manually position arm and record poses")
    ap.add_argument("--skip-load-test", action="store_true", dest="skip_load_test",
                    help="Skip Phase 1 (1 kg load test)")
    ap.add_argument("--robstride-calib", default=None, dest="robstride_calib")
    ap.add_argument("--dry-run", action="store_true", dest="dry_run",
                    help="Simulate without hardware (for testing)")
    args = ap.parse_args()

    if args.record_poses:
        record_poses_mode(args)
    else:
        calibrate_mode(args)


if __name__ == "__main__":
    main()
