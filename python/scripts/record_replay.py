#!/usr/bin/env python3
"""record_replay.py — Arm Trajectory Record, Visualize, and Replay

Usage:
    python record_replay.py record    [options]
    python record_replay.py visualize <file.hdf5> [options]
    python record_replay.py replay    <file.hdf5> [options]

The visualize and replay subcommands accept both recordings/recording_XXXX.hdf5
and episode_XXXX.hdf5 files from collect_demo.py (reads /observations/qpos).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_msg

# ---------------------------------------------------------------------------
# Constants (from rerun_bridge.py / teleop.yaml)
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
# Copied inline from act_policy_node.py
# ---------------------------------------------------------------------------

def drain_sub(sock) -> Optional[dict]:
    """Drain a ZMQ SUB socket (msgpack), return latest message or None."""
    import zmq

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
# Copied inline from collect_demo.py
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


# ---------------------------------------------------------------------------
# Rerun helpers — extracted from rerun_bridge.py
# ---------------------------------------------------------------------------

_DEFAULT_ARM_ROOT = "world/rover/arm"

_DEFAULT_LINK_COLORS = (
    (255, 180, 0),   # link_0
    (255, 140, 0),   # link_1
    (255, 100, 0),   # link_2
    (255,  60, 0),   # link_3
    (255,   0, 50),  # link_5
)


def _log_static_arm(
    root: str = _DEFAULT_ARM_ROOT,
    *,
    color_override: Optional[Tuple[int, int, int, int]] = None,
    include_rover_body: bool = True,
) -> None:
    """Log static arm link geometry.

    Args:
        root: Entity path where the swivel transform sits. Defaults to the
            on-robot path; the episode visualizer passes
            ``world/ghosts/h_NN/arm`` to mount independent ghost copies.
        color_override: When set, paint every link strip with this RGBA so a
            ghost arm reads as a single tint instead of the per-link
            orange-gradient used for the actual robot.
        include_rover_body: Skip the rover Boxes3D (and its parent path) for
            ghost copies so we don't draw 16 overlapping rover bodies.
    """
    import rerun as rr

    _jb  = f"{root}/joint_base"
    _jm  = f"{_jb}/joint_mid"
    _je  = f"{_jm}/joint_end"
    _jwp = f"{_je}/joint_wrist_pitch"
    _jwr = f"{_jwp}/joint_wrist_roll"

    if include_rover_body:
        rover_path = root.rsplit("/", 1)[0] if "/" in root else "world/rover"
        rr.log(
            rover_path,
            rr.Boxes3D(half_sizes=[[0.3, 0.2, 0.1]], colors=[[80, 80, 80]]),
            static=True,
        )

    rr.log(
        root,
        rr.Transform3D(translation=[0.0, 0.0, ARM_MOUNT_Z]),
        static=True,
    )

    link_paths = [
        (f"{_jb}/link_0",  L0),
        (f"{_jm}/link_1",  L1),
        (f"{_je}/link_2",  L2),
        (f"{_jwp}/link_3", L3),
        (f"{_jwr}/link_5", L5),
    ]
    for (path, length), default_color in zip(link_paths, _DEFAULT_LINK_COLORS):
        color = color_override if color_override is not None else default_color
        rr.log(
            path,
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]], colors=[color]),
            static=True,
        )


def _log_arm_fk(qpos: np.ndarray, root: str = _DEFAULT_ARM_ROOT) -> None:
    """Log FK transforms for the arm hierarchy.

    Args:
        qpos: [7] array in ARM_JOINTS order
              [swivel, base, mid, end, wrist_pitch, wrist_roll, gripper].
              The swivel transform is logged on the rover→arm link.
        root: Entity path where the swivel transform sits. Pass a per-ghost
              root from the visualizer to drive an independent ghost copy.
    """
    import rerun as rr

    swivel_pos      = float(qpos[0])
    base_pos        = float(qpos[1])
    mid_pos         = float(qpos[2])
    end_pos         = float(qpos[3])
    wrist_pitch_pos = float(qpos[4])
    wrist_roll_pos  = float(qpos[5])
    gripper_pos     = float(qpos[6])

    _je = f"{root}/joint_base/joint_mid/joint_end"

    rr.log(
        root,
        rr.Transform3D(
            translation=[0.0, 0.0, ARM_MOUNT_Z],
            rotation=rr.RotationAxisAngle([0, 0, 1], angle=swivel_pos),
        ),
    )
    rr.log(
        f"{root}/joint_base",
        rr.Transform3D(rotation=rr.RotationAxisAngle([0, 0, 1], angle=base_pos)),
    )
    rr.log(
        f"{root}/joint_base/joint_mid",
        rr.Transform3D(
            translation=[L0, 0.0, 0.0],
            rotation=rr.RotationAxisAngle([0, 1, 0], angle=mid_pos),
        ),
    )
    rr.log(
        f"{root}/joint_base/joint_mid/joint_end",
        rr.Transform3D(
            translation=[L1, 0.0, 0.0],
            rotation=rr.RotationAxisAngle([0, 1, 0], angle=end_pos),
        ),
    )
    rr.log(
        f"{_je}/joint_wrist_pitch",
        rr.Transform3D(
            translation=[L2, 0.0, 0.0],
            rotation=rr.RotationAxisAngle([0, 1, 0], angle=wrist_pitch_pos),
        ),
    )
    rr.log(
        f"{_je}/joint_wrist_pitch/joint_wrist_roll",
        rr.Transform3D(
            translation=[L3, 0.0, 0.0],
            rotation=rr.RotationAxisAngle([1, 0, 0], angle=wrist_roll_pos),
        ),
    )
    rr.log(
        f"{_je}/joint_wrist_pitch/joint_wrist_roll/joint_gripper",
        rr.Transform3D(
            translation=[L5, 0.0, 0.0],
            rotation=rr.RotationAxisAngle([0, 0, 1], angle=gripper_pos),
        ),
    )


# ---------------------------------------------------------------------------
# HDF5 I/O helpers
# ---------------------------------------------------------------------------

def _next_recording_path(recordings_dir: Path) -> Path:
    """Find the next available recordings/recording_XXXX.hdf5 path."""
    recordings_dir.mkdir(parents=True, exist_ok=True)
    existing = sorted(recordings_dir.glob("recording_*.hdf5"))
    if existing:
        last_num = int(existing[-1].stem.split("_")[1])
        num = last_num + 1
    else:
        num = 0
    return recordings_dir / f"recording_{num:04d}.hdf5"


def save_recording(path: Path, qpos: np.ndarray, velocities: np.ndarray, timestamps: np.ndarray) -> None:
    """Save a recording to HDF5."""
    with h5py.File(path, "w") as f:
        f.create_dataset("qpos",       data=qpos.astype(np.float32),       compression="gzip")
        f.create_dataset("velocities", data=velocities.astype(np.float32), compression="gzip")
        f.create_dataset("timestamps", data=timestamps.astype(np.float64), compression="gzip")
        f.attrs["hz"]          = RECORD_HZ
        f.attrs["arm_joints"]  = json.dumps(ARM_JOINTS)
        f.attrs["recorded_at"] = datetime.now(timezone.utc).isoformat()


def _maybe_prepend_swivel(
    qpos: np.ndarray, swivel: Optional[np.ndarray],
) -> np.ndarray:
    """Back-compat shim: pre-7-DOF recordings stored qpos as 6-element gantry
    rows with swivel as a sibling dataset.  Re-stitch to a 7-DOF array so
    every loader downstream sees the unified shape.
    """
    if qpos.ndim != 2 or qpos.shape[1] != NUM_GANTRY_JOINTS:
        return qpos.astype(np.float32, copy=False)
    if swivel is None:
        # Old recording without a swivel dataset — fill with zeros so the
        # array shape is consistent.  Old data without swivel is rare and
        # marker-quality at best; downstream code should treat sw=0 as
        # "unknown".
        sw_col = np.zeros((qpos.shape[0], 1), dtype=np.float32)
    else:
        sw = np.asarray(swivel, dtype=np.float32).reshape(-1)
        if sw.shape[0] != qpos.shape[0]:
            # Truncate/pad rather than crash; old episodes occasionally
            # have a trailing extra swivel sample.
            n  = min(sw.shape[0], qpos.shape[0])
            sw = sw[:n]
            qpos = qpos[:n]
        sw_col = sw.reshape(-1, 1)
    return np.hstack([sw_col, qpos.astype(np.float32, copy=False)])


def load_recording(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load a recording or episode HDF5.

    Supports:
      - recordings/recording_XXXX.hdf5  → /qpos, /velocities, /timestamps
      - episode_XXXX.hdf5               → /observations/qpos (velocities/timestamps synthesized)
      - Pre-unification files where qpos was 6-DOF and `swivel` was a
        sibling dataset — those are stitched into a 7-DOF array on load.

    Returns:
        qpos:       [T, 7] float32 in ARM_JOINTS order (swivel-first)
        velocities: [T, 7] float32
        timestamps: [T]    float64
    """
    with h5py.File(path, "r") as f:
        sw_top = f["swivel"][:] if "swivel" in f else None
        sw_obs = (f["observations/swivel"][:]
                  if "observations" in f and "swivel" in f["observations"]
                  else None)
        if "qpos" in f:
            qpos       = f["qpos"][:]
            velocities = f["velocities"][:]
            timestamps = f["timestamps"][:]
            qpos       = _maybe_prepend_swivel(qpos, sw_top)
            if velocities.ndim == 2 and velocities.shape[1] == NUM_GANTRY_JOINTS:
                velocities = _maybe_prepend_swivel(velocities, None)
        elif "observations" in f and "qpos" in f["observations"]:
            # episode_XXXX.hdf5 from collect_demo.py
            qpos = f["observations/qpos"][:]
            qpos = _maybe_prepend_swivel(qpos, sw_obs)
            T    = len(qpos)
            velocities = np.zeros_like(qpos)
            timestamps = np.arange(T, dtype=np.float64) / RECORD_HZ
        else:
            raise ValueError(f"Unrecognised HDF5 format: {path}")
    return (
        qpos.astype(np.float32),
        velocities.astype(np.float32),
        timestamps.astype(np.float64),
    )


# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------

def check_recording_continuity(qpos: np.ndarray, max_delta: float) -> list[tuple[int, int, float]]:
    """Scan for large inter-frame deltas (discontinuities).

    Returns list of (frame_idx, joint_idx, delta) for deltas > 2 * max_delta.
    """
    warnings = []
    threshold = 2.0 * max_delta
    for i in range(1, len(qpos)):
        for j in range(qpos.shape[1]):
            delta = abs(float(qpos[i, j]) - float(qpos[i - 1, j]))
            if delta > threshold:
                warnings.append((i, j, delta))
    return warnings


def check_start_position(recording_start: np.ndarray, current_qpos: np.ndarray, tol: float = 0.1) -> list[tuple[int, float]]:
    """Compare recording first frame to current arm position.

    Returns list of (joint_idx, mismatch_rad) for mismatches > tol.
    """
    mismatches = []
    for j in range(len(recording_start)):
        diff = abs(float(recording_start[j]) - float(current_qpos[j]))
        if diff > tol:
            mismatches.append((j, diff))
    return mismatches


# ---------------------------------------------------------------------------
# subcommand: record
# ---------------------------------------------------------------------------

def cmd_record(args: argparse.Namespace) -> None:
    import zmq

    ctx  = zmq.Context()
    sub  = ctx.socket(zmq.SUB)
    sub.connect(args.telem)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, 100)

    recordings_dir = Path(args.output_dir)
    get_key = setup_keyboard()

    recording   = False
    qpos_buf:       list[np.ndarray] = []
    vel_buf:        list[np.ndarray] = []
    ts_buf:         list[float]      = []
    record_start_t: float            = 0.0

    print("record_replay  [record mode]")
    print("  R = start/stop recording   Q = quit")
    print(f"  Subscribing to {args.telem} at {RECORD_HZ} Hz\n")

    period = 1.0 / RECORD_HZ
    try:
        while True:
            t0 = time.time()

            key = get_key()
            if key == "Q":
                print("Quit.")
                break
            if key == "R":
                if not recording:
                    recording      = True
                    record_start_t = time.time()
                    qpos_buf.clear()
                    vel_buf.clear()
                    ts_buf.clear()
                    print("*** Recording started ***")
                else:
                    recording = False
                    print(f"\n*** Recording stopped — {len(qpos_buf)} frames ***")
                    if qpos_buf:
                        out_path = _next_recording_path(recordings_dir)
                        save_recording(
                            out_path,
                            np.stack(qpos_buf),
                            np.stack(vel_buf),
                            np.array(ts_buf),
                        )
                        print(f"Saved → {out_path}")

            telem = drain_sub(sub)
            if telem is not None:
                qpos = extract_qpos(telem)
                vels = extract_velocities(telem)
                if qpos is not None and vels is not None:
                    if recording:
                        qpos_buf.append(qpos)
                        vel_buf.append(vels)
                        ts_buf.append(time.time())
                        elapsed = time.time() - record_start_t
                        print(
                            f"\r  steps={len(qpos_buf):5d}  t={elapsed:6.1f}s  "
                            f"qpos=[{', '.join(f'{v:.3f}' for v in qpos)}]",
                            end="",
                            flush=True,
                        )
                    else:
                        print(
                            f"\r  qpos=[{', '.join(f'{v:.3f}' for v in qpos)}]",
                            end="",
                            flush=True,
                        )

            elapsed = time.time() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sub.close()
        ctx.term()


# ---------------------------------------------------------------------------
# subcommand: visualize
# ---------------------------------------------------------------------------

def cmd_visualize(args: argparse.Namespace) -> None:
    import rerun as rr

    path = Path(args.file)
    qpos, velocities, timestamps = load_recording(path)

    rr.init("arm_replay", spawn=True)
    if args.save:
        rr.save(args.save)

    _log_static_arm()

    print(f"Visualizing {path.name}: {len(qpos)} frames")

    for i in range(len(qpos)):
        rr.set_time("time", timestamp=timestamps[i])

        _log_arm_fk(qpos[i])

        for j, joint in enumerate(ARM_JOINTS):
            rr.log(f"joints/position/{joint}", rr.Scalars(float(qpos[i, j])))
            rr.log(f"joints/velocity/{joint}", rr.Scalars(float(velocities[i, j])))

        if i < len(qpos) - 1:
            dt = float(timestamps[i + 1] - timestamps[i])
            sleep_t = dt / args.speed
            if sleep_t > 0:
                time.sleep(sleep_t)

    print("Done.")


# ---------------------------------------------------------------------------
# subcommand: replay
# ---------------------------------------------------------------------------

def cmd_replay(args: argparse.Namespace) -> None:
    path = Path(args.file)
    qpos, velocities, timestamps = load_recording(path)

    # --- Pre-flight: continuity check ---
    continuity_warnings = check_recording_continuity(qpos, args.max_delta)
    if continuity_warnings:
        print(f"WARNING: {len(continuity_warnings)} discontinuity/ies detected in recording:")
        for frame_i, joint_i, delta in continuity_warnings[:10]:
            print(f"  frame={frame_i}, joint={ARM_JOINTS[joint_i]}, delta={delta:.4f} rad")
        if len(continuity_warnings) > 10:
            print(f"  ... and {len(continuity_warnings) - 10} more")

    if args.dry_run:
        _replay_dry_run(args, qpos, velocities, timestamps)
    else:
        _replay_live(args, qpos, velocities, timestamps)


def _replay_dry_run(
    args: argparse.Namespace,
    qpos: np.ndarray,
    velocities: np.ndarray,
    timestamps: np.ndarray,
) -> None:
    import rerun as rr

    rr.init("arm_replay", spawn=True)
    if args.save:
        rr.save(args.save)

    _log_static_arm()

    print(f"[DRY-RUN] Replaying {len(qpos)} frames (no hardware connection)")

    for i in range(len(qpos)):
        rr.set_time("time", timestamp=timestamps[i])
        _log_arm_fk(qpos[i])

        for j, joint in enumerate(ARM_JOINTS):
            rr.log(f"arm/commanded/{joint}", rr.Scalars(float(qpos[i, j])))

        print(
            f"\r  frame={i+1:5d}/{len(qpos)}  "
            f"cmd=[{', '.join(f'{v:.3f}' for v in qpos[i])}]",
            end="",
            flush=True,
        )

        if i < len(qpos) - 1:
            dt = float(timestamps[i + 1] - timestamps[i])
            sleep_t = dt / args.speed
            if sleep_t > 0:
                time.sleep(sleep_t)

    print("\nDone.")


def _replay_live(
    args: argparse.Namespace,
    qpos: np.ndarray,
    velocities: np.ndarray,
    timestamps: np.ndarray,
) -> None:
    import zmq
    import rerun as rr

    # Safety confirmation with countdown
    print("=" * 60)
    print("MOTORS WILL MOVE — press Enter to confirm or Ctrl+C to abort")
    print("=" * 60)
    for remaining in range(5, 0, -1):
        print(f"  Starting in {remaining}s ...", end="\r", flush=True)
        time.sleep(1.0)
    print()
    try:
        input("Press Enter to confirm, or Ctrl+C to abort: ")
    except KeyboardInterrupt:
        print("\nAborted.")
        return

    arm_limits = load_arm_limits()
    if arm_limits:
        print(f"Arm limits loaded ({len(arm_limits)} joints)")

    ctx = zmq.Context()

    # Command socket (PUSH/PUB)
    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.connect(args.cmd)

    # Telemetry socket
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.connect(args.telem)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sock.setsockopt(zmq.RCVTIMEO, 50)

    rr.init("arm_replay", spawn=True)
    if args.save:
        rr.save(args.save)

    _log_static_arm()

    # --- Pre-flight: start position check ---
    print("Checking start position...")
    telem = None
    for _ in range(20):
        telem = drain_sub(telem_sock)
        if telem is not None:
            break
        time.sleep(0.05)

    current_qpos = extract_qpos(telem) if telem else None
    if current_qpos is not None:
        mismatches = check_start_position(qpos[0], current_qpos)
        if mismatches:
            print("WARNING: Start position mismatch:")
            for joint_i, diff in mismatches:
                print(f"  {ARM_JOINTS[joint_i]}: recording={qpos[0, joint_i]:.3f}  current={current_qpos[joint_i]:.3f}  diff={diff:.3f} rad")
    else:
        print("WARNING: Could not read current arm position for start-position check.")
        current_qpos = qpos[0].copy()

    # --- goto-start ---
    if args.goto_start:
        print("Moving to start position...")
        _goto_start(cmd_sock, telem_sock, qpos[0], args.max_delta, arm_limits)

    # --- Replay loop ---
    print(f"\n[LIVE] Replaying {len(qpos)} frames")
    period = (timestamps[-1] - timestamps[0]) / max(len(timestamps) - 1, 1) / args.speed if len(timestamps) > 1 else 0.05 / args.speed

    try:
        for i in range(len(qpos)):
            t0 = time.time()

            target = qpos[i]

            # Apply delta clamping relative to current position
            safe_target, clamped = apply_safety_limits(target, current_qpos, args.max_delta)
            if arm_limits:
                safe_target = np.array(clamp_arm_positions(safe_target.tolist(), arm_limits), dtype=np.float32)

            # Send command (7-DOF: swivel + gantry + gripper)
            cmd = {
                "type": "arm_joints",
                "positions": safe_target.tolist(),
                "velocities": [0.0] * NUM_ARM_JOINTS,
                "kp": KP,
                "kd": KD,
                "torques": [0.0] * NUM_ARM_JOINTS,
            }
            cmd_sock.send(pack_msg(cmd))

            # Read telemetry
            telem = drain_sub(telem_sock)
            actual_qpos = extract_qpos(telem) if telem else current_qpos
            if actual_qpos is not None:
                current_qpos = actual_qpos

            # Log to Rerun
            rr.set_time("time", timestamp=timestamps[i])
            _log_arm_fk(current_qpos)
            for j, joint in enumerate(ARM_JOINTS):
                rr.log(f"arm/commanded/{joint}", rr.Scalars(float(safe_target[j])))
                rr.log(f"arm/actual/{joint}",    rr.Scalars(float(current_qpos[j])))

            print(
                f"\r  frame={i+1:5d}/{len(qpos)}  "
                f"cmd=[{', '.join(f'{v:.3f}' for v in safe_target)}]  "
                f"actual=[{', '.join(f'{v:.3f}' for v in current_qpos)}]",
                end="",
                flush=True,
            )

            elapsed = time.time() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\nInterrupted — watchdog will hold last position.")
    finally:
        print("\nDone.")
        cmd_sock.close()
        telem_sock.close()
        ctx.term()


def _goto_start(
    cmd_sock,
    telem_sock,
    target: np.ndarray,
    max_delta: float,
    arm_limits=None,
    tol: float = 0.02,
    timeout: float = 30.0,
) -> None:
    """Smoothly move arm to target, waiting until within tol on all joints."""
    t_start = time.time()
    current = target.copy()

    # Read current position
    for _ in range(20):
        telem = drain_sub(telem_sock)
        if telem is not None:
            q = extract_qpos(telem)
            if q is not None:
                current = q
                break
        time.sleep(0.05)

    print(f"  Moving to start: target=[{', '.join(f'{v:.3f}' for v in target)}]")

    while time.time() - t_start < timeout:
        safe, _ = apply_safety_limits(target, current, max_delta)
        if arm_limits:
            safe = np.array(clamp_arm_positions(safe.tolist(), arm_limits), dtype=np.float32)
        cmd = {
            "type": "arm_joints",
            "positions": safe.tolist(),
            "velocities": [0.0] * NUM_ARM_JOINTS,
            "kp": KP,
            "kd": KD,
            "torques": [0.0] * NUM_ARM_JOINTS,
        }
        cmd_sock.send(pack_msg(cmd))
        time.sleep(1.0 / RECORD_HZ)

        telem = drain_sub(telem_sock)
        if telem is not None:
            q = extract_qpos(telem)
            if q is not None:
                current = q

        if np.all(np.abs(current - target) < tol):
            print("  Reached start position.")
            return

    print("  WARNING: timeout reaching start position — proceeding anyway.")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Arm trajectory record, visualize, and replay.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="subcommand", required=True)

    # -- record --
    rec = sub.add_parser("record", help="Record arm trajectory from telemetry")
    rec.add_argument("--telem",      default="tcp://localhost:5556", help="ZMQ telemetry address")
    rec.add_argument("--output-dir", default="recordings",           help="Directory for output HDF5 files")

    # -- visualize --
    viz = sub.add_parser("visualize", help="Visualize a recording or episode in Rerun")
    viz.add_argument("file",           help="HDF5 file to visualize")
    viz.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    viz.add_argument("--save",  default=None,            help="Also save to MCAP file")

    # -- replay --
    rep = sub.add_parser("replay", help="Replay a recording on the arm")
    rep.add_argument("file",                                                  help="HDF5 file to replay")
    rep.add_argument("--cmd",       default="tcp://localhost:5555",           help="ZMQ command address")
    rep.add_argument("--telem",     default="tcp://localhost:5556",           help="ZMQ telemetry address")
    rep.add_argument("--speed",     type=float, default=1.0,                  help="Playback speed multiplier")
    rep.add_argument("--max-delta", type=float, default=0.05, dest="max_delta", help="Max joint delta per step (rad)")
    rep.add_argument("--save",      default=None,                             help="Also save Rerun data to MCAP file")
    rep.add_argument("--goto-start", action="store_true", dest="goto_start",  help="Move arm to recording start before replay (live only)")

    mode = rep.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true",  dest="dry_run", default=True,  help="Simulate replay without hardware (default)")
    mode.add_argument("--live",    action="store_false", dest="dry_run",                help="Send commands to real arm")

    return p


def main() -> None:
    parser = build_parser()
    args   = parser.parse_args()

    if args.subcommand == "record":
        cmd_record(args)
    elif args.subcommand == "visualize":
        cmd_visualize(args)
    elif args.subcommand == "replay":
        cmd_replay(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
