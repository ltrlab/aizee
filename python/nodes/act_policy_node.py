#!/usr/bin/env python3
"""
act_policy_node.py — ACT policy inference node for AIZEE arm.

Runs at 20 Hz, subscribes to arm telemetry and the single gripper camera
(ELP UVC), runs the ACT policy, and sends a single 7-DOF arm_joints command
per tick (swivel is joint 0 of the unified arm).

WARNING: Do NOT run teleop.py and this node simultaneously.
Both push to :5555. Interleaved commands are dangerous.

Usage:
    python act_policy_node.py --checkpoint checkpoints/act_best.pt
    python act_policy_node.py --checkpoint checkpoints/act_best.pt --dry-run
    python act_policy_node.py --checkpoint checkpoints/act_best.pt --device cpu

Safety:
    - Waits for all three sources before sending any commands.
    - Skips command if any source is >200 ms stale.
    - --dry-run: runs inference and logs actions, does NOT push to :5555.
    - Warns if forward pass takes >80 ms (near 100 ms watchdog limit).
    - Clamps predicted positions (or relative deltas) to training-data range.
    - Clamps per-step delta to --max-delta rad/step (velocity guard).
"""

from __future__ import annotations

import argparse
import io
import json
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import torch
import yaml
import zmq
from PIL import Image

try:
    import cv2
    _cv2_available = True
except ImportError:
    _cv2_available = False

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_camera, unpack_msg

try:
    import rerun as rr
    import rerun.blueprint as rrb
    _rerun_available = True
except ImportError:
    _rerun_available = False

# Allow running from repo root or python/nodes/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.act_model import ACTPolicy
from python.training.inference import (
    _IMAGENET_MEAN, _IMAGENET_STD, _STATE_MODE_K,
    normalize_image, normalize_qpos, normalize_qcmd, normalize_torques,
    denormalize_actions, build_state_vector, load_checkpoint,
)
from python.scripts.record_replay import (
    ARM_JOINTS, POLICY_JOINTS, NUM_POLICY_JOINTS,
    extract_policy_qpos, extract_policy_torques,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# After the swivel-unification refactor, ARM_JOINTS is 7-DOF (swivel-first)
# and equals POLICY_JOINTS — there's no separate Swivel command type and the
# wire `arm_joints` payload carries 7 floats.
NUM_ARM_JOINTS = len(ARM_JOINTS)   # 7

# Fallback gains if teleop.yaml is not found
# 7-DOF defaults (swivel-first), matching record_replay.KP/KD.
_DEFAULT_KP = [150.0, 75.0, 65.0, 10.0, 5.0, 10.0, 10.0]
_DEFAULT_KD = [5.0,   7.0,  5.5,  0.2,  0.2, 2.0,  2.0]


def _load_gains():
    """Load 7-DOF arm KP/KD from config/teleop.yaml, falling back to defaults.

    Reads `arm.kp` / `arm.kd` (post-unification).  Older configs that still
    have split `gantry.kp` + `drive.swivel_kp` are stitched together so the
    policy still gets a swivel-first 7-element vector.
    """
    here = Path(__file__).parent
    for candidate in [
        here / ".." / ".." / "config" / "teleop.yaml",
        Path("config") / "teleop.yaml",
    ]:
        p = candidate.resolve()
        if p.exists():
            cfg = yaml.safe_load(p.read_text()) or {}
            arm = cfg.get("arm", {})
            if "kp" in arm and "kd" in arm:
                return list(arm["kp"]), list(arm["kd"])
            gantry = cfg.get("gantry", {})
            drive  = cfg.get("drive",  {})
            kp = [float(drive.get("swivel_kp", _DEFAULT_KP[0]))] + list(gantry.get("kp", _DEFAULT_KP[1:]))
            kd = [float(drive.get("swivel_kd", _DEFAULT_KD[0]))] + list(gantry.get("kd", _DEFAULT_KD[1:]))
            return list(kp), list(kd)
    return list(_DEFAULT_KP), list(_DEFAULT_KD)

STALE_THRESH = 0.200   # 200 ms
WARN_LATENCY = 0.080   # 80 ms — warn if inference takes longer


def _setup_keyboard():
    """Non-blocking key reader. Returns a callable that returns a key or None."""
    if sys.platform == "win32":
        import msvcrt
        def _get():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch.upper()
            return None
    else:
        import select, tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        def _get():
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                return ch.upper() if ch else None
            return None
    return _get


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def drain_sub(sock) -> Optional[dict]:
    """Drain an msgpack-over-ZMQ SUB socket, return latest message or None."""
    latest = None
    while True:
        try:
            latest = unpack_msg(sock.recv(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def drain_camera(sock) -> Optional[dict]:
    """Drain a multipart camera SUB socket, return latest decoded message."""
    latest = None
    while True:
        try:
            frames = sock.recv_multipart(zmq.NOBLOCK)
            latest = unpack_camera(frames)
        except zmq.Again:
            break
        except Exception:
            break
    return latest


# ---------------------------------------------------------------------------
# Image / normalization
# ---------------------------------------------------------------------------

def decode_image(msg: dict, target_size=(1024, 768)) -> Optional[np.ndarray]:
    """Decode camera message to uint8 [H, W, 3]. target_size = (width, height).

    Default matches the gripper-camera capture resolution so no resize fires
    when the publisher and the policy agree. Uses cv2 (libjpeg-turbo) when
    available; falls back to PIL.
    """
    color = msg.get("color", {})
    raw   = color.get("data_bytes")
    if raw is None:
        return None
    if _cv2_available:
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if (bgr.shape[1], bgr.shape[0]) != target_size:
            bgr = cv2.resize(bgr, target_size, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if (img.width, img.height) != target_size:
        img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)


def apply_safety_limits(
    action: np.ndarray,
    qpos_raw: np.ndarray,
    stats: dict,
    action_mode: str,
    max_delta_arm: float,
    max_delta_swivel: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Clamp a predicted absolute-action vector to safe bounds.

    Layer 1 — position bounds:
        absolute mode: clamp each joint to [action_min, action_max]
        relative mode: clamp (action - qpos) to [rel_action_min, rel_action_max]
                       (i.e. deltas must stay within the deltas seen in training)
    Layer 2 — per-step delta guard: clamp |action - qpos| ≤ per-joint max_delta.
        swivel uses max_delta_swivel, arm joints use max_delta_arm.

    Returns (clamped_action [J], delta_clamped [J] bool).
    """
    if action_mode == "absolute":
        action = np.clip(action, stats["action_min"], stats["action_max"])
    else:
        delta = action - qpos_raw
        delta = np.clip(delta, stats["rel_action_min"], stats["rel_action_max"])
        action = qpos_raw + delta

    # Per-joint velocity guard
    max_delta_vec = np.full(action.shape, max_delta_arm, dtype=np.float32)
    max_delta_vec[0] = max_delta_swivel   # swivel is index 0
    delta = action - qpos_raw
    delta_clamped = np.abs(delta) > max_delta_vec
    delta_clipped = np.clip(delta, -max_delta_vec, max_delta_vec)
    action = qpos_raw + delta_clipped

    return action.astype(np.float32), delta_clamped


# ---------------------------------------------------------------------------
# Temporal ensemble
# ---------------------------------------------------------------------------

class TemporalEnsemble:
    """Maintain rolling buffer of predicted action chunks.

    Each tick we have K recent chunks, each predicting future actions. The
    ensemble action at the current step is an exponentially-weighted average
    of all available predictions for this timestep.

    NOTE: for relative-mode checkpoints we ensemble *absolute* positions after
    each chunk has been denormalized and anchored to its own qpos_at_prediction.
    That way if the arm drifts between predictions the ensemble still converges
    to the correct absolute target rather than a stale delta.
    """

    def __init__(self, chunk_size: int, ensemble_steps: int, action_dim: int):
        self.chunk_size = chunk_size
        self.ensemble_steps = ensemble_steps
        self.action_dim = action_dim
        self._chunks: Deque[Tuple[np.ndarray, int]] = deque(maxlen=ensemble_steps)
        self._step = 0

    def add_chunk(self, chunk: np.ndarray):
        """Add a newly predicted chunk. chunk: [chunk_size, action_dim]."""
        self._chunks.append((chunk.copy(), 0))

    def get_action(self) -> Optional[np.ndarray]:
        if not self._chunks:
            return None
        alpha = 0.1
        weighted_sum = np.zeros(self.action_dim, dtype=np.float64)
        weight_total = 0.0
        for chunk, age in self._chunks:
            if age < self.chunk_size:
                w = np.exp(-alpha * age)
                weighted_sum += w * chunk[age]
                weight_total += w
        if weight_total < 1e-9:
            return None
        return (weighted_sum / weight_total).astype(np.float32)

    def step(self):
        updated = [(c, a + 1) for c, a in self._chunks]
        self._chunks = deque(updated, maxlen=self.ensemble_steps)
        self._step += 1

    def clear(self):
        self._chunks.clear()


# ---------------------------------------------------------------------------
# Commanding
# ---------------------------------------------------------------------------

def _send_joint_command(
    cmd_push,
    action: np.ndarray,
    arm_kp: List[float],
    arm_kd: List[float],
):
    """Send the 7-DOF action as a single arm_joints command (swivel-first)."""
    try:
        cmd_push.send(pack_msg({
            "type": "arm_joints",
            "positions": action[:NUM_ARM_JOINTS].astype(np.float32).tolist(),
            "velocities": [0.0] * NUM_ARM_JOINTS,
            "kp": arm_kp, "kd": arm_kd,
            "torques": [0.0] * NUM_ARM_JOINTS,
        }), zmq.NOBLOCK)
    except zmq.Again:
        print("\n[WARN] Command queue full, skipped")


# ---------------------------------------------------------------------------
# State / ready-pose helpers
# ---------------------------------------------------------------------------

def _closest_start_pose(qpos_now: np.ndarray, stats: dict) -> np.ndarray:
    """Select the training-set start pose nearest to `qpos_now`.

    Falls back to `ready_pose` (the mean) if individual start_poses are absent
    (older checkpoints). Using the *closest* real start pose instead of the mean
    keeps the first observation on-distribution for all tasks in the dataset.
    """
    if "start_poses" in stats:
        starts = np.asarray(stats["start_poses"], dtype=np.float32)  # [N, J]
        if starts.ndim == 2 and starts.shape[0] > 0:
            dists = np.linalg.norm(starts - qpos_now[None, :], axis=1)
            return starts[int(np.argmin(dists))].copy()
    return np.asarray(stats["ready_pose"], dtype=np.float32).copy()


def _ramp_to_pose(
    target: np.ndarray,
    *,
    cmd_push,
    telem_sub,
    gripper_sub,
    arm_kp: List[float],
    arm_kd: List[float],
    ramp_speed_rad_s: float,
    tick: float,
    tol: float = 0.03,
) -> None:
    """Ramp all 7 joints from current qpos to `target` at `ramp_speed_rad_s`.

    Sends a swivel command + arm_joints command each tick. Blocks until the
    arm is within `tol` rad of the target (max-norm).
    """
    ramp_step = ramp_speed_rad_s * tick
    while True:
        rt0 = time.monotonic()
        t = drain_sub(telem_sub)
        drain_camera(gripper_sub)
        qpos = extract_policy_qpos(t) if t is not None else None
        if qpos is None:
            time.sleep(tick)
            continue

        err = float(np.max(np.abs(qpos - target)))
        delta = np.clip(target - qpos, -ramp_step, ramp_step)
        q_cmd = qpos + delta
        _send_joint_command(cmd_push, q_cmd, arm_kp, arm_kd)

        print(f"\rRamping... max_err={err:.3f} rad   ", end="", flush=True)
        if err < tol:
            break

        elapsed = time.monotonic() - rt0
        time.sleep(max(0, tick - elapsed))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="ACT Policy Inference Node")
    parser.add_argument("--checkpoint", required=True, help="Path to .pt checkpoint")
    parser.add_argument("--telem", default="tcp://localhost:5556")
    parser.add_argument("--gripper-cam", default="tcp://localhost:5563", dest="gripper_cam",
                        help="Gripper camera ZMQ endpoint (single ELP UVC stream)")
    parser.add_argument("--cmd", default="tcp://localhost:5555")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run inference but do NOT send commands to :5555")
    parser.add_argument("--ensemble-steps", type=int, default=16,
                        help="Number of past chunks to ensemble (0 = disable)")
    parser.add_argument("--max-delta", type=float, default=0.3,
                        help="Max arm-joint position change per step in rad "
                             "(velocity guard, default 0.3 = 6 rad/s at 20 Hz)")
    parser.add_argument("--max-delta-swivel", type=float, default=0.15,
                        help="Max swivel position change per step in rad "
                             "(default 0.15 = 3 rad/s at 20 Hz)")
    parser.add_argument("--ramp-speed", type=float, default=1.5, dest="ramp_speed",
                        help="Ramp speed to ready pose [rad/s] (default 1.5)")
    parser.add_argument("--no-rerun", action="store_true", dest="no_rerun",
                        help="Disable Rerun visualization")
    args = parser.parse_args()

    # Safety warning
    print()
    print("=" * 60)
    print("WARNING: Do NOT run teleop.py and this node simultaneously!")
    print("Both push to :5555. Interleaved commands are dangerous.")
    print("=" * 60)
    print()

    if args.dry_run:
        print("[DRY RUN MODE — inference only, no commands will be sent]")
        print()

    device = torch.device(args.device)
    print(f"Device: {device}")
    print(f"Checkpoint: {args.checkpoint}")

    arm_kp, arm_kd = _load_gains()
    print(f"Arm gains (swivel-first 7-DOF): kp={arm_kp}  kd={arm_kd}")

    # Load model
    print("Loading checkpoint...")
    policy, dataset_stats, config = load_checkpoint(args.checkpoint, device)
    chunk_size = config["chunk_size"]
    num_joints = config["num_joints"]
    state_mode = config["state_mode"]
    action_mode = config["action_mode"]
    state_dim = config["state_dim"]
    print(f"Policy: chunk_size={chunk_size}  num_joints={num_joints}  "
          f"state_mode={state_mode} (dim={state_dim})  action_mode={action_mode}")

    if num_joints != NUM_POLICY_JOINTS:
        print(f"[WARN] checkpoint num_joints={num_joints} but ARM_JOINTS has "
              f"{NUM_POLICY_JOINTS} entries (swivel + 6 gantry, swivel-first).")

    # Training-data safety ranges
    print()
    print("Safety limits (from training data):")
    print(f"  {'Joint':<14} {'act_min':>10}  {'act_max':>10}  {'rel_min':>8}  {'rel_max':>8}")
    print(f"  {'-'*14} {'-'*10}  {'-'*10}  {'-'*8}  {'-'*8}")
    for i, joint in enumerate(POLICY_JOINTS[:num_joints]):
        lo = dataset_stats["action_min"][i]
        hi = dataset_stats["action_max"][i]
        rlo = dataset_stats.get("rel_action_min", [np.nan] * num_joints)[i]
        rhi = dataset_stats.get("rel_action_max", [np.nan] * num_joints)[i]
        print(f"  {joint:<14} {lo:>10.3f}  {hi:>10.3f}  {rlo:>8.3f}  {rhi:>8.3f}")
    print(f"  Per-step delta limit: swivel=±{args.max_delta_swivel:.3f} rad  "
          f"arm=±{args.max_delta:.3f} rad")
    print()

    # ZMQ setup
    ctx = zmq.Context()

    telem_sub = ctx.socket(zmq.SUB)
    telem_sub.setsockopt(zmq.LINGER, 0)
    telem_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sub.connect(args.telem)

    gripper_sub = ctx.socket(zmq.SUB)
    gripper_sub.setsockopt(zmq.LINGER, 0)
    gripper_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    gripper_sub.connect(args.gripper_cam)

    cmd_push = None
    if not args.dry_run:
        cmd_push = ctx.socket(zmq.PUSH)
        cmd_push.setsockopt(zmq.LINGER, 0)
        cmd_push.connect(args.cmd)

    print(f"Subscribing to telem:       {args.telem}")
    print(f"Subscribing to gripper cam: {args.gripper_cam}")
    if not args.dry_run:
        print(f"Pushing commands to:     {args.cmd}")
    print()

    # Rerun
    use_rerun = not args.no_rerun and _rerun_available
    if use_rerun:
        rr.init("aizee_policy")
        rr.spawn(memory_limit="512MiB")
        for jn in POLICY_JOINTS[:num_joints]:
            rr.set_time("frame", sequence=0)
            rr.log(f"qpos/{jn}", rr.Scalars(0.0))
            rr.log(f"action/{jn}", rr.Scalars(0.0))
            rr.log(f"error/{jn}", rr.Scalars(0.0))
        rr.send_blueprint(rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial2DView(name="Gripper", origin="cameras/gripper"),
                rrb.TimeSeriesView(
                    name="Joint Positions (qpos=amber, action=green)",
                    contents=[f"qpos/{j}" for j in POLICY_JOINTS[:num_joints]]
                           + [f"action/{j}" for j in POLICY_JOINTS[:num_joints]],
                ),
                rrb.TimeSeriesView(
                    name="Per-Joint Error |action - qpos|",
                    contents=[f"error/{j}" for j in POLICY_JOINTS[:num_joints]],
                ),
                rrb.TimeSeriesView(name="Inference Time (ms)", contents=["inference_ms"]),
                row_shares=[2, 2, 1, 1],
            )
        ))
        print("Rerun: viewer spawned")
    elif not args.no_rerun:
        print("NOTE: rerun not installed — run with --no-rerun to suppress this")
        use_rerun = False

    # Temporal ensemble
    use_ensemble = args.ensemble_steps > 0
    ensemble = (TemporalEnsemble(chunk_size, args.ensemble_steps, num_joints)
                if use_ensemble else None)

    # State
    last_telem_time = 0.0
    last_gripper_time = 0.0
    latest_telem: Optional[dict] = None
    latest_gripper: Optional[dict] = None
    last_action: Optional[np.ndarray] = None

    tick = 1.0 / 20.0   # 20 Hz
    commands_sent = 0
    rr_frame = 0
    all_sources_ready = False
    paused = False
    shutting_down = False
    shutdown_target: Optional[np.ndarray] = None
    shutdown_countdown = 0.0
    quit_requested = False
    hold_position: Optional[np.ndarray] = None
    get_key = _setup_keyboard()

    # Rerun colors
    if use_rerun:
        for jn in POLICY_JOINTS[:num_joints]:
            rr.log(f"qpos/{jn}", rr.SeriesLines(colors=[[255, 200, 60]], names=[f"qpos_{jn}"]), static=True)
            rr.log(f"action/{jn}", rr.SeriesLines(colors=[[80, 220, 80]], names=[f"act_{jn}"]), static=True)
            rr.log(f"error/{jn}", rr.SeriesLines(colors=[[255, 80, 80]], names=[jn]), static=True)
        rr.log("inference_ms", rr.SeriesLines(colors=[[160, 160, 160]], names=["ms"]), static=True)

    print("Waiting for all sources to become ready...")
    print("Controls: SPACE=pause/resume  Q=quit (confirm twice)")

    try:
        while True:
            t0 = time.monotonic()

            # Drain sockets
            telem = drain_sub(telem_sub)
            if telem is not None:
                latest_telem = telem
                last_telem_time = t0
            gripper_msg = drain_camera(gripper_sub)
            if gripper_msg is not None:
                latest_gripper = gripper_msg
                last_gripper_time = t0

            # Keyboard
            key = get_key()
            if key == "Q":
                if quit_requested:
                    if not args.dry_run and cmd_push is not None:
                        qpos_now = extract_policy_qpos(latest_telem) if latest_telem else None
                        shutdown_target = (qpos_now.copy() if qpos_now is not None
                                           else np.zeros(num_joints, dtype=np.float32))
                        shutdown_countdown = 1.0
                        shutting_down = True
                        print("\n[SHUTDOWN] Holding 1s then ramping to zero...")
                    else:
                        break
                else:
                    quit_requested = True
                    print("\n[Q] Press Q again to confirm shutdown, or SPACE to cancel.")
            elif key == " ":
                quit_requested = False
                if shutting_down:
                    shutting_down = False
                    paused = True
                    qpos_now = extract_policy_qpos(latest_telem) if latest_telem else None
                    hold_position = qpos_now.copy() if qpos_now is not None else last_action
                    print("\n[PAUSED] Shutdown cancelled. Holding position.")
                elif paused:
                    paused = False
                    hold_position = None
                    if ensemble is not None:
                        ensemble.clear()
                    print("\n[RESUMED] Inference active.")
                else:
                    paused = True
                    qpos_now = extract_policy_qpos(latest_telem) if latest_telem else None
                    hold_position = qpos_now.copy() if qpos_now is not None else last_action
                    print("\n[PAUSED] Holding position. SPACE=resume  Q=quit")

            # Shutdown
            if shutting_down and not args.dry_run and cmd_push is not None:
                dt = tick
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    if shutdown_target is not None:
                        _send_joint_command(cmd_push, shutdown_target, arm_kp, arm_kd)
                else:
                    max_change = 0.2 * dt
                    if shutdown_target is None:
                        shutdown_target = np.zeros(num_joints, dtype=np.float32)
                    qpos_now = extract_policy_qpos(latest_telem) if latest_telem else shutdown_target
                    ref = qpos_now if qpos_now is not None else shutdown_target
                    for i in range(num_joints):
                        shutdown_target[i] = (0.0 if abs(shutdown_target[i]) < max_change
                                              else shutdown_target[i] - np.sign(shutdown_target[i]) * max_change)
                    ramp_done = bool(np.all(np.abs(shutdown_target) < 0.01))
                    actual_close = qpos_now is None or bool(np.all(np.abs(qpos_now) < 0.05))
                    if ramp_done and actual_close:
                        try:
                            cmd_push.send(pack_msg({
                                "type": "disable",
                                "motor_ids": list(POLICY_JOINTS[:num_joints]),
                            }), zmq.NOBLOCK)
                        except zmq.Again:
                            pass
                        print("\n[SHUTDOWN] Motors disabled. Exiting.")
                        break
                    else:
                        delta = np.clip(shutdown_target - ref, -0.3, 0.3)
                        q_cmd = ref + delta
                        _send_joint_command(cmd_push, q_cmd, arm_kp, arm_kd)
                        print(f"\r[SHUTDOWN] Returning to zero... max_err={np.max(np.abs(shutdown_target)):.3f}    ",
                              end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Hold-position paused
            if paused and not args.dry_run and cmd_push is not None and hold_position is not None:
                _send_joint_command(cmd_push, hold_position, arm_kp, arm_kd)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue
            elif paused:
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Freshness
            telem_age = t0 - last_telem_time if last_telem_time > 0 else 999.0
            gripper_age = t0 - last_gripper_time if last_gripper_time > 0 else 999.0
            telem_ok = telem_age < STALE_THRESH
            gripper_ok = gripper_age < STALE_THRESH
            all_ok = telem_ok and gripper_ok

            if not all_sources_ready:
                if all_ok:
                    all_sources_ready = True
                    print("\nAll sources ready.")

                    # Pick closest start pose and ramp to it
                    qpos_now = extract_policy_qpos(latest_telem)
                    if qpos_now is not None:
                        target = _closest_start_pose(qpos_now, dataset_stats)
                        print(f"Closest start pose: {target}")
                        if not args.dry_run and cmd_push is not None:
                            try:
                                cmd_push.send(pack_msg({
                                    "type": "enable",
                                    "motor_ids": list(POLICY_JOINTS[:num_joints]),
                                }), zmq.NOBLOCK)
                            except zmq.Again:
                                pass
                            time.sleep(0.1)
                            _ramp_to_pose(
                                target, cmd_push=cmd_push,
                                telem_sub=telem_sub, gripper_sub=gripper_sub,
                                arm_kp=arm_kp, arm_kd=arm_kd,
                                ramp_speed_rad_s=args.ramp_speed, tick=tick,
                            )
                            print("\nReady pose reached. Stabilising camera...")
                            time.sleep(0.5)
                            drain_camera(gripper_sub)
                            drain_sub(telem_sub)
                    print("Starting inference loop.")
                else:
                    flags = []
                    if not telem_ok: flags.append("telem")
                    if not gripper_ok: flags.append("gripper_cam")
                    print(f"\rWaiting: missing {', '.join(flags)}    ", end="", flush=True)
                    elapsed = time.monotonic() - t0
                    time.sleep(max(0, tick - elapsed))
                    continue

            if not all_ok:
                stale = []
                if not telem_ok: stale.append(f"telem({telem_age*1000:.0f}ms)")
                if not gripper_ok: stale.append(f"gripper_cam({gripper_age*1000:.0f}ms)")
                print(f"\r[SKIP] Stale sources: {', '.join(stale)}    ", end="", flush=True)
                if ensemble is not None:
                    ensemble.step()
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Decode observations
            qpos_raw = extract_policy_qpos(latest_telem)
            gripper_img = decode_image(latest_gripper) if latest_gripper else None

            if qpos_raw is None or gripper_img is None:
                fails = []
                if qpos_raw is None: fails.append("qpos")
                if gripper_img is None: fails.append("gripper_img")
                print(f"\r[SKIP] Decode failed: {','.join(fails)}    ", end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            torques_raw = (extract_policy_torques(latest_telem)
                           if state_mode == "qpos_qcmd_tq" else None)

            # Normalize observations
            qpos_norm = normalize_qpos(qpos_raw, dataset_stats)
            gripper_norm = normalize_image(gripper_img)

            state_vec = build_state_vector(
                qpos_norm, state_mode, last_action, qpos_raw, torques_raw,
                dataset_stats, num_joints,
            )

            qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)
            state_t = torch.from_numpy(state_vec).unsqueeze(0).to(device)
            gripper_t = torch.from_numpy(gripper_norm).unsqueeze(0).to(device)

            # Inference
            infer_start = time.monotonic()
            with torch.no_grad():
                pred_chunk = policy.select_action(qpos_t, state_t, gripper_t)
            infer_time = time.monotonic() - infer_start
            if infer_time > WARN_LATENCY:
                print(f"\n[WARN] Inference took {infer_time*1000:.1f}ms "
                      f"(>{WARN_LATENCY*1000:.0f}ms threshold)")

            # Denormalize chunk to absolute positions (relative mode needs qpos anchor)
            pred_np = pred_chunk[0].cpu().numpy()
            pred_abs = denormalize_actions(pred_np, dataset_stats, action_mode, qpos=qpos_raw)

            if use_ensemble:
                ensemble.add_chunk(pred_abs)
                action = ensemble.get_action()
                ensemble.step()
            else:
                action = pred_abs[0]

            if action is None:
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            action, delta_clamped = apply_safety_limits(
                action, qpos_raw, dataset_stats, action_mode,
                max_delta_arm=args.max_delta,
                max_delta_swivel=args.max_delta_swivel,
            )
            last_action = action.copy()

            # Rerun
            if use_rerun:
                rr.set_time("frame", sequence=rr_frame)
                for j, jn in enumerate(POLICY_JOINTS[:num_joints]):
                    rr.log(f"qpos/{jn}", rr.Scalars(float(qpos_raw[j])))
                    rr.log(f"action/{jn}", rr.Scalars(float(action[j])))
                    rr.log(f"error/{jn}", rr.Scalars(float(abs(action[j] - qpos_raw[j]))))
                rr.log("inference_ms", rr.Scalars(infer_time * 1000.0))
                if rr_frame % 2 == 0:
                    rr.log("cameras/gripper", rr.Image(gripper_img))
                rr_frame += 1

            # Log
            action_str = " ".join(f"{v:+6.3f}" for v in action)
            qpos_str = " ".join(f"{v:+6.3f}" for v in qpos_raw)
            clamp_str = ""
            if delta_clamped.any():
                clamped = [POLICY_JOINTS[i] for i in range(num_joints) if delta_clamped[i]]
                clamp_str = f" [DELTA CLAMPED: {','.join(clamped)}]"
            print(
                f"\r[{'DRY' if args.dry_run else 'CMD'}#{commands_sent:5d}] "
                f"qpos:[{qpos_str}] → act:[{action_str}] "
                f"inf={infer_time*1000:.1f}ms{clamp_str}    ",
                end="", flush=True,
            )

            # Send command
            if not args.dry_run and cmd_push is not None:
                _send_joint_command(cmd_push, action, arm_kp, arm_kd)
                commands_sent += 1

            # Sleep remainder
            elapsed = time.monotonic() - t0
            remaining = tick - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        telem_sub.close()
        gripper_sub.close()
        if cmd_push is not None:
            cmd_push.close()
        ctx.term()
        print(f"Done. Commands sent: {commands_sent}")


if __name__ == "__main__":
    main()
