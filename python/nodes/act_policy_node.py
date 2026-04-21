#!/usr/bin/env python3
"""
act_policy_node.py — ACT policy inference node for AIZEE arm.

Runs at 20 Hz, subscribes to arm telemetry and both wrist cameras,
runs the ACT policy, and sends arm_joints commands.

WARNING: Do NOT run teleop.py and this node simultaneously.
Both push to :5555. Interleaved commands are dangerous.

Usage:
    python act_policy_node.py --checkpoint checkpoints/act_epoch_0100.pt
    python act_policy_node.py --checkpoint checkpoints/act_epoch_0100.pt --dry-run
    python act_policy_node.py --checkpoint checkpoints/act_epoch_0100.pt --device cpu

Safety:
    - Waits for all three sources before sending any commands.
    - Skips command if any source is >200 ms stale.
    - --dry-run: runs inference and logs actions, does NOT push to :5555.
    - Warns if forward pass takes >80 ms (near 100 ms watchdog limit).
    - Clamps predicted positions to [action_min, action_max] from training data.
    - Clamps per-step delta to --max-delta rad/step (velocity guard).
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import torch
import yaml
import zmq
from PIL import Image

try:
    import rerun as rr
    import rerun.blueprint as rrb
    _rerun_available = True
except ImportError:
    _rerun_available = False

# Allow running from repo root or python/nodes/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.act_model import ACTPolicy

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ARM_JOINTS = ["gantry_base", "gantry_mid", "gantry_end", "wrist_pitch", "wrist_roll", "gripper"]
NUM_JOINTS = 6

# Fallback gains if teleop.yaml is not found
_DEFAULT_KP = [75.0, 65.0, 10.0, 5.0, 10.0, 10.0]
_DEFAULT_KD = [7.0, 5.5, 0.2, 0.2, 2.0, 2.0]


def _load_gains():
    """Load arm KP/KD from config/teleop.yaml, falling back to defaults."""
    here = Path(__file__).parent
    for candidate in [
        here / ".." / ".." / "config" / "teleop.yaml",
        Path("config") / "teleop.yaml",
    ]:
        p = candidate.resolve()
        if p.exists():
            cfg = yaml.safe_load(p.read_text()) or {}
            gantry = cfg.get("gantry", {})
            kp = gantry.get("kp", _DEFAULT_KP)
            kd = gantry.get("kd", _DEFAULT_KD)
            return list(kp), list(kd)
    return list(_DEFAULT_KP), list(_DEFAULT_KD)

# ImageNet normalization
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

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
    """Drain a ZMQ SUB socket, return latest message or None."""
    latest = None
    while True:
        try:
            raw = sock.recv_string(zmq.NOBLOCK)
            latest = json.loads(raw)
        except zmq.Again:
            break
        except Exception:
            break
    return latest


# ---------------------------------------------------------------------------
# Observation helpers
# ---------------------------------------------------------------------------

def extract_qpos(telem: dict) -> Optional[np.ndarray]:
    """Extract [6] float32 arm joint positions from telemetry."""
    if telem is None or "motors" not in telem:
        return None
    motors = telem["motors"]
    qpos = []
    for joint in ARM_JOINTS:
        m = motors.get(joint)
        if m is None:
            return None
        qpos.append(float(m.get("position", 0.0)))
    return np.array(qpos, dtype=np.float32)


def extract_torques(telem: dict) -> Optional[np.ndarray]:
    """Extract [6] float32 arm joint torques from telemetry."""
    if telem is None or "motors" not in telem:
        return None
    motors = telem["motors"]
    torques = []
    for joint in ARM_JOINTS:
        m = motors.get(joint)
        if m is None:
            return None
        torques.append(float(m.get("torque", 0.0)))
    return np.array(torques, dtype=np.float32)


def decode_image(msg: dict, target_size=(320, 240)) -> Optional[np.ndarray]:
    """Decode camera message to uint8 [H, W, 3]. target_size = (width, height)."""
    color = msg.get("color", {})
    data_b64 = color.get("data")
    if data_b64 is None:
        return None
    jpeg_bytes = base64.b64decode(data_b64)
    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)


def normalize_image(img: np.ndarray) -> np.ndarray:
    """uint8 [H,W,3] → float32 [3,H,W] ImageNet normalized."""
    x = img.astype(np.float32) / 255.0
    x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
    return x.transpose(2, 0, 1)


def normalize_qpos(qpos: np.ndarray, stats: dict) -> np.ndarray:
    return (qpos - stats["qpos_mean"]) / stats["qpos_std"]


def normalize_qcmd(qcmd: np.ndarray, stats: dict) -> np.ndarray:
    return (qcmd - stats["qcmd_mean"]) / stats["qcmd_std"]


def normalize_torques(torques: np.ndarray, stats: dict) -> np.ndarray:
    return (torques - stats["torque_mean"]) / stats["torque_std"]


def denormalize_actions(actions: np.ndarray, stats: dict) -> np.ndarray:
    """actions: [..., 6] z-score normalized → absolute joint positions."""
    return actions * stats["action_std"] + stats["action_mean"]


def apply_safety_limits(
    action: np.ndarray,
    qpos_raw: np.ndarray,
    stats: dict,
    max_delta: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Apply two layers of safety clamping to a predicted action.

    Layer 1 — absolute bounds: clamp to [action_min, action_max] from training.
    The policy was trained only on positions within this range; anything outside
    is extrapolation.

    Layer 2 — delta (velocity) guard: clamp |action - current_qpos| ≤ max_delta
    per joint per step. Prevents violent jumps even if the absolute position is
    technically within range. At 20 Hz, max_delta=0.05 rad/step ≈ 1 rad/s.

    Returns:
        clamped_action: [6] float32, safe to send
        delta_clamped:  [6] bool, True for joints where delta was the binding constraint
    """
    # Layer 1: absolute bounds
    action = np.clip(action, stats["action_min"], stats["action_max"])

    # Layer 2: delta per joint
    delta = action - qpos_raw
    delta_clamped = np.abs(delta) > max_delta
    delta_clipped = np.clip(delta, -max_delta, max_delta)
    action = qpos_raw + delta_clipped

    return action.astype(np.float32), delta_clamped


# ---------------------------------------------------------------------------
# Temporal ensemble
# ---------------------------------------------------------------------------

class TemporalEnsemble:
    """Maintain rolling buffer of predicted action chunks, compute ensemble action.

    At each timestep we have K recent chunks, each predicting future actions.
    The ensemble action at the current step is an exponentially-weighted
    average of all available predictions for this timestep.

    This smooths out jitter between successive chunk predictions.
    """

    def __init__(self, chunk_size: int, ensemble_steps: int, action_dim: int = 6):
        self.chunk_size = chunk_size
        self.ensemble_steps = ensemble_steps  # how many past chunks to keep
        self.action_dim = action_dim
        # Each entry: (chunk array [chunk_size, 6], age in steps)
        self._chunks: Deque = deque(maxlen=ensemble_steps)
        self._step = 0

    def add_chunk(self, chunk: np.ndarray):
        """Add a newly predicted chunk. chunk: [chunk_size, 6]."""
        self._chunks.append((chunk.copy(), 0))

    def get_action(self) -> Optional[np.ndarray]:
        """Compute exponentially-weighted ensemble action for current step.

        Returns [6] action, or None if no chunks available.
        """
        if not self._chunks:
            return None

        # Exponential weights: newer chunks get higher weight
        # weight = exp(-alpha * age), alpha chosen so oldest gets ~0.1x
        alpha = 0.1

        weighted_sum = np.zeros(self.action_dim, dtype=np.float64)
        weight_total = 0.0

        for i, (chunk, age) in enumerate(self._chunks):
            if age < self.chunk_size:
                w = np.exp(-alpha * age)
                weighted_sum += w * chunk[age]
                weight_total += w

        if weight_total < 1e-9:
            return None

        return (weighted_sum / weight_total).astype(np.float32)

    def step(self):
        """Advance all chunk ages by 1 step. Call once per control tick."""
        updated = []
        for chunk, age in self._chunks:
            updated.append((chunk, age + 1))
        self._chunks = deque(updated, maxlen=self.ensemble_steps)
        self._step += 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def load_checkpoint(path: str, device: torch.device):
    """Load checkpoint, return (policy, dataset_stats, config)."""
    ckpt = torch.load(path, map_location=device, weights_only=False)
    config = ckpt["config"]
    dataset_stats = ckpt["dataset_stats"]

    # Convert to float32 numpy (handles both numpy arrays and torch tensors)
    for k in dataset_stats:
        v = dataset_stats[k]
        if hasattr(v, "numpy"):
            dataset_stats[k] = v.cpu().numpy().astype(np.float32)
        elif hasattr(v, "astype"):
            dataset_stats[k] = v.astype(np.float32)

    state_dim = config.get("state_dim", 6)
    dim_feedforward = config.get("dim_feedforward", 2048)

    policy = ACTPolicy(
        chunk_size=config["chunk_size"],
        d_model=config["d_model"],
        dim_feedforward=dim_feedforward,
        z_dim=config["z_dim"],
        nhead=config["nhead"],
        num_encoder_layers=config["num_encoder_layers"],
        num_decoder_layers=config["num_decoder_layers"],
        kl_weight=config.get("kl_weight", 10.0),
        pretrained_encoder=False,  # weights loaded from checkpoint
        state_dim=state_dim,
    ).to(device)

    policy.load_state_dict(ckpt["model_state_dict"])
    policy.eval()

    return policy, dataset_stats, config


def _build_state_vector(
    qpos_norm: np.ndarray,
    state_dim: int,
    last_action: Optional[np.ndarray],
    qpos_raw: np.ndarray,
    torques_raw: Optional[np.ndarray],
    stats: dict,
) -> np.ndarray:
    """Build the extended state vector for inference.

    At inference, qcmd = last predicted action (same trick ALOHA uses with leader).
    First step uses qpos_raw as bootstrap (zero compliance = safe).
    """
    if state_dim == 6:
        return qpos_norm.copy()

    # qcmd = last_action if available, else bootstrap with current qpos
    qcmd_raw = last_action if last_action is not None else qpos_raw
    qcmd_norm = normalize_qcmd(qcmd_raw, stats)

    if state_dim == 12:
        return np.concatenate([qpos_norm, qcmd_norm])  # [12]

    # state_dim == 18
    tq_raw = torques_raw if torques_raw is not None else np.zeros(NUM_JOINTS, dtype=np.float32)
    tq_norm = normalize_torques(tq_raw, stats)
    return np.concatenate([qpos_norm, qcmd_norm, tq_norm])  # [18]


def main():
    parser = argparse.ArgumentParser(description="ACT Policy Inference Node")
    parser.add_argument("--checkpoint", required=True, help="Path to .pt checkpoint")
    parser.add_argument("--telem", default="tcp://localhost:5556")
    parser.add_argument("--cam-left", default="tcp://localhost:5563")
    parser.add_argument("--cam-right", default="tcp://localhost:5564")
    parser.add_argument("--cmd", default="tcp://localhost:5555")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run inference but do NOT send commands to :5555")
    parser.add_argument("--ensemble-steps", type=int, default=25,
                        help="Number of past chunks to ensemble (0 = disable)")
    parser.add_argument("--max-delta", type=float, default=0.3,
                        help="Max joint position change per step in rad "
                             "(velocity guard, default 0.3 = 6 rad/s at 20 Hz)")
    parser.add_argument("--ready-pose", default=None, dest="ready_pose",
                        help="Path to ready_pose.json (auto-detected from config/ if not set)")
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

    kp, kd = _load_gains()
    print(f"Arm gains: kp={kp}  kd={kd}")

    # Load model
    print("Loading checkpoint...")
    policy, dataset_stats, config = load_checkpoint(args.checkpoint, device)
    chunk_size = config["chunk_size"]
    state_dim = config.get("state_dim", 6)
    print(f"Policy loaded: chunk_size={chunk_size}, state_dim={state_dim}")

    # Print safety limits derived from training data — operator should verify these
    # look physically plausible before running live.
    print()
    print("Safety limits (from training data):")
    print(f"  {'Joint':<14} {'Min (rad)':>10}  {'Max (rad)':>10}  {'Range (rad)':>12}")
    print(f"  {'-'*14} {'-'*10}  {'-'*10}  {'-'*12}")
    for i, joint in enumerate(ARM_JOINTS):
        lo = dataset_stats["action_min"][i]
        hi = dataset_stats["action_max"][i]
        print(f"  {joint:<14} {lo:>10.3f}  {hi:>10.3f}  {hi-lo:>12.3f}")
    print(f"  Per-step delta limit: ±{args.max_delta:.3f} rad/step "
          f"(≈ ±{args.max_delta * 20:.1f} rad/s at 20 Hz)")
    print()

    # Load ready pose: prefer checkpoint's dataset_stats, fall back to JSON
    ready_pose: Optional[np.ndarray] = None
    if "ready_pose" in dataset_stats:
        ready_pose = dataset_stats["ready_pose"]
        print(f"Ready pose from checkpoint: {ready_pose}")
    elif args.ready_pose:
        rp_data = json.loads(Path(args.ready_pose).read_text())
        ready_pose = np.array(rp_data["positions"], dtype=np.float32)
        print(f"Ready pose from {args.ready_pose}: {ready_pose}")
    else:
        rp_path = Path(__file__).resolve().parent.parent.parent / "config" / "ready_pose.json"
        if rp_path.exists():
            rp_data = json.loads(rp_path.read_text())
            ready_pose = np.array(rp_data["positions"], dtype=np.float32)
            print(f"Ready pose from {rp_path.name}: {ready_pose}")
        else:
            print("No ready pose found — skipping pre-ramp")
    print()

    # ZMQ setup
    ctx = zmq.Context()

    telem_sub = ctx.socket(zmq.SUB)
    telem_sub.setsockopt(zmq.LINGER, 0)
    telem_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sub.connect(args.telem)

    left_sub = ctx.socket(zmq.SUB)
    left_sub.setsockopt(zmq.LINGER, 0)
    left_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    left_sub.connect(args.cam_left)

    right_sub = ctx.socket(zmq.SUB)
    right_sub.setsockopt(zmq.LINGER, 0)
    right_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    right_sub.connect(args.cam_right)

    cmd_push = None
    if not args.dry_run:
        cmd_push = ctx.socket(zmq.PUSH)
        cmd_push.setsockopt(zmq.LINGER, 0)
        cmd_push.connect(args.cmd)

    print(f"Subscribing to telem:    {args.telem}")
    print(f"Subscribing to left cam: {args.cam_left}")
    print(f"Subscribing to right cam:{args.cam_right}")
    if not args.dry_run:
        print(f"Pushing commands to:     {args.cmd}")
    print()

    # Rerun visualization — streams to a local viewer via IPC or network
    use_rerun = not args.no_rerun and _rerun_available
    if use_rerun:
        rr.init("aizee_policy")
        rr.spawn(memory_limit="512MiB")
        for jn in ARM_JOINTS:
            rr.set_time("frame", sequence=0)
            rr.log(f"qpos/{jn}", rr.Scalars(0.0))
            rr.log(f"action/{jn}", rr.Scalars(0.0))
            rr.log(f"error/{jn}", rr.Scalars(0.0))
        rr.send_blueprint(rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Spatial2DView(name="Left", origin="cameras/left"),
                    rrb.Spatial2DView(name="Right", origin="cameras/right"),
                    column_shares=[1, 1],
                ),
                rrb.TimeSeriesView(
                    name="Joint Positions (qpos=amber, action=green)",
                    contents=[f"qpos/{j}" for j in ARM_JOINTS]
                           + [f"action/{j}" for j in ARM_JOINTS],
                ),
                rrb.TimeSeriesView(
                    name="Per-Joint Error |action - qpos|",
                    contents=[f"error/{j}" for j in ARM_JOINTS],
                ),
                rrb.TimeSeriesView(
                    name="Inference Time (ms)",
                    contents=["inference_ms"],
                ),
                row_shares=[2, 2, 1, 1],
            )
        ))
        print("Rerun: viewer spawned")
    elif not args.no_rerun:
        print("NOTE: rerun not installed — run with --no-rerun to suppress this")
        use_rerun = False

    # Temporal ensemble
    use_ensemble = args.ensemble_steps > 0
    ensemble = TemporalEnsemble(chunk_size, args.ensemble_steps) if use_ensemble else None

    # State
    last_telem_time = 0.0
    last_left_time = 0.0
    last_right_time = 0.0
    latest_telem = None
    latest_left = None
    latest_right = None
    last_action: Optional[np.ndarray] = None  # tracks last predicted action for qcmd

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

    # Rerun series colors
    if use_rerun:
        for jn in ARM_JOINTS:
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

            left_msg = drain_sub(left_sub)
            if left_msg is not None:
                latest_left = left_msg
                last_left_time = t0

            right_msg = drain_sub(right_sub)
            if right_msg is not None:
                latest_right = right_msg
                last_right_time = t0

            # Keyboard
            key = get_key()
            if key == "Q":
                if quit_requested:
                    # Second Q — begin shutdown
                    if not args.dry_run and cmd_push is not None:
                        qpos_now = extract_qpos(latest_telem) if latest_telem else None
                        shutdown_target = qpos_now.copy() if qpos_now is not None else np.zeros(NUM_JOINTS)
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
                    qpos_now = extract_qpos(latest_telem) if latest_telem else None
                    hold_position = qpos_now.copy() if qpos_now is not None else last_action
                    print("\n[PAUSED] Shutdown cancelled. Holding position.")
                elif paused:
                    paused = False
                    hold_position = None
                    if ensemble is not None:
                        ensemble._chunks.clear()
                    print("\n[RESUMED] Inference active.")
                else:
                    paused = True
                    qpos_now = extract_qpos(latest_telem) if latest_telem else None
                    hold_position = qpos_now.copy() if qpos_now is not None else last_action
                    print("\n[PAUSED] Holding position. SPACE=resume  Q=quit")

            # Shutdown state: hold 1s, ramp to zero, disable
            if shutting_down and not args.dry_run and cmd_push is not None:
                dt = tick
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    if shutdown_target is not None:
                        cmd_push.send_string(json.dumps({
                            "type": "arm_joints",
                            "positions": shutdown_target.tolist(),
                            "velocities": [0.0] * NUM_JOINTS,
                            "kp": kp, "kd": kd,
                            "torques": [0.0] * NUM_JOINTS,
                        }), zmq.NOBLOCK)
                else:
                    max_change = 0.2 * dt
                    if shutdown_target is None:
                        shutdown_target = np.zeros(NUM_JOINTS)
                    qpos_now = extract_qpos(latest_telem) if latest_telem else shutdown_target
                    ref = qpos_now if qpos_now is not None else shutdown_target
                    for i in range(NUM_JOINTS):
                        shutdown_target[i] = (0.0 if abs(shutdown_target[i]) < max_change
                                              else shutdown_target[i] - np.sign(shutdown_target[i]) * max_change)
                    ramp_done = np.all(np.abs(shutdown_target) < 0.01)
                    actual_close = qpos_now is None or np.all(np.abs(qpos_now) < 0.05)
                    if ramp_done and actual_close:
                        cmd_push.send_string(json.dumps({
                            "type": "disable",
                            "motor_ids": list(ARM_JOINTS),
                        }), zmq.NOBLOCK)
                        print("\n[SHUTDOWN] Motors disabled. Exiting.")
                        break
                    else:
                        delta = np.clip(shutdown_target - ref, -0.3, 0.3)
                        q_cmd = ref + delta
                        cmd_push.send_string(json.dumps({
                            "type": "arm_joints",
                            "positions": q_cmd.tolist(),
                            "velocities": [0.0] * NUM_JOINTS,
                            "kp": kp, "kd": kd,
                            "torques": [0.0] * NUM_JOINTS,
                        }), zmq.NOBLOCK)
                        print(f"\r[SHUTDOWN] Returning to zero... max_err={np.max(np.abs(shutdown_target)):.3f}    ",
                              end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # While paused, hold position and skip inference
            if paused and not args.dry_run and cmd_push is not None and hold_position is not None:
                cmd_push.send_string(json.dumps({
                    "type": "arm_joints",
                    "positions": hold_position.tolist(),
                    "velocities": [0.0] * NUM_JOINTS,
                    "kp": kp, "kd": kd,
                    "torques": [0.0] * NUM_JOINTS,
                }), zmq.NOBLOCK)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue
            elif paused:
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Check freshness
            telem_age = t0 - last_telem_time if last_telem_time > 0 else 999.0
            left_age = t0 - last_left_time if last_left_time > 0 else 999.0
            right_age = t0 - last_right_time if last_right_time > 0 else 999.0

            telem_ok = telem_age < STALE_THRESH
            left_ok = left_age < STALE_THRESH
            right_ok = right_age < STALE_THRESH
            all_ok = telem_ok and left_ok and right_ok

            if not all_sources_ready:
                if all_ok:
                    all_sources_ready = True
                    print("\nAll sources ready.")

                    # Ramp to ready pose before inference
                    if ready_pose is not None and not args.dry_run and cmd_push is not None:
                        print("Enabling motors and ramping to ready pose...")
                        cmd_push.send_string(json.dumps({
                            "type": "enable",
                            "motor_ids": list(ARM_JOINTS),
                        }), zmq.NOBLOCK)
                        time.sleep(0.1)

                        ramp_delta = args.ramp_speed / 20.0  # per tick at 20 Hz
                        while True:
                            rt0 = time.monotonic()
                            # Keep sockets drained
                            t = drain_sub(telem_sub)
                            if t is not None:
                                latest_telem = t
                                last_telem_time = rt0
                            drain_sub(left_sub)
                            drain_sub(right_sub)

                            qpos = extract_qpos(latest_telem)
                            if qpos is None:
                                time.sleep(tick)
                                continue

                            err = np.max(np.abs(qpos - ready_pose))
                            delta = np.clip(ready_pose - qpos, -ramp_delta, ramp_delta)
                            q_cmd = qpos + delta
                            cmd_push.send_string(json.dumps({
                                "type": "arm_joints",
                                "positions": q_cmd.tolist(),
                                "velocities": [0.0] * NUM_JOINTS,
                                "kp": kp, "kd": kd,
                                "torques": [0.0] * NUM_JOINTS,
                            }), zmq.NOBLOCK)

                            print(f"\rRamping to ready pose... err={err:.3f} rad   ",
                                  end="", flush=True)
                            if err < 0.03:
                                print("\nReady pose reached. Stabilising cameras...")
                                time.sleep(0.5)
                                # Drain stale frames accumulated during ramp
                                drain_sub(left_sub)
                                drain_sub(right_sub)
                                drain_sub(telem_sub)
                                break

                            elapsed = time.monotonic() - rt0
                            time.sleep(max(0, tick - elapsed))

                    print("Starting inference loop.")
                else:
                    # Status while waiting
                    flags = []
                    if not telem_ok:
                        flags.append("telem")
                    if not left_ok:
                        flags.append("left_cam")
                    if not right_ok:
                        flags.append("right_cam")
                    print(f"\rWaiting: missing {', '.join(flags)}    ", end="", flush=True)
                    elapsed = time.monotonic() - t0
                    time.sleep(max(0, tick - elapsed))
                    continue

            if not all_ok:
                stale = []
                if not telem_ok:
                    stale.append(f"telem({telem_age*1000:.0f}ms)")
                if not left_ok:
                    stale.append(f"left_cam({left_age*1000:.0f}ms)")
                if not right_ok:
                    stale.append(f"right_cam({right_age*1000:.0f}ms)")
                print(f"\r[SKIP] Stale sources: {', '.join(stale)}    ", end="", flush=True)
                if ensemble is not None:
                    ensemble.step()
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Decode observations
            qpos_raw = extract_qpos(latest_telem)
            left_img = decode_image(latest_left)
            right_img = decode_image(latest_right)

            if qpos_raw is None or left_img is None or right_img is None:
                fails = []
                if qpos_raw is None: fails.append("qpos")
                if left_img is None: fails.append("left_img")
                if right_img is None: fails.append("right_img")
                print(f"\r[SKIP] Decode failed: {','.join(fails)}    ", end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Extract torques (needed for state_dim=18)
            torques_raw = extract_torques(latest_telem) if state_dim >= 18 else None

            # Normalize
            qpos_norm = normalize_qpos(qpos_raw, dataset_stats)
            left_norm = normalize_image(left_img)
            right_norm = normalize_image(right_img)

            # Build extended state vector
            state_vec = _build_state_vector(
                qpos_norm, state_dim, last_action, qpos_raw, torques_raw, dataset_stats,
            )

            # Build tensors
            qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)          # [1, 6]
            state_t = torch.from_numpy(state_vec).unsqueeze(0).to(device)          # [1, state_dim]
            left_t = torch.from_numpy(left_norm).unsqueeze(0).to(device)           # [1, 3, H, W]
            right_t = torch.from_numpy(right_norm).unsqueeze(0).to(device)         # [1, 3, H, W]

            # Inference
            infer_start = time.monotonic()
            with torch.no_grad():
                pred_chunk = policy.select_action(qpos_t, state_t, left_t, right_t)  # [1, chunk_size, 6]
            infer_time = time.monotonic() - infer_start

            if infer_time > WARN_LATENCY:
                print(
                    f"\n[WARN] Inference took {infer_time*1000:.1f}ms "
                    f"(>{WARN_LATENCY*1000:.0f}ms threshold)"
                )

            # Denormalize
            pred_np = pred_chunk[0].cpu().numpy()                    # [chunk_size, 6]
            pred_abs = denormalize_actions(pred_np, dataset_stats)   # [chunk_size, 6]

            # Temporal ensemble
            if use_ensemble:
                ensemble.add_chunk(pred_abs)
                action = ensemble.get_action()
                ensemble.step()
            else:
                action = pred_abs[0]  # first action only

            if action is None:
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Safety: clamp to training range + delta limit
            action, delta_clamped = apply_safety_limits(
                action, qpos_raw, dataset_stats, args.max_delta
            )

            # Track last action for qcmd in next step
            last_action = action.copy()

            # Rerun logging
            if use_rerun:
                rr.set_time("frame", sequence=rr_frame)
                for j, jn in enumerate(ARM_JOINTS):
                    rr.log(f"qpos/{jn}", rr.Scalars(float(qpos_raw[j])))
                    rr.log(f"action/{jn}", rr.Scalars(float(action[j])))
                    rr.log(f"error/{jn}", rr.Scalars(float(abs(action[j] - qpos_raw[j]))))
                rr.log("inference_ms", rr.Scalars(infer_time * 1000.0))
                if rr_frame % 2 == 0:
                    if left_img is not None:
                        rr.log("cameras/left", rr.Image(left_img))
                    if right_img is not None:
                        rr.log("cameras/right", rr.Image(right_img))
                rr_frame += 1

            # Log
            action_str = " ".join(f"{v:+6.3f}" for v in action)
            qpos_str = " ".join(f"{v:+6.3f}" for v in qpos_raw)
            clamp_str = ""
            if delta_clamped.any():
                clamped_joints = [ARM_JOINTS[i] for i in range(NUM_JOINTS) if delta_clamped[i]]
                clamp_str = f" [DELTA CLAMPED: {','.join(clamped_joints)}]"
            print(
                f"\r[{'DRY' if args.dry_run else 'CMD'}#{commands_sent:5d}] "
                f"qpos:[{qpos_str}] → act:[{action_str}] "
                f"inf={infer_time*1000:.1f}ms{clamp_str}    ",
                end="", flush=True,
            )

            # Send command (unless dry-run)
            if not args.dry_run and cmd_push is not None:
                cmd = {
                    "type": "arm_joints",
                    "positions": action.tolist(),
                    "velocities": [0.0] * NUM_JOINTS,
                    "kp": kp,
                    "kd": kd,
                    "torques": [0.0] * NUM_JOINTS,
                }
                try:
                    cmd_push.send_string(json.dumps(cmd), zmq.NOBLOCK)
                    commands_sent += 1
                except zmq.Again:
                    print("\n[WARN] Command queue full, skipped")
                except zmq.ZMQError as e:
                    print(f"\n[ERROR] ZMQ send error: {e}")

            # Sleep remainder
            elapsed = time.monotonic() - t0
            remaining = tick - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        telem_sub.close()
        left_sub.close()
        right_sub.close()
        if cmd_push is not None:
            cmd_push.close()
        ctx.term()
        print(f"Done. Commands sent: {commands_sent}")


if __name__ == "__main__":
    main()
