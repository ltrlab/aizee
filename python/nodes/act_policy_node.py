#!/usr/bin/env python3
"""
act_policy_node.py — ACT policy inference node for AIZEE arm.

Runs at 20 Hz, subscribes to arm telemetry and both wrist cameras,
runs the ACT policy, and sends swivel + arm_joints commands.

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
import base64
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
    import rerun as rr
    import rerun.blueprint as rrb
    _rerun_available = True
except ImportError:
    _rerun_available = False

# Allow running from repo root or python/nodes/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.act_model import ACTPolicy
from python.scripts.record_replay import (
    ARM_JOINTS, POLICY_JOINTS, NUM_POLICY_JOINTS,
    SWIVEL_KP, SWIVEL_KD,
    extract_policy_qpos, extract_policy_torques,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
NUM_ARM_JOINTS = len(ARM_JOINTS)   # 6 — dimension of the arm_joints ZMQ message

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

# State mode → per-joint feature block count
_STATE_MODE_K = {"qpos": 1, "qpos_qcmd": 2, "qpos_qcmd_tq": 3}


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
# Image / normalization
# ---------------------------------------------------------------------------

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


def normalize_qpos(qpos, stats):    return (qpos - stats["qpos_mean"])   / stats["qpos_std"]
def normalize_qcmd(qcmd, stats):    return (qcmd - stats["qcmd_mean"])   / stats["qcmd_std"]
def normalize_torques(tq, stats):   return (tq   - stats["torque_mean"]) / stats["torque_std"]


def denormalize_actions(
    actions: np.ndarray, stats: dict, action_mode: str, qpos: Optional[np.ndarray] = None
) -> np.ndarray:
    """Denormalize a predicted chunk → absolute joint positions.

    actions shape: [chunk_size, J] or [J]
    action_mode = "absolute": inverse of z-score normalization.
    action_mode = "relative": inverse z-score gives per-step delta, then add qpos.
    """
    if action_mode == "absolute":
        return actions * stats["action_std"] + stats["action_mean"]
    if qpos is None:
        raise ValueError("qpos anchor required for relative action mode")
    deltas = actions * stats["rel_action_std"] + stats["rel_action_mean"]
    if deltas.ndim == 2 and qpos.ndim == 1:
        return deltas + qpos[None, :]
    return deltas + qpos


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
    """Split the 7-DOF action into a swivel command + arm_joints command."""
    swivel_pos = float(action[0])
    arm_pos = action[1:1 + NUM_ARM_JOINTS].tolist()

    try:
        cmd_push.send_string(json.dumps({
            "type": "swivel",
            "position": swivel_pos,
            "velocity": 0.0,
            "kp": SWIVEL_KP,
            "kd": SWIVEL_KD,
            "torque": 0.0,
        }), zmq.NOBLOCK)
        cmd_push.send_string(json.dumps({
            "type": "arm_joints",
            "positions": arm_pos,
            "velocities": [0.0] * NUM_ARM_JOINTS,
            "kp": arm_kp, "kd": arm_kd,
            "torques": [0.0] * NUM_ARM_JOINTS,
        }), zmq.NOBLOCK)
    except zmq.Again:
        print("\n[WARN] Command queue full, skipped")


# ---------------------------------------------------------------------------
# Checkpoint loading
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

    num_joints = config.get("num_joints", NUM_POLICY_JOINTS)
    state_mode = config.get("state_mode", "qpos_qcmd")
    state_dim = config.get("state_dim", num_joints * _STATE_MODE_K.get(state_mode, 1))
    action_mode = config.get("action_mode", "absolute")
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
        pretrained_encoder=False,
        num_joints=num_joints,
        state_dim=state_dim,
    ).to(device)

    policy.load_state_dict(ckpt["model_state_dict"])
    policy.eval()

    # Annotate the derived fields onto config for the caller's convenience
    config["num_joints"] = num_joints
    config["state_mode"] = state_mode
    config["state_dim"] = state_dim
    config["action_mode"] = action_mode

    return policy, dataset_stats, config


# ---------------------------------------------------------------------------
# State / ready-pose helpers
# ---------------------------------------------------------------------------

def _build_state_vector(
    qpos_norm: np.ndarray,
    state_mode: str,
    last_action: Optional[np.ndarray],
    qpos_raw: np.ndarray,
    torques_raw: Optional[np.ndarray],
    stats: dict,
    num_joints: int,
) -> np.ndarray:
    """Build the extended state vector for inference.

    At inference, qcmd = last predicted action (same trick ALOHA uses with leader).
    First step uses qpos_raw as bootstrap (zero compliance = safe).
    """
    if state_mode == "qpos":
        return qpos_norm.copy()

    qcmd_raw = last_action if last_action is not None else qpos_raw
    qcmd_norm = normalize_qcmd(qcmd_raw, stats)

    if state_mode == "qpos_qcmd":
        return np.concatenate([qpos_norm, qcmd_norm])

    # qpos_qcmd_tq
    tq_raw = torques_raw if torques_raw is not None else np.zeros(num_joints, dtype=np.float32)
    tq_norm = normalize_torques(tq_raw, stats)
    return np.concatenate([qpos_norm, qcmd_norm, tq_norm])


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
    left_sub,
    right_sub,
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
        drain_sub(left_sub)
        drain_sub(right_sub)
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
    parser.add_argument("--cam-left", default="tcp://localhost:5563")
    parser.add_argument("--cam-right", default="tcp://localhost:5564")
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
    print(f"Arm gains: kp={arm_kp}  kd={arm_kd}")
    print(f"Swivel gains: kp={SWIVEL_KP}  kd={SWIVEL_KD}")

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
        print(f"[WARN] checkpoint num_joints={num_joints} but POLICY_JOINTS has "
              f"{NUM_POLICY_JOINTS} entries. Command dispatch assumes "
              f"index 0 = swivel and indices 1..{NUM_ARM_JOINTS} = ARM_JOINTS.")

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
                rrb.Horizontal(
                    rrb.Spatial2DView(name="Left", origin="cameras/left"),
                    rrb.Spatial2DView(name="Right", origin="cameras/right"),
                    column_shares=[1, 1],
                ),
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
    last_left_time = 0.0
    last_right_time = 0.0
    latest_telem: Optional[dict] = None
    latest_left: Optional[dict] = None
    latest_right: Optional[dict] = None
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
                            cmd_push.send_string(json.dumps({
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

                    # Pick closest start pose and ramp to it
                    qpos_now = extract_policy_qpos(latest_telem)
                    if qpos_now is not None:
                        target = _closest_start_pose(qpos_now, dataset_stats)
                        print(f"Closest start pose: {target}")
                        if not args.dry_run and cmd_push is not None:
                            try:
                                cmd_push.send_string(json.dumps({
                                    "type": "enable",
                                    "motor_ids": list(POLICY_JOINTS[:num_joints]),
                                }), zmq.NOBLOCK)
                            except zmq.Again:
                                pass
                            time.sleep(0.1)
                            _ramp_to_pose(
                                target, cmd_push=cmd_push,
                                telem_sub=telem_sub, left_sub=left_sub, right_sub=right_sub,
                                arm_kp=arm_kp, arm_kd=arm_kd,
                                ramp_speed_rad_s=args.ramp_speed, tick=tick,
                            )
                            print("\nReady pose reached. Stabilising cameras...")
                            time.sleep(0.5)
                            drain_sub(left_sub)
                            drain_sub(right_sub)
                            drain_sub(telem_sub)
                    print("Starting inference loop.")
                else:
                    flags = []
                    if not telem_ok: flags.append("telem")
                    if not left_ok: flags.append("left_cam")
                    if not right_ok: flags.append("right_cam")
                    print(f"\rWaiting: missing {', '.join(flags)}    ", end="", flush=True)
                    elapsed = time.monotonic() - t0
                    time.sleep(max(0, tick - elapsed))
                    continue

            if not all_ok:
                stale = []
                if not telem_ok: stale.append(f"telem({telem_age*1000:.0f}ms)")
                if not left_ok: stale.append(f"left_cam({left_age*1000:.0f}ms)")
                if not right_ok: stale.append(f"right_cam({right_age*1000:.0f}ms)")
                print(f"\r[SKIP] Stale sources: {', '.join(stale)}    ", end="", flush=True)
                if ensemble is not None:
                    ensemble.step()
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Decode observations
            qpos_raw = extract_policy_qpos(latest_telem)
            left_img = decode_image(latest_left) if latest_left else None
            right_img = decode_image(latest_right) if latest_right else None

            if qpos_raw is None or left_img is None or right_img is None:
                fails = []
                if qpos_raw is None: fails.append("qpos")
                if left_img is None: fails.append("left_img")
                if right_img is None: fails.append("right_img")
                print(f"\r[SKIP] Decode failed: {','.join(fails)}    ", end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            torques_raw = (extract_policy_torques(latest_telem)
                           if state_mode == "qpos_qcmd_tq" else None)

            # Normalize observations
            qpos_norm = normalize_qpos(qpos_raw, dataset_stats)
            left_norm = normalize_image(left_img)
            right_norm = normalize_image(right_img)

            state_vec = _build_state_vector(
                qpos_norm, state_mode, last_action, qpos_raw, torques_raw,
                dataset_stats, num_joints,
            )

            qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)
            state_t = torch.from_numpy(state_vec).unsqueeze(0).to(device)
            left_t = torch.from_numpy(left_norm).unsqueeze(0).to(device)
            right_t = torch.from_numpy(right_norm).unsqueeze(0).to(device)

            # Inference
            infer_start = time.monotonic()
            with torch.no_grad():
                pred_chunk = policy.select_action(qpos_t, state_t, left_t, right_t)
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
                    rr.log("cameras/left", rr.Image(left_img))
                    rr.log("cameras/right", rr.Image(right_img))
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
        left_sub.close()
        right_sub.close()
        if cmd_push is not None:
            cmd_push.close()
        ctx.term()
        print(f"Done. Commands sent: {commands_sent}")


if __name__ == "__main__":
    main()
