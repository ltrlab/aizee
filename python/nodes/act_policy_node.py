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
        except (json.JSONDecodeError, Exception):
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


def denormalize_actions(actions: np.ndarray, stats: dict) -> np.ndarray:
    """actions: [..., 6] in [-1,1] → absolute joint positions."""
    mn = stats["action_min"]
    rng = stats["action_range"]
    return (actions + 1.0) / 2.0 * rng + mn


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
    ckpt = torch.load(path, map_location=device)
    config = ckpt["config"]
    dataset_stats = ckpt["dataset_stats"]

    # Convert numpy arrays to float32
    for k in dataset_stats:
        if hasattr(dataset_stats[k], "astype"):
            dataset_stats[k] = dataset_stats[k].astype(np.float32)

    policy = ACTPolicy(
        chunk_size=config["chunk_size"],
        d_model=config["d_model"],
        z_dim=config["z_dim"],
        nhead=config["nhead"],
        num_encoder_layers=config["num_encoder_layers"],
        num_decoder_layers=config["num_decoder_layers"],
        kl_weight=config.get("kl_weight", 10.0),
        pretrained_encoder=False,  # weights loaded from checkpoint
    ).to(device)

    policy.load_state_dict(ckpt["model_state_dict"])
    policy.eval()

    return policy, dataset_stats, config


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
    parser.add_argument("--max-delta", type=float, default=0.05,
                        help="Max joint position change per step in rad "
                             "(velocity guard, default 0.05 = 1 rad/s at 20 Hz)")
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
    print(f"Policy loaded: chunk_size={chunk_size}")

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

    tick = 1.0 / 20.0   # 20 Hz
    commands_sent = 0
    all_sources_ready = False

    print("Waiting for all sources to become ready...")

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
                    print("All sources ready. Starting inference loop.")
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
                print("\r[SKIP] Decode failed    ", end="", flush=True)
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))
                continue

            # Normalize
            qpos_norm = normalize_qpos(qpos_raw, dataset_stats)
            left_norm = normalize_image(left_img)
            right_norm = normalize_image(right_img)

            # Build tensors
            qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)         # [1, 6]
            left_t = torch.from_numpy(left_norm).unsqueeze(0).to(device)          # [1, 3, H, W]
            right_t = torch.from_numpy(right_norm).unsqueeze(0).to(device)        # [1, 3, H, W]

            # Inference
            infer_start = time.monotonic()
            with torch.no_grad():
                pred_chunk = policy.select_action(qpos_t, left_t, right_t)  # [1, chunk_size, 6]
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
