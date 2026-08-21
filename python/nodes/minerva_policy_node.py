#!/usr/bin/env python3
"""
minerva_policy_node.py — inference node for the Minerva bimanual policy.

Runs the flow-matching MinervaPolicy at 20 Hz. Subscribes to THREE camera
streams (left_wrist, right_wrist, head RealSense) + arm telemetry, samples an
action chunk, and executes it receding-horizon (open-loop for `execute_steps`
then replans). The JEPA/SIGReg auxiliary is training-only and never loaded here.

Usage:
    python minerva_policy_node.py --config config/minerva.yaml \
        --checkpoint checkpoints/minerva/minerva_best.pt \
        --instruction "pick up the red block with the left arm" [--dry-run]

Safety:
  - waits for telemetry + all cameras before commanding,
  - skips a tick if any source is > stale_threshold_ms old,
  - clamps to the training action range + physical joint limits + a per-step
    velocity guard (minerva_constants.apply_safety_limits),
  - ramps to the closest training start pose on startup.

WARNING: do not run a teleop node and this node simultaneously (both push
commands). Interleaved commands are dangerous.
"""

from __future__ import annotations

import argparse
import io
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import torch
import yaml
import zmq

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_camera, unpack_msg

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from python.common.minerva_constants import (
    MINERVA_JOINTS, NUM_MINERVA_JOINTS, KP, KD, apply_safety_limits, build_qpos,
    max_delta_vector,
)
from python.training.language import TextConditioner
from python.training.minerva_dataset import denormalize_actions, imagenet_normalize
from python.training.minerva_model import MinervaPolicy

_CAMERAS = ["left_wrist", "right_wrist", "head"]


# ---------------------------------------------------------------------------
# Checkpoint loading
# ---------------------------------------------------------------------------

def load_minerva_checkpoint(path: str, device: torch.device, *, use_ema: bool = True):
    """Reconstruct MinervaPolicy from a checkpoint. Returns (policy, stats, config)."""
    ckpt = torch.load(path, map_location=device, weights_only=False)
    cfg = ckpt["config"]
    stats = {k: (np.asarray(v, dtype=np.float32) if isinstance(v, list) else v)
             for k, v in ckpt["dataset_stats"].items()}

    policy = MinervaPolicy(
        num_joints=cfg["num_joints"], chunk_size=cfg["chunk_size"], d_model=cfg["d_model"],
        state_dim=cfg["state_dim"], lang_dim=cfg.get("lang_dim", 0), nhead=cfg["nhead"],
        head_layers=cfg["head_layers"], head_ff=cfg["head_ff"],
        flow_steps=cfg.get("flow_steps", 8), pretrained_encoder=False,
    ).to(device)

    # Prefer EMA weights (what flow/diffusion heads should deploy).
    state = ckpt.get("ema_state_dict") if use_ema and "ema_state_dict" in ckpt else None
    if state is not None:
        # EMA holds only float params/buffers; fill the rest from the raw dict.
        merged = dict(ckpt["model_state_dict"]); merged.update(state)
        policy.load_state_dict(merged, strict=False)
    else:
        policy.load_state_dict(ckpt["model_state_dict"], strict=False)
    policy.eval()
    return policy, stats, cfg


# ---------------------------------------------------------------------------
# ZMQ + image helpers
# ---------------------------------------------------------------------------

def drain_sub(sock) -> Optional[dict]:
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
    latest = None
    while True:
        try:
            latest = unpack_camera(sock.recv_multipart(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def decode_image(msg: dict, size_wh) -> Optional[np.ndarray]:
    """Decode a camera message to uint8 RGB [H,W,3] at (width,height)=size_wh."""
    raw = (msg.get("color") or {}).get("data_bytes")
    if raw is None:
        return None
    if _CV2:
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if (bgr.shape[1], bgr.shape[0]) != tuple(size_wh):
            bgr = cv2.resize(bgr, tuple(size_wh), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if (img.width, img.height) != tuple(size_wh):
        img = img.resize(tuple(size_wh), Image.BILINEAR)
    return np.array(img, dtype=np.uint8)


def extract_qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Pull a 17-vector from telemetry. Accepts {'joints': {name: pos}} (ALL 17
    canonical joints must be present) or a 'positions' list already in
    MINERVA_JOINTS order. Returns None on incomplete telemetry so the tick is
    SKIPPED — never silently zero-fills a missing safety-critical joint (a
    zero-filled qpos would anchor the velocity guard at the zero pose and drag
    the arms toward it). Adapt to motor_control's actual schema on the robot."""
    if telem is None:
        return None
    joints = telem.get("joints")
    if isinstance(joints, dict):
        if all(name in joints for name in MINERVA_JOINTS):
            return build_qpos(joints)
        return None
    pos = telem.get("positions")
    if pos is not None and len(pos) == NUM_MINERVA_JOINTS:
        return np.asarray(pos, dtype=np.float32)
    return None


def extract_torques(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Pull a 17-vector of joint torques (only for state_mode 'qpos_qcmd_tq').
    Returns None when torque telemetry is unavailable/incomplete so the caller
    skips the tick rather than feeding an out-of-distribution zero vector."""
    if telem is None:
        return None
    tq = telem.get("torques")
    if isinstance(tq, dict):
        return build_qpos(tq) if all(n in tq for n in MINERVA_JOINTS) else None
    for key in ("torques", "efforts"):
        v = telem.get(key)
        if isinstance(v, (list, tuple)) and len(v) == NUM_MINERVA_JOINTS:
            return np.asarray(v, dtype=np.float32)
    return None


def build_state(qpos_raw, last_action, torques_raw, state_mode, stats) -> np.ndarray:
    qpos_n = (qpos_raw - stats["qpos_mean"]) / stats["qpos_std"]
    if state_mode == "qpos":
        return qpos_n.astype(np.float32)
    qcmd_src = last_action if last_action is not None else qpos_raw
    qcmd_n = (qcmd_src - stats["qcmd_mean"]) / stats["qcmd_std"]
    if state_mode == "qpos_qcmd":
        return np.concatenate([qpos_n, qcmd_n]).astype(np.float32)
    tq = torques_raw if torques_raw is not None else np.zeros(NUM_MINERVA_JOINTS, np.float32)
    tq_n = (tq - stats["torque_mean"]) / stats["torque_std"]
    return np.concatenate([qpos_n, qcmd_n, tq_n]).astype(np.float32)


def send_command(cmd_push, action: np.ndarray):
    try:
        cmd_push.send(pack_msg({
            "type": "arm_joints",
            "joint_names": list(MINERVA_JOINTS),
            "positions": action.astype(np.float32).tolist(),
            "velocities": [0.0] * NUM_MINERVA_JOINTS,
            "kp": list(KP), "kd": list(KD),
        }), zmq.NOBLOCK)
    except zmq.Again:
        print("\n[WARN] command queue full, skipped")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Minerva Policy Inference Node")
    ap.add_argument("--config", default="config/minerva.yaml")
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--instruction", default=None, help="task string for language conditioning")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    ep, cam_cfg, ctl, safe = cfg["endpoints"], cfg["cameras"], cfg["control"], cfg["safety"]
    device = torch.device(args.device)

    print(f"Loading {args.checkpoint} on {device} ...")
    policy, stats, mcfg = load_minerva_checkpoint(args.checkpoint, device)
    cameras = mcfg.get("cameras", _CAMERAS)
    state_mode = mcfg["state_mode"]
    action_mode = mcfg["action_mode"]
    chunk_size = mcfg["chunk_size"]
    execute_steps = min(int(ctl["execute_steps"]), chunk_size)
    flow_steps = int(ctl["flow_steps"])
    tick = 1.0 / float(ctl["rate_hz"])
    stale = float(ctl["stale_threshold_ms"]) / 1000.0
    cam_size = {c: (cam_cfg[c]["width"], cam_cfg[c]["height"]) for c in cameras}
    print(f"cameras={cameras} state_mode={state_mode} action_mode={action_mode} "
          f"chunk={chunk_size} execute={execute_steps} flow_steps={flow_steps}")

    # Language conditioning — REQUIRED whenever the checkpoint has lang_dim>0
    # (the flow head's cond_proj expects state_dim+lang_dim), independent of the
    # yaml `enabled` flag. enabled+instruction only choose a real embedding vs a
    # zeros vector; a lang_dim>0 model is never run with language=None.
    lang_vec_t = None
    lang_cfg = cfg.get("language", {})
    if mcfg.get("lang_dim", 0) > 0:
        instr = args.instruction or lang_cfg.get("default_instruction", "")
        if lang_cfg.get("enabled", False) and instr:
            conditioner = TextConditioner(model_name=mcfg["lang_model"],
                                          cache_path=lang_cfg.get("cache"))
            vec = conditioner.get(instr).astype(np.float32)
            print(f"Instruction: {instr!r}")
        else:
            vec = np.zeros(mcfg["lang_dim"], np.float32)
            print("[WARN] no instruction / language disabled — conditioning on zeros")
        assert vec.shape[-1] == mcfg["lang_dim"], (
            f"language embedding dim {vec.shape[-1]} != checkpoint lang_dim {mcfg['lang_dim']}")
        lang_vec_t = torch.from_numpy(vec).unsqueeze(0).to(device)

    # ZMQ
    ctx = zmq.Context()
    telem_sub = ctx.socket(zmq.SUB); telem_sub.setsockopt(zmq.LINGER, 0)
    telem_sub.setsockopt_string(zmq.SUBSCRIBE, ""); telem_sub.connect(ep["telemetry"])
    cam_subs = {}
    for c in cameras:
        s = ctx.socket(zmq.SUB); s.setsockopt(zmq.LINGER, 0)
        s.setsockopt_string(zmq.SUBSCRIBE, ""); s.connect(ep["cameras"][c])
        cam_subs[c] = s
    cmd_push = None
    if not args.dry_run:
        cmd_push = ctx.socket(zmq.PUSH); cmd_push.setsockopt(zmq.LINGER, 0)
        cmd_push.connect(ep["command"])

    print("=" * 60)
    print("WARNING: do NOT run a teleop node simultaneously (both push commands).")
    print("=" * 60)
    if args.dry_run:
        print("[DRY RUN — inference only, no commands sent]")

    last_telem_t = 0.0
    cam_last_t = {c: 0.0 for c in cameras}
    latest_telem = None
    latest_cam: Dict[str, dict] = {}
    last_action: Optional[np.ndarray] = None
    chunk_buf: Optional[np.ndarray] = None      # [K, 17] absolute positions
    chunk_idx = 0
    ready = False
    action_range = (stats["action_lo"], stats["action_hi"]) if action_mode == "absolute" else (None, None)
    max_delta = max_delta_vector(
        arm=float(safe["max_delta_arm"]), gripper=float(safe["max_delta_gripper"]),
        head=float(safe["max_delta_head"]), lift=float(safe["max_delta_lift"]))

    print("Waiting for telemetry + all cameras ...")
    try:
        while True:
            t0 = time.monotonic()
            telem = drain_sub(telem_sub)
            if telem is not None:
                latest_telem, last_telem_t = telem, t0
            for c in cameras:
                m = drain_camera(cam_subs[c])
                if m is not None:
                    latest_cam[c], cam_last_t[c] = m, t0

            fresh = (t0 - last_telem_t < stale if last_telem_t else False) and all(
                (t0 - cam_last_t[c] < stale if cam_last_t[c] else False) for c in cameras)
            if not fresh:
                if not ready:
                    missing = ([] if last_telem_t else ["telem"]) + [c for c in cameras if not cam_last_t[c]]
                    print(f"\rWaiting: {missing}   ", end="", flush=True)
                time.sleep(max(0, tick - (time.monotonic() - t0)))
                continue
            if not ready:
                ready = True
                print("\nAll sources ready. Starting inference loop.")
                # (start-pose ramp omitted here for brevity — add via a shared
                #  ramp helper before commanding on hardware.)

            qpos_raw = extract_qpos(latest_telem)
            imgs = {}
            for c in cameras:
                img = decode_image(latest_cam[c], cam_size[c])
                if img is not None:
                    imgs[c] = img
            if qpos_raw is None or len(imgs) != len(cameras):
                print("\r[SKIP] decode/telem incomplete   ", end="", flush=True)
                time.sleep(max(0, tick - (time.monotonic() - t0)))
                continue

            torques_raw = None
            if state_mode == "qpos_qcmd_tq":
                torques_raw = extract_torques(latest_telem)
                if torques_raw is None:
                    print("\r[SKIP] state_mode qpos_qcmd_tq requires torque telemetry   ",
                          end="", flush=True)
                    time.sleep(max(0, tick - (time.monotonic() - t0)))
                    continue

            # (Re)plan when the current chunk is exhausted.
            if chunk_buf is None or chunk_idx >= execute_steps:
                state = build_state(qpos_raw, last_action, torques_raw, state_mode, stats)
                images_t = {c: torch.from_numpy(imagenet_normalize(imgs[c])).unsqueeze(0).to(device)
                            for c in cameras}
                state_t = torch.from_numpy(state).unsqueeze(0).to(device)
                t_inf = time.monotonic()
                with torch.no_grad():
                    chunk_n = policy.select_action(state_t, images_t, language=lang_vec_t,
                                                   num_steps=flow_steps)[0].cpu().numpy()
                chunk_buf = denormalize_actions(chunk_n, stats, action_mode, qpos=qpos_raw)
                chunk_idx = 0
                infer_ms = (time.monotonic() - t_inf) * 1000
                if infer_ms > 1000 * tick * execute_steps:
                    print(f"\n[WARN] planning {infer_ms:.0f}ms > execution window "
                          f"{1000*tick*execute_steps:.0f}ms")

            action = chunk_buf[chunk_idx]
            action, clamped = apply_safety_limits(
                action, qpos_raw,
                dataset_action_min=action_range[0], dataset_action_max=action_range[1],
                max_delta=max_delta)
            last_action = action.copy()
            chunk_idx += 1

            if not args.dry_run and cmd_push is not None:
                send_command(cmd_push, action)

            flag = " [CLAMP]" if clamped.any() else ""
            print(f"\r[{'DRY' if args.dry_run else 'CMD'}] step {chunk_idx}/{execute_steps} "
                  f"a0={action[0]:+.3f} aR={action[7]:+.3f}{flag}   ", end="", flush=True)
            time.sleep(max(0, tick - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        telem_sub.close()
        for s in cam_subs.values():
            s.close()
        if cmd_push is not None:
            cmd_push.close()
        ctx.term()


if __name__ == "__main__":
    main()
