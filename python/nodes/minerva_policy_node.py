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
  - requires fresh telemetry from both arm controllers,
  - treats inactive head/lift slots as historical model placeholders; nonzero
    predictions for those slots are rejected before sending.

The policy does not enable motors or perform a startup ramp. Its commands use
the same shared session/backend as collection, with one endpoint per arm.

WARNING: do not run a teleop node and this node simultaneously (both push
commands). Interleaved commands are dangerous.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional
from urllib.parse import urlsplit

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
from robot_platform.transport.wire import unpack_camera
from robot_platform.control import Observation, RobotSession, StaleTelemetryError, ZmqRobotClient
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings, merge_settings

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from python.common.minerva_constants import (
    MINERVA_JOINTS, NUM_MINERVA_JOINTS, KP, KD, apply_safety_limits,
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


def model_observation(observation: Observation, state_mode: str):
    """Adapt a shared observation to the historical Minerva model input.

    Only explicitly inactive slots become zero, matching collected v6/v7 data.
    An absent/stale active joint always rejects the entire observation. Torque
    is required only for models whose state actually includes torque.
    """
    if observation.joint_names != tuple(MINERVA_JOINTS):
        raise ValueError("Minerva checkpoint requires the canonical 17-joint order")
    if state_mode not in ("qpos", "qpos_qcmd", "qpos_qcmd_tq"):
        raise ValueError(f"Unsupported checkpoint state_mode: {state_mode!r}")
    active = np.asarray(observation.active_mask, dtype=bool)
    if not all(valid for enabled, valid in zip(active, observation.position_valid_mask) if enabled):
        return None
    qpos = np.where(active, observation.positions, 0.0).astype(np.float32)
    torques = None
    if state_mode == "qpos_qcmd_tq":
        if not all(valid for enabled, valid in zip(active, observation.torque_valid_mask) if enabled):
            return None
        torques = np.where(active, observation.torques, 0.0).astype(np.float32)
    return qpos, torques


def policy_settings(cfg: dict, *, settings_paths=(), host=None, endpoint_overrides=None):
    """Resolve legacy policy YAML and shared settings without network access.

    YAML supplies the old left endpoint/cameras and control timeout. The second
    arm uses the packaged 5575/5576 defaults unless explicitly configured. JSON
    overlays, launcher settings, then explicit CLI flags have increasing priority.
    Matching YAML hostnames become placeholders so a host overlay moves both
    arms and cameras together while preserving explicitly separate camera hosts.
    """
    spec = load_robot("minerva")
    ep = cfg.get("endpoints", {})
    primary = ep.get("arms", {}).get("left", {}).get("command", ep.get("command", "tcp://localhost:5555"))
    yaml_host = urlsplit(primary).hostname or "localhost"

    def template(value):
        if isinstance(value, dict):
            return {key: template(item) for key, item in value.items()}
        if isinstance(value, str) and value.startswith("tcp://"):
            parsed = urlsplit(value)
            if parsed.hostname == yaml_host and parsed.port is not None:
                return f"tcp://{{host}}:{parsed.port}"
        return value

    overrides = {"host": yaml_host, "endpoints": {"cameras": template(ep.get("cameras", {}))},
                 "control": {"stale_after_s": float(cfg["control"]["stale_threshold_ms"]) / 1000.0}}
    if "arms" in ep:
        overrides["endpoints"]["arms"] = template(ep["arms"])
    else:
        left = {key: template(ep[key]) for key in ("command", "telemetry") if key in ep}
        overrides["endpoints"]["arms"] = {"left": left}
    for filename in settings_paths:
        overlay = json.loads(Path(filename).expanduser().read_text(encoding="utf-8"))
        if not isinstance(overlay, dict):
            raise ValueError(f"Settings must be an object: {filename}")
        overrides = merge_settings(overrides, overlay)
    launcher = os.environ.get("ROBOT_PLATFORM_SETTINGS_JSON")
    if launcher:
        overlay = json.loads(launcher)
        if not isinstance(overlay, dict):
            raise ValueError("ROBOT_PLATFORM_SETTINGS_JSON must be an object")
        overrides = merge_settings(overrides, overlay)
    if host is not None:
        old_host = overrides["host"]

        def retarget(value):
            if isinstance(value, dict):
                return {key: retarget(item) for key, item in value.items()}
            if isinstance(value, str) and value.startswith("tcp://"):
                parsed = urlsplit(value)
                if parsed.hostname == old_host and parsed.port is not None:
                    return f"tcp://{{host}}:{parsed.port}"
            return value

        overrides["endpoints"] = retarget(overrides["endpoints"])
        overrides["host"] = host
    if endpoint_overrides:
        overrides = merge_settings(overrides, {"endpoints": {"arms": endpoint_overrides}})
    return spec, load_settings(spec, overrides=overrides)


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


def policy_target(action: np.ndarray, session: RobotSession) -> np.ndarray:
    """Reject unsupported motion and normalize only floating-point residue.

    Zero-range action denormalization can leave tiny residuals in inactive
    placeholder slots; values within 1e-5 are rounded to zero. Material head or
    lift predictions require a robot definition/backend that supports them.
    """
    action = np.asarray(action, dtype=np.float32).copy()
    if action.shape != (len(session.joint_names),) or not np.isfinite(action).all():
        raise ValueError("Policy action must be a finite canonical joint vector")
    active = np.asarray(session.active_mask, dtype=bool)
    unsupported = np.flatnonzero(~active & (np.abs(action) > 1e-5))
    if unsupported.size:
        names = [session.joint_names[index] for index in unsupported]
        raise ValueError(f"Policy predicts motion for inactive joints: {names}")
    action[~active] = 0.0
    return action


def send_command(client: ZmqRobotClient, action: np.ndarray, *, require_torque: bool = True):
    """Route one canonical target to the two real arm controllers."""
    action = policy_target(action, client.session)
    active = np.asarray(client.session.active_mask, dtype=bool)
    client.send_target(action, kp=np.where(active, KP, 0.0), kd=np.where(active, KD, 0.0),
                       require_torque=require_torque)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description="Minerva Policy Inference Node")
    ap.add_argument("--config", default="config/minerva.yaml")
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--instruction", default=None, help="task string for language conditioning")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--settings", action="append", default=[], help="shared deployment JSON overlay (repeatable)")
    ap.add_argument("--host", help="override the robot host")
    for side in ("left", "right"):
        ap.add_argument(f"--{side}-cmd", help=f"{side} arm command endpoint")
        ap.add_argument(f"--{side}-telem", help=f"{side} arm telemetry endpoint")
    args = ap.parse_args(argv)

    cfg = yaml.safe_load(Path(args.config).read_text())
    endpoint_overrides = {}
    for side in ("left", "right"):
        values = {key: getattr(args, f"{side}_{flag}") for key, flag in (("command", "cmd"), ("telemetry", "telem"))}
        endpoint_overrides[side] = {key: value for key, value in values.items() if value is not None}
    spec, settings = policy_settings(cfg, settings_paths=args.settings, host=args.host,
                                     endpoint_overrides=endpoint_overrides)
    ep, cam_cfg, ctl, safe = settings["endpoints"], cfg["cameras"], cfg["control"], cfg["safety"]
    device = torch.device(args.device)

    print(f"Loading {args.checkpoint} on {device} ...")
    policy, stats, mcfg = load_minerva_checkpoint(args.checkpoint, device)
    cameras = mcfg.get("cameras", _CAMERAS)
    state_mode = mcfg["state_mode"]
    action_mode = mcfg["action_mode"]
    chunk_size = mcfg["chunk_size"]
    execute_steps = min(int(ctl["execute_steps"]), chunk_size)
    if execute_steps < 1 or mcfg["num_joints"] != len(spec.joints):
        raise ValueError("Checkpoint must use the Minerva 17-joint layout and positive execution steps")
    flow_steps = int(ctl["flow_steps"])
    tick = 1.0 / float(ctl["rate_hz"])
    stale = float(settings["control"]["stale_after_s"])
    cam_size = {c: (cam_cfg[c]["width"], cam_cfg[c]["height"]) for c in cameras}
    print(f"cameras={cameras} state_mode={state_mode} action_mode={action_mode} "
          f"chunk={chunk_size} execute={execute_steps} flow_steps={flow_steps}")
    if state_mode not in ("qpos", "qpos_qcmd", "qpos_qcmd_tq"):
        raise ValueError(f"Unsupported checkpoint state_mode: {state_mode!r}")
    require_torque = state_mode == "qpos_qcmd_tq"
    action_range = (stats["action_lo"], stats["action_hi"]) if action_mode == "absolute" else (None, None)
    max_delta = max_delta_vector(
        arm=float(safe["max_delta_arm"]), gripper=float(safe["max_delta_gripper"]),
        head=float(safe["max_delta_head"]), lift=float(safe["max_delta_lift"]))

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

    # The shared backend owns both arm transports; cameras remain policy inputs.
    session = RobotSession(spec, settings, max_age_s=stale)
    ctx = zmq.Context()
    client = ZmqRobotClient(session, context=ctx, receive_only=args.dry_run)
    cam_subs = {}
    try:
        client.connect()
        for c in cameras:
            s = ctx.socket(zmq.SUB)
            cam_subs[c] = s
            s.setsockopt(zmq.LINGER, 0)
            s.setsockopt_string(zmq.SUBSCRIBE, "")
            s.connect(ep["cameras"][c])
    except BaseException:
        client.close()
        for s in cam_subs.values():
            s.close()
        ctx.term()
        raise

    print("=" * 60)
    print("WARNING: do NOT run a teleop node simultaneously (both push commands).")
    print("=" * 60)
    if args.dry_run:
        print("[DRY RUN — inference only, no commands sent]")

    cam_last_t = {c: 0.0 for c in cameras}
    latest_cam: Dict[str, dict] = {}
    last_action: Optional[np.ndarray] = None
    chunk_buf: Optional[np.ndarray] = None      # [K, 17] absolute positions
    chunk_idx = 0
    ready = False

    print("Waiting for telemetry + all cameras ...")
    try:
        while True:
            t0 = time.monotonic()
            observation = client.poll()
            measured = model_observation(observation, state_mode)
            for c in cameras:
                m = drain_camera(cam_subs[c])
                if m is not None:
                    latest_cam[c], cam_last_t[c] = m, t0

            fresh = measured is not None and all(
                (t0 - cam_last_t[c] < stale if cam_last_t[c] else False) for c in cameras)
            if not fresh:
                chunk_buf, last_action = None, None
                if not ready:
                    missing = ([] if measured is not None else ["arm telemetry"]) + [c for c in cameras if not cam_last_t[c]]
                    print(f"\rWaiting: {missing}   ", end="", flush=True)
                time.sleep(max(0, tick - (time.monotonic() - t0)))
                continue
            if not ready:
                ready = True
                print("\nAll sources ready. Starting inference loop.")

            qpos_raw, torques_raw = measured
            imgs = {}
            for c in cameras:
                img = decode_image(latest_cam[c], cam_size[c])
                if img is not None:
                    imgs[c] = img
            if len(imgs) != len(cameras):
                chunk_buf, last_action = None, None
                print("\r[SKIP] decode/telem incomplete   ", end="", flush=True)
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

            # Inference can outlast the stale threshold. Drain fresh telemetry
            # again, then clamp against the measured pose at command time.
            measured = model_observation(client.poll(), state_mode)
            camera_stale = any(time.monotonic() - cam_last_t[c] >= stale for c in cameras)
            if measured is None or camera_stale:
                chunk_buf, last_action = None, None
                print("\r[SKIP] observations expired during inference   ", end="", flush=True)
                time.sleep(max(0, tick - (time.monotonic() - t0)))
                continue
            qpos_raw, _ = measured
            action = policy_target(chunk_buf[chunk_idx], session)
            action, clamped = apply_safety_limits(
                action, qpos_raw,
                dataset_action_min=action_range[0], dataset_action_max=action_range[1],
                max_delta=max_delta)
            action = policy_target(action, session)
            try:
                if args.dry_run:
                    session.route_target(action, require_torque=require_torque)
                else:
                    send_command(client, action, require_torque=require_torque)
            except StaleTelemetryError:
                chunk_buf, last_action = None, None
                print("\r[SKIP] arm telemetry expired before sending   ", end="", flush=True)
                time.sleep(max(0, tick - (time.monotonic() - t0)))
                continue
            last_action = action.copy()
            chunk_idx += 1

            flag = " [CLAMP]" if clamped.any() else ""
            print(f"\r[{'DRY' if args.dry_run else 'CMD'}] step {chunk_idx}/{execute_steps} "
                  f"a0={action[0]:+.3f} aR={action[7]:+.3f}{flag}   ", end="", flush=True)
            time.sleep(max(0, tick - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        client.close()
        for s in cam_subs.values():
            s.close()
        ctx.term()


if __name__ == "__main__":
    main()
