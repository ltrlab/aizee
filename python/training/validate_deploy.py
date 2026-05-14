"""
validate_deploy.py — Pre-deployment sanity check for ACT / ACT-JEPA checkpoints.

Run this before pushing a checkpoint to the Jetson. It exercises only the
inference path (so any leakage of training-only JEPA modules is caught),
checks that the model produces finite, in-bounds actions with reasonable
per-step deltas, confirms determinism, measures inference latency, and
— if you supply --data-dir — compares predicted actions against recorded
ground-truth actions on real episodes so you know the model didn't
load-but-produce-garbage.

Failures are printed with [FAIL] prefixes and a non-zero exit code. Pass
--strict to also fail on [WARN] items.

Usage:
    python -m python.training.validate_deploy \\
        --checkpoint checkpoints/jepa/act_jepa_best.pt \\
        --data-dir episodes/ --device cuda

    # Strip JEPA-only weights and save a deploy-ready ACT-only checkpoint:
    python -m python.training.validate_deploy \\
        --checkpoint checkpoints/jepa/act_jepa_best.pt \\
        --strip checkpoints/jepa/act_jepa_best_deploy.pt
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import h5py
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.act_model import ACTPolicy
from python.training.jepa_model import ACTJEPAPolicy


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

class Report:
    """Collects check results and prints a summary at the end."""

    def __init__(self):
        self.results: List[Tuple[str, str, str]] = []  # (severity, name, detail)

    def _add(self, severity: str, name: str, detail: str) -> None:
        tag = {"PASS": "[ PASS ]", "FAIL": "[ FAIL ]",
               "WARN": "[ WARN ]", "INFO": "[ INFO ]"}[severity]
        print(f"  {tag} {name}" + (f" - {detail}" if detail else ""))
        self.results.append((severity, name, detail))

    def passed(self, name: str, detail: str = "") -> None: self._add("PASS", name, detail)
    def failed(self, name: str, detail: str = "") -> None: self._add("FAIL", name, detail)
    def warned(self, name: str, detail: str = "") -> None: self._add("WARN", name, detail)
    def info(self, name: str, detail: str = "") -> None:   self._add("INFO", name, detail)

    def section(self, title: str) -> None:
        print(f"\n=== {title} ===")

    def summary(self, *, strict: bool) -> int:
        n_pass = sum(1 for s, _, _ in self.results if s == "PASS")
        n_fail = sum(1 for s, _, _ in self.results if s == "FAIL")
        n_warn = sum(1 for s, _, _ in self.results if s == "WARN")
        print("\n" + "-" * 64)
        print(f"PASS: {n_pass}    WARN: {n_warn}    FAIL: {n_fail}")
        if n_fail > 0 or (strict and n_warn > 0):
            print("RESULT: NOT SAFE TO DEPLOY")
            return 1
        if n_warn > 0:
            print("RESULT: deploy-able, but review WARNings first")
            return 0
        print("RESULT: OK to deploy")
        return 0


# ---------------------------------------------------------------------------
# Checkpoint loading + model rebuild
# ---------------------------------------------------------------------------

_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_STATE_MODE_K = {"qpos": 1, "qpos_qcmd": 2, "qpos_qcmd_tq": 3}


def _stats_to_numpy(stats: Dict) -> Dict[str, np.ndarray]:
    out = {}
    for k, v in stats.items():
        if hasattr(v, "cpu"):
            out[k] = v.cpu().numpy().astype(np.float32)
        elif hasattr(v, "astype"):
            out[k] = v.astype(np.float32)
        else:
            out[k] = v
    return out


def _detect_model_kind(config: Dict, state_dict: Dict[str, torch.Tensor]) -> str:
    """Return 'act_jepa' if the checkpoint has JEPA modules, else 'act'."""
    if config.get("model") == "act_jepa":
        return "act_jepa"
    if any(k.startswith("jepa_predictor.") or k.startswith("sigreg.")
           for k in state_dict):
        return "act_jepa"
    return "act"


def load_for_validation(
    path: str, device: torch.device, report: Report,
) -> Tuple[torch.nn.Module, Dict, Dict, str]:
    """Load a checkpoint and rebuild the matching policy on `device`."""
    report.section("1. Checkpoint integrity")

    ckpt_path = Path(path)
    if not ckpt_path.exists():
        report.failed("checkpoint file exists", str(ckpt_path))
        sys.exit(1)
    report.passed("checkpoint file exists", str(ckpt_path))

    try:
        ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    except Exception as e:
        report.failed("torch.load", repr(e))
        sys.exit(1)
    report.passed("torch.load")

    for required in ("model_state_dict", "dataset_stats", "config"):
        if required not in ckpt:
            report.failed(f"key '{required}' in checkpoint")
            sys.exit(1)
    report.passed("checkpoint has model_state_dict / dataset_stats / config")

    state_dict = ckpt["model_state_dict"]
    config = dict(ckpt["config"])  # copy so we can annotate
    stats = _stats_to_numpy(ckpt["dataset_stats"])

    kind = _detect_model_kind(config, state_dict)
    report.info("model kind", kind)

    num_joints = config.get("num_joints", 7)
    state_mode = config.get("state_mode", "qpos_qcmd")
    state_dim = config.get("state_dim",
                           num_joints * _STATE_MODE_K.get(state_mode, 1))
    action_mode = config.get("action_mode", "absolute")
    config["num_joints"] = num_joints
    config["state_mode"] = state_mode
    config["state_dim"] = state_dim
    config["action_mode"] = action_mode

    common_kwargs = dict(
        chunk_size=config["chunk_size"],
        d_model=config["d_model"],
        dim_feedforward=config.get("dim_feedforward", 2048),
        z_dim=config["z_dim"],
        nhead=config["nhead"],
        num_encoder_layers=config["num_encoder_layers"],
        num_decoder_layers=config["num_decoder_layers"],
        kl_weight=config.get("kl_weight", 10.0),
        pretrained_encoder=False,
        num_joints=num_joints,
        state_dim=state_dim,
    )

    report.section("2. Architecture rebuild")
    if kind == "act_jepa":
        policy = ACTJEPAPolicy(
            **common_kwargs,
            predictor_layers=config.get("predictor_layers", 4),
            predictor_heads=config.get("predictor_heads", 8),
            predictor_ff=config.get("predictor_ff", 1024),
            lambda_obs=config.get("lambda_obs", 0.5),
            lambda_reg=config.get("lambda_reg", 0.05),
            sigreg_slices=config.get("sigreg_slices", 1024),
            sigreg_points=config.get("sigreg_points", 17),
        ).to(device)
    else:
        policy = ACTPolicy(**common_kwargs).to(device)

    missing, unexpected = policy.load_state_dict(state_dict, strict=False)
    if missing:
        report.failed("missing keys", f"{len(missing)} (e.g. {missing[:3]})")
    else:
        report.passed("no missing keys")
    if unexpected:
        report.failed("unexpected keys", f"{len(unexpected)} (e.g. {unexpected[:3]})")
    else:
        report.passed("no unexpected keys")
    policy.eval()
    return policy, stats, config, kind


# ---------------------------------------------------------------------------
# Inference-time helpers (replicate the deploy normalization pipeline)
# ---------------------------------------------------------------------------

def _norm_image(img_u8: np.ndarray) -> np.ndarray:
    x = img_u8.astype(np.float32) / 255.0
    x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
    return x.transpose(2, 0, 1)


def _build_state(
    qpos_raw: np.ndarray,
    qcmd_raw: Optional[np.ndarray],
    torque_raw: Optional[np.ndarray],
    stats: Dict, state_mode: str,
) -> np.ndarray:
    qn = (qpos_raw - stats["qpos_mean"]) / stats["qpos_std"]
    if state_mode == "qpos":
        return qn.astype(np.float32)
    qc = qcmd_raw if qcmd_raw is not None else qpos_raw
    qcn = (qc - stats["qcmd_mean"]) / stats["qcmd_std"]
    if state_mode == "qpos_qcmd":
        return np.concatenate([qn, qcn]).astype(np.float32)
    tq = torque_raw if torque_raw is not None else np.zeros_like(qpos_raw)
    tqn = (tq - stats["torque_mean"]) / stats["torque_std"]
    return np.concatenate([qn, qcn, tqn]).astype(np.float32)


def _denormalize(actions_norm: np.ndarray, stats: Dict, action_mode: str,
                 qpos_raw: Optional[np.ndarray]) -> np.ndarray:
    if action_mode == "absolute":
        return actions_norm * stats["action_std"] + stats["action_mean"]
    deltas = actions_norm * stats["rel_action_std"] + stats["rel_action_mean"]
    if deltas.ndim == 2 and qpos_raw.ndim == 1:
        return deltas + qpos_raw[None, :]
    return deltas + qpos_raw


def _make_synthetic_input(stats: Dict, config: Dict, device: torch.device):
    """Build a single plausible inference input from dataset stats.

    Returns (qpos_t, state_t, img_t, qpos_raw) — single-camera (gripper)
    pipeline since 2026-05-13. Image shape matches the deployed capture
    resolution (1024x768).
    """
    qpos_raw = stats["ready_pose"].astype(np.float32)
    state_np = _build_state(qpos_raw, None, None, stats, config["state_mode"])
    qpos_norm = (qpos_raw - stats["qpos_mean"]) / stats["qpos_std"]
    img = np.full((768, 1024, 3), 128, dtype=np.uint8)  # neutral gray
    img_norm = _norm_image(img)

    qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)
    state_t = torch.from_numpy(state_np).unsqueeze(0).to(device)
    img_t = torch.from_numpy(img_norm).unsqueeze(0).to(device)
    return qpos_t, state_t, img_t, qpos_raw


# ---------------------------------------------------------------------------
# Individual checks
# ---------------------------------------------------------------------------

def check_inference_isolation(
    policy: torch.nn.Module, inputs: Tuple, kind: str, report: Report,
) -> None:
    """Confirm select_action does not touch training-only JEPA modules."""
    report.section("3. Inference-path isolation")
    qpos, state, img, _ = inputs

    if kind != "act_jepa":
        report.info("not an ACT-JEPA checkpoint, skipping JEPA-isolation check")
        return

    hits = {"jepa_predictor": 0, "sigreg": 0}
    handles = [
        policy.jepa_predictor.register_forward_pre_hook(
            lambda m, i, name="jepa_predictor": hits.__setitem__(name, hits[name] + 1)
        ),
        policy.sigreg.register_forward_pre_hook(
            lambda m, i, name="sigreg": hits.__setitem__(name, hits[name] + 1)
        ),
    ]
    try:
        _ = policy.select_action(qpos, state, img)
    finally:
        for h in handles:
            h.remove()

    if hits["jepa_predictor"] == 0 and hits["sigreg"] == 0:
        report.passed("select_action never touches jepa_predictor / sigreg")
    else:
        report.failed("JEPA modules ran during inference",
                      f"predictor={hits['jepa_predictor']}  sigreg={hits['sigreg']}")


def check_output_sanity(
    policy: torch.nn.Module, inputs: Tuple, stats: Dict, config: Dict, report: Report,
) -> Optional[np.ndarray]:
    """Forward pass + shape / finiteness / bounds checks."""
    report.section("4. Output sanity")
    qpos, state, img, qpos_raw = inputs
    chunk_size = config["chunk_size"]
    J = config["num_joints"]

    with torch.no_grad():
        out = policy.select_action(qpos, state, img)
    expected = (1, chunk_size, J)
    if tuple(out.shape) != expected:
        report.failed("output shape", f"got {tuple(out.shape)}, expected {expected}")
        return None
    report.passed("output shape", f"{tuple(out.shape)}")

    actions_np = out.squeeze(0).cpu().numpy()
    if not np.all(np.isfinite(actions_np)):
        report.failed("output finiteness", "contains NaN/Inf")
        return None
    report.passed("output finiteness")

    abs_actions = _denormalize(actions_np, stats, config["action_mode"], qpos_raw)

    # Bounds check
    a_min = stats["action_min"]
    a_max = stats["action_max"]
    out_of_range = ((abs_actions < a_min[None, :]) | (abs_actions > a_max[None, :])).any(axis=0)
    n_oor = int(out_of_range.sum())
    if n_oor == 0:
        report.passed("predicted absolute actions within learned [min, max]")
    else:
        worst_joint = int(np.argmax(np.abs(abs_actions).max(axis=0) - a_max))
        report.warned("predicted actions exceed learned range on some joints",
                      f"{n_oor}/{J} joints; deploy-time safety clamp will catch this")

    # Per-step delta check (absolute -> consecutive deltas)
    per_step = np.diff(abs_actions, axis=0)
    max_step = float(np.abs(per_step).max()) if per_step.size else 0.0
    # Heuristic threshold: 0.5 rad ~= 29 deg per chunk step is already aggressive
    if max_step > 0.5:
        report.warned("large per-step delta inside chunk",
                      f"max |dq|={max_step:.3f} rad")
    else:
        report.passed("per-step deltas reasonable",
                      f"max |dq|={max_step:.3f} rad")
    return abs_actions


def check_determinism(
    policy: torch.nn.Module, inputs: Tuple, report: Report,
) -> None:
    report.section("5. Determinism (eval mode)")
    qpos, state, img, _ = inputs
    with torch.no_grad():
        a1 = policy.select_action(qpos, state, img)
        a2 = policy.select_action(qpos, state, img)
    diff = (a1 - a2).abs().max().item()
    if diff < 1e-6:
        report.passed("two passes match exactly", f"max |d|={diff:.2e}")
    elif diff < 1e-4:
        report.warned("two passes differ slightly", f"max |d|={diff:.2e}")
    else:
        report.failed("non-deterministic inference", f"max |d|={diff:.2e}")


def check_latency(
    policy: torch.nn.Module, inputs: Tuple, device: torch.device,
    n_iters: int, warn_ms: float, report: Report,
) -> None:
    report.section("6. Inference latency")
    qpos, state, img, _ = inputs

    # Warm-up
    for _ in range(5):
        with torch.no_grad():
            _ = policy.select_action(qpos, state, img)
    if device.type == "cuda":
        torch.cuda.synchronize()

    t0 = time.perf_counter()
    for _ in range(n_iters):
        with torch.no_grad():
            _ = policy.select_action(qpos, state, img)
    if device.type == "cuda":
        torch.cuda.synchronize()
    t = (time.perf_counter() - t0) / n_iters * 1000.0

    detail = f"{t:.2f} ms/call  (device={device})"
    if t > warn_ms:
        report.warned(f"latency exceeds {warn_ms:.0f} ms on this device", detail)
    else:
        report.passed("latency", detail)
    report.info("note", "Jetson Orin Nano latency will differ - re-run on target")


def check_deploy_loader_compat(
    config: Dict, state_dict: Dict[str, torch.Tensor], kind: str, report: Report,
) -> None:
    """Confirm the deployment-side loader (act_policy_node.load_checkpoint)
    will accept this checkpoint. It builds a plain ACTPolicy, so JEPA
    state-dict keys would be 'unexpected' under strict=True."""
    report.section("7. Deployment loader (act_policy_node) compatibility")
    num_joints = config["num_joints"]
    state_dim = config["state_dim"]
    act_only = ACTPolicy(
        chunk_size=config["chunk_size"],
        d_model=config["d_model"],
        dim_feedforward=config.get("dim_feedforward", 2048),
        z_dim=config["z_dim"],
        nhead=config["nhead"],
        num_encoder_layers=config["num_encoder_layers"],
        num_decoder_layers=config["num_decoder_layers"],
        kl_weight=config.get("kl_weight", 10.0),
        pretrained_encoder=False,
        num_joints=num_joints,
        state_dim=state_dim,
    )
    missing, unexpected = act_only.load_state_dict(state_dict, strict=False)
    extra_jepa = [k for k in unexpected if k.startswith(("jepa_predictor.", "sigreg."))]
    extra_other = [k for k in unexpected if k not in extra_jepa]

    if missing:
        report.failed("deploy loader missing keys",
                      f"{len(missing)} keys — checkpoint is broken (e.g. {missing[:3]})")
        return
    if extra_other:
        report.failed("deploy loader has unexpected non-JEPA keys",
                      f"{extra_other[:3]}")
        return
    if extra_jepa:
        report.warned(
            "JEPA-only weights present in checkpoint",
            f"{len(extra_jepa)} keys will be ignored by ACTPolicy. "
            "Re-save with --strip for a clean deploy checkpoint.",
        )
    else:
        report.passed("clean ACT-only state dict")


def check_episode_fidelity(
    policy: torch.nn.Module, stats: Dict, config: Dict, device: torch.device,
    data_dir: str, n_frames: int, report: Report,
) -> None:
    """Open-loop replay on real episodes: compare predicted vs ground-truth actions."""
    report.section("8. Behavioral fidelity (real episodes)")
    paths = sorted(Path(data_dir).glob("episode_*.hdf5"))
    if not paths:
        report.warned("no episodes found", data_dir)
        return

    state_mode = config["state_mode"]
    action_mode = config["action_mode"]
    J = config["num_joints"]
    chunk_size = config["chunk_size"]

    per_joint_err: List[np.ndarray] = []
    for path in paths[:2]:  # cap at 2 episodes for speed
        with h5py.File(path, "r") as f:
            T = f["observations/qpos"].shape[0]
            n = min(n_frames, T - chunk_size)
            if n <= 0:
                continue
            idxs = np.linspace(0, T - chunk_size - 1, num=n, dtype=np.int64)
            for t in idxs:
                qpos = f["observations/qpos"][t]
                qcmd = (f["observations/qcmd"][t]
                        if "observations/qcmd" in f else None)
                torques = (f["observations/torques"][t]
                           if "observations/torques" in f else None)
                gripper = f["observations/images/gripper"][t]
                gt_chunk = f["actions"][t:t + chunk_size]

                qn = (qpos - stats["qpos_mean"]) / stats["qpos_std"]
                state_v = _build_state(qpos, qcmd, torques, stats, state_mode)
                gripper_n = _norm_image(gripper)

                qt = torch.from_numpy(qn).unsqueeze(0).to(device)
                st = torch.from_numpy(state_v).unsqueeze(0).to(device)
                gt = torch.from_numpy(gripper_n).unsqueeze(0).to(device)
                with torch.no_grad():
                    pred = policy.select_action(qt, st, gt).squeeze(0).cpu().numpy()
                pred_abs = _denormalize(pred, stats, action_mode, qpos)
                per_joint_err.append(np.abs(pred_abs - gt_chunk).mean(axis=0))

    if not per_joint_err:
        report.warned("no frames evaluated")
        return

    mae = np.stack(per_joint_err).mean(axis=0)
    overall = float(mae.mean())
    report.info("per-joint MAE (rad)",
                "[" + ", ".join(f"{v:.3f}" for v in mae) + "]")
    # Heuristic: if mean MAE > 0.20 rad ≈ 11°, the model probably hasn't
    # learned the task well enough to deploy
    if overall > 0.20:
        report.failed("behavioral fidelity",
                      f"mean MAE={overall:.3f} rad - undertrained / wrong stats")
    elif overall > 0.10:
        report.warned("behavioral fidelity marginal",
                      f"mean MAE={overall:.3f} rad")
    else:
        report.passed("behavioral fidelity",
                      f"mean MAE={overall:.3f} rad")


def strip_jepa_weights(in_path: str, out_path: str, report: Report) -> None:
    """Save a deploy-only checkpoint with JEPA training weights removed."""
    report.section("9. Strip JEPA-only weights")
    ckpt = torch.load(in_path, map_location="cpu", weights_only=False)
    sd = ckpt["model_state_dict"]
    keep = {k: v for k, v in sd.items()
            if not k.startswith(("jepa_predictor.", "sigreg."))}
    dropped = len(sd) - len(keep)
    ckpt["model_state_dict"] = keep
    ckpt.setdefault("config", {})["model"] = "act"  # mark as deploy-only
    torch.save(ckpt, out_path)
    report.info("dropped tensors", str(dropped))
    report.passed("stripped checkpoint saved", out_path)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Pre-deployment ACT / ACT-JEPA validator")
    p.add_argument("--checkpoint", required=True, help="Path to .pt checkpoint")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--data-dir", default=None,
                   help="Optional episodes dir for behavioral-fidelity check")
    p.add_argument("--frames-per-episode", type=int, default=20)
    p.add_argument("--latency-iters", type=int, default=50)
    p.add_argument("--latency-warn-ms", type=float, default=80.0,
                   help="Warn if average forward pass exceeds this (matches "
                        "act_policy_node WARN_LATENCY).")
    p.add_argument("--strip", default=None, metavar="OUT",
                   help="If checkpoint is ACT-JEPA, also save a JEPA-stripped "
                        "copy to this path for deployment.")
    p.add_argument("--strict", action="store_true",
                   help="Treat WARNings as deploy-blocking failures.")
    return p.parse_args()


def main():
    args = parse_args()
    device = torch.device(args.device)
    report = Report()

    print(f"validate_deploy  checkpoint={args.checkpoint}  device={device}")

    policy, stats, config, kind = load_for_validation(args.checkpoint, device, report)
    inputs = _make_synthetic_input(stats, config, device)

    check_inference_isolation(policy, inputs, kind, report)
    check_output_sanity(policy, inputs, stats, config, report)
    check_determinism(policy, inputs, report)
    check_latency(policy, inputs, device,
                  args.latency_iters, args.latency_warn_ms, report)

    # Pull the raw state dict back from disk for the deploy-loader compat check
    raw_sd = torch.load(args.checkpoint, map_location="cpu",
                        weights_only=False)["model_state_dict"]
    check_deploy_loader_compat(config, raw_sd, kind, report)

    if args.data_dir:
        check_episode_fidelity(policy, stats, config, device,
                               args.data_dir, args.frames_per_episode, report)
    else:
        report.section("8. Behavioral fidelity (real episodes)")
        report.info("skipped", "pass --data-dir episodes/ to enable")

    if args.strip:
        if kind != "act_jepa":
            report.section("9. Strip JEPA-only weights")
            report.info("skipped", "checkpoint is already ACT-only")
        else:
            strip_jepa_weights(args.checkpoint, args.strip, report)

    sys.exit(report.summary(strict=args.strict))


if __name__ == "__main__":
    main()
