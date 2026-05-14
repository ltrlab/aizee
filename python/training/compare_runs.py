"""
compare_runs.py — Side-by-side comparison of ACT / ACT-JEPA checkpoints.

Replays the same recorded episodes through each checkpoint open-loop and
prints a per-joint MAE table so you can see whether the JEPA objective
actually helps on your data.

Usage:
    python -m python.training.compare_runs \\
        --data-dir episodes/ \\
        --checkpoint act:checkpoints/act_v2/act_best.pt \\
        --checkpoint jepa:checkpoints/jepa/act_jepa_best.pt

    # Add a third reference (e.g. the older 40-epoch baseline):
    python -m python.training.compare_runs \\
        --data-dir episodes/ \\
        --checkpoint act_v1:checkpoints/act_best.pt \\
        --checkpoint act_v2:checkpoints/act_v2/act_best.pt \\
        --checkpoint jepa:checkpoints/jepa/act_jepa_best.pt
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import h5py
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.validate_deploy import (
    _STATE_MODE_K,
    _build_state,
    _denormalize,
    _detect_model_kind,
    _norm_image,
    _stats_to_numpy,
)
from python.training.act_model import ACTPolicy
from python.training.jepa_model import ACTJEPAPolicy


def load_policy(path: str, device: torch.device):
    ckpt = torch.load(path, map_location=device, weights_only=False)
    config = dict(ckpt["config"])
    stats = _stats_to_numpy(ckpt["dataset_stats"])
    state_dict = ckpt["model_state_dict"]
    kind = _detect_model_kind(config, state_dict)

    num_joints = config.get("num_joints", 7)
    state_mode = config.get("state_mode", "qpos_qcmd")
    state_dim = config.get("state_dim", num_joints * _STATE_MODE_K.get(state_mode, 1))
    config["num_joints"] = num_joints
    config["state_mode"] = state_mode
    config["state_dim"] = state_dim
    config["action_mode"] = config.get("action_mode", "absolute")

    kwargs = dict(
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
    if kind == "act_jepa":
        policy = ACTJEPAPolicy(
            **kwargs,
            predictor_layers=config.get("predictor_layers", 4),
            predictor_heads=config.get("predictor_heads", 8),
            predictor_ff=config.get("predictor_ff", 1024),
            lambda_obs=config.get("lambda_obs", 0.5),
            lambda_reg=config.get("lambda_reg", 0.05),
            sigreg_slices=config.get("sigreg_slices", 1024),
            sigreg_points=config.get("sigreg_points", 17),
        )
    else:
        policy = ACTPolicy(**kwargs)
    policy.load_state_dict(state_dict, strict=False)
    policy.to(device).eval()
    return policy, stats, config, kind


def replay_episode_mae(
    policy: torch.nn.Module, stats: Dict, config: Dict, device: torch.device,
    episode_path: Path, frames: int,
) -> np.ndarray:
    """Return per-joint MAE [J] over `frames` evenly-spaced frames."""
    state_mode = config["state_mode"]
    action_mode = config["action_mode"]
    chunk_size = config["chunk_size"]
    errs: List[np.ndarray] = []
    with h5py.File(episode_path, "r") as f:
        T = f["observations/qpos"].shape[0]
        n = min(frames, max(1, T - chunk_size))
        idxs = np.linspace(0, T - chunk_size - 1, num=n, dtype=np.int64)
        for t in idxs:
            qpos = f["observations/qpos"][t]
            qcmd = f["observations/qcmd"][t] if "observations/qcmd" in f else None
            torques = f["observations/torques"][t] if "observations/torques" in f else None
            gripper = f["observations/images/gripper"][t]
            gt_chunk = f["actions"][t:t + chunk_size]

            qn = (qpos - stats["qpos_mean"]) / stats["qpos_std"]
            state_v = _build_state(qpos, qcmd, torques, stats, state_mode)
            qt = torch.from_numpy(qn).unsqueeze(0).to(device)
            st = torch.from_numpy(state_v).unsqueeze(0).to(device)
            gt = torch.from_numpy(_norm_image(gripper)).unsqueeze(0).to(device)
            with torch.no_grad():
                pred = policy.select_action(qt, st, gt).squeeze(0).cpu().numpy()
            pred_abs = _denormalize(pred, stats, action_mode, qpos)
            errs.append(np.abs(pred_abs - gt_chunk).mean(axis=0))
    return np.stack(errs).mean(axis=0)


def parse_checkpoints(values: List[str]) -> List[Tuple[str, Path]]:
    """Parse name:path pairs; if no colon, derive name from filename stem."""
    out = []
    for v in values:
        if ":" in v and not v[1:3] == ":\\":  # avoid eating Windows drive letters
            name, p = v.split(":", 1)
        else:
            p = v
            name = Path(v).parent.name or Path(v).stem
        out.append((name, Path(p)))
    return out


def main():
    p = argparse.ArgumentParser(description="Compare ACT vs ACT-JEPA checkpoints")
    p.add_argument("--data-dir", required=True)
    p.add_argument("--checkpoint", "-c", action="append", required=True,
                   help="name:path pair, may be passed multiple times")
    p.add_argument("--episodes-cap", type=int, default=5,
                   help="Number of episodes to evaluate (default 5).")
    p.add_argument("--frames-per-episode", type=int, default=40)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--csv", default=None, help="Optional CSV output path")
    p.add_argument("--json", default=None, help="Optional JSON output path")
    args = p.parse_args()

    device = torch.device(args.device)
    pairs = parse_checkpoints(args.checkpoint)
    paths = sorted(Path(args.data_dir).glob("episode_*.hdf5"))[: args.episodes_cap]
    if not paths:
        print(f"No episodes in {args.data_dir}", file=sys.stderr); sys.exit(1)

    print(f"Comparing {len(pairs)} checkpoint(s) on {len(paths)} episode(s) "
          f"({args.frames_per_episode} frames each)  device={device}\n")

    results: Dict[str, np.ndarray] = {}
    J = None
    for name, path in pairs:
        if not path.exists():
            print(f"[skip] {name}: {path} not found"); continue
        policy, stats, config, kind = load_policy(str(path), device)
        J = config["num_joints"]
        ep_errs = [replay_episode_mae(policy, stats, config, device, ep,
                                      args.frames_per_episode) for ep in paths]
        mae = np.stack(ep_errs).mean(axis=0)
        results[name] = mae
        print(f"[done] {name}  kind={kind}  mean MAE={mae.mean():.4f}")
        del policy
        if device.type == "cuda":
            torch.cuda.empty_cache()

    if not results:
        sys.exit(1)

    # Pretty table
    print("\nPer-joint MAE (rad)")
    print("-" * (12 + 12 * len(results)))
    header = f"{'joint':<10}" + "".join(f"{name:>12}" for name in results)
    print(header)
    print("-" * (12 + 12 * len(results)))
    for j in range(J):
        row = f"j{j:<9}" + "".join(f"{results[name][j]:>12.4f}" for name in results)
        print(row)
    print("-" * (12 + 12 * len(results)))
    row = f"{'mean':<10}" + "".join(f"{results[name].mean():>12.4f}" for name in results)
    print(row)

    if len(results) >= 2:
        names = list(results.keys())
        base = results[names[0]]
        print(f"\nDelta vs '{names[0]}' (negative = better):")
        for nm in names[1:]:
            delta = results[nm] - base
            print(f"  {nm}: mean delta = {delta.mean():+.4f} rad  "
                  f"(per joint: [{', '.join(f'{d:+.3f}' for d in delta)}])")

    if args.csv:
        import csv
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["joint"] + list(results.keys()))
            for j in range(J):
                w.writerow([f"j{j}"] + [f"{results[n][j]:.6f}" for n in results])
            w.writerow(["mean"] + [f"{results[n].mean():.6f}" for n in results])
        print(f"\nWrote CSV: {args.csv}")
    if args.json:
        out = {name: results[name].tolist() for name in results}
        Path(args.json).write_text(json.dumps(out, indent=2))
        print(f"Wrote JSON: {args.json}")


if __name__ == "__main__":
    main()
