"""
probe_jepa.py - Direct test of whether the JEPA predictor is real.

For each held-out frame at time t we compute three latent-space distances
(Frobenius norm over the [N=160, 256] context-token bag), all in the
SAME 256-d space the predictor was trained in:

    d_pred = || predictor(ctx_t)         - target_encoder(frame_{t+offset}) ||
    d_id   = || ctx_t                    - target_encoder(frame_{t+offset}) ||
    d_rand = || predictor(ctx_t)         - target_encoder(random_other_frame) ||

Two ratios summarize the predictor's behavior:

    r_id   = d_pred / d_id            < 1: predictor moved forward in time
                                       = 1: predictor collapsed to identity
                                       > 1: predictor is worse than nothing

    r_rand = d_pred / d_rand          < 1: prediction is calibrated to THIS
                                            frame's future, not just any
                                            plausible future state

The script saves a 2-panel histogram (r_id and r_rand) and prints
summary stats. r_id mean << 1 and r_rand mean << 1 are the green lights.

Usage:
    python -m python.training.probe_jepa \\
        --checkpoint checkpoints/jepa/act_jepa_best.pt \\
        --data-dir episodes/ --output viz/probe.png
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import List, Tuple

import h5py
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.dataset import split_episodes
from python.training.validate_deploy import (
    _STATE_MODE_K,
    _detect_model_kind,
    _norm_image,
    _stats_to_numpy,
)
from python.training.jepa_model import ACTJEPAPolicy


def load_jepa(path: str, device: torch.device) -> Tuple[ACTJEPAPolicy, dict]:
    ckpt = torch.load(path, map_location=device, weights_only=False)
    config = dict(ckpt["config"])
    state_dict = ckpt["model_state_dict"]
    kind = _detect_model_kind(config, state_dict)
    if kind != "act_jepa":
        raise SystemExit(
            f"{path} is not an ACT-JEPA checkpoint (kind={kind}); "
            "this probe requires the JEPA predictor."
        )
    num_joints = config.get("num_joints", 7)
    state_mode = config.get("state_mode", "qpos_qcmd")
    state_dim = config.get(
        "state_dim", num_joints * _STATE_MODE_K.get(state_mode, 1)
    )
    policy = ACTJEPAPolicy(
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
        predictor_layers=config.get("predictor_layers", 4),
        predictor_heads=config.get("predictor_heads", 8),
        predictor_ff=config.get("predictor_ff", 1024),
        lambda_obs=config.get("lambda_obs", 0.5),
        lambda_reg=config.get("lambda_reg", 0.05),
        sigreg_slices=config.get("sigreg_slices", 1024),
        sigreg_points=config.get("sigreg_points", 17),
    )
    policy.load_state_dict(state_dict, strict=True)
    policy.to(device).eval()
    return policy, config


@torch.no_grad()
def encode(policy: ACTJEPAPolicy, gripper_u8: np.ndarray,
           device: torch.device) -> torch.Tensor:
    gt = torch.from_numpy(_norm_image(gripper_u8)).unsqueeze(0).to(device)
    return policy._encode_images(gt)  # [1, N, D]


def main():
    p = argparse.ArgumentParser(
        description="Probe JEPA predictor distance ratios"
    )
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--data-dir", required=True)
    p.add_argument("--val-fraction", type=float, default=0.15)
    p.add_argument("--val-seed", type=int, default=0)
    p.add_argument(
        "--future-offset", type=int, default=None,
        help="Frames ahead to predict (default: config.future_offset, "
             "falling back to chunk_size).",
    )
    p.add_argument("--frame-stride", type=int, default=10)
    p.add_argument(
        "--n-random-anchors", type=int, default=50,
        help="Size of the pool of random other frames used as the d_rand "
             "baseline.",
    )
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--output", default="viz/probe.png")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rng = random.Random(args.seed)
    device = torch.device(args.device)
    policy, config = load_jepa(args.checkpoint, device)

    offset = (args.future_offset
              or config.get("future_offset")
              or config["chunk_size"])

    _, val_paths = split_episodes(
        args.data_dir, val_fraction=args.val_fraction, seed=args.val_seed
    )
    if not val_paths:
        raise SystemExit("No val episodes; check --val-fraction.")
    print(f"Probing predictor at future_offset={offset} on "
          f"{len(val_paths)} held-out episode(s); stride={args.frame_stride}")

    # ------------------------------------------------------------------
    # 1. Build a pool of random "other-frame" targets for d_rand baseline
    # ------------------------------------------------------------------
    other_targets: List[torch.Tensor] = []
    other_id: List[Tuple[int, int]] = []
    for ei, path in enumerate(val_paths):
        with h5py.File(path, "r") as f:
            T = f["observations/qpos"].shape[0]
            for _ in range(max(1, args.n_random_anchors // len(val_paths))):
                t = rng.randrange(T)
                gripper = f["observations/images/gripper"][t]
                other_targets.append(encode(policy, gripper, device))
                other_id.append((ei, t))

    # ------------------------------------------------------------------
    # 2. Sweep frames, compute the three distances
    # ------------------------------------------------------------------
    r_id_list: List[float] = []
    r_rand_list: List[float] = []
    d_pred_list: List[float] = []

    for ei, path in enumerate(val_paths):
        with h5py.File(path, "r") as f:
            T = f["observations/qpos"].shape[0]
            for t in range(0, T - offset, args.frame_stride):
                ctx = encode(policy,
                             f["observations/images/gripper"][t], device)
                future = encode(
                    policy,
                    f["observations/images/gripper"][t + offset],
                    device,
                )
                pred = policy.jepa_predictor(ctx)

                d_pred = (pred - future).norm().item()
                d_id = (ctx - future).norm().item()
                # Pick a random other-frame target that is NOT (ei, t+offset)
                while True:
                    j = rng.randrange(len(other_targets))
                    if other_id[j] != (ei, t + offset):
                        break
                d_rand = (pred - other_targets[j]).norm().item()

                r_id_list.append(d_pred / max(d_id, 1e-9))
                r_rand_list.append(d_pred / max(d_rand, 1e-9))
                d_pred_list.append(d_pred)

    r_id = np.array(r_id_list)
    r_rand = np.array(r_rand_list)
    d_pred_arr = np.array(d_pred_list)

    # ------------------------------------------------------------------
    # 3. Stats + plot
    # ------------------------------------------------------------------
    def _stats(name: str, x: np.ndarray, note: str):
        print(f"  {name:7s}  n={x.size:>5d}  mean={x.mean():.3f}  "
              f"median={np.median(x):.3f}  "
              f"frac<1={(x < 1).mean()*100:5.1f}%   {note}")

    print("\nDistance ratios (latent space, Frobenius over [N, D] tokens):")
    _stats("r_id",   r_id,   "lower = predictor moves forward vs identity")
    _stats("r_rand", r_rand, "lower = prediction tied to THIS frame's future")
    print(f"  d_pred raw: mean={d_pred_arr.mean():.2f}")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    axes[0].hist(r_id, bins=40, color="tab:blue", edgecolor="white")
    axes[0].axvline(1.0, color="k", ls="--", lw=1.2, label="identity (r=1)")
    axes[0].axvline(r_id.mean(), color="tab:red", ls="-", lw=1.2,
                    label=f"mean={r_id.mean():.2f}")
    axes[0].set_xlabel("r_id = d_pred / d_identity")
    axes[0].set_ylabel("frame count")
    axes[0].set_title("Predictor vs identity")
    axes[0].legend()

    axes[1].hist(r_rand, bins=40, color="tab:green", edgecolor="white")
    axes[1].axvline(1.0, color="k", ls="--", lw=1.2, label="no calibration (r=1)")
    axes[1].axvline(r_rand.mean(), color="tab:red", ls="-", lw=1.2,
                    label=f"mean={r_rand.mean():.2f}")
    axes[1].set_xlabel("r_rand = d_pred / d_random_other")
    axes[1].set_ylabel("frame count")
    axes[1].set_title("Predictor vs random-other future")
    axes[1].legend()

    fig.suptitle(f"JEPA predictor probe  (offset={offset} frames)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    print(f"\nSaved {args.output}")

    if r_id.mean() >= 0.95:
        print("[WARN] r_id mean is close to 1 - predictor may be collapsing to identity.")
    if r_rand.mean() >= 0.95:
        print("[WARN] r_rand mean is close to 1 - prediction is not specific to this frame.")
    if r_id.mean() < 0.9 and r_rand.mean() < 0.9:
        print("[OK] World model is doing real work: predictions are both")
        print("     forward-in-time AND calibrated to the specific input.")


if __name__ == "__main__":
    main()
