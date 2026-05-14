"""
visualize_embeddings.py - Side-by-side 2D projection of context-token
embeddings from ACT vs ACT-JEPA checkpoints.

For every held-out validation frame, we:
  1. Encode the stereo pair through the policy's image encoder + img_proj.
  2. Mean-pool the resulting [N=160, 256] token bag into a single 256-d
     per-frame embedding (one point in the scatter).
  3. Project all per-frame embeddings to 2D with t-SNE.
  4. Plot each checkpoint's embeddings side by side, colored two ways:
       row 1: 'task phase'   - timestep / episode_length, in [0, 1]
       row 2: 'gripper'       - thresholded last joint (rough open/closed)

If the JEPA objective captured useful semantics, you should see tighter,
more separated clusters in the JEPA panel relative to ACT - even though
the backbones started from the same ImageNet weights.

Usage:
    python -m python.training.visualize_embeddings \\
        --data-dir episodes/ \\
        --checkpoint act:checkpoints/act_v2/act_best.pt \\
        --checkpoint jepa:checkpoints/jepa/act_jepa_best.pt \\
        --output viz/embeddings.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple

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
    state_dim = config.get(
        "state_dim", num_joints * _STATE_MODE_K.get(state_mode, 1)
    )
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


@torch.no_grad()
def encode_frames(
    policy: torch.nn.Module,
    device: torch.device,
    episode_paths: List[Path],
    frame_stride: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (embeddings [F, D], episode_id [F], phase [F], gripper [F])."""
    embs: List[np.ndarray] = []
    ep_ids: List[int] = []
    phases: List[float] = []
    grippers: List[float] = []

    for ei, path in enumerate(episode_paths):
        with h5py.File(path, "r") as f:
            T = f["observations/qpos"].shape[0]
            for t in range(0, T, frame_stride):
                gripper = f["observations/images/gripper"][t]
                qpos = f["observations/qpos"][t]
                gt = torch.from_numpy(_norm_image(gripper)).unsqueeze(0).to(device)
                tokens = policy._encode_images(gt)            # [1, N, D]
                pooled = tokens.mean(dim=1).squeeze(0)        # [D]
                embs.append(pooled.cpu().numpy().astype(np.float32))
                ep_ids.append(ei)
                phases.append(t / max(T - 1, 1))
                grippers.append(float(qpos[-1]))
    return (
        np.stack(embs),
        np.array(ep_ids, dtype=np.int32),
        np.array(phases, dtype=np.float32),
        np.array(grippers, dtype=np.float32),
    )


def project_2d(embs: np.ndarray, *, seed: int) -> np.ndarray:
    """t-SNE -> [F, 2]. Falls back to PCA if too few samples."""
    from sklearn.manifold import TSNE
    n = embs.shape[0]
    perplexity = float(max(5, min(30, n // 4)))
    return TSNE(
        n_components=2, perplexity=perplexity,
        init="pca", random_state=seed, max_iter=1000,
    ).fit_transform(embs)


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
    p = argparse.ArgumentParser(
        description="Visualize ACT vs ACT-JEPA embeddings with t-SNE"
    )
    p.add_argument("--data-dir", required=True)
    p.add_argument(
        "--checkpoint", "-c", action="append", required=True,
        help="name:path pair, may be passed multiple times",
    )
    p.add_argument(
        "--val-fraction", type=float, default=0.15,
        help="Same split as training (default 0.15).",
    )
    p.add_argument("--val-seed", type=int, default=0)
    p.add_argument(
        "--frame-stride", type=int, default=10,
        help="Encode every Nth frame per episode (default 10 = 2 Hz @ 20 Hz).",
    )
    p.add_argument(
        "--gripper-threshold", type=float, default=None,
        help="Median by default; pass a value to override.",
    )
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--output", default="viz/embeddings.png")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    device = torch.device(args.device)
    pairs = parse_checkpoints(args.checkpoint)

    _, val_paths = split_episodes(
        args.data_dir, val_fraction=args.val_fraction, seed=args.val_seed
    )
    if not val_paths:
        print("No val episodes found; check --val-fraction.", file=sys.stderr)
        sys.exit(1)
    print(f"Encoding {len(val_paths)} held-out episode(s); stride={args.frame_stride}")
    for v in val_paths:
        print(f"  {v.name}")

    by_name: Dict[str, Dict] = {}
    for name, path in pairs:
        if not path.exists():
            print(f"[skip] {name}: {path} not found")
            continue
        policy, _, _, kind = load_policy(str(path), device)
        e, eid, ph, gr = encode_frames(policy, device, val_paths, args.frame_stride)
        proj = project_2d(e, seed=args.seed)
        by_name[name] = dict(emb=e, proj=proj, ep=eid, phase=ph, gripper=gr, kind=kind)
        print(f"[done] {name} (kind={kind}): {e.shape[0]} frames, "
              f"emb_dim={e.shape[1]}")
        del policy
        if device.type == "cuda":
            torch.cuda.empty_cache()

    if not by_name:
        sys.exit(1)

    # Compute a shared gripper threshold so the same dichotomy is applied
    # to every checkpoint
    all_grip = np.concatenate([r["gripper"] for r in by_name.values()])
    threshold = (args.gripper_threshold if args.gripper_threshold is not None
                 else float(np.median(all_grip)))

    n_ckpt = len(by_name)
    fig, axes = plt.subplots(
        2, n_ckpt,
        figsize=(5.0 * n_ckpt, 9.0),
        squeeze=False,
    )

    for col, (name, r) in enumerate(by_name.items()):
        proj = r["proj"]
        # Row 0: colored by task phase
        ax = axes[0, col]
        sc = ax.scatter(proj[:, 0], proj[:, 1], c=r["phase"],
                        cmap="viridis", s=8, alpha=0.85)
        ax.set_title(f"{name} ({r['kind']})  -  task phase")
        ax.set_xticks([]); ax.set_yticks([])
        plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)

        # Row 1: colored by gripper open/closed
        ax = axes[1, col]
        closed = r["gripper"] > threshold
        ax.scatter(proj[closed, 0], proj[closed, 1], c="tab:red",
                   label="closed", s=8, alpha=0.85)
        ax.scatter(proj[~closed, 0], proj[~closed, 1], c="tab:blue",
                   label="open", s=8, alpha=0.85)
        ax.set_title(f"{name} ({r['kind']})  -  gripper (thr={threshold:.2f})")
        ax.set_xticks([]); ax.set_yticks([])
        ax.legend(loc="best", fontsize=8)

    fig.suptitle("Per-frame context-token embeddings (t-SNE)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    print(f"\nSaved {args.output}")

    # Lightweight quantitative diagnostic: silhouette of episode clusters
    try:
        from sklearn.metrics import silhouette_score
        for name, r in by_name.items():
            if len(set(r["ep"].tolist())) >= 2:
                s = silhouette_score(r["proj"], r["ep"], metric="euclidean")
                print(f"silhouette by episode  {name}: {s:+.3f}  "
                      f"(higher = episodes more separated in 2D)")
    except Exception as e:
        print(f"silhouette skipped: {e}")


if __name__ == "__main__":
    main()
