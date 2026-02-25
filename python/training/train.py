"""
train.py — ACT policy training loop.

Usage:
    python train.py --data-dir episodes/ --output-dir checkpoints/
    python train.py --data-dir episodes/ --output-dir checkpoints/ \\
        --epochs 100 --batch-size 32 --chunk-size 100 --lr 1e-4 --device cuda
    python train.py --data-dir episodes/ --output-dir checkpoints/ --resume

Checkpoints are saved every 5 epochs to:
    checkpoints/act_epoch_XXXX.pt

Each checkpoint includes:
    - model_state_dict
    - optimizer_state_dict
    - epoch
    - dataset_stats  (for denormalization at inference)
    - config         (hyperparameters)
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

# Allow running from repo root or python/training/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.dataset import EpisodeDataset
from python.training.act_model import ACTPolicy


def parse_args():
    parser = argparse.ArgumentParser(description="Train ACT policy")
    parser.add_argument("--data-dir", required=True, help="Directory with episode_*.hdf5")
    parser.add_argument("--output-dir", default="checkpoints", help="Checkpoint output directory")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--chunk-size", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--kl-weight", type=float, default=10.0)
    parser.add_argument("--d-model", type=int, default=512)
    parser.add_argument("--z-dim", type=int, default=32)
    parser.add_argument("--nhead", type=int, default=8)
    parser.add_argument("--num-encoder-layers", type=int, default=4)
    parser.add_argument("--num-decoder-layers", type=int, default=7)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--save-every", type=int, default=5, help="Save checkpoint every N epochs")
    parser.add_argument("--resume", action="store_true", help="Resume from latest checkpoint")
    parser.add_argument("--cache", action="store_true", help="Cache all HDF5 data in RAM")
    return parser.parse_args()


def find_latest_checkpoint(output_dir: Path) -> Path | None:
    """Return the latest checkpoint path, or None."""
    ckpts = sorted(output_dir.glob("act_epoch_*.pt"))
    return ckpts[-1] if ckpts else None


def collate_fn(batch):
    """Custom collate to handle nested obs dicts."""
    obs_list, action_list = zip(*batch)
    actions = torch.stack(action_list, dim=0)

    qpos = torch.stack([o["qpos"] for o in obs_list], dim=0)
    imgs_left = torch.stack([o["images"]["left"] for o in obs_list], dim=0)
    imgs_right = torch.stack([o["images"]["right"] for o in obs_list], dim=0)

    obs = {
        "qpos": qpos,
        "images": {"left": imgs_left, "right": imgs_right},
    }
    return obs, actions


def main():
    args = parse_args()
    device = torch.device(args.device)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Device: {device}")
    print(f"Data dir: {args.data_dir}")
    print(f"Output dir: {output_dir}")

    # Dataset
    print("Loading dataset...")
    dataset = EpisodeDataset(
        args.data_dir,
        chunk_size=args.chunk_size,
        cache=args.cache,
    )
    print(
        f"Dataset: {len(dataset)} samples across {dataset.num_episodes} episodes "
        f"(chunk_size={args.chunk_size})"
    )

    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        drop_last=True,
        collate_fn=collate_fn,
    )

    # Model
    policy = ACTPolicy(
        chunk_size=args.chunk_size,
        d_model=args.d_model,
        z_dim=args.z_dim,
        nhead=args.nhead,
        num_encoder_layers=args.num_encoder_layers,
        num_decoder_layers=args.num_decoder_layers,
        kl_weight=args.kl_weight,
        pretrained_encoder=True,
    ).to(device)

    n_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"Model parameters: {n_params:,}")

    # Optimizer
    optimizer = torch.optim.AdamW(
        policy.parameters(),
        lr=args.lr,
        weight_decay=args.weight_decay,
    )

    start_epoch = 0

    # Resume
    if args.resume:
        ckpt_path = find_latest_checkpoint(output_dir)
        if ckpt_path is None:
            print("No checkpoint found to resume from. Starting fresh.")
        else:
            print(f"Resuming from {ckpt_path}")
            ckpt = torch.load(ckpt_path, map_location=device)
            policy.load_state_dict(ckpt["model_state_dict"])
            optimizer.load_state_dict(ckpt["optimizer_state_dict"])
            start_epoch = ckpt["epoch"] + 1
            print(f"Resuming from epoch {start_epoch}")

    # Training loop
    for epoch in range(start_epoch, args.epochs):
        policy.train()
        epoch_l1 = 0.0
        epoch_kl = 0.0
        epoch_total = 0.0
        n_batches = 0

        for batch_idx, (obs, actions) in enumerate(loader):
            qpos = obs["qpos"].to(device)
            imgs_left = obs["images"]["left"].to(device)
            imgs_right = obs["images"]["right"].to(device)
            actions = actions.to(device)

            optimizer.zero_grad()
            loss_dict = policy(qpos, imgs_left, imgs_right, actions)
            loss = loss_dict["total"]
            loss.backward()
            # Gradient clipping for stability
            nn.utils.clip_grad_norm_(policy.parameters(), max_norm=10.0)
            optimizer.step()

            epoch_l1 += loss_dict["l1"].item()
            epoch_kl += loss_dict["kl"].item()
            epoch_total += loss_dict["total"].item()
            n_batches += 1

            if (batch_idx + 1) % 20 == 0:
                print(
                    f"  Epoch {epoch+1}/{args.epochs} "
                    f"batch {batch_idx+1}/{len(loader)} "
                    f"l1={loss_dict['l1'].item():.4f} "
                    f"kl={loss_dict['kl'].item():.4f} "
                    f"total={loss_dict['total'].item():.4f}"
                )

        avg_l1 = epoch_l1 / max(n_batches, 1)
        avg_kl = epoch_kl / max(n_batches, 1)
        avg_total = epoch_total / max(n_batches, 1)

        print(
            f"Epoch {epoch+1}/{args.epochs} — "
            f"l1={avg_l1:.4f}  kl={avg_kl:.4f}  total={avg_total:.4f}"
        )

        # Save checkpoint
        if (epoch + 1) % args.save_every == 0 or (epoch + 1) == args.epochs:
            ckpt_path = output_dir / f"act_epoch_{epoch+1:04d}.pt"
            torch.save(
                {
                    "epoch": epoch,
                    "model_state_dict": policy.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "dataset_stats": {
                        k: v.numpy() if hasattr(v, "numpy") else v
                        for k, v in dataset.dataset_stats.items()
                    },
                    "config": {
                        "chunk_size": args.chunk_size,
                        "d_model": args.d_model,
                        "z_dim": args.z_dim,
                        "nhead": args.nhead,
                        "num_encoder_layers": args.num_encoder_layers,
                        "num_decoder_layers": args.num_decoder_layers,
                        "kl_weight": args.kl_weight,
                    },
                    "train_loss": {
                        "l1": avg_l1,
                        "kl": avg_kl,
                        "total": avg_total,
                    },
                },
                ckpt_path,
            )
            print(f"  Saved checkpoint: {ckpt_path}")

    print("Training complete.")


if __name__ == "__main__":
    main()
