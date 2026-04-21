"""
train.py — ACT policy training loop.

Usage:
    python train.py --data-dir episodes/ --output-dir checkpoints/
    python train.py --data-dir episodes/ --output-dir checkpoints/ \\
        --epochs 200 --batch-size 32 --chunk-size 32 --lr 1e-4 --device cuda
    python train.py --data-dir episodes/ --output-dir checkpoints/ --resume
    python train.py --data-dir episodes/ --output-dir checkpoints/ \\
        --state-mode qpos_qcmd --action-mode relative --augment --val-fraction 0.15

Checkpoints are saved every --save-every epochs to:
    checkpoints/act_epoch_XXXX.pt

The checkpoint with the lowest validation total-loss is also saved to:
    checkpoints/act_best.pt

Each checkpoint includes:
    - model_state_dict
    - optimizer_state_dict
    - scheduler_state_dict
    - epoch
    - dataset_stats  (for denormalization at inference)
    - config         (hyperparameters, including num_joints, state_mode, action_mode)
    - train_loss, val_loss
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter

# Allow running from repo root or python/training/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.dataset import EpisodeDataset, split_episodes, STATE_MODES, ACTION_MODES
from python.training.act_model import ACTPolicy


def parse_args():
    parser = argparse.ArgumentParser(description="Train ACT policy")
    parser.add_argument("--data-dir", required=True, help="Directory with episode_*.hdf5")
    parser.add_argument("--output-dir", default="checkpoints", help="Checkpoint output directory")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--chunk-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--lr-backbone", type=float, default=1e-5,
                        help="Learning rate for image backbone (10x lower than main)")
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--kl-weight", type=float, default=10.0)
    parser.add_argument("--d-model", type=int, default=256)
    parser.add_argument("--dim-feedforward", type=int, default=2048)
    parser.add_argument("--z-dim", type=int, default=32)
    parser.add_argument("--nhead", type=int, default=8)
    parser.add_argument("--num-encoder-layers", type=int, default=4)
    parser.add_argument("--num-decoder-layers", type=int, default=7)
    parser.add_argument("--state-mode", default="qpos_qcmd", choices=list(STATE_MODES),
                        help="State vector layout (qpos, qpos_qcmd, qpos_qcmd_tq)")
    parser.add_argument("--action-mode", default="relative", choices=list(ACTION_MODES),
                        help="absolute = predict joint targets; relative = predict (target - qpos)")
    parser.add_argument("--augment", action="store_true",
                        help="Enable train-time image augmentation")
    parser.add_argument("--val-fraction", type=float, default=0.15,
                        help="Fraction of episodes held out for validation (0 disables)")
    parser.add_argument("--val-seed", type=int, default=0,
                        help="Seed for train/val episode split")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--save-every", type=int, default=10,
                        help="Save periodic checkpoint every N epochs")
    parser.add_argument("--resume", action="store_true", help="Resume from latest checkpoint")
    parser.add_argument("--cache", action="store_true", help="Cache all HDF5 data in RAM")
    parser.add_argument("--no-tensorboard", action="store_true", dest="no_tensorboard",
                        help="Disable TensorBoard logging")
    return parser.parse_args()


def find_latest_checkpoint(output_dir: Path) -> Path | None:
    """Return the latest periodic checkpoint path, or None."""
    ckpts = sorted(output_dir.glob("act_epoch_*.pt"))
    return ckpts[-1] if ckpts else None


def collate_fn(batch):
    """Custom collate to handle nested obs dicts."""
    obs_list, action_list = zip(*batch)
    actions = torch.stack(action_list, dim=0)

    qpos = torch.stack([o["qpos"] for o in obs_list], dim=0)
    state = torch.stack([o["state"] for o in obs_list], dim=0)
    imgs_left = torch.stack([o["images"]["left"] for o in obs_list], dim=0)
    imgs_right = torch.stack([o["images"]["right"] for o in obs_list], dim=0)

    obs = {
        "qpos": qpos,
        "state": state,
        "images": {"left": imgs_left, "right": imgs_right},
    }
    return obs, actions


def run_epoch(policy, loader, device, *, optimizer=None, clip_grad=0.1):
    """Run one pass over `loader`. If optimizer is None, runs in eval mode."""
    is_train = optimizer is not None
    policy.train(is_train)

    sum_l1 = 0.0
    sum_kl = 0.0
    sum_total = 0.0
    n_batches = 0

    ctx = torch.enable_grad() if is_train else torch.no_grad()
    with ctx:
        for obs, actions in loader:
            qpos = obs["qpos"].to(device, non_blocking=True)
            state = obs["state"].to(device, non_blocking=True)
            imgs_left = obs["images"]["left"].to(device, non_blocking=True)
            imgs_right = obs["images"]["right"].to(device, non_blocking=True)
            actions = actions.to(device, non_blocking=True)

            loss_dict = policy(qpos, state, imgs_left, imgs_right, actions)

            if is_train:
                optimizer.zero_grad()
                loss_dict["total"].backward()
                nn.utils.clip_grad_norm_(policy.parameters(), max_norm=clip_grad)
                optimizer.step()

            sum_l1 += loss_dict["l1"].item()
            sum_kl += loss_dict["kl"].item()
            sum_total += loss_dict["total"].item()
            n_batches += 1

    n = max(n_batches, 1)
    return {"l1": sum_l1 / n, "kl": sum_kl / n, "total": sum_total / n}


def main():
    args = parse_args()
    device = torch.device(args.device)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Device: {device}")
    print(f"Data dir: {args.data_dir}")
    print(f"Output dir: {output_dir}")
    print(f"State mode: {args.state_mode}  Action mode: {args.action_mode}  "
          f"Augment: {args.augment}")

    # ------------------------------------------------------------------
    # Train / val split
    # ------------------------------------------------------------------
    train_paths, val_paths = split_episodes(
        args.data_dir, val_fraction=args.val_fraction, seed=args.val_seed
    )
    print(f"Episodes: {len(train_paths)} train / {len(val_paths)} val "
          f"(val_fraction={args.val_fraction})")

    train_set = EpisodeDataset(
        episode_paths=train_paths,
        chunk_size=args.chunk_size,
        state_mode=args.state_mode,
        action_mode=args.action_mode,
        augment=args.augment,
        cache=args.cache,
    )
    print(
        f"Train: {len(train_set)} samples across {train_set.num_episodes} episodes "
        f"(num_joints={train_set.num_joints}, state_dim={train_set.state_dim})"
    )

    val_set = None
    if val_paths:
        val_set = EpisodeDataset(
            episode_paths=val_paths,
            chunk_size=args.chunk_size,
            state_mode=args.state_mode,
            action_mode=args.action_mode,
            augment=False,
            cache=args.cache,
            dataset_stats=train_set.dataset_stats,
        )
        print(f"Val:   {len(val_set)} samples across {val_set.num_episodes} episodes")

    pin = (device.type == "cuda")
    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=pin,
        drop_last=True,
        collate_fn=collate_fn,
    )
    val_loader = None
    if val_set is not None:
        val_loader = DataLoader(
            val_set,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=pin,
            drop_last=False,
            collate_fn=collate_fn,
        )

    # ------------------------------------------------------------------
    # Model
    # ------------------------------------------------------------------
    policy = ACTPolicy(
        chunk_size=args.chunk_size,
        d_model=args.d_model,
        dim_feedforward=args.dim_feedforward,
        z_dim=args.z_dim,
        nhead=args.nhead,
        num_encoder_layers=args.num_encoder_layers,
        num_decoder_layers=args.num_decoder_layers,
        kl_weight=args.kl_weight,
        pretrained_encoder=True,
        num_joints=train_set.num_joints,
        state_dim=train_set.state_dim,
    ).to(device)

    n_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"Model parameters: {n_params:,}")

    # Optimizer with separate backbone LR (10x lower)
    param_groups = [
        {"params": policy.backbone_parameters(), "lr": args.lr_backbone},
        {"params": policy.non_backbone_parameters(), "lr": args.lr},
    ]
    optimizer = torch.optim.AdamW(param_groups, weight_decay=args.weight_decay)
    print(f"LR: {args.lr:.1e} (backbone: {args.lr_backbone:.1e})")

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=args.lr * 0.01
    )

    # TensorBoard
    writer = None
    if not args.no_tensorboard:
        tb_dir = output_dir / "tb_logs"
        writer = SummaryWriter(log_dir=str(tb_dir))
        print(f"TensorBoard: {tb_dir}")

    # ------------------------------------------------------------------
    # Resume
    # ------------------------------------------------------------------
    start_epoch = 0
    best_val = float("inf")

    if args.resume:
        ckpt_path = find_latest_checkpoint(output_dir)
        if ckpt_path is None:
            print("No checkpoint found to resume from. Starting fresh.")
        else:
            print(f"Resuming from {ckpt_path}")
            ckpt = torch.load(ckpt_path, map_location=device)
            policy.load_state_dict(ckpt["model_state_dict"])
            optimizer.load_state_dict(ckpt["optimizer_state_dict"])
            if "scheduler_state_dict" in ckpt:
                scheduler.load_state_dict(ckpt["scheduler_state_dict"])
            start_epoch = ckpt["epoch"] + 1
            best_val = ckpt.get("best_val", best_val)
            print(f"Resuming from epoch {start_epoch}  (best_val={best_val:.4f})")

    # ------------------------------------------------------------------
    # Training loop
    # ------------------------------------------------------------------
    def save_checkpoint(path: Path, epoch: int, train_loss, val_loss):
        torch.save(
            {
                "epoch": epoch,
                "best_val": best_val,
                "model_state_dict": policy.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "scheduler_state_dict": scheduler.state_dict(),
                "dataset_stats": {
                    k: (v.numpy() if hasattr(v, "numpy") else v)
                    for k, v in train_set.dataset_stats.items()
                },
                "config": {
                    "chunk_size": args.chunk_size,
                    "d_model": args.d_model,
                    "dim_feedforward": args.dim_feedforward,
                    "z_dim": args.z_dim,
                    "nhead": args.nhead,
                    "num_encoder_layers": args.num_encoder_layers,
                    "num_decoder_layers": args.num_decoder_layers,
                    "kl_weight": args.kl_weight,
                    "num_joints": train_set.num_joints,
                    "state_mode": args.state_mode,
                    "state_dim": train_set.state_dim,
                    "action_mode": args.action_mode,
                },
                "train_loss": train_loss,
                "val_loss": val_loss,
            },
            path,
        )

    for epoch in range(start_epoch, args.epochs):
        train_loss = run_epoch(policy, train_loader, device, optimizer=optimizer)
        val_loss = None
        if val_loader is not None:
            val_loss = run_epoch(policy, val_loader, device, optimizer=None)

        scheduler.step()
        lrs = scheduler.get_last_lr()
        lr_backbone = lrs[0]
        lr_main = lrs[1] if len(lrs) > 1 else lrs[0]

        msg = (
            f"Epoch {epoch+1}/{args.epochs} — "
            f"train l1={train_loss['l1']:.4f} kl={train_loss['kl']:.4f} "
            f"total={train_loss['total']:.4f}"
        )
        if val_loss is not None:
            msg += (f"  |  val l1={val_loss['l1']:.4f} kl={val_loss['kl']:.4f} "
                    f"total={val_loss['total']:.4f}")
        msg += f"  lr={lr_main:.2e} (backbone={lr_backbone:.2e})"
        print(msg)

        if writer is not None:
            writer.add_scalar("train/l1", train_loss["l1"], epoch + 1)
            writer.add_scalar("train/kl", train_loss["kl"], epoch + 1)
            writer.add_scalar("train/total", train_loss["total"], epoch + 1)
            if val_loss is not None:
                writer.add_scalar("val/l1", val_loss["l1"], epoch + 1)
                writer.add_scalar("val/kl", val_loss["kl"], epoch + 1)
                writer.add_scalar("val/total", val_loss["total"], epoch + 1)
            writer.add_scalar("lr/main", lr_main, epoch + 1)
            writer.add_scalar("lr/backbone", lr_backbone, epoch + 1)

        # Best-val checkpoint
        if val_loss is not None and val_loss["total"] < best_val:
            best_val = val_loss["total"]
            best_path = output_dir / "act_best.pt"
            save_checkpoint(best_path, epoch, train_loss, val_loss)
            print(f"  New best val total={best_val:.4f} → {best_path}")

        # Periodic checkpoint
        if (epoch + 1) % args.save_every == 0 or (epoch + 1) == args.epochs:
            ckpt_path = output_dir / f"act_epoch_{epoch+1:04d}.pt"
            save_checkpoint(ckpt_path, epoch, train_loss, val_loss)
            print(f"  Saved checkpoint: {ckpt_path}")

    if writer is not None:
        writer.close()
    print("Training complete.")


if __name__ == "__main__":
    main()
