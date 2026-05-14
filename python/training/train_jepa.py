"""
train_jepa.py — ACT-JEPA training loop.

Mirrors train.py but instantiates ACTJEPAPolicy, fetches future image
pairs from the dataset, and adds the JEPA observation loss + SIGReg
regularizer to the existing ACT objective. Checkpoints are
drop-in compatible at *inference* time with the plain ACT loader, since
ACTJEPAPolicy is an ACTPolicy subclass and the JEPA modules are
training-only.

Usage:
    python -m python.training.train_jepa \\
        --data-dir episodes/ --output-dir checkpoints/jepa \\
        --epochs 200 --batch-size 32 --chunk-size 32 --augment \\
        --future-offset 32 --lambda-obs 0.5 --lambda-reg 0.05
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.dataset import EpisodeDataset, split_episodes, STATE_MODES, ACTION_MODES
from python.training.jepa_model import ACTJEPAPolicy


def parse_args():
    p = argparse.ArgumentParser(description="Train ACT-JEPA policy")
    # Standard ACT args
    p.add_argument("--data-dir", required=True)
    p.add_argument("--output-dir", default="checkpoints/jepa")
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--chunk-size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--lr-backbone", type=float, default=1e-5)
    p.add_argument("--weight-decay", type=float, default=1e-4)
    p.add_argument("--kl-weight", type=float, default=10.0)
    p.add_argument("--d-model", type=int, default=256)
    p.add_argument("--dim-feedforward", type=int, default=2048)
    p.add_argument("--z-dim", type=int, default=32)
    p.add_argument("--nhead", type=int, default=8)
    p.add_argument("--num-encoder-layers", type=int, default=4)
    p.add_argument("--num-decoder-layers", type=int, default=7)
    p.add_argument("--state-mode", default="qpos_qcmd", choices=list(STATE_MODES))
    p.add_argument("--action-mode", default="relative", choices=list(ACTION_MODES))
    p.add_argument("--augment", action="store_true")
    p.add_argument("--val-fraction", type=float, default=0.15)
    p.add_argument("--val-seed", type=int, default=0)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--save-every", type=int, default=10)
    p.add_argument("--resume", action="store_true")
    p.add_argument("--cache", action="store_true")
    p.add_argument("--no-tensorboard", action="store_true", dest="no_tensorboard")
    # JEPA-specific args
    p.add_argument("--future-offset", type=int, default=None,
                   help="Frames into the future for the JEPA target. "
                        "Defaults to --chunk-size (predict end-of-chunk).")
    p.add_argument("--lambda-obs", type=float, default=0.5,
                   help="Weight on the latent-space prediction loss.")
    p.add_argument("--lambda-reg", type=float, default=0.05,
                   help="Weight on the SIGReg isotropic-Gaussian regularizer.")
    p.add_argument("--predictor-layers", type=int, default=4)
    p.add_argument("--predictor-heads", type=int, default=8)
    p.add_argument("--predictor-ff", type=int, default=1024)
    p.add_argument("--sigreg-slices", type=int, default=1024)
    p.add_argument("--sigreg-points", type=int, default=17)
    p.add_argument("--target-grad", action="store_true",
                   help="Let gradients flow through the target encoder "
                        "(default: stop-gradient on the target path).")
    return p.parse_args()


def find_latest_checkpoint(output_dir: Path) -> Path | None:
    ckpts = sorted(output_dir.glob("act_jepa_epoch_*.pt"))
    return ckpts[-1] if ckpts else None


def collate_fn(batch):
    obs_list, action_list = zip(*batch)
    actions = torch.stack(action_list, dim=0)
    qpos = torch.stack([o["qpos"] for o in obs_list], dim=0)
    state = torch.stack([o["state"] for o in obs_list], dim=0)
    imgs_gripper = torch.stack([o["images"]["gripper"] for o in obs_list], dim=0)
    obs = {
        "qpos": qpos,
        "state": state,
        "images": {"gripper": imgs_gripper},
    }
    if "future_images" in obs_list[0]:
        obs["future_images"] = {
            "gripper": torch.stack([o["future_images"]["gripper"] for o in obs_list], dim=0),
        }
    return obs, actions


def run_epoch(policy, loader, device, *, optimizer=None, clip_grad=0.1):
    is_train = optimizer is not None
    policy.train(is_train)

    sums = {"l1": 0.0, "kl": 0.0, "obs": 0.0, "reg": 0.0, "total": 0.0}
    n_batches = 0

    ctx = torch.enable_grad() if is_train else torch.no_grad()
    with ctx:
        for obs, actions in loader:
            qpos = obs["qpos"].to(device, non_blocking=True)
            state = obs["state"].to(device, non_blocking=True)
            imgs_gripper = obs["images"]["gripper"].to(device, non_blocking=True)
            actions = actions.to(device, non_blocking=True)

            future_gripper = None
            if "future_images" in obs:
                future_gripper = obs["future_images"]["gripper"].to(device, non_blocking=True)

            loss_dict = policy(
                qpos, state, imgs_gripper, actions,
                future_images_gripper=future_gripper,
            )

            if is_train:
                optimizer.zero_grad()
                loss_dict["total"].backward()
                nn.utils.clip_grad_norm_(policy.parameters(), max_norm=clip_grad)
                optimizer.step()

            for k in sums:
                sums[k] += loss_dict[k].item()
            n_batches += 1

    n = max(n_batches, 1)
    return {k: v / n for k, v in sums.items()}


def main():
    args = parse_args()
    device = torch.device(args.device)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    future_offset = args.future_offset if args.future_offset is not None else args.chunk_size

    print(f"Device: {device}")
    print(f"Data dir: {args.data_dir}")
    print(f"Output dir: {output_dir}")
    print(f"State mode: {args.state_mode}  Action mode: {args.action_mode}  "
          f"Augment: {args.augment}  future_offset: {future_offset}")
    print(f"JEPA weights: lambda_obs={args.lambda_obs}  lambda_reg={args.lambda_reg}")

    train_paths, val_paths = split_episodes(
        args.data_dir, val_fraction=args.val_fraction, seed=args.val_seed
    )
    print(f"Episodes: {len(train_paths)} train / {len(val_paths)} val")

    train_set = EpisodeDataset(
        episode_paths=train_paths,
        chunk_size=args.chunk_size,
        state_mode=args.state_mode,
        action_mode=args.action_mode,
        augment=args.augment,
        cache=args.cache,
        future_offset=future_offset,
    )
    print(f"Train: {len(train_set)} samples  num_joints={train_set.num_joints}  "
          f"state_dim={train_set.state_dim}")

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
            future_offset=future_offset,
        )
        print(f"Val:   {len(val_set)} samples")

    pin = (device.type == "cuda")
    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True,
        num_workers=args.num_workers, pin_memory=pin, drop_last=True,
        collate_fn=collate_fn,
    )
    val_loader = None
    if val_set is not None:
        val_loader = DataLoader(
            val_set, batch_size=args.batch_size, shuffle=False,
            num_workers=args.num_workers, pin_memory=pin, drop_last=False,
            collate_fn=collate_fn,
        )

    policy = ACTJEPAPolicy(
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
        predictor_layers=args.predictor_layers,
        predictor_heads=args.predictor_heads,
        predictor_ff=args.predictor_ff,
        lambda_obs=args.lambda_obs,
        lambda_reg=args.lambda_reg,
        sigreg_slices=args.sigreg_slices,
        sigreg_points=args.sigreg_points,
        target_no_grad=not args.target_grad,
    ).to(device)

    n_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    n_jepa = sum(p.numel() for p in policy.jepa_parameters())
    print(f"Total parameters: {n_params:,}  (of which JEPA predictor: {n_jepa:,})")

    param_groups = [
        {"params": policy.backbone_parameters(), "lr": args.lr_backbone},
        {"params": policy.non_backbone_parameters(), "lr": args.lr},
    ]
    optimizer = torch.optim.AdamW(param_groups, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=args.lr * 0.01
    )

    writer = None
    if not args.no_tensorboard:
        writer = SummaryWriter(log_dir=str(output_dir / "tb_logs"))
        print(f"TensorBoard: {output_dir / 'tb_logs'}")

    start_epoch = 0
    best_val = float("inf")

    if args.resume:
        ckpt_path = find_latest_checkpoint(output_dir)
        if ckpt_path is None:
            print("No JEPA checkpoint found, starting fresh.")
        else:
            print(f"Resuming from {ckpt_path}")
            ckpt = torch.load(ckpt_path, map_location=device)
            policy.load_state_dict(ckpt["model_state_dict"])
            optimizer.load_state_dict(ckpt["optimizer_state_dict"])
            if "scheduler_state_dict" in ckpt:
                scheduler.load_state_dict(ckpt["scheduler_state_dict"])
            start_epoch = ckpt["epoch"] + 1
            best_val = ckpt.get("best_val", best_val)
            print(f"Resuming from epoch {start_epoch}  best_val={best_val:.4f}")

    def save_checkpoint(path: Path, epoch, train_loss, val_loss):
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
                    "model": "act_jepa",
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
                    "future_offset": future_offset,
                    "lambda_obs": args.lambda_obs,
                    "lambda_reg": args.lambda_reg,
                    "predictor_layers": args.predictor_layers,
                    "predictor_heads": args.predictor_heads,
                    "predictor_ff": args.predictor_ff,
                    "sigreg_slices": args.sigreg_slices,
                    "sigreg_points": args.sigreg_points,
                    "target_no_grad": not args.target_grad,
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
            f"obs={train_loss['obs']:.4f} reg={train_loss['reg']:.4f} "
            f"total={train_loss['total']:.4f}"
        )
        if val_loss is not None:
            msg += (f"  |  val l1={val_loss['l1']:.4f} obs={val_loss['obs']:.4f} "
                    f"total={val_loss['total']:.4f}")
        msg += f"  lr={lr_main:.2e} (bb={lr_backbone:.2e})"
        print(msg)

        if writer is not None:
            for k, v in train_loss.items():
                writer.add_scalar(f"train/{k}", v, epoch + 1)
            if val_loss is not None:
                for k, v in val_loss.items():
                    writer.add_scalar(f"val/{k}", v, epoch + 1)
            writer.add_scalar("lr/main", lr_main, epoch + 1)
            writer.add_scalar("lr/backbone", lr_backbone, epoch + 1)

        if val_loss is not None and val_loss["total"] < best_val:
            best_val = val_loss["total"]
            best_path = output_dir / "act_jepa_best.pt"
            save_checkpoint(best_path, epoch, train_loss, val_loss)
            print(f"  New best val total={best_val:.4f} → {best_path}")

        if (epoch + 1) % args.save_every == 0 or (epoch + 1) == args.epochs:
            ckpt_path = output_dir / f"act_jepa_epoch_{epoch+1:04d}.pt"
            save_checkpoint(ckpt_path, epoch, train_loss, val_loss)
            print(f"  Saved checkpoint: {ckpt_path}")

    if writer is not None:
        writer.close()
    print("ACT-JEPA training complete.")


if __name__ == "__main__":
    main()
