"""
train_minerva.py — training loop for the Minerva flow-matching + JEPA policy.

Mirrors train_jepa.py but instantiates MinervaPolicy over the 3-camera
MinervaEpisodeDataset, adds an EMA of the weights (required for stable
flow/diffusion heads), and logs the flow / JEPA-obs / SIGReg loss terms.

The language cache must be built first (once, offline):
    python -m python.training.build_lang_cache --data-dir episodes/ \
        --out checkpoints/minerva/task_embeddings.npz

Then:
    python -m python.training.train_minerva --data-dir episodes/ \
        --output-dir checkpoints/minerva --epochs 300 --batch-size 16 \
        --chunk-size 32 --future-offset 32 --augment \
        --lang-cache checkpoints/minerva/task_embeddings.npz
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import torch
import torch.nn as nn
from contextlib import contextmanager
import random
import numpy as np
from torch.utils.data import DataLoader, WeightedRandomSampler
from torch.utils.tensorboard import SummaryWriter

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.training.language import TextConditioner
from python.training.minerva_dataset import (
    MinervaEpisodeDataset, collate_minerva, split_episodes, STATE_MODES, ACTION_MODES,
)
from python.training.minerva_model import MinervaPolicy


class EMA:
    """Exponential moving average of floating-point model parameters/buffers with step warmup."""

    def __init__(self, model: nn.Module, decay: float = 0.9999, *, warmup: bool = True):
        self.decay = float(decay)
        self.warmup = bool(warmup)
        self.num_updates = 0
        self.shadow = {
            k: v.detach().clone()
            for k, v in model.state_dict().items()
            if torch.is_floating_point(v)
        }

    @torch.no_grad()
    def update(self, model: nn.Module):
        self.num_updates += 1
        d = (
            min(self.decay, (1.0 + self.num_updates) / (10.0 + self.num_updates))
            if self.warmup else self.decay
        )
        for k, v in model.state_dict().items():
            if k in self.shadow:
                self.shadow[k].mul_(d).add_(v.detach(), alpha=1.0 - d)

    def state_dict(self):
        return self.shadow

    def load_state_dict(self, shadow: dict, num_updates: int = 0):
        self.num_updates = int(num_updates)
        for k, v in shadow.items():
            if k in self.shadow:
                self.shadow[k].copy_(v)

    @contextmanager
    def average_parameters(self, model: nn.Module):
        """Temporarily swap the EMA shadow weights into `model` (e.g. for validation)."""
        backup = {}
        with torch.no_grad():
            for name, param in model.named_parameters():
                if name in self.shadow:
                    backup[name] = param.detach().clone()
                    param.copy_(self.shadow[name])
            for name, buf in model.named_buffers():
                if name in self.shadow:
                    backup[name] = buf.detach().clone()
                    buf.copy_(self.shadow[name])
        try:
            yield
        finally:
            with torch.no_grad():
                for name, param in model.named_parameters():
                    if name in backup:
                        param.copy_(backup[name])
                for name, buf in model.named_buffers():
                    if name in backup:
                        buf.copy_(backup[name])


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Train Minerva flow-matching + JEPA policy")
    p.add_argument("--data-dir", required=True)
    p.add_argument("--output-dir", default="checkpoints/minerva")
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--batch-size", type=int, default=16)
    p.add_argument("--chunk-size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--lr-backbone", type=float, default=1e-5)
    p.add_argument("--weight-decay", type=float, default=1e-4)
    p.add_argument("--d-model", type=int, default=512)
    p.add_argument("--nhead", type=int, default=8)
    p.add_argument("--head-layers", type=int, default=6)
    p.add_argument("--head-ff", type=int, default=2048)
    p.add_argument("--flow-steps", type=int, default=8)
    p.add_argument("--camera-dropout", type=float, default=0.15)
    p.add_argument("--state-mode", default="qpos_qcmd", choices=list(STATE_MODES))
    p.add_argument("--action-mode", default="absolute", choices=list(ACTION_MODES))
    p.add_argument("--augment", action="store_true")
    p.add_argument("--future-offset", type=int, default=None,
                   help="JEPA target horizon (defaults to --chunk-size). 0 disables JEPA.")
    p.add_argument("--lambda-obs", type=float, default=0.3)
    p.add_argument("--lambda-reg", type=float, default=0.05)
    p.add_argument("--predictor-layers", type=int, default=3)
    p.add_argument("--ema-decay", type=float, default=0.9999)
    p.add_argument("--val-fraction", type=float, default=0.15)
    p.add_argument("--val-seed", type=int, default=0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--save-every", type=int, default=10)
    p.add_argument("--clip-grad", type=float, default=1.0)
    p.add_argument("--include-fixture", action="store_true",
                   help="Include scripted_fixture / capture_eligible=False episodes (for sim smoke tests).")
    p.add_argument("--correction-weight", type=float, default=3.0,
                   help="IWR sampling weight for authority='correction' frames relative to 'human' frames.")
    p.add_argument("--no-pretrained-encoder", action="store_true",
                   help="Initialize ResNet18 encoders from scratch (offline/CPU test mode).")
    p.add_argument("--resume", default=None, help="Path to checkpoint to resume training from.")
    # Language
    p.add_argument("--lang-model", default="sentence-transformers/all-MiniLM-L6-v2")
    p.add_argument("--lang-cache", default=None,
                   help="Prebuilt embedding cache (.npz). Omit to disable language conditioning.")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--no-tensorboard", action="store_true")
    return p.parse_args(argv)


def _seed_everything(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def run_epoch(policy, loader, device, has_lang, *, optimizer=None, ema=None, clip=1.0):
    is_train = optimizer is not None
    policy.train(is_train)
    sums = {"flow": 0.0, "obs": 0.0, "reg": 0.0, "total": 0.0}
    n = 0
    ctx = torch.enable_grad() if is_train else torch.no_grad()
    with ctx:
        for obs, actions in loader:
            state = obs["state"].to(device, non_blocking=True)
            images = {c: v.to(device, non_blocking=True) for c, v in obs["images"].items()}
            actions = actions.to(device, non_blocking=True)
            future = ({c: v.to(device, non_blocking=True) for c, v in obs["future_images"].items()}
                      if "future_images" in obs else None)
            language = obs["language"].to(device, non_blocking=True) if has_lang and "language" in obs else None

            loss = policy(state, images, actions, language=language, future_images=future)
            for k, val in loss.items():
                if not torch.isfinite(val):
                    raise RuntimeError(f"Non-finite {k} loss ({val.item()}) encountered during {'train' if is_train else 'val'}")
            if is_train:
                optimizer.zero_grad()
                loss["total"].backward()
                nn.utils.clip_grad_norm_(policy.parameters(), clip)
                optimizer.step()
                if ema is not None:
                    ema.update(policy)
            for k in sums:
                sums[k] += loss[k].item()
            n += 1
    if n == 0:
        raise RuntimeError("DataLoader yielded zero batches")
    return {k: v / n for k, v in sums.items()}


def main(argv=None):
    args = parse_args(argv)
    _seed_everything(args.seed)
    device = torch.device(args.device)
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    future_offset = args.future_offset if args.future_offset is not None else args.chunk_size

    conditioner = None
    if args.lang_cache:
        conditioner = TextConditioner(model_name=args.lang_model, cache_path=args.lang_cache)
        print(f"Language ON: {args.lang_model} (dim={conditioner.embed_dim}), cache={args.lang_cache}")
    else:
        print("Language OFF (no --lang-cache)")
    lang_dim = conditioner.embed_dim if conditioner is not None else 0

    train_paths, val_paths = split_episodes(
        args.data_dir, args.val_fraction, args.val_seed, include_ineligible=args.include_fixture,
    )
    print(f"Episodes: {len(train_paths)} train / {len(val_paths)} val")

    train_set = MinervaEpisodeDataset(
        train_paths, chunk_size=args.chunk_size, state_mode=args.state_mode,
        action_mode=args.action_mode, augment=args.augment, future_offset=future_offset,
        conditioner=conditioner, include_fixture=args.include_fixture,
        correction_weight=args.correction_weight,
    )
    val_set = MinervaEpisodeDataset(
        val_paths, chunk_size=args.chunk_size, state_mode=args.state_mode,
        action_mode=args.action_mode, augment=False, future_offset=future_offset,
        dataset_stats=train_set.dataset_stats, conditioner=conditioner,
        include_fixture=args.include_fixture,
    ) if val_paths else None
    print(f"num_joints={train_set.num_joints} joints={train_set.joint_names} "
          f"state_dim={train_set.state_dim} cameras={train_set.cameras} "
          f"image_shapes={train_set.image_shapes}")

    pin = device.type == "cuda"
    loader_gen = torch.Generator()
    loader_gen.manual_seed(args.seed)
    use_weighted = np.any(train_set.sample_weights != 1.0)
    if use_weighted:
        sampler = WeightedRandomSampler(
            weights=torch.from_numpy(train_set.sample_weights),
            num_samples=len(train_set),
            replacement=True,
            generator=loader_gen,
        )
        train_loader = DataLoader(
            train_set, batch_size=args.batch_size, sampler=sampler,
            num_workers=args.num_workers, pin_memory=pin, drop_last=len(train_set) >= args.batch_size,
            collate_fn=collate_minerva,
        )
    else:
        train_loader = DataLoader(
            train_set, batch_size=args.batch_size, shuffle=True, generator=loader_gen,
            num_workers=args.num_workers, pin_memory=pin, drop_last=len(train_set) >= args.batch_size,
            collate_fn=collate_minerva,
        )
    val_loader = (DataLoader(val_set, batch_size=args.batch_size, shuffle=False,
                             num_workers=args.num_workers, pin_memory=pin,
                             collate_fn=collate_minerva) if val_set else None)

    policy = MinervaPolicy(
        num_joints=train_set.num_joints, chunk_size=args.chunk_size, d_model=args.d_model,
        state_dim=train_set.state_dim, lang_dim=lang_dim, nhead=args.nhead,
        head_layers=args.head_layers, head_ff=args.head_ff, camera_dropout=args.camera_dropout,
        flow_steps=args.flow_steps, lambda_obs=args.lambda_obs, lambda_reg=args.lambda_reg,
        predictor_layers=args.predictor_layers,
        pretrained_encoder=not args.no_pretrained_encoder,
    ).to(device)
    n_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"Parameters: {n_params:,}")

    optimizer = torch.optim.AdamW(
        [{"params": policy.backbone_parameters(), "lr": args.lr_backbone},
         {"params": policy.non_backbone_parameters(), "lr": args.lr}],
        weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs, 1), eta_min=args.lr * 0.01)
    ema = EMA(policy, decay=args.ema_decay)

    start_epoch = 0
    best_val = float("inf")
    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=False)
        policy.load_state_dict(ckpt["model_state_dict"], strict=True)
        if "ema_state_dict" in ckpt:
            ema.load_state_dict(ckpt["ema_state_dict"], ckpt.get("ema_num_updates", 0))
        if "optimizer_state_dict" in ckpt:
            optimizer.load_state_dict(ckpt["optimizer_state_dict"])
        if "scheduler_state_dict" in ckpt:
            scheduler.load_state_dict(ckpt["scheduler_state_dict"])
        start_epoch = int(ckpt.get("epoch", -1)) + 1
        if ckpt.get("val_loss") and isinstance(ckpt["val_loss"], dict):
            best_val = float(ckpt["val_loss"].get("total", float("inf")))
        print(f"Resumed from {args.resume} at epoch {start_epoch}")

    writer = None if args.no_tensorboard else SummaryWriter(log_dir=str(out / "tb_logs"))

    def save(path, epoch, train_loss, val_loss):
        torch.save({
            "epoch": epoch,
            "model_state_dict": policy.state_dict(),
            "ema_state_dict": ema.state_dict(),
            "ema_num_updates": ema.num_updates,
            "optimizer_state_dict": optimizer.state_dict(),
            "scheduler_state_dict": scheduler.state_dict(),
            "dataset_stats": {k: (v.tolist() if hasattr(v, "tolist") else v)
                              for k, v in train_set.dataset_stats.items()},
            "config": {
                "model": "minerva", "num_joints": train_set.num_joints,
                "joint_names": list(train_set.joint_names),
                "chunk_size": args.chunk_size, "d_model": args.d_model, "nhead": args.nhead,
                "head_layers": args.head_layers, "head_ff": args.head_ff,
                "flow_steps": args.flow_steps, "state_mode": args.state_mode,
                "state_dim": train_set.state_dim, "action_mode": args.action_mode,
                "cameras": train_set.cameras,
                "image_shapes": {c: list(s) for c, s in train_set.image_shapes.items()},
                "lang_dim": lang_dim,
                "lang_model": args.lang_model if conditioner else None,
                "camera_dropout": args.camera_dropout,
                "seed": args.seed,
                "correction_weight": args.correction_weight,
            },
            "train_loss": train_loss, "val_loss": val_loss,
        }, path)

    has_lang = conditioner is not None
    for epoch in range(start_epoch, args.epochs):
        tr = run_epoch(policy, train_loader, device, has_lang,
                       optimizer=optimizer, ema=ema, clip=args.clip_grad)
        if val_loader:
            with ema.average_parameters(policy):
                va = run_epoch(policy, val_loader, device, has_lang)
        else:
            va = None
        scheduler.step()
        msg = (f"Epoch {epoch+1}/{args.epochs} — flow={tr['flow']:.4f} obs={tr['obs']:.4f} "
               f"reg={tr['reg']:.4f} total={tr['total']:.4f}")
        if va:
            msg += f" | val(EMA) flow={va['flow']:.4f} total={va['total']:.4f}"
        print(msg)
        if writer:
            for k, v in tr.items():
                writer.add_scalar(f"train/{k}", v, epoch + 1)
            if va:
                for k, v in va.items():
                    writer.add_scalar(f"val/{k}", v, epoch + 1)
        if va and va["total"] < best_val:
            best_val = va["total"]
            save(out / "minerva_best.pt", epoch, tr, va)
            print(f"  New best val(EMA) total={best_val:.4f}")
        elif not va and (epoch + 1) == args.epochs:
            save(out / "minerva_best.pt", epoch, tr, va)
        if (epoch + 1) % args.save_every == 0 or (epoch + 1) == args.epochs:
            save(out / f"minerva_epoch_{epoch+1:04d}.pt", epoch, tr, va)

    if writer:
        writer.close()
    print("Minerva training complete.")


if __name__ == "__main__":
    main()
