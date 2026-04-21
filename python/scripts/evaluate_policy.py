#!/usr/bin/env python3
"""
evaluate_policy.py — Offline ACT policy evaluation.

Replays recorded HDF5 episodes through a trained ACT model in open-loop
(feeding ground truth observations), compares predicted vs ground truth
actions, and visualizes results in Rerun with per-joint error metrics.

Usage:
    python evaluate_policy.py --checkpoint ckpt.pt --episode episodes/episode_0001.hdf5
    python evaluate_policy.py --checkpoint ckpt.pt --episode-dir episodes/
    python evaluate_policy.py --checkpoint ckpt.pt --episode-dir episodes/ --ensemble --no-images
    python evaluate_policy.py --checkpoint ckpt.pt --episode-dir episodes/ --save eval.rrd --csv eval.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import List

import h5py
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import torch

# Allow running from repo root or python/scripts/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from python.nodes.act_policy_node import (
    POLICY_JOINTS,
    NUM_POLICY_JOINTS,
    TemporalEnsemble,
    _build_state_vector,
    denormalize_actions,
    load_checkpoint,
    normalize_image,
    normalize_qpos,
)


# ---------------------------------------------------------------------------
# Episode loading
# ---------------------------------------------------------------------------

def load_episode(path: str) -> dict:
    """Load one HDF5 episode into memory. Returns dict of numpy arrays.

    Expects format_version=2 episodes (7-DOF qpos/qcmd/actions with swivel first).
    """
    with h5py.File(path, "r") as f:
        ep = {
            "qpos": f["observations/qpos"][:],          # [T, J]
            "actions": f["actions"][:],                  # [T, J]
        }
        if "observations/images/left" in f:
            ep["img_left"] = f["observations/images/left"][:]
        if "observations/images/right" in f:
            ep["img_right"] = f["observations/images/right"][:]
        if "observations/qcmd" in f:
            ep["qcmd"] = f["observations/qcmd"][:]
        if "observations/torques" in f:
            ep["torques"] = f["observations/torques"][:]
        ep["hz"] = int(f.attrs.get("hz", 20))
        ep["num_joints"] = int(ep["qpos"].shape[1])
    return ep


def collect_episodes(args) -> List[str]:
    if args.episode:
        return args.episode
    ep_dir = Path(args.episode_dir)
    files = sorted(ep_dir.glob("episode_*.hdf5"))
    if not files:
        print(f"No episode_*.hdf5 files found in {ep_dir}")
        sys.exit(1)
    return [str(f) for f in files]


# ---------------------------------------------------------------------------
# Rerun blueprint
# ---------------------------------------------------------------------------

def build_eval_blueprint(show_images: bool, joint_names: List[str]) -> rrb.Blueprint:
    action_contents = []
    for joint in joint_names:
        action_contents.append(f"eval/gt_action/{joint}")
        action_contents.append(f"eval/pred_action/{joint}")
    actions_view = rrb.TimeSeriesView(
        name="GT vs Predicted Actions", contents=action_contents,
    )
    error_view = rrb.TimeSeriesView(
        name="Per-Joint L1 Error",
        contents=[f"eval/l1_error/{joint}" for joint in joint_names],
    )
    infer_view = rrb.TimeSeriesView(
        name="Inference Time (ms)", contents=["eval/inference_ms"],
    )
    info_view = rrb.TextDocumentView(name="Info", origin="eval/info")

    right_col = rrb.Vertical(
        actions_view, error_view, infer_view,
        row_shares=[3, 2, 1],
    )

    if show_images:
        left_col = rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial2DView(name="Left", origin="cameras/left"),
                rrb.Spatial2DView(name="Right", origin="cameras/right"),
                column_shares=[1, 1],
            ),
            info_view,
            row_shares=[3, 1],
        )
        return rrb.Blueprint(
            rrb.Horizontal(left_col, right_col, column_shares=[2, 3])
        )
    return rrb.Blueprint(
        rrb.Horizontal(right_col, info_view, column_shares=[4, 1])
    )


# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------

GT_COLOR = [255, 200, 60]
PRED_COLOR = [80, 220, 80]
ERROR_COLOR = [255, 80, 80]
INFER_COLOR = [160, 160, 160]


def _log_series_colors(joint_names: List[str]):
    for joint in joint_names:
        rr.log(f"eval/gt_action/{joint}", rr.SeriesLines(colors=[GT_COLOR], names=[f"gt_{joint}"]), static=True)
        rr.log(f"eval/pred_action/{joint}", rr.SeriesLines(colors=[PRED_COLOR], names=[f"pred_{joint}"]), static=True)
        rr.log(f"eval/l1_error/{joint}", rr.SeriesLines(colors=[ERROR_COLOR], names=[joint]), static=True)
    rr.log("eval/inference_ms", rr.SeriesLines(colors=[INFER_COLOR], names=["infer_ms"]), static=True)


# ---------------------------------------------------------------------------
# Core evaluation
# ---------------------------------------------------------------------------

def evaluate_episode(
    ep: dict,
    policy,
    stats: dict,
    config: dict,
    device: torch.device,
    use_ensemble: bool,
    ensemble_steps: int,
    show_images: bool,
    frame_offset: int,
    speed: float,
    joint_names: List[str],
) -> dict:
    """Run open-loop evaluation on one episode. Returns metrics dict."""
    chunk_size = config["chunk_size"]
    num_joints = config["num_joints"]
    state_mode = config["state_mode"]
    action_mode = config["action_mode"]
    T = ep["qpos"].shape[0]

    if ep["num_joints"] != num_joints:
        raise ValueError(
            f"episode has {ep['num_joints']} joints, checkpoint expects {num_joints}"
        )

    ensemble = (TemporalEnsemble(chunk_size, ensemble_steps, num_joints)
                if use_ensemble else None)

    pred_actions = np.zeros((T, num_joints), dtype=np.float32)
    inference_times = np.zeros(T, dtype=np.float64)

    hz = ep.get("hz", 20)
    dt = 1.0 / hz
    has_images = "img_left" in ep and "img_right" in ep

    for t in range(T):
        frame = frame_offset + t

        qpos_raw = ep["qpos"][t].astype(np.float32)
        gt_action = ep["actions"][t].astype(np.float32)

        # For state_mode with qcmd, use ground truth qcmd (not recursive prediction)
        qcmd_raw = None
        if state_mode in ("qpos_qcmd", "qpos_qcmd_tq"):
            qcmd_raw = ep.get("qcmd", ep["qpos"])[t].astype(np.float32)
        torques_raw = None
        if state_mode == "qpos_qcmd_tq":
            torques_raw = ep.get("torques", np.zeros_like(ep["qpos"]))[t].astype(np.float32)

        qpos_norm = normalize_qpos(qpos_raw, stats)

        state_vec = _build_state_vector(
            qpos_norm, state_mode, qcmd_raw, qpos_raw, torques_raw, stats, num_joints,
        )

        if has_images:
            left_norm = normalize_image(ep["img_left"][t])
            right_norm = normalize_image(ep["img_right"][t])
        else:
            left_norm = np.zeros((3, 240, 320), dtype=np.float32)
            right_norm = np.zeros((3, 240, 320), dtype=np.float32)

        qpos_t = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)
        state_t = torch.from_numpy(state_vec).unsqueeze(0).to(device)
        left_t = torch.from_numpy(left_norm).unsqueeze(0).to(device)
        right_t = torch.from_numpy(right_norm).unsqueeze(0).to(device)

        t0 = time.perf_counter()
        with torch.no_grad():
            pred_chunk = policy.select_action(qpos_t, state_t, left_t, right_t)
        infer_ms = (time.perf_counter() - t0) * 1000.0

        pred_np = pred_chunk[0].cpu().numpy()
        pred_abs = denormalize_actions(pred_np, stats, action_mode, qpos=qpos_raw)

        if use_ensemble and ensemble is not None:
            ensemble.add_chunk(pred_abs)
            action = ensemble.get_action()
            ensemble.step()
            if action is None:
                action = pred_abs[0]
        else:
            action = pred_abs[0]

        pred_actions[t] = action
        inference_times[t] = infer_ms

        rr.set_time("frame", sequence=frame)
        for j, joint in enumerate(joint_names):
            rr.log(f"eval/gt_action/{joint}", rr.Scalars(float(gt_action[j])))
            rr.log(f"eval/pred_action/{joint}", rr.Scalars(float(action[j])))
            rr.log(f"eval/l1_error/{joint}", rr.Scalars(float(abs(action[j] - gt_action[j]))))
        rr.log("eval/inference_ms", rr.Scalars(infer_ms))

        if show_images and has_images:
            rr.log("cameras/left", rr.Image(ep["img_left"][t]))
            rr.log("cameras/right", rr.Image(ep["img_right"][t]))

        if speed > 0:
            time.sleep(dt / speed)

    gt_actions = ep["actions"][:T].astype(np.float32)
    l1_errors = np.abs(pred_actions - gt_actions)
    return {
        "per_joint_mean": l1_errors.mean(axis=0),
        "per_joint_max": l1_errors.max(axis=0),
        "overall_mean": l1_errors.mean(),
        "overall_max": l1_errors.max(),
        "mean_infer_ms": inference_times.mean(),
        "num_frames": T,
        "l1_errors": l1_errors,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Offline ACT policy evaluation — replay episodes through model",
    )
    parser.add_argument("--checkpoint", required=True, help="Path to .pt checkpoint")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--episode", nargs="+", help="One or more HDF5 episode files")
    group.add_argument("--episode-dir", help="Directory of episode_*.hdf5 files")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--ensemble", action="store_true", help="Enable temporal ensemble")
    parser.add_argument("--ensemble-steps", type=int, default=16,
                        help="Past chunks for ensemble (default: 16)")
    parser.add_argument("--no-images", action="store_true",
                        help="Skip camera images in Rerun (faster)")
    parser.add_argument("--speed", type=float, default=0,
                        help="Rerun playback speed (0 = fast as possible)")
    parser.add_argument("--save", default=None,
                        help="Save .rrd file instead of spawning viewer")
    parser.add_argument("--csv", default=None,
                        help="Export per-episode stats to CSV")
    args = parser.parse_args()

    device = torch.device(args.device)
    show_images = not args.no_images

    print(f"Loading checkpoint: {args.checkpoint}")
    policy, stats, config = load_checkpoint(args.checkpoint, device)
    chunk_size = config["chunk_size"]
    num_joints = config["num_joints"]
    state_mode = config["state_mode"]
    action_mode = config["action_mode"]
    print(f"  chunk_size={chunk_size}  num_joints={num_joints}  "
          f"state_mode={state_mode}  action_mode={action_mode}  device={device}")
    if args.ensemble:
        print(f"  ensemble enabled, steps={args.ensemble_steps}")

    joint_names = list(POLICY_JOINTS[:num_joints])
    if num_joints != NUM_POLICY_JOINTS:
        print(f"[WARN] num_joints={num_joints} but POLICY_JOINTS has "
              f"{NUM_POLICY_JOINTS} entries; using first {num_joints} names.")

    episode_paths = collect_episodes(args)
    print(f"Episodes: {len(episode_paths)}")

    rr.init("aizee_eval", spawn=args.save is None)
    if args.save:
        rr.save(args.save)
    rr.send_blueprint(build_eval_blueprint(show_images, joint_names))
    _log_series_colors(joint_names)

    all_metrics = []
    global_frame = 0

    print()
    print(f"{'Episode':<40s} {'Mean L1':>8s} {'Max L1':>8s} {'Inf (ms)':>10s}")
    print(f"{'-'*40} {'-'*8} {'-'*8} {'-'*10}")

    for ep_path in episode_paths:
        ep_name = Path(ep_path).stem
        print(f"  Loading {ep_name}...", end="", flush=True)
        ep = load_episode(ep_path)
        T = ep["qpos"].shape[0]
        print(f" {T} frames", flush=True)

        rr.set_time("frame", sequence=global_frame)
        rr.log("eval/info", rr.TextDocument(
            f"**Episode**: {ep_name}\n\n"
            f"**Frames**: {T}\n\n"
            f"**Ensemble**: {'on' if args.ensemble else 'off'}\n\n"
            f"**State mode**: {state_mode}\n\n"
            f"**Action mode**: {action_mode}",
            media_type=rr.MediaType.MARKDOWN,
        ))

        metrics = evaluate_episode(
            ep=ep,
            policy=policy,
            stats=stats,
            config=config,
            device=device,
            use_ensemble=args.ensemble,
            ensemble_steps=args.ensemble_steps,
            show_images=show_images,
            frame_offset=global_frame,
            speed=args.speed,
            joint_names=joint_names,
        )
        all_metrics.append((ep_name, metrics))
        global_frame += metrics["num_frames"]

        print(f"  {ep_name:<40s} {metrics['overall_mean']:8.4f} "
              f"{metrics['overall_max']:8.4f} {metrics['mean_infer_ms']:10.1f}")
        del ep

    print()
    if len(all_metrics) > 1:
        all_l1 = np.concatenate([m["l1_errors"] for _, m in all_metrics], axis=0)
    else:
        all_l1 = all_metrics[0][1]["l1_errors"]

    print(f"{'Joint':<14s} {'Mean L1':>8s} {'Max L1':>8s}")
    print(f"{'-'*14} {'-'*8} {'-'*8}")
    for j, joint in enumerate(joint_names):
        print(f"{joint:<14s} {all_l1[:, j].mean():8.4f} {all_l1[:, j].max():8.4f}")
    print(f"{'OVERALL':<14s} {all_l1.mean():8.4f} {all_l1.max():8.4f}")

    summary_lines = ["## Evaluation Summary\n"]
    summary_lines.append(f"| Joint | Mean L1 | Max L1 |")
    summary_lines.append(f"|-------|---------|--------|")
    for j, joint in enumerate(joint_names):
        summary_lines.append(f"| {joint} | {all_l1[:, j].mean():.4f} | {all_l1[:, j].max():.4f} |")
    summary_lines.append(f"| **OVERALL** | **{all_l1.mean():.4f}** | **{all_l1.max():.4f}** |")
    rr.set_time("frame", sequence=global_frame)
    rr.log("eval/info", rr.TextDocument(
        "\n".join(summary_lines), media_type=rr.MediaType.MARKDOWN,
    ))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["episode", "frames", "overall_mean_l1", "overall_max_l1", "mean_infer_ms"]
            header += [f"{j}_mean_l1" for j in joint_names]
            header += [f"{j}_max_l1" for j in joint_names]
            writer.writerow(header)
            for ep_name, m in all_metrics:
                row = [ep_name, m["num_frames"], f"{m['overall_mean']:.6f}",
                       f"{m['overall_max']:.6f}", f"{m['mean_infer_ms']:.2f}"]
                row += [f"{m['per_joint_mean'][j]:.6f}" for j in range(num_joints)]
                row += [f"{m['per_joint_max'][j]:.6f}" for j in range(num_joints)]
                writer.writerow(row)
        print(f"\nCSV saved: {args.csv}")

    if args.save:
        print(f"Rerun .rrd saved: {args.save}")

    print("\nDone.")


if __name__ == "__main__":
    main()
