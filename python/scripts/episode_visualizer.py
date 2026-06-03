#!/usr/bin/env python3
"""episode_visualizer.py — offline 3D replay + ACT policy prediction viewer.

Loads a recorded episode + an ACT/ACT-JEPA checkpoint and streams everything
into a Rerun viewer on a single timeline:

  * The actual robot pose (stick-figure FK from `record_replay._log_arm_fk`).
  * `--horizon` ghost arms showing the policy's predicted action chunk
    (h=0 closest in time, h=N farthest), each on its own entity subtree so
    individual horizon steps can be toggled in Rerun's entity panel.
  * The gripper camera frame at every step.
  * Per-joint scalar plots: recorded qpos / qcmd / action vs. predicted h=0.

Usage:
    python python/scripts/episode_visualizer.py
        --episode episodes/episode_0100.hdf5
        --checkpoint checkpoints/jepa/act_jepa_best.pt
        --horizon 16

Defaults to CPU because a training job typically owns the GPU; pass
`--device cuda` if your GPU is free.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import h5py
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rerun as rr
import rerun.blueprint as rrb

from python.scripts.record_replay import ARM_JOINTS, _log_static_arm, _log_arm_fk
from python.training.inference import load_checkpoint, predict_chunk


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Offline 3D episode + ACT prediction visualizer (Rerun).",
    )
    ap.add_argument("--episode",    required=True, type=Path)
    ap.add_argument("--checkpoint", required=True, type=Path)
    ap.add_argument("--horizon",    type=int, default=16,
                    help="Number of future-step ghost arms to draw (≤ chunk_size).")
    ap.add_argument("--stride",     type=int, default=1,
                    help="Run inference every N-th frame (intermediate frames "
                         "show stale ghosts). 1 = every frame.")
    ap.add_argument("--max-steps",  type=int, default=None,
                    help="Truncate the episode to the first N frames "
                         "(handy for fast iteration).")
    ap.add_argument("--device",     default="cpu",
                    help="Inference device. Defaults to CPU because the GPU "
                         "is usually busy with training.")
    ap.add_argument("--qcmd-source", choices=["recorded", "predicted"], default="recorded",
                    help="`recorded`: feed observations/qcmd[t] from the HDF5 "
                         "(matches the training distribution exactly). "
                         "`predicted`: feed the previous predicted action "
                         "(closed-loop sim; shows compounding drift).")
    ap.add_argument("--no-image",   action="store_true", dest="no_image",
                    help="Skip logging the gripper image stream (faster).")
    ap.add_argument("--memory-limit", default="2GiB",
                    help="Rerun viewer memory limit.")
    return ap.parse_args()


def _ghost_color(j: int, n: int) -> Tuple[int, int, int, int]:
    """Per-horizon color: orange (j=0, most certain) → blue (j=N, most distant),
    with alpha decay so far-future ghosts read as faint hints."""
    t = j / max(n - 1, 1)
    r = int(255 * (1.0 - t))
    g = int(140 * (1.0 - t) + 100 * t)
    b = int(255 * t)
    a = int(200 - 150 * t)
    return (r, g, b, a)


def _load_episode(
    path: Path, max_steps: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Load qpos / qcmd / torques / gripper images / recorded actions / hz."""
    with h5py.File(path, "r") as f:
        qpos    = f["observations/qpos"][:]
        qcmd    = (f["observations/qcmd"][:]
                   if "observations/qcmd" in f else qpos.copy())
        torques = (f["observations/torques"][:]
                   if "observations/torques" in f else np.zeros_like(qpos))
        gripper = f["observations/images/gripper"][:]
        actions = f["actions"][:]
        hz = float(f.attrs.get("hz", 20))
    if max_steps is not None and max_steps < len(qpos):
        qpos    = qpos[:max_steps]
        qcmd    = qcmd[:max_steps]
        torques = torques[:max_steps]
        gripper = gripper[:max_steps]
        actions = actions[:max_steps]
    return qpos, qcmd, torques, gripper, actions, hz


def _precompute_predictions(
    policy, qpos, qcmd, torques, gripper,
    *, dataset_stats, config, qcmd_source: str, stride: int, device: torch.device,
) -> np.ndarray:
    """Run inference at every stride-th frame and return [T, chunk_size, J]."""
    T, J = qpos.shape
    chunk_size = config["chunk_size"]
    pred = np.full((T, chunk_size, J), np.nan, dtype=np.float32)

    last_pred: Optional[np.ndarray] = None
    t0 = time.monotonic()
    n_done = 0
    n_total = (T + stride - 1) // stride
    for t in range(0, T, stride):
        if qcmd_source == "recorded":
            qcmd_in = qcmd[t]
        else:
            qcmd_in = last_pred if last_pred is not None else qpos[t]
        chunk = predict_chunk(
            policy, qpos[t], gripper[t],
            dataset_stats=dataset_stats, config=config,
            qcmd_raw=qcmd_in, torques_raw=torques[t],
            device=device,
        )
        pred[t] = chunk
        last_pred = chunk[0].copy()
        n_done += 1
        if n_done % 10 == 0 or n_done == n_total:
            elapsed = time.monotonic() - t0
            rate = n_done / max(elapsed, 1e-6)
            eta = (n_total - n_done) / max(rate, 1e-6)
            print(f"\r  {n_done:4d}/{n_total}  {rate:5.1f} step/s  ETA {eta:5.0f}s   ",
                  end="", flush=True)
    print(f"\n  done in {time.monotonic() - t0:.1f}s")
    return pred


def _build_blueprint(horizon: int) -> rrb.Blueprint:
    """Right column: gripper image + scalar plots. Left: 3D arm + ghosts."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="3D — robot + predicted ghosts", origin="world"),
            rrb.Vertical(
                rrb.Spatial2DView(name="Gripper cam", origin="cameras/gripper"),
                rrb.TimeSeriesView(name="qpos (recorded)", origin="qpos"),
                rrb.TimeSeriesView(name="action (recorded vs predicted h=0)",
                                   origin="action"),
                row_shares=[3, 2, 2],
            ),
            column_shares=[3, 2],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded"),
    )


def main() -> None:
    args = _parse_args()

    print(f"Episode:    {args.episode}")
    print(f"Checkpoint: {args.checkpoint}")
    qpos, qcmd, torques, gripper, actions, hz = _load_episode(
        args.episode, max_steps=args.max_steps,
    )
    T, J = qpos.shape
    print(f"Loaded T={T} steps @ {hz:.0f} Hz, J={J}, gripper {gripper.shape[1]}x{gripper.shape[2]}")

    device = torch.device(args.device)
    policy, dataset_stats, config = load_checkpoint(
        str(args.checkpoint), device, strict=False,
    )
    chunk_size = config["chunk_size"]
    horizon = max(1, min(args.horizon, chunk_size))
    print(f"Policy:     chunk_size={chunk_size}  state_mode={config['state_mode']}  "
          f"action_mode={config['action_mode']}  device={device}")
    print(f"Drawing {horizon} ghost horizons (max = chunk_size = {chunk_size}).")

    print("Pre-computing predictions...")
    pred = _precompute_predictions(
        policy, qpos, qcmd, torques, gripper,
        dataset_stats=dataset_stats, config=config,
        qcmd_source=args.qcmd_source, stride=args.stride, device=device,
    )

    print("Streaming to Rerun...")
    rr.init(
        "aizee-episode-visualizer",
        spawn=True,
        default_blueprint=_build_blueprint(horizon),
    )

    # Static link geometry — once for the actual robot, once per ghost subtree.
    _log_static_arm()
    for j in range(horizon):
        _log_static_arm(
            root=f"world/ghosts/h_{j:02d}/arm",
            color_override=_ghost_color(j, horizon),
            include_rover_body=False,
        )

    # Per-frame: actual pose, ghost FKs, image, scalars.
    for t in range(T):
        rr.set_time("step", sequence=t)
        rr.set_time("time", duration=t / hz)

        _log_arm_fk(qpos[t])

        if not np.isnan(pred[t, 0, 0]):
            for j in range(horizon):
                _log_arm_fk(pred[t, j], root=f"world/ghosts/h_{j:02d}/arm")

        if not args.no_image:
            rr.log("cameras/gripper", rr.Image(gripper[t]))

        for ji, jn in enumerate(ARM_JOINTS):
            rr.log(f"qpos/{jn}",            rr.Scalars(float(qpos[t, ji])))
            rr.log(f"qpos/cmd_{jn}",        rr.Scalars(float(qcmd[t, ji])))
            rr.log(f"action/actual_{jn}",   rr.Scalars(float(actions[t, ji])))
            if not np.isnan(pred[t, 0, ji]):
                rr.log(f"action/pred_h00_{jn}",
                       rr.Scalars(float(pred[t, 0, ji])))

    print(f"Logged {T} frames. Rerun viewer should be open; use the timeline "
          f"to scrub. Toggle ghost horizons under `world/ghosts/h_*` in the entity tree.")


if __name__ == "__main__":
    main()
