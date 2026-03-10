#!/usr/bin/env python3
"""
view_episode.py — Visualize an HDF5 demonstration episode in Rerun.

Replays qpos, actions, and camera images from a single episode file.
The /timestamps group (if present) is used to show sync skew between
cameras and telemetry.

Usage:
    python view_episode.py episodes/episode_0001.hdf5
    python view_episode.py episodes/episode_0001.hdf5 --speed 2.0
    python view_episode.py episodes/episode_0001.hdf5 --no-images
    python view_episode.py episodes/episode_0001.hdf5 --save episode_0001.rrd
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import h5py
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

ARM_JOINTS = ["gantry_base", "gantry_mid", "gantry_end", "wrist_pitch", "wrist_roll", "gripper"]

# Per-joint colors (RGBA 0-255)
_QPOS_COLOR   = [80,  180, 255, 255]   # blue  — recorded position
_ACTION_COLOR = [255, 200,  60, 255]   # amber — recorded action target


def build_blueprint(show_images: bool, has_sync: bool) -> rrb.Blueprint:
    joint_panel = rrb.Vertical(
        rrb.TimeSeriesView(
            name="Joint Positions  qpos  (rad)",
            contents=["episode/qpos/**"],
        ),
        rrb.TimeSeriesView(
            name="Actions  (rad)",
            contents=["episode/actions/**"],
        ),
        row_shares=[1, 1],
    )

    if has_sync:
        data_col = rrb.Vertical(
            joint_panel,
            rrb.TimeSeriesView(
                name="Sync skew  (ms)",
                contents=["episode/sync/**"],
            ),
            row_shares=[4, 1],
        )
    else:
        data_col = joint_panel

    info_col = rrb.TextDocumentView(name="Episode Info", origin="episode/info")

    if show_images:
        cam_col = rrb.Vertical(
            rrb.Spatial2DView(name="Left camera",  origin="cameras/left"),
            rrb.Spatial2DView(name="Right camera", origin="cameras/right"),
            row_shares=[1, 1],
        )
        return rrb.Blueprint(
            rrb.Horizontal(
                cam_col,
                data_col,
                info_col,
                column_shares=[2, 4, 1],
            )
        )
    else:
        return rrb.Blueprint(
            rrb.Horizontal(
                data_col,
                info_col,
                column_shares=[5, 1],
            )
        )


def load_episode(path: Path):
    with h5py.File(path, "r") as f:
        qpos    = f["observations/qpos"][:]           # [T, 6]
        left    = f["observations/images/left"][:]    # [T, 240, 320, 3]
        right   = f["observations/images/right"][:]   # [T, 240, 320, 3]
        actions = f["actions"][:]                     # [T, 6]

        hz     = float(f.attrs.get("hz", 20))
        joints = f.attrs.get("arm_joints", ",".join(ARM_JOINTS))
        joint_names = [j.strip() for j in joints.split(",")]

        ts_telem = ts_left = ts_right = None
        if "timestamps" in f:
            ts_telem = f["timestamps/telem"][:]
            ts_left  = f["timestamps/camera_left"][:]
            ts_right = f["timestamps/camera_right"][:]

    return {
        "qpos": qpos, "left": left, "right": right, "actions": actions,
        "hz": hz, "joint_names": joint_names,
        "ts_telem": ts_telem, "ts_left": ts_left, "ts_right": ts_right,
    }


def print_stats(path: Path, ep: dict) -> str:
    T   = ep["qpos"].shape[0]
    hz  = ep["hz"]
    dur = T / hz

    lines = [
        f"## Episode: {path.name}",
        f"",
        f"**Steps:** {T}  ({dur:.1f} s @ {hz:.0f} Hz)",
        f"**Joints:** {', '.join(ep['joint_names'])}",
    ]

    if ep["ts_telem"] is not None:
        ts_t = ep["ts_telem"]
        ts_l = ep["ts_left"]
        ts_r = ep["ts_right"]

        # Drop NaN frames
        valid = ~(np.isnan(ts_t) | np.isnan(ts_l) | np.isnan(ts_r))
        n_valid = valid.sum()

        skew_lt = np.abs(ts_l[valid] - ts_t[valid]) * 1000
        skew_rt = np.abs(ts_r[valid] - ts_t[valid]) * 1000
        skew_lr = np.abs(ts_l[valid] - ts_r[valid]) * 1000
        worst   = np.maximum(np.maximum(skew_lt, skew_rt), skew_lr)

        lines += [
            f"",
            f"**Sync ({n_valid}/{T} frames with timestamps)**",
            f"- L-telem: mean {skew_lt.mean():.1f} ms, max {skew_lt.max():.1f} ms",
            f"- R-telem: mean {skew_rt.mean():.1f} ms, max {skew_rt.max():.1f} ms",
            f"- L-R:     mean {skew_lr.mean():.1f} ms, max {skew_lr.max():.1f} ms",
            f"- Worst overall: {worst.mean():.1f} ms mean, {worst.max():.1f} ms max",
        ]
    else:
        lines.append("")
        lines.append("*No /timestamps group — sync data not available.*")

    text = "\n".join(lines)
    print(text.replace("**", "").replace("*", "").replace("## ", "").replace("# ", ""))
    return text


def log_episode(ep: dict, show_images: bool, speed: float) -> None:
    T           = ep["qpos"].shape[0]
    hz          = ep["hz"]
    joint_names = ep["joint_names"]
    tick        = 1.0 / (hz * speed)
    has_sync    = ep["ts_telem"] is not None

    # Pre-style each joint series once
    for j, name in enumerate(joint_names):
        rr.log(f"episode/qpos/{name}",
               rr.SeriesLines(colors=_QPOS_COLOR,   names=name), static=True)
        rr.log(f"episode/actions/{name}",
               rr.SeriesLines(colors=_ACTION_COLOR, names=name), static=True)

    if has_sync:
        rr.log("episode/sync/L_telem_ms", rr.SeriesLines(colors=[255,  80,  80, 200], names="L-telem"), static=True)
        rr.log("episode/sync/R_telem_ms", rr.SeriesLines(colors=[ 80, 200,  80, 200], names="R-telem"), static=True)
        rr.log("episode/sync/L_R_ms",     rr.SeriesLines(colors=[200, 200,  80, 200], names="L-R"),     static=True)

    t0 = time.monotonic()

    for t in range(T):
        rr.set_time("frame", sequence=t)

        if has_sync:
            ts = ep["ts_telem"][t]
            if not np.isnan(ts):
                rr.set_time("wall_time", timestamp=float(ts))

        # Joint positions and actions
        for j, name in enumerate(joint_names):
            rr.log(f"episode/qpos/{name}",    rr.Scalars(float(ep["qpos"][t, j])))
            rr.log(f"episode/actions/{name}", rr.Scalars(float(ep["actions"][t, j])))

        # Sync skew
        if has_sync:
            ts_t = ep["ts_telem"][t]
            ts_l = ep["ts_left"][t]
            ts_r = ep["ts_right"][t]
            if not (np.isnan(ts_t) or np.isnan(ts_l) or np.isnan(ts_r)):
                rr.log("episode/sync/L_telem_ms", rr.Scalars(abs(ts_l - ts_t) * 1000))
                rr.log("episode/sync/R_telem_ms", rr.Scalars(abs(ts_r - ts_t) * 1000))
                rr.log("episode/sync/L_R_ms",     rr.Scalars(abs(ts_l - ts_r) * 1000))

        # Camera images
        if show_images:
            rr.log("cameras/left",  rr.Image(ep["left"][t],  color_model="RGB"))
            rr.log("cameras/right", rr.Image(ep["right"][t], color_model="RGB"))

        # Pace to real-time (or scaled)
        elapsed  = time.monotonic() - t0
        expected = t * tick
        remaining = expected - elapsed
        if remaining > 0:
            time.sleep(remaining)


def main():
    ap = argparse.ArgumentParser(description="View an AIZEE HDF5 episode in Rerun")
    ap.add_argument("episode", help="Path to episode_XXXX.hdf5")
    ap.add_argument("--speed",      type=float, default=1.0,  help="Playback speed multiplier (default 1.0)")
    ap.add_argument("--no-images",  action="store_true",       help="Skip camera images (faster)")
    ap.add_argument("--save",       default=None,              help="Save .rrd recording to this path instead of spawning viewer")
    args = ap.parse_args()

    path = Path(args.episode)
    if not path.exists():
        print(f"ERROR: {path} not found", file=sys.stderr)
        sys.exit(1)

    ep = load_episode(path)
    has_sync = ep["ts_telem"] is not None
    show_images = not args.no_images

    info_md = print_stats(path, ep)

    if args.save:
        rr.init("view_episode", recording_id=path.stem)
        rr.save(args.save)
    else:
        rr.init("view_episode", spawn=True)

    rr.send_blueprint(build_blueprint(show_images, has_sync))
    rr.log("episode/info", rr.TextDocument(info_md, media_type="text/markdown"), static=True)

    print(f"\nLogging {ep['qpos'].shape[0]} frames at {args.speed}x speed...")
    log_episode(ep, show_images, args.speed)
    print("Done.")


if __name__ == "__main__":
    main()
