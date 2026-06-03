"""Compute the reachable EE workspace box from the (collision-aware) IK limits.

Used to size the operator-facing workspace clamp in `config/quest_teleop.yaml`
so the in-VR workspace wireframe matches what the IK can actually solve to.
A target outside this box is unreachable no matter the clutch motion — the
IK will saturate at the boundary and the operator just won't see the arm
follow.

Two ways the box differs from the IK's joint-space limits:
  * The mapping from joint-space to Cartesian is nonlinear, so a box-shaped
    joint region maps to an irregular Cartesian volume.  We take the
    axis-aligned bounding box of that volume.
  * Some EE positions ARE reachable but only at singular / awkward
    postures; we percentile-trim to avoid letting those define the box.

Run with:
    python -m ik.workspace                          # print + show
    python -m ik.workspace --update-config          # patch quest_teleop.yaml
    python -m ik.workspace --samples 50000          # tighter Monte Carlo
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from . import load_aizee_arm
from .kinematics import Kinematics


_DEFAULT_LIMITS_YAML = Path(__file__).resolve().parents[2] / "config" / "joint_limits.yaml"
_DEFAULT_QUEST_YAML  = Path(__file__).resolve().parents[2] / "config" / "quest_teleop.yaml"


def compute_reachable_box(
    kin: Kinematics,
    *,
    n_samples: int = 20000,
    percentile: float = 2.0,
    margin: float = 0.02,
    seed: int = 0,
) -> dict:
    """Monte-Carlo sample joint-space within (post-overlay) limits, compute
    FK, and take the percentile-trimmed AABB of the resulting EE positions.

    `percentile=2.0` clips the worst 2% on each side — drops singular /
    awkward configurations that distort the box.

    `margin=0.02` (2 cm) is shaved off each side so the IK has slack to
    actually solve at the box boundary (otherwise IK targets right on the
    boundary need exact joint extrema, which the DLS solver chases poorly).
    """
    rng = np.random.default_rng(seed)
    lo = kin.lower.astype(np.float64)
    hi = kin.upper.astype(np.float64)
    n = kin.n
    # Uniform within limits; this is a coarse-but-honest sample because
    # joint distribution at runtime is operator-driven, not physically
    # weighted.  N=20k FKs runs in ~0.3 s.
    Q = rng.uniform(lo, hi, size=(n_samples, n))
    pts = np.empty((n_samples, 3), dtype=np.float64)
    for i in range(n_samples):
        pts[i] = kin.fk_pose(Q[i])[0]
    p_lo = np.percentile(pts, percentile,       axis=0) + margin
    p_hi = np.percentile(pts, 100 - percentile, axis=0) - margin
    # Guarantee lower < upper post-margin even on degenerate axes.
    p_lo = np.minimum(p_lo, p_hi - 1e-3)
    return {
        "lower":          p_lo.tolist(),
        "upper":          p_hi.tolist(),
        "raw_min":        pts.min(axis=0).tolist(),
        "raw_max":        pts.max(axis=0).tolist(),
        "samples":        n_samples,
        "percentile_clip": percentile,
        "margin_m":       margin,
    }


def _update_quest_yaml(path: Path, lower: list[float], upper: list[float]) -> None:
    """Patch reachable_min / reachable_max in quest_teleop.yaml.  These are
    the IK's HARD reach limits, drawn as an informational thin wireframe in
    VR — distinct from workspace_min/max which is the operator clutch
    safety clamp (stays as you set it)."""
    import yaml as _yaml
    data: dict = {}
    if path.exists():
        data = _yaml.safe_load(path.read_text()) or {}
    data["reachable_min"] = [round(float(v), 4) for v in lower]
    data["reachable_max"] = [round(float(v), 4) for v in upper]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_yaml.safe_dump(data, sort_keys=False, default_flow_style=None))


def _cli() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--limits-yaml", type=Path, default=_DEFAULT_LIMITS_YAML,
                    help="joint_limits.yaml to overlay (default: config/joint_limits.yaml)")
    ap.add_argument("--samples", type=int, default=20000,
                    help="Number of FK samples (default 20000)")
    ap.add_argument("--percentile", type=float, default=2.0,
                    help="Percentile clip per side (default 2 = drop worst 2%% outliers)")
    ap.add_argument("--margin", type=float, default=0.02,
                    help="Inset per side [m] so IK can settle (default 0.02 = 2 cm)")
    ap.add_argument("--update-config", action="store_true",
                    help="Patch config/quest_teleop.yaml with the new workspace_min/max")
    ap.add_argument("--quest-yaml", type=Path, default=_DEFAULT_QUEST_YAML)
    args = ap.parse_args()

    kin = load_aizee_arm()
    if args.limits_yaml.exists():
        kin.apply_limits_overlay(args.limits_yaml)
        print(f"[workspace] applied limits from {args.limits_yaml}")
    else:
        print(f"[workspace] no limits YAML — using raw URDF bounds")
    print(f"[workspace] joint range:")
    for i, n in enumerate(kin.joint_names):
        print(f"    {n:14s}  [{kin.lower[i]:+.3f}, {kin.upper[i]:+.3f}]")

    t0 = time.perf_counter()
    box = compute_reachable_box(
        kin, n_samples=args.samples, percentile=args.percentile, margin=args.margin,
    )
    dt = time.perf_counter() - t0
    lo, hi = box["lower"], box["upper"]
    raw_lo, raw_hi = box["raw_min"], box["raw_max"]
    print(f"[workspace] {args.samples} FK samples in {dt*1000:.0f} ms")
    print(f"[workspace] raw reachable AABB:")
    print(f"    x: [{raw_lo[0]:+.3f}, {raw_hi[0]:+.3f}]   ({(raw_hi[0]-raw_lo[0])*100:.1f} cm)")
    print(f"    y: [{raw_lo[1]:+.3f}, {raw_hi[1]:+.3f}]   ({(raw_hi[1]-raw_lo[1])*100:.1f} cm)")
    print(f"    z: [{raw_lo[2]:+.3f}, {raw_hi[2]:+.3f}]   ({(raw_hi[2]-raw_lo[2])*100:.1f} cm)")
    print(f"[workspace] trimmed (p{args.percentile} clip, {args.margin*100:.0f} cm inset):")
    print(f"    workspace_min: [{lo[0]:+.4f}, {lo[1]:+.4f}, {lo[2]:+.4f}]")
    print(f"    workspace_max: [{hi[0]:+.4f}, {hi[1]:+.4f}, {hi[2]:+.4f}]")
    print(f"    size:  {(hi[0]-lo[0])*100:.1f}  x  {(hi[1]-lo[1])*100:.1f}  x  {(hi[2]-lo[2])*100:.1f}  cm")

    if args.update_config:
        _update_quest_yaml(args.quest_yaml, lo, hi)
        print(f"[workspace] wrote reachable_min/max -> {args.quest_yaml}")
        print(f"[workspace] (workspace_min/max — the OPERATOR clutch clamp — was left untouched)")
    else:
        print(f"[workspace] (re-run with --update-config to patch {args.quest_yaml})")
    return 0


if __name__ == "__main__":
    sys.exit(_cli())
