"""Adjacency-graph + n-joint collision sweep for the AIZEE URDF.

Computes effective joint limits — tighter than the raw URDF declared
limits — by sweeping each joint across its range while sampling its
kinematic ancestors at their previously-computed safe ranges.

Hierarchical sweep:
  swivel        →  no ancestors, sweep once
  gantry_base   →  sample swivel in [swivel.lo, swivel.hi]
  gantry_mid    →  sample (swivel, gantry_base) in their safe ranges
  ... and so on through wrist_swivel.

This is more efficient than naive Cartesian sampling because it avoids
wasting time on ancestor postures that are themselves already in collision.

A joint value is recorded as SAFE iff the world is collision-free for that
value across EVERY sampled ancestor posture.  The effective `(lower,
upper)` is the largest contiguous safe interval containing q=0 — i.e. the
range the operator can drive that joint to from home without the IK ever
producing a colliding pose, regardless of where the other joints are.

Writes `config/joint_limits.yaml`, which the IK overlay loads to clamp
solutions tighter than the raw URDF.

Usage:
    python -m ik.collision_sweep                                 # defaults
    python -m ik.collision_sweep --samples-ancestor 7 --samples-sweep 51
    python -m ik.collision_sweep --raw-meshes                    # no convex hull
    python -m ik.collision_sweep --joints swivel,gantry_base     # subset
"""

from __future__ import annotations

import argparse
import datetime as _dt
import itertools
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from .mesh_world import MeshWorld


# Default sweep order — matches AIZEE_ARM_IK_JOINTS in kinematics.py,
# root-first.  Wheels and other continuous joints are skipped (no
# meaningful "limit" to compute).
_DEFAULT_SWEEP_JOINTS = [
    "swivel",
    "gantry_base",
    "gantry_mid",
    "gantry_end",
    "wrist_pitch",
    "wrist_swivel",
]

_DEFAULT_URDF = Path(__file__).resolve().parents[2] / "urdf" / "aizee" / "aizee.urdf"
_DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "config" / "joint_limits.yaml"


def _samples_with_zero(lo: float, hi: float, n: int) -> list[float]:
    """n evenly-spaced samples in [lo, hi] with 0.0 forcibly included
    (if the range contains 0).  Always returns a sorted, deduplicated list."""
    if n < 2:
        return [0.0] if lo <= 0.0 <= hi else [(lo + hi) / 2.0]
    pts = list(np.linspace(lo, hi, n))
    if lo <= 0.0 <= hi:
        pts.append(0.0)
    pts = sorted(set(round(float(p), 6) for p in pts))
    return pts


def _largest_run_containing(values: list[float], mask: list[bool], anchor: float = 0.0) -> tuple[float, float]:
    """Given sorted `values` + parallel `mask` (True=safe), return (lo, hi)
    bracketing the longest contiguous run of True values that contains the
    sample nearest `anchor`.  If the anchor sample itself is False, returns
    (anchor, anchor) — degenerate, signaling no safe interval at home."""
    n = len(values)
    if n == 0:
        return (anchor, anchor)
    # Find sample closest to anchor.
    diffs = [abs(v - anchor) for v in values]
    i0 = diffs.index(min(diffs))
    if not mask[i0]:
        return (anchor, anchor)
    # Walk left from i0 while safe.
    lo_i = i0
    while lo_i > 0 and mask[lo_i - 1]:
        lo_i -= 1
    hi_i = i0
    while hi_i < n - 1 and mask[hi_i + 1]:
        hi_i += 1
    return (values[lo_i], values[hi_i])


def sweep_one_joint(
    world: MeshWorld,
    joint: str,
    *,
    effective_limits_so_far: dict[str, tuple[float, float]],
    n_ancestor: int,
    n_sweep: int,
    verbose: bool = True,
) -> dict:
    """Run the hierarchical sweep for `joint` and return a result dict."""
    urdf_lo, urdf_hi = world.joint_limit(joint)
    ancestors = world.ancestors_of(joint)
    # Filter out ancestors we don't have a safe range for (e.g. continuous
    # wheels at the rover base — they're parents of the swivel link's
    # parent but don't affect collision context).  Sample those at 0 only.
    ancestor_samples: dict[str, list[float]] = {}
    for A in ancestors:
        if A in effective_limits_so_far:
            lo, hi = effective_limits_so_far[A]
            ancestor_samples[A] = _samples_with_zero(lo, hi, n_ancestor)
        else:
            ancestor_samples[A] = [0.0]
    posture_keys = list(ancestor_samples.keys())
    posture_values = [ancestor_samples[k] for k in posture_keys]
    posture_grid = list(itertools.product(*posture_values)) if posture_values else [()]

    j_samples = _samples_with_zero(urdf_lo, urdf_hi, n_sweep)
    safe_count = [0] * len(j_samples)
    valid_postures = 0
    failure_pairs: dict[frozenset[str], int] = {}  # pair -> count of failures

    t0 = time.perf_counter()
    for posture in posture_grid:
        qd = dict(zip(posture_keys, posture))
        # Verify the ancestor posture itself is valid (collision-free with
        # the swept joint at 0).  Otherwise this posture can't constrain
        # the safe range and we'd just produce false negatives.
        qd[joint] = 0.0
        world.set_qpos(qd)
        if world.in_collision():
            continue
        valid_postures += 1
        for i, jv in enumerate(j_samples):
            qd[joint] = float(jv)
            world.set_qpos(qd)
            pairs = world.colliding_pairs()
            if not pairs:
                safe_count[i] += 1
            else:
                for p in pairs:
                    failure_pairs[p] = failure_pairs.get(p, 0) + 1
    elapsed = time.perf_counter() - t0

    if valid_postures == 0:
        # No reachable ancestor posture is collision-free at joint=0.
        # The URDF likely has a structural issue or our home-pose
        # auto-allowlist missed something.  Fall back to URDF limits and
        # flag in the output.
        eff_lo, eff_hi = urdf_lo, urdf_hi
        safe_at_zero_pct = 0.0
        if verbose:
            print(f"  [warn] {joint}: 0 valid ancestor postures — using URDF limits as fallback")
    else:
        safe_mask = [c == valid_postures for c in safe_count]
        eff_lo, eff_hi = _largest_run_containing(j_samples, safe_mask, anchor=0.0)
        safe_at_zero_pct = 100.0 * sum(safe_mask) / len(safe_mask)

    urdf_range = urdf_hi - urdf_lo
    eff_range = eff_hi - eff_lo
    reduction_pct = 100.0 * (1.0 - eff_range / urdf_range) if urdf_range > 0 else 0.0

    top_offenders = sorted(failure_pairs.items(), key=lambda kv: -kv[1])[:5]
    top_offenders_str = [
        {"pair": sorted(list(p)), "fail_count": c} for p, c in top_offenders
    ]

    return {
        "joint": joint,
        "urdf_lower": float(urdf_lo),
        "urdf_upper": float(urdf_hi),
        "effective_lower": float(eff_lo),
        "effective_upper": float(eff_hi),
        "reduction_pct": float(reduction_pct),
        "ancestors": ancestors,
        "posture_grid_size": len(posture_grid),
        "valid_postures": valid_postures,
        "sweep_samples": len(j_samples),
        "safe_sample_pct": float(safe_at_zero_pct),
        "top_colliding_pairs": top_offenders_str,
        "elapsed_s": float(elapsed),
    }


def run_full_sweep(
    urdf_path: Path,
    *,
    joints: Optional[list[str]] = None,
    n_ancestor: int = 5,
    n_sweep: int = 31,
    use_convex: bool = True,
    verbose: bool = True,
) -> dict:
    if verbose:
        print(f"[sweep] loading URDF + meshes ({'convex hulls' if use_convex else 'raw meshes'})")
    world = MeshWorld(urdf_path, use_convex=use_convex)
    if verbose:
        print(f"[sweep] {len(world.link_names)} links meshed, "
              f"{len(world._home_spurious_pairs)} home-spurious pairs auto-allowed")
        for p in world._home_spurious_pairs:
            print(f"          {sorted(list(p))}")

    joint_order = list(joints) if joints else _DEFAULT_SWEEP_JOINTS
    # Drop any joint that's continuous (wheels) — meaningless to sweep.
    joint_order = [j for j in joint_order if world.joint_type(j) != "continuous"]

    if verbose:
        print(f"[sweep] joints (root-first): {joint_order}")
        print(f"[sweep] samples: ancestor={n_ancestor}, sweep={n_sweep}")

    effective_limits: dict[str, tuple[float, float]] = {}
    per_joint_reports: list[dict] = []

    for j in joint_order:
        if verbose:
            ancs = world.ancestors_of(j)
            print(f"[sweep] {j}  (ancestors: {ancs})")
        report = sweep_one_joint(
            world, j,
            effective_limits_so_far=effective_limits,
            n_ancestor=n_ancestor,
            n_sweep=n_sweep,
            verbose=verbose,
        )
        effective_limits[j] = (report["effective_lower"], report["effective_upper"])
        per_joint_reports.append(report)
        if verbose:
            print(f"  -> [{report['effective_lower']:+.3f}, {report['effective_upper']:+.3f}]  "
                  f"(URDF: [{report['urdf_lower']:+.3f}, {report['urdf_upper']:+.3f}])  "
                  f"reduction {report['reduction_pct']:.1f}%  "
                  f"in {report['elapsed_s']:.1f}s  "
                  f"({report['valid_postures']}/{report['posture_grid_size']} postures valid)")
            if report["top_colliding_pairs"]:
                for entry in report["top_colliding_pairs"][:3]:
                    print(f"      colliding: {entry['pair']}  ({entry['fail_count']} fails)")

    return {
        "generated_at":           _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "urdf":                   str(urdf_path),
        "use_convex":             use_convex,
        "samples_per_ancestor":   n_ancestor,
        "samples_per_sweep":      n_sweep,
        "auto_allowed_home_pairs": [sorted(list(p)) for p in world._home_spurious_pairs],
        "joints":                 {r["joint"]: r for r in per_joint_reports},
    }


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def _cli() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--urdf", type=Path, default=_DEFAULT_URDF,
                    help=f"URDF path (default: {_DEFAULT_URDF})")
    ap.add_argument("--output", type=Path, default=_DEFAULT_OUTPUT,
                    help=f"YAML output path (default: {_DEFAULT_OUTPUT})")
    ap.add_argument("--joints", type=str, default=None,
                    help="Comma-separated joint subset (default: full arm chain)")
    ap.add_argument("--samples-ancestor", type=int, default=5)
    ap.add_argument("--samples-sweep",    type=int, default=31)
    ap.add_argument("--raw-meshes", action="store_true",
                    help="Use raw STL geometry instead of convex hulls "
                         "(slower, more accurate)")
    ap.add_argument("--no-write", action="store_true",
                    help="Print results but don't write the YAML")
    args = ap.parse_args()

    joints = [s.strip() for s in args.joints.split(",")] if args.joints else None
    t0 = time.perf_counter()
    result = run_full_sweep(
        args.urdf,
        joints=joints,
        n_ancestor=args.samples_ancestor,
        n_sweep=args.samples_sweep,
        use_convex=not args.raw_meshes,
        verbose=True,
    )
    print(f"\n[sweep] total {time.perf_counter() - t0:.1f}s")

    if not args.no_write:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(yaml.safe_dump(result, sort_keys=False, default_flow_style=False))
        print(f"[sweep] wrote {args.output}")

    return 0


if __name__ == "__main__":
    sys.exit(_cli())
