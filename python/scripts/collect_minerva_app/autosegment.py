"""autosegment.py — propose action-segment boundaries from an episode's qpos.

Heuristic boundary detection so the operator only has to LABEL pre-cut phases
instead of also finding the cut points. Two signals:

  1. gripper open<->close transitions (each gripper channel binarized at its
     mid-range; a state flip is a boundary), and
  2. motion pause<->resume transitions (per-frame arm-joint speed crossing a
     threshold), which bracket reach / manipulate / retract phases.

Boundaries closer than `min_gap` frames are merged. `segments_from_boundaries`
turns the boundary list into contiguous [start, end) spans covering [0, T).
"""

from __future__ import annotations

from typing import List, Sequence

import numpy as np

from common.minerva_constants import ARM_INDICES, GRIPPER_INDICES


def auto_segment_boundaries(
    qpos,
    *,
    gripper_indices: Sequence[int] = GRIPPER_INDICES,
    arm_indices: Sequence[int] | None = None,
    min_gap: int = 8,
    pause_thresh: float = 0.015,
) -> List[int]:
    """Return sorted interior boundary frames (0 < b < T)."""
    q = np.asarray(qpos, dtype=float)
    if q.ndim != 2 or q.shape[0] < 3:
        return []
    T, J = q.shape
    ai = [i for i in (arm_indices if arm_indices is not None else ARM_INDICES) if i < J]
    bnds: set = set()

    # (1) gripper open<->close edges
    for g in gripper_indices:
        if g >= J:
            continue
        col = q[:, g]
        lo, hi = float(col.min()), float(col.max())
        if hi - lo < 1e-6:
            continue
        state = col > 0.5 * (lo + hi)
        for t in range(1, T):
            if state[t] != state[t - 1]:
                bnds.add(t)

    # (2) motion pause<->resume edges on the arm joints
    if ai:
        speed = np.zeros(T)
        speed[1:] = np.linalg.norm(np.diff(q[:, ai], axis=0), axis=1)
        moving = speed > pause_thresh
        for t in range(1, T):
            if moving[t] != moving[t - 1]:
                bnds.add(t)

    out: List[int] = []
    for b in sorted(b for b in bnds if 0 < b < T):
        if not out or b - out[-1] >= min_gap:
            out.append(b)
    return out


def segments_from_boundaries(boundaries: Sequence[int], T: int, label: str = "") -> list:
    """Contiguous [start, end) spans split at `boundaries`, covering [0, T)."""
    pts = sorted({0, *[int(b) for b in boundaries if 0 < b < T], int(T)})
    return [{"start": pts[i], "end": pts[i + 1], "label": label}
            for i in range(len(pts) - 1) if pts[i + 1] > pts[i]]


__all__ = ["auto_segment_boundaries", "segments_from_boundaries"]
