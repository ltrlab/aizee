"""
test_minerva_autosegment.py — heuristic action-segment boundary proposal.

Builds a synthetic episode with a motion → pause → motion profile plus a gripper
close, and checks auto_segment_boundaries finds the transitions and that
segments_from_boundaries yields contiguous spans covering [0, T).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

_PY = Path(__file__).resolve().parents[3] / "python"
sys.path.insert(0, str(_PY))
sys.path.insert(0, str(_PY / "scripts"))

from collect_minerva_app.autosegment import auto_segment_boundaries, segments_from_boundaries


def run():
    T = 40
    q = np.zeros((T, 17), dtype=np.float32)
    q[:15, 0] = np.linspace(0, 1, 15)   # arm moves
    q[15:25, 0] = 1.0                   # pause
    q[25:, 0] = np.linspace(1, 2, 15)   # arm moves again
    q[:20, 6] = 0.0                     # left gripper open ...
    q[20:, 6] = 1.0                     # ... then closed at frame 20

    bnds = auto_segment_boundaries(q, min_gap=3, pause_thresh=0.02)
    assert len(bnds) >= 2, bnds
    assert any(abs(b - 20) <= 2 for b in bnds), f"gripper edge near 20 not found: {bnds}"
    assert any(abs(b - 15) <= 2 for b in bnds), f"motion pause near 15 not found: {bnds}"
    print(f"  OK: boundaries {bnds}")

    segs = segments_from_boundaries(bnds, T, label="")
    assert segs[0]["start"] == 0 and segs[-1]["end"] == T
    for a, b in zip(segs, segs[1:]):
        assert a["end"] == b["start"], (a, b)   # contiguous, no gaps/overlaps
    assert all(s["end"] > s["start"] for s in segs)
    print(f"  OK: {len(segs)} contiguous spans covering [0,{T})")

    assert auto_segment_boundaries(np.zeros((5, 17))) == []   # no motion -> no cuts
    print("  OK: flat episode -> no boundaries")
    print("AUTOSEGMENT TEST PASS")


def test_minerva_autosegment():
    run()


if __name__ == "__main__":
    run()
