"""
test_minerva_segments.py — v7 temporal action segments (per-phase labels).

Covers: RecordingSession.set_label/finalize_segments -> save_minerva_episode
(v7 `segments` attr) -> MinervaEpisodeDataset per-frame label resolution.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

_AIZEE = Path(__file__).resolve().parents[3]
_PY = _AIZEE / "python"
sys.path.insert(0, str(_AIZEE))
sys.path.insert(0, str(_PY))
sys.path.insert(0, str(_PY / "scripts"))

import h5py

from collect_demo_app.minerva_recording import save_minerva_episode
from collect_minerva_app.recording import RecordingSession
from python.training.language import TextConditioner
from python.training.minerva_dataset import MinervaEpisodeDataset, parse_segments

CAMERAS = ["left_wrist", "right_wrist", "head"]


def _frame():
    return (np.random.default_rng(0).random((16, 16, 3)) * 255).astype(np.uint8)


def _append(s, n):
    for _ in range(n):
        s.append(np.zeros(17, np.float32), np.zeros(17, np.float32), None,
                 {c: _frame() for c in CAMERAS}, 0.0, {c: 0.0 for c in CAMERAS})


def test_session_labeling():
    s = RecordingSession(CAMERAS)
    s.set_label("reach for the block")   # opens at frame 0
    _append(s, 3)
    s.set_label("grasp")                 # closes reach@3, opens grasp@3
    _append(s, 2)
    s.finalize_segments()                # closes grasp@5
    assert s.segments == [
        {"start": 0, "end": 3, "label": "reach for the block"},
        {"start": 3, "end": 5, "label": "grasp"},
    ], s.segments
    print("  OK: RecordingSession segment tracking (reach 0-3, grasp 3-5)")
    return s


def test_save_and_dataset(s):
    tmp = Path(tempfile.mkdtemp(prefix="minerva_seg_"))
    path, n = save_minerva_episode(
        tmp, s.qpos, s.cam, qcmd_buf=s.qcmd, telem_ts_buf=s.telem_ts,
        camera_ts_bufs=s.cam_ts, language_instruction="stack the blocks",
        task_id=1, segments=s.segments)
    assert n == 5
    with h5py.File(path, "r") as f:
        assert int(f.attrs["format_version"]) == 7, f.attrs["format_version"]
        assert len(parse_segments(f.attrs.get("segments"))) == 2

    tc = TextConditioner(model_name="hash")
    tc.build_cache(["stack the blocks", "reach for the block", "grasp"])
    ds = MinervaEpisodeDataset([path], chunk_size=2, future_offset=0, conditioner=tc)

    labels = [ds._label_at(0, t) for t in range(5)]
    assert labels == ["reach for the block", "reach for the block",
                      "reach for the block", "grasp", "grasp"], labels
    # a sample's language embedding follows its frame's segment label
    obs, _ = ds[3]   # frame 3 -> "grasp"
    assert "language" in obs and obs["language"].shape[0] == tc.embed_dim
    print("  OK: v7 save + dataset per-frame labels + language embedding")


def run():
    s = test_session_labeling()
    test_save_and_dataset(s)
    print("SEGMENTS TEST PASS")


def test_minerva_segments():
    run()


if __name__ == "__main__":
    run()
