"""Tests for python/scripts/record_replay.py."""

from __future__ import annotations

import sys
from pathlib import Path

import h5py
import numpy as np
import pytest

# Make the scripts directory importable
SCRIPTS_DIR = Path(__file__).parent.parent.parent / "python" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import record_replay as rr_mod

J = rr_mod.NUM_ARM_JOINTS  # 7, swivel-first


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_recording(path, T: int = 20, *, start_qpos: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Write a synthetic recording to *path* and return (qpos, vels, ts)."""
    if start_qpos is None:
        start_qpos = np.zeros(J, dtype=np.float32)
    qpos       = np.tile(start_qpos, (T, 1)).astype(np.float32)
    velocities = np.zeros((T, J), dtype=np.float32)
    timestamps = np.linspace(0.0, (T - 1) / rr_mod.RECORD_HZ, T)
    rr_mod.save_recording(path, qpos, velocities, timestamps)
    return qpos, velocities, timestamps


# ---------------------------------------------------------------------------
# 1. HDF5 roundtrip
# ---------------------------------------------------------------------------

def test_hdf5_roundtrip(tmp_path):
    """Write a recording, read it back, verify arrays match exactly."""
    p = tmp_path / "recording_0000.hdf5"
    T = 30
    rng = np.random.default_rng(42)
    qpos_orig = rng.uniform(-1.0, 1.0, (T, J)).astype(np.float32)
    vels_orig = rng.uniform(-0.5, 0.5, (T, J)).astype(np.float32)
    ts_orig   = np.linspace(0.0, (T - 1) / rr_mod.RECORD_HZ, T)

    rr_mod.save_recording(p, qpos_orig, vels_orig, ts_orig)

    qpos_r, vels_r, ts_r = rr_mod.load_recording(p)

    np.testing.assert_array_almost_equal(qpos_r, qpos_orig, decimal=5)
    np.testing.assert_array_almost_equal(vels_r, vels_orig, decimal=5)
    np.testing.assert_array_almost_equal(ts_r,   ts_orig,   decimal=8)

    # Verify HDF5 attributes were stored
    with h5py.File(p, "r") as f:
        assert f.attrs["hz"] == rr_mod.RECORD_HZ
        assert "recorded_at" in f.attrs


# ---------------------------------------------------------------------------
# 2. Episode file loading (collect_demo.py format)
# ---------------------------------------------------------------------------

def test_episode_file_loading(tmp_path):
    """Verify the script correctly reads /observations/qpos from episode_XXXX.hdf5."""
    episode_path = tmp_path / "episode_0000.hdf5"
    T = 15
    rng = np.random.default_rng(7)
    qpos_orig = rng.uniform(-0.5, 0.5, (T, J)).astype(np.float32)

    with h5py.File(episode_path, "w") as f:
        grp = f.create_group("observations")
        grp.create_dataset("qpos", data=qpos_orig)
        # Also store action, images, etc. as collect_demo.py would — but only qpos matters
        grp.create_dataset("qvel", data=np.zeros((T, J), dtype=np.float32))

    qpos_r, vels_r, ts_r = rr_mod.load_recording(episode_path)

    assert qpos_r.shape == (T, J)
    np.testing.assert_array_almost_equal(qpos_r, qpos_orig, decimal=5)

    # Velocities should be zero-padded when not present in episode file
    assert vels_r.shape == (T, J)
    np.testing.assert_array_equal(vels_r, 0.0)

    # Timestamps should be synthesized at RECORD_HZ
    assert ts_r.shape == (T,)
    assert ts_r[0] == pytest.approx(0.0)
    assert ts_r[-1] == pytest.approx((T - 1) / rr_mod.RECORD_HZ, rel=1e-5)


# ---------------------------------------------------------------------------
# 3. Pre-flight delta scanner
# ---------------------------------------------------------------------------

def test_preflight_delta_scanner(tmp_path):
    """Craft a recording with one large jump; verify warning is raised."""
    T = 10
    qpos = np.zeros((T, J), dtype=np.float32)

    # Insert a 0.5 rad jump on joint 2 at frame 5 (threshold 2 * 0.05 = 0.1)
    qpos[5, 2] = 0.5

    max_delta = 0.05
    warnings = rr_mod.check_recording_continuity(qpos, max_delta)

    assert len(warnings) >= 1, "Expected at least one continuity warning"

    # The warning should point to frame 5, joint 2
    frames    = [w[0] for w in warnings]
    joints    = [w[1] for w in warnings]
    assert 5 in frames
    assert 2 in joints

    # All reported deltas should exceed the threshold
    for _, _, delta in warnings:
        assert delta > 2.0 * max_delta


def test_preflight_delta_scanner_clean_recording():
    """Smooth trajectory should produce no warnings."""
    T = 50
    t  = np.linspace(0, 2 * np.pi, T)
    qpos = np.stack([np.sin(t) * 0.01] * J, axis=1).astype(np.float32)  # tiny smooth motion
    warnings = rr_mod.check_recording_continuity(qpos, max_delta=0.05)
    assert warnings == [], f"Unexpected warnings on clean recording: {warnings}"


# ---------------------------------------------------------------------------
# 4. Start-position mismatch check
# ---------------------------------------------------------------------------

def test_start_position_mismatch():
    """Given recording starting at [0.5, ...] and current [0.0, ...], mismatch is detected."""
    recording_start = np.full(J, 0.5, dtype=np.float32)
    current_qpos    = np.zeros(J,    dtype=np.float32)

    mismatches = rr_mod.check_start_position(recording_start, current_qpos, tol=0.1)

    assert len(mismatches) == J, "All joints should mismatch"
    for joint_i, diff in mismatches:
        assert diff == pytest.approx(0.5, rel=1e-5)


def test_start_position_no_mismatch():
    """Identical positions should produce no mismatch."""
    q = np.array([0.1, -0.2, 0.3, 0.0, -0.1, 0.05, 0.2], dtype=np.float32)
    mismatches = rr_mod.check_start_position(q, q.copy(), tol=0.1)
    assert mismatches == []


def test_start_position_partial_mismatch():
    """Only joints outside tolerance should be flagged."""
    recording_start = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    current_qpos    = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    mismatches = rr_mod.check_start_position(recording_start, current_qpos, tol=0.1)

    # Only joint 1 (gantry_base — swivel-first order) should mismatch
    assert len(mismatches) == 1
    assert mismatches[0][0] == 1
    assert mismatches[0][1] == pytest.approx(0.5, rel=1e-5)


# ---------------------------------------------------------------------------
# 5. _log_arm_fk smoke test
# ---------------------------------------------------------------------------

def test_log_arm_fk_smoke():
    """Call _log_arm_fk with known qpos; verify no exception (Rerun memory mode)."""
    import rerun as rerun_sdk

    # Use in-memory recording to avoid spawning a viewer or writing files
    rerun_sdk.init("test_fk_smoke", spawn=False)

    qpos = np.array([0.0, 0.1, 0.2, -0.1, 0.05, 0.0, 0.3], dtype=np.float32)

    # Should not raise
    rr_mod._log_arm_fk(qpos)

    # Also verify static geometry logs without exception
    rr_mod._log_static_arm()


def test_log_arm_fk_zero():
    """FK at zero position should not raise."""
    import rerun as rerun_sdk

    rerun_sdk.init("test_fk_zero", spawn=False)
    rr_mod._log_arm_fk(np.zeros(J, dtype=np.float32))


# ---------------------------------------------------------------------------
# Extra: next_recording_path numbering
# ---------------------------------------------------------------------------

def test_next_recording_path_empty_dir(tmp_path):
    """First recording in an empty dir should be recording_0000.hdf5."""
    path = rr_mod._next_recording_path(tmp_path)
    assert path.name == "recording_0000.hdf5"
    assert path.parent == tmp_path


def test_next_recording_path_increments(tmp_path):
    """Should increment past existing recordings."""
    # Create fake existing recordings
    (tmp_path / "recording_0000.hdf5").touch()
    (tmp_path / "recording_0001.hdf5").touch()
    path = rr_mod._next_recording_path(tmp_path)
    assert path.name == "recording_0002.hdf5"


# ---------------------------------------------------------------------------
# Extra: legacy 6-DoF recording loads as 7-DoF (swivel shim)
# ---------------------------------------------------------------------------

def test_legacy_6dof_recording_gains_swivel(tmp_path):
    """Pre-7-DoF files stored 6 gantry columns; loader must prepend swivel=0."""
    p = tmp_path / "recording_0000.hdf5"
    T = 12
    qpos6 = np.random.default_rng(3).uniform(-0.5, 0.5, (T, 6)).astype(np.float32)
    with h5py.File(p, "w") as f:
        f.create_dataset("qpos", data=qpos6)
        f.create_dataset("velocities", data=np.zeros((T, 6), dtype=np.float32))
        f.create_dataset("timestamps", data=np.linspace(0, (T - 1) / rr_mod.RECORD_HZ, T))
        f.attrs["hz"] = rr_mod.RECORD_HZ

    qpos_r, vels_r, _ = rr_mod.load_recording(p)
    assert qpos_r.shape == (T, J)
    np.testing.assert_array_equal(qpos_r[:, 0], 0.0)          # shimmed swivel
    np.testing.assert_array_almost_equal(qpos_r[:, 1:], qpos6, decimal=5)
