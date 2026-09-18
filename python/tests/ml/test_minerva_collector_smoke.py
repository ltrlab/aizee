"""
test_minerva_collector_smoke.py — headless end-to-end test for the Minerva
data-collection app (collect_minerva.py + collect_minerva_app + GUI).

Uses real in-process ZMQ (inproc PUB/SUB) with fake camera publishers, exercises camera receiver/decoder threads,
jog-only teleop, the recording→save round-trip, and constructs the PySide6 GUI offscreen
to verify snapshot application + raw-JPEG painting.

Run:    python python/tests/ml/test_minerva_collector_smoke.py
Pytest: pytest python/tests/ml/test_minerva_collector_smoke.py
"""

from __future__ import annotations

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # must precede PySide6 import

import queue
import sys
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np

for _s in (sys.stdout, sys.stderr):   # Windows console defaults to cp1252
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

_AIZEE = Path(__file__).resolve().parents[3]
_PY = _AIZEE / "python"
sys.path.insert(0, str(_PY))                 # common.*
sys.path.insert(0, str(_PY / "scripts"))     # collect_minerva_app.*, collect_demo_app.*

import cv2
import zmq

from common.minerva_constants import CAMERAS, JOINT_LIMITS, NUM_MINERVA_JOINTS
from common.wire import pack_camera
from collect_minerva_app.images import start_image_decoder
from collect_minerva_app.receivers import start_cam_receiver
from collect_minerva_app.recording import RecordingSession, start_async_save
from collect_minerva_app.teleop import MinervaTeleop

_CAM_WH = (80, 60)   # (w, h) published; decoder resizes to _SIZE
_SIZE = {c: (64, 48) for c in CAMERAS}


def _jpeg(w=_CAM_WH[0], h=_CAM_WH[1]) -> bytes:
    bgr = (np.random.default_rng(0).random((h, w, 3)) * 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", bgr)
    assert ok
    return buf.tobytes()


def _test_receivers_and_decoder(ctx):
    print("running: receivers+decoder", flush=True)
    cam_pubs = {c: ctx.socket(zmq.PUB) for c in CAMERAS}
    for c, p in cam_pubs.items():
        p.bind(f"inproc://{c}")

    cam_stop, _, cam_lock, cam_cache = start_cam_receiver(
        ctx, {c: f"inproc://{c}" for c in CAMERAS})
    dec_stop, _, dec_lock, dec_cache = start_image_decoder(
        cam_lock, cam_cache, CAMERAS, _SIZE, always_on=True, hz=60)

    time.sleep(0.3)  # SUB slow-joiner
    for _ in range(25):
        for c, p in cam_pubs.items():
            hdr = {"color": {"format": "jpeg", "width": _CAM_WH[0], "height": _CAM_WH[1]},
                   "timestamp": time.time()}
            p.send_multipart(pack_camera(hdr, color_bytes=_jpeg()))
        time.sleep(0.02)
    time.sleep(0.25)

    with cam_lock:
        assert all(cam_cache[c] is not None for c in CAMERAS), "camera(s) not received"
    with dec_lock:
        for c in CAMERAS:
            img = dec_cache[c]
            assert img is not None and img.shape == (_SIZE[c][1], _SIZE[c][0], 3), (c, None if img is None else img.shape)

    for st in (dec_stop, cam_stop):
        st.set()
    time.sleep(0.1)  # let receiver threads close their SUB sockets
    for p in cam_pubs.values():
        p.close(linger=0)
    print("  ✓ receivers + decoder (3 cams decoded/resized)")


def _test_teleop():
    tel = MinervaTeleop(hardware_enabled=False)
    tel.connect(verbose=False)   # explicit hardware isolation
    assert tel.status == {"left": False, "right": False}
    q0 = np.zeros(NUM_MINERVA_JOINTS, dtype=np.float32)
    assert tel.target(q0) is None, "target before engage must be None"
    tel.engage(q0)
    assert tel.engaged
    tel.jog(0, 1.0); tel.jog(0, 1.0); tel.jog(16, 1.0)
    tgt = tel.target(q0)
    assert tgt is not None and tgt.shape == (NUM_MINERVA_JOINTS,) and np.isfinite(tgt).all()
    assert (tgt >= JOINT_LIMITS[:, 0] - 1e-4).all() and (tgt <= JOINT_LIMITS[:, 1] + 1e-4).all()
    assert tgt[0] > 0, "jog on joint 0 should have moved the target"
    assert tel.take_record_edges() == 0
    print("  ✓ teleop jog-only (engage/target/jog, clamped to limits)")
    return tel


def _test_recording_save():
    print("running: recording+save", flush=True)
    sess = RecordingSession(CAMERAS)
    for _ in range(6):
        frames = {c: (np.random.default_rng(1).random((_SIZE[c][1], _SIZE[c][0], 3)) * 255).astype(np.uint8)
                  for c in CAMERAS}
        sess.append(np.zeros(NUM_MINERVA_JOINTS, dtype=np.float32),
                    np.zeros(NUM_MINERVA_JOINTS, dtype=np.float32),
                    np.zeros(NUM_MINERVA_JOINTS, dtype=np.float32),
                    frames, time.time(), {c: time.time() for c in CAMERAS})
    assert sess.steps == 6
    tmp = tempfile.mkdtemp(prefix="minerva_collect_")
    res, rlock = {}, threading.Lock()
    t = start_async_save(sess, tmp, language_instruction="pick up the block",
                         task_id=2, notes="smoke", result_holder=res, result_lock=rlock)
    t.join(10)
    with rlock:
        assert "error" not in res, res.get("error")
        assert "path" in res and Path(res["path"]).exists(), res
    # verify schema round-trips through the dataset reader
    sys.path.insert(0, str(_AIZEE))  # aizee/ on path for the python.* package
    from python.training.minerva_dataset import MinervaEpisodeDataset  # noqa: E402
    ds = MinervaEpisodeDataset([Path(res["path"])], chunk_size=4, future_offset=0)
    assert ds.num_joints == 17 and set(ds.cameras) == set(CAMERAS)
    print(f"  ✓ recording→save→v6 read-back ({res['steps']} steps, cams={ds.cameras})")
    return tmp


def _test_gui(tel, tmp):
    print("running: gui (offscreen)", flush=True)
    from PySide6.QtWidgets import QApplication
    from collect_minerva_gui import MinervaMainWindow, _ACTION_VOCAB
    app = QApplication.instance() or QApplication([])
    cq: "queue.Queue[str]" = queue.Queue()
    meta = {"language_instruction": "", "notes": "", "task_id": None}
    win = MinervaMainWindow(cq, meta, tel, CAMERAS, tmp)
    snap = {
        "state": "TELEOP", "recording": True, "rec_steps": 6, "dropped": 1,
        "qpos": [0.1] * 17, "target": [0.2] * 17, "torque": [0.0] * 17,
        "telem_age": 0.01, "cam_ages": {c: 0.01 for c in CAMERAS},
        "leaders": {"left": False, "right": False},
        "last_saved": str(Path(tmp) / "episode_0000.hdf5"),
        "language_instruction": "", "robot_ok": True,
    }
    win.apply_snapshot(snap)
    win.set_camera_frames({c: _jpeg() for c in CAMERAS}, {c: 1.0 for c in CAMERAS})
    app.processEvents()

    # RECORD button posts "R"; instruction edit updates meta; jog buttons hit teleop.
    win._key("R")
    assert cq.get_nowait() == "R"
    win.instr.setText("grab the cube")
    assert meta["language_instruction"] == "grab the cube"

    # episode viewer: it should list the episode saved by _test_recording_save
    # (same tmp dir), and load + scrub + play it.
    v = win.viewer
    assert v.list.count() >= 1, "viewer should list the recorded episode"
    v.list.setCurrentRow(0)
    app.processEvents()
    assert v._T == 6, v._T
    assert v.slider.maximum() == 5, v.slider.maximum()
    v._render(3)
    assert v.frame_lbl.text() == "4/6", v.frame_lbl.text()
    v._toggle_play()
    assert v._timer.isActive()
    v._advance()
    v._stop()
    assert not v._timer.isActive()

    # joint trace + numeric readout populated on load/scrub
    assert v.trace._T == 6 and v._qpos_all is not None and v._qpos_all.shape == (6, 17)
    assert v.qpos_lbl.text() != ""

    # post-hoc segment annotation: mark In/Out -> Add -> Save -> persisted (v7)
    v.slider.setValue(1); v._mark_in()
    v.slider.setValue(3); v._mark_out()
    v.seg_combo.setCurrentText("grasp"); v._add_segment()
    assert v._edit_segments == [{"start": 1, "end": 4, "label": "grasp"}], v._edit_segments
    v._save_segments()
    from python.training.minerva_dataset import MinervaEpisodeDataset as _DS
    _ds = _DS([Path(v._path)], chunk_size=2, future_offset=0)
    assert _ds._label_at(0, 2) == "grasp", _ds._label_at(0, 2)
    assert _ds._label_at(0, 0) == "pick up the block", _ds._label_at(0, 0)

    # auto-segment (flat episode -> one span) + relabel the selected span
    v._auto_segment()
    assert len(v._edit_segments) == 1 and v._edit_segments[0]["end"] == 6, v._edit_segments
    v.seg_list.setCurrentRow(0)
    v.seg_combo.setCurrentText("reach")
    v._relabel_selected()
    assert v._edit_segments[0]["label"] == "reach"

    # delete (no-modal core) removes the file and empties the list
    _p = Path(v._path)
    v._delete_path(_p)
    assert not _p.exists(), "delete should remove the episode file"
    assert v.list.count() == 0, "list should be empty after deleting the only episode"

    # live action-label control pushes onto the label queue
    win.action_combo.setCurrentText("grasp")
    win._set_phase()
    assert win.label_queue.get_nowait() == "grasp"
    # voice apply-label path (bypasses mic/modal); no STT backend here -> mic disabled
    win._apply_voice_label("insert the peg")
    assert win.action_combo.currentText() == "insert the peg"
    assert win.label_queue.get_nowait() == "insert the peg"
    # phase hotkeys cycle / pick from the vocab and queue the label
    win.action_combo.setCurrentText("")
    win._cycle_phase(1)
    assert win.action_combo.currentText() == _ACTION_VOCAB[0]
    assert win.label_queue.get_nowait() == _ACTION_VOCAB[0]
    win._pick_phase(2)
    assert win.action_combo.currentText() == _ACTION_VOCAB[2]
    assert win.label_queue.get_nowait() == _ACTION_VOCAB[2]
    # voice button reflects local STT availability
    from collect_minerva_app.speech import SpeechToText
    assert win.voice_btn.isEnabled() == SpeechToText().available()

    # settings: dialog builds, values complete, and apply reconfigures the STT
    from collect_minerva_gui import _SettingsDialog
    dlg = _SettingsDialog(win.settings, win.params)
    _vals = dlg.values()
    assert {"mic_device", "voice_model", "voice_seconds"} <= set(_vals)
    # new tunables round-trip through the tabbed dialog
    assert {"kp_scale", "grip_strength", "grav_comp", "grav_scale", "grip_ff",
            "grip_ff_gain", "grip_ff_invert", "arm_kp", "arm_kd", "arm_sat"} <= set(_vals)
    assert len(_vals["arm_kp"]) == 6 and len(_vals["arm_kd"]) == 6 and len(_vals["arm_sat"]) == 6
    win.settings.update({"voice_seconds": 3.0, "voice_model": "tiny.en"})
    win._voice_seconds = float(win.settings.get("voice_seconds"))
    win._reload_stt()
    assert win._voice_seconds == 3.0
    assert win._stt is not None and win._stt.whisper_model == "tiny.en"

    win.close()
    print("  ✓ GUI offscreen (viewer, auto-segment, labels, voice, hotkeys, settings)")


def run_smoke():
    ctx = zmq.Context()
    try:
        _test_receivers_and_decoder(ctx)
        tel = _test_teleop()
        tmp = _test_recording_save()
        # GUI tests must not load operator preferences, enumerate speech
        # backends or download a model on machines that have STT installed.
        with patch("collect_minerva_app.settings._DEFAULT_PATH", Path(tmp) / "settings.json"), \
             patch("collect_minerva_app.speech._have", return_value=False):
            _test_gui(tel, tmp)
        tel.close()
    finally:
        ctx.destroy(linger=0)   # force-close ALL sockets so we never deadlock on term()
    print("COLLECTOR SMOKE PASS")


def test_minerva_collector_smoke():
    run_smoke()


if __name__ == "__main__":
    run_smoke()
