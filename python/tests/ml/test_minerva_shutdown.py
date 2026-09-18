"""
test_minerva_shutdown.py — clean-exit paths for the collector.

Covers the two things that made the app not exit cleanly:
  1. receiver threads must be joined so ctx.term() doesn't hang on open sockets,
  2. the QtRenderer QApplication (on a worker thread) must actually stop on
     request_quit().
"""

from __future__ import annotations

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import queue
import sys
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

_PY = Path(__file__).resolve().parents[3] / "python"
sys.path.insert(0, str(_PY))
sys.path.insert(0, str(_PY / "scripts"))

import zmq

from common.minerva_constants import CAMERAS
from collect_minerva_app.images import start_image_decoder
from collect_minerva_app.receivers import start_cam_receiver
from collect_minerva_app.teleop import MinervaTeleop


def test_backend_shutdown():
    ctx = zmq.Context()
    cam_pubs = {c: ctx.socket(zmq.PUB) for c in CAMERAS}
    for c, p in cam_pubs.items():
        p.bind(f"inproc://c_{c}")
    cs, ct, cl, cc = start_cam_receiver(ctx, {c: f"inproc://c_{c}" for c in CAMERAS})
    ds, dt, _, _ = start_image_decoder(cl, cc, CAMERAS, {c: (64, 48) for c in CAMERAS},
                                       always_on=True)
    time.sleep(0.3)

    for st in (ds, cs):
        st.set()
    for th in (dt, ct):
        th.join(timeout=1.5)
    assert not any(th.is_alive() for th in (dt, ct)), "receiver threads did not stop"

    for p in cam_pubs.values():
        p.close(linger=0)
    # ctx.term() must not hang now that every socket is closed.
    done = threading.Event()
    threading.Thread(target=lambda: (ctx.term(), done.set()), daemon=True).start()
    assert done.wait(5.0), "ctx.term() hung — a receiver socket was still open"
    print("  OK: receivers join + ctx.term() returns (no hang)")


def test_qt_shutdown():
    from collect_minerva_gui import QtRenderer
    tel = MinervaTeleop(hardware_enabled=False)
    tel.connect(verbose=False)
    qt = QtRenderer(cmd_queue=queue.Queue(),
                    meta={"language_instruction": "", "notes": "", "task_id": None},
                    teleop=tel, cameras=CAMERAS,
                    output_dir=tempfile.mkdtemp(prefix="minerva_sd_"),
                    label_queue=queue.Queue())
    with patch("collect_minerva_app.settings._DEFAULT_PATH", Path(qt.output_dir) / "settings.json"), \
         patch("collect_minerva_app.speech._have", return_value=False):
        qt.start()
        deadline = time.monotonic() + 15.0
        while qt._win is None and not qt.should_quit() and time.monotonic() < deadline:
            time.sleep(0.05)
        try:
            assert qt._win is not None, "QtRenderer did not finish initialization"
            assert qt._thread is not None and qt._thread.is_alive()
        finally:
            qt.request_quit()
            qt.join(timeout=5.0)
        assert not qt._thread.is_alive(), "QtRenderer thread did not stop on request_quit"
    tel.close()
    print("  OK: QtRenderer QApplication thread stops on request_quit")


def run():
    test_backend_shutdown()
    test_qt_shutdown()
    print("SHUTDOWN TEST PASS")


def test_minerva_shutdown():
    run()


if __name__ == "__main__":
    run()
    # Same guard as the app: a worker-thread QApplication segfaults on teardown.
    sys.stdout.flush()
    os._exit(0)
