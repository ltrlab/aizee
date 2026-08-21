"""images.py — JPEG decode helpers + a background decoder thread.

The GUI paints raw JPEG straight to QPixmap (Qt's C++ decoder), so the only
consumer that needs decoded numpy frames is the RECORDER. This thread decodes
each camera's latest cached frame to uint8 RGB at the target resolution,
carrying the publisher timestamp alongside (paired under one lock acquisition
so a frame and its ts never desync across streams). Gated on `rec_flag` so we
don't burn CPU decoding when not recording.
"""

from __future__ import annotations

import io
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False
from PIL import Image


def decode_jpeg(raw: bytes, size_wh: Tuple[int, int]) -> Optional[np.ndarray]:
    """Decode JPEG bytes to uint8 RGB [H, W, 3] at (width, height)=size_wh."""
    if raw is None:
        return None
    if _CV2:
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if (bgr.shape[1], bgr.shape[0]) != tuple(size_wh):
            bgr = cv2.resize(bgr, tuple(size_wh), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if (img.width, img.height) != tuple(size_wh):
        img = img.resize(tuple(size_wh), Image.BILINEAR)
    return np.array(img, dtype=np.uint8)


def raw_jpeg(cam_msg: Optional[dict]) -> Optional[bytes]:
    if not cam_msg:
        return None
    return (cam_msg.get("color") or {}).get("data_bytes")


def start_image_decoder(
    cam_lock: threading.Lock,
    cam_cache: dict,
    cameras: Sequence[str],
    sizes: Dict[str, Tuple[int, int]],
    rec_flag: Optional[threading.Event] = None,
    always_on: bool = False,
    hz: int = 40,
) -> Tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    """Decode the latest frame of each camera into `dec_cache`.

    dec_cache keys per <name>: <name> (uint8 RGB [H,W,3]) and <name>_ts (float).
    Returns (stop, thread, lock, dec_cache).
    """
    lock = threading.Lock()
    dec: dict = {}
    for name in cameras:
        dec[name] = None
        dec[f"{name}_ts"] = None
        dec[f"{name}_recv_time"] = None
    stop = threading.Event()
    period = 1.0 / max(hz, 1)

    def _run() -> None:
        while not stop.is_set():
            if not always_on and rec_flag is not None and not rec_flag.is_set():
                stop.wait(0.02)
                continue
            t0 = time.monotonic()
            for name in cameras:
                with cam_lock:
                    msg = cam_cache.get(name)
                    ts = cam_cache.get(f"{name}_ts")
                    recv_t = cam_cache.get(f"{name}_time")
                raw = raw_jpeg(msg)
                size = sizes.get(name)
                if raw is None or size is None:
                    continue
                try:
                    img = decode_jpeg(raw, size)
                except Exception:
                    img = None   # a decode error must NOT kill the decoder thread
                if img is not None:
                    with lock:
                        dec[name] = img
                        dec[f"{name}_ts"] = ts
                        dec[f"{name}_recv_time"] = recv_t
            stop.wait(max(0.0, period - (time.monotonic() - t0)))

    thread = threading.Thread(target=_run, daemon=True, name="ImgDec")
    thread.start()
    return stop, thread, lock, dec


__all__ = ["decode_jpeg", "raw_jpeg", "start_image_decoder"]
