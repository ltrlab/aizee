"""Camera JPEG decoding + background image-decoder thread (from collect_demo.py)."""
from __future__ import annotations

import io
import threading
from typing import Optional

import numpy as np
from PIL import Image

try:
    import cv2
    _cv2_available = True
except ImportError:
    _cv2_available = False

def _start_image_decoder(
    cam_lock: threading.Lock,
    cam_cache: dict,
    img_size: tuple,
    always_on: bool = False,
    scene_proj_stride: int = 20,
    scene_proj_z_near: float = 0.15,
    scene_proj_z_far:  float = 3.0,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict, threading.Event]:
    """Background thread that decodes + resizes camera JPEGs.

    Gated on *rec_flag* by default (decoding only matters for the record
    buffer).  Pass always_on=True for GUI mode where live preview needs
    decoded frames even outside recording.

    Scene cam additionally has its depth backprojected to a CAMERA-FRAME
    pointcloud right here on the worker thread, so the GUI never pays
    that cost. The final pose transform (R·p + t to world frame) is the
    only remaining numpy work for the GUI side — that's a single matmul
    on ~700 points, sub-millisecond.

    Each `_dec_cache["{cam}"]` is paired with `_dec_cache["{cam}_ts_pub"]`
    (the publisher's capture timestamp of the same frame), so the recorder
    can pull image + ts atomically — eliminates the off-by-one-frame sync
    bug where `latest_*_ts` referred to a newer frame than the decoded
    ndarray. Critical for training-time alignment across streams.
    """
    lock     = threading.Lock()
    decoded: dict = {
        # gripper: image (np.ndarray) + ts_pub (publisher's capture ts) +
        # time (local arrival ts, used to drive the freshness check by
        # callers that already do age-based gating).
        "gripper": None, "gripper_time": 0.0, "gripper_ts_pub": None,
        # scene: same as gripper, plus pre-projected camera-frame point
        # cloud + per-point depth (for the GUI's 3D viz). All five fields
        # are written under the same lock acquisition so consumers get a
        # consistent snapshot of the same frame.
        "scene":            None,
        "scene_time":       0.0,
        "scene_ts_pub":     None,
        "scene_pts_cam":    None,   # (N, 3) float32, camera-frame XYZ
        "scene_pts_depth":  None,   # (N,)  float32, per-point Z for color ramp
    }
    rec_flag = threading.Event()
    stop     = threading.Event()

    def _scene_depth_to_cam_points(depth_blob: dict) -> tuple:
        """Backproject Z16 depth → (N, 3) camera-frame XYZ + (N,) depths.

        Returns (None, None) if the depth payload or intrinsics are
        missing — the decoder skips updating the pointcloud fields in
        that case, leaving the previous cache values intact.

        Camera optical frame here is the librealsense / OpenCV
        convention: +X right, +Y down, +Z forward. The GUI applies a
        single rotation+translation to land in URDF world frame.
        """
        raw   = depth_blob.get("data_bytes")
        intr  = depth_blob.get("intrinsics")
        scale = depth_blob.get("scale")
        w     = depth_blob.get("width")
        h     = depth_blob.get("height")
        if (raw is None or intr is None or scale is None
                or w is None or h is None):
            return None, None
        try:
            depth = np.frombuffer(raw, dtype=np.uint16).reshape(int(h), int(w))
        except Exception:
            return None, None
        fx = float(intr["fx"]); fy = float(intr["fy"])
        cx = float(intr["cx"]); cy = float(intr["cy"])
        us = np.arange(0, int(w), scene_proj_stride, dtype=np.int32)
        vs = np.arange(0, int(h), scene_proj_stride, dtype=np.int32)
        UU, VV = np.meshgrid(us, vs)
        z = depth[VV, UU].astype(np.float32) * float(scale)
        m = (z > scene_proj_z_near) & (z < scene_proj_z_far)
        if not m.any():
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, np.zeros((0,), dtype=np.float32)
        u = UU[m].astype(np.float32); v = VV[m].astype(np.float32)
        zf = z[m]
        X = (u - cx) / fx * zf
        Y = (v - cy) / fy * zf
        pts_cam = np.stack([X, Y, zf], axis=1).astype(np.float32)
        return pts_cam, zf

    def _run() -> None:
        prev_gt = 0.0
        prev_st = 0.0
        while not stop.is_set():
            if not (always_on or rec_flag.is_set()):
                stop.wait(0.05)
                continue

            with cam_lock:
                g_msg = cam_cache["gripper"]
                g_t   = cam_cache["gripper_time"]
                s_msg = cam_cache.get("scene")
                s_t   = cam_cache.get("scene_time", 0.0)

            changed = False
            if g_t > prev_gt and g_msg is not None:
                # Orientation correction is applied at the publisher
                # (config/hardware_jetson_gripper_cam.yaml → streams.color.flip),
                # so the consumer doesn't need to flip anything here.
                img = decode_image(g_msg, img_size)
                pub_ts = g_msg.get("timestamp")
                with lock:
                    decoded["gripper"]        = img
                    decoded["gripper_time"]   = g_t
                    decoded["gripper_ts_pub"] = (
                        float(pub_ts) if pub_ts is not None else None)
                prev_gt = g_t
                changed = True

            if s_t > prev_st and s_msg is not None:
                # Decode color + project depth, then commit both under one
                # lock acquisition so consumers (recorder, GUI push) see a
                # consistent (image, points, ts) triple from the same frame.
                img = decode_image(s_msg, img_size)
                depth_blob = s_msg.get("depth", {}) or {}
                pts_cam, pts_depth = _scene_depth_to_cam_points(depth_blob)
                pub_ts = s_msg.get("timestamp")
                with lock:
                    decoded["scene"]           = img
                    decoded["scene_time"]      = s_t
                    decoded["scene_ts_pub"]    = (
                        float(pub_ts) if pub_ts is not None else None)
                    if pts_cam is not None:
                        decoded["scene_pts_cam"]   = pts_cam
                        decoded["scene_pts_depth"] = pts_depth
                prev_st = s_t
                changed = True

            if not changed:
                stop.wait(0.005)   # 200 Hz poll when idle

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, decoded, rec_flag


# ---------------------------------------------------------------------------
# Image decoding
# ---------------------------------------------------------------------------

def decode_image(msg: dict, target_size: tuple, flip_v: bool = False) -> Optional[np.ndarray]:
    """Decode a camera message to uint8 [H, W, 3]. target_size is (W, H).

    Uses cv2 (libjpeg-turbo + INTER_AREA) when available — ~5-10x faster
    than PIL+LANCZOS and releases the GIL for the JPEG decode and resize.
    Falls back to PIL when cv2 isn't installed.

    Expects the multipart wire format: `msg["color"]["data_bytes"]` is
    the raw JPEG.  (Old base64 path under `["data"]` is no longer used.)
    """
    color = msg.get("color", {})
    raw   = color.get("data_bytes")
    if raw is None:
        return None
    if _cv2_available:
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if flip_v:
            bgr = cv2.flip(bgr, 0)
        # target_size is (W, H); cv2.resize takes (W, H) too.  Skip the
        # resize when the publisher already produced the target size.
        if (bgr.shape[1], bgr.shape[0]) != target_size:
            bgr = cv2.resize(bgr, target_size, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if flip_v:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
    if (img.width, img.height) != target_size:
        img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)
