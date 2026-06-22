#!/usr/bin/env python3
"""AIZEE RealSense self-test.

Three checks, run in order. Exits with code 0 only when every requested
stage passed; stops at the first failure (no point trying ZMQ if the
SDK can't see the camera).

  1. SDK + device enumeration via pyrealsense2.
  2. Short streaming pipeline (color + optional depth) — pulls a few
     frames so we know librealsense can actually negotiate the requested
     mode, not just see the USB endpoint.
  3. Optional ZMQ subscriber — connects to the running publisher and
     waits for one multipart camera frame.

Invoked at the end of scripts/deploy_scene_cam.sh so the deploy ends with
a green / red signal. Also usable standalone on the Jetson:

    python3 scripts/test_realsense.py \\
        --config config/hardware_jetson_scene_cam.yaml \\
        --zmq tcp://127.0.0.1:5564
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import yaml
except Exception:
    yaml = None  # config flag becomes a no-op


def _stage(label: str) -> None:
    sys.stdout.write(f"\n--- {label} ---\n")
    sys.stdout.flush()


def _ok(msg: str) -> None:
    sys.stdout.write(f"  OK  {msg}\n")
    sys.stdout.flush()


def _fail(msg: str) -> int:
    sys.stdout.write(f"  FAIL  {msg}\n")
    sys.stdout.flush()
    return 1


def stage_sdk_enum() -> tuple[int, list]:
    """Stage 1: import the SDK and enumerate connected devices.

    Returns rc=0 on success; rc=2 when the SDK loaded but enumeration
    failed in a way consistent with "another process holds the device"
    (librealsense raises `RuntimeError: bad optional access` or returns
    an empty list when the device is exclusively held by the publisher).
    The deploy driver downgrades rc=2 to a warning when the ZMQ stage
    proves liveness anyway.
    """
    _stage("RealSense SDK + device enumeration")
    try:
        import pyrealsense2 as rs
    except Exception as e:
        return _fail(f"pyrealsense2 import failed: {e}"), []

    try:
        ctx = rs.context()
        devices = list(ctx.devices)
    except RuntimeError as e:
        # Typical when the systemd publisher is already streaming — the
        # USB endpoint is held exclusively and libusb can't enumerate
        # again from this process.
        _fail(f"enumeration raised: {e}  (publisher likely holds the device)")
        return 2, []
    if not devices:
        _fail("no RealSense devices detected (check USB cable / power, "
              "or publisher may already hold the device)")
        return 2, []

    out = []
    for d in devices:
        try:
            name   = d.get_info(rs.camera_info.name)
            serial = d.get_info(rs.camera_info.serial_number)
            fw     = d.get_info(rs.camera_info.firmware_version)
        except Exception as e:
            return _fail(f"failed to query device info: {e}"), []
        try:
            usb = d.get_info(rs.camera_info.usb_type_descriptor)
        except Exception:
            usb = "?"
        _ok(f"{name}  serial={serial}  fw={fw}  usb={usb}")
        out.append((name, serial, fw, usb))
    return 0, out


def stage_stream(color_wh: tuple[int, int], depth_wh: tuple[int, int],
                 fps: int, want_depth: bool, n_frames: int) -> int:
    """Stage 2: bring the pipeline up and pull a few frames."""
    _stage(f"streaming test  ({n_frames} frames @ {fps}fps, "
           f"color {color_wh[0]}x{color_wh[1]}"
           f"{', depth ' + str(depth_wh[0]) + 'x' + str(depth_wh[1]) if want_depth else ''})")
    import pyrealsense2 as rs

    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, color_wh[0], color_wh[1], rs.format.rgb8, fps)
    if want_depth:
        cfg.enable_stream(rs.stream.depth, depth_wh[0], depth_wh[1], rs.format.z16, fps)

    try:
        profile = pipe.start(cfg)
    except RuntimeError as e:
        return _fail(f"pipeline.start failed: {e}")

    try:
        # Warm-up. AE/AWB stabilise within the first second on every D4xx
        # we've used — bail before then and `wait_for_frames` can time out.
        t0 = time.monotonic()
        last_ts = None
        for i in range(n_frames):
            frames = pipe.wait_for_frames(timeout_ms=5000)
            c = frames.get_color_frame()
            d = frames.get_depth_frame() if want_depth else None
            if not c or (want_depth and not d):
                return _fail(f"frame {i}: missing color={bool(c)} depth={bool(d)}")
            last_ts = float(frames.get_timestamp())
        dt = time.monotonic() - t0
        eff_fps = n_frames / dt if dt > 0 else 0.0
        _ok(f"captured {n_frames} frames in {dt:.2f}s (~{eff_fps:.1f} fps)  "
            f"last_rs_ts={last_ts:.1f}ms")

        # Depth intrinsics + scale — proves the depth path is fully configured.
        if want_depth:
            ds = profile.get_device().first_depth_sensor().get_depth_scale()
            di = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            _ok(f"depth intrinsics  {di.width}x{di.height}  "
                f"fx={di.fx:.1f} fy={di.fy:.1f}  scale={ds:.6f} m/unit")
    finally:
        try:
            pipe.stop()
        except Exception:
            pass
    return 0


def stage_zmq(endpoint: str, timeout_s: float) -> int:
    """Stage 3: confirm a running publisher is putting frames on the wire."""
    _stage(f"ZMQ subscriber  {endpoint}  (timeout {timeout_s:.0f}s)")
    try:
        import zmq
    except Exception as e:
        return _fail(f"pyzmq import failed: {e}")

    ctx  = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(endpoint)

    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            if sock.poll(timeout=200) == 0:
                continue
            parts = sock.recv_multipart(zmq.NOBLOCK)
            n_parts = len(parts)
            sizes = [len(p) for p in parts]
            _ok(f"got multipart frame: {n_parts} parts, sizes={sizes}")
            return 0
        return _fail(f"no frame received within {timeout_s:.0f}s — is the "
                     f"publisher running?  (systemctl status aizee-scene-cam)")
    finally:
        sock.close(linger=0)


def main() -> int:
    ap = argparse.ArgumentParser(description="AIZEE RealSense self-test")
    ap.add_argument("--config", default=None,
                    help="hardware_jetson_*_cam.yaml — pulls stream sizes / "
                         "fps so the test matches what the service will run")
    ap.add_argument("--color", default="640x480",
                    help="Color WxH for stage 2 (overridden by --config)")
    ap.add_argument("--depth", default="640x480",
                    help="Depth WxH for stage 2 (overridden by --config)")
    ap.add_argument("--fps",   type=int, default=30,
                    help="Stream FPS for stage 2 (overridden by --config)")
    ap.add_argument("--no-depth", action="store_true",
                    help="Skip the depth stream in stage 2 (RGB-only check)")
    ap.add_argument("--frames", type=int, default=15,
                    help="Frames to pull in stage 2 (default: 15)")
    ap.add_argument("--zmq", default=None,
                    help="If set, stage 3 connects to this PUB endpoint "
                         "and waits for one frame.  Skipped when omitted.")
    ap.add_argument("--zmq-timeout", type=float, default=10.0,
                    help="How long to wait for the first ZMQ frame (s)")
    ap.add_argument("--skip-stream", action="store_true",
                    help="Skip stage 2 (enumeration-only check)")
    args = ap.parse_args()

    # Pull stream params out of the YAML when provided so the test exactly
    # matches the service config. CLI flags still win over YAML defaults.
    want_depth = not args.no_depth
    color_w, color_h = (int(v) for v in args.color.lower().split("x"))
    depth_w, depth_h = (int(v) for v in args.depth.lower().split("x"))
    fps = args.fps
    if args.config:
        if yaml is None:
            return _fail("--config given but PyYAML is not installed")
        cfg_path = Path(args.config)
        if not cfg_path.exists():
            return _fail(f"--config path does not exist: {cfg_path}")
        cfg = yaml.safe_load(cfg_path.read_text()) or {}
        streams = (cfg.get("camera", {}) or {}).get("streams", {}) or {}
        c = streams.get("color", {}) or {}
        d = streams.get("depth", {}) or {}
        color_w, color_h = int(c.get("width",  color_w)), int(c.get("height", color_h))
        depth_w, depth_h = int(d.get("width",  depth_w)), int(d.get("height", depth_h))
        fps        = int(c.get("fps", fps))
        want_depth = bool(d.get("enabled", want_depth))

    rc, _devs = stage_sdk_enum()
    # rc=2 means the publisher likely holds the device — keep going if
    # the caller asked for a ZMQ check, since that's the authoritative
    # liveness signal in deploy context.
    enum_busy = (rc == 2)
    if rc and rc != 2:
        return rc
    if rc == 2 and not args.zmq:
        return rc   # standalone run: enum failure IS the failure

    if not args.skip_stream and not enum_busy:
        rc = stage_stream((color_w, color_h), (depth_w, depth_h),
                          fps, want_depth, args.frames)
        if rc:
            return rc
    elif enum_busy and not args.skip_stream:
        sys.stdout.write("\n  SKIP  stream test (publisher holds the device)\n")

    if args.zmq:
        rc = stage_zmq(args.zmq, args.zmq_timeout)
        if rc:
            return rc

    if enum_busy:
        sys.stdout.write("\nLive ZMQ frame confirms the publisher is "
                         "streaming — enumeration was skipped because the "
                         "service holds the device exclusively.\n")
    else:
        sys.stdout.write("\nAll requested stages passed.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
