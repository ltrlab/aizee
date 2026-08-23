#!/usr/bin/env python3
"""
AIZEE Gripper Camera Node - ELP-USBFHD01M-L21 via V4L2/OpenCV

Single USB UVC color camera (MJPG/YUYV) USB-connected to the Jetson and
mounted on the gripper looking at the workspace. Replaces the previous
stereo D435 arm-camera pair (arm_cam_left + arm_cam_right).

Publishes color-only frames on a single ZeroMQ PUB endpoint. No depth.

Usage:
    python gripper_camera_node.py --config config/hardware_jetson_gripper_cam.yaml
"""

import argparse
import io
import logging
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
import zmq
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_camera


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


_FOURCC = {
    "MJPG": cv2.VideoWriter_fourcc("M", "J", "P", "G"),
    "YUYV": cv2.VideoWriter_fourcc("Y", "U", "Y", "V"),
}


class GripperCameraNode:
    """ELP UVC color camera publishing JPEG frames over ZMQ."""

    def __init__(self, config: dict):
        cam_cfg = config.get("camera", {})
        color_cfg = cam_cfg.get("streams", {}).get("color", {})
        zmq_cfg = config.get("network", {}).get("device", {}).get("zmq", {})

        self.camera_id: str = cam_cfg.get("id", "gripper_cam")
        # /dev/videoN device path. Prefer the stable symlink set by udev.
        self.device: str = str(cam_cfg.get("device", "/dev/aizee_gripper_cam"))
        # Optional USB VID:PID ("32e4:9230"). When set, the node can re-find THIS
        # camera by identity after an unplug/replug that renumbers /dev/videoN
        # (video0 -> video4), instead of being stuck on a now-dead path.
        self.usb_id: str = str(cam_cfg.get("usb_id", "")).strip().lower()
        self.fourcc: str = str(cam_cfg.get("fourcc", "MJPG")).upper()

        self.capture_w: int = int(color_cfg.get("width", 1024))
        self.capture_h: int = int(color_cfg.get("height", 768))
        self.fps: int = int(color_cfg.get("fps", 30))
        self.jpeg_quality: int = int(color_cfg.get("quality", 85))

        # Optional downscale before JPEG encode — pushes the wire payload down
        # without changing capture resolution (better for AE/AWB stability).
        self.output_w: Optional[int] = color_cfg.get("output_width")
        self.output_h: Optional[int] = color_cfg.get("output_height")

        _flip = str(color_cfg.get("flip", "none")).lower()
        if _flip not in ("none", "horizontal", "vertical", "180"):
            logger.warning(f"Unknown color.flip={_flip!r}; using 'none'")
            _flip = "none"
        self.color_flip: str = _flip

        # Optional MJPG passthrough: avoid decode-then-reencode when the camera
        # is already producing MJPG at the desired resolution/quality.
        self.passthrough_mjpg: bool = bool(color_cfg.get("passthrough_mjpg", False))

        # Optional V4L2 controls (gain, exposure, white balance, etc.) applied
        # after the device opens. Dict order matters — `auto_exposure` must be
        # set before `exposure_time_absolute` becomes writable. See
        # `v4l2-ctl --device <dev> --list-ctrls` for valid names/ranges.
        v4l2 = cam_cfg.get("v4l2_controls") or {}
        if not isinstance(v4l2, dict):
            logger.warning(f"camera.v4l2_controls should be a dict, got {type(v4l2)}; ignoring")
            v4l2 = {}
        self.v4l2_controls: dict = dict(v4l2)

        self.zmq_endpoint: str = zmq_cfg.get("camera_pub", "tcp://*:5563")
        # REP socket for runtime camera-control changes (sliders in the
        # collect-demo GUI). Empty string disables the control channel.
        self.zmq_ctrl_endpoint: str = zmq_cfg.get("camera_ctrl", "tcp://*:5573")

        log_level = config.get("logging", {}).get("level", "INFO")
        logger.setLevel(getattr(logging, log_level, logging.INFO))

        self.cap: Optional[cv2.VideoCapture] = None
        # The device path we actually opened (may differ from self.device after a
        # replug renumber); v4l2-ctl targets this so runtime controls hit the live node.
        self._active_device: str = self.device
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        self.ctrl_socket: Optional[zmq.Socket] = None
        self.running = False

        self.frame_count = 0
        self.last_stats_time = time.time()
        self._stat_encode_ms: list = []
        self._stat_send_ms: list = []

    # --- reconnect tuning ---
    _READ_FAIL_LIMIT = 30            # consecutive failed reads => device lost (~0.6s @ 50Hz)
    _REOPEN_BACKOFF_S = (0.5, 1.0, 2.0, 3.0)   # escalating wait between reopen attempts

    @staticmethod
    def _node_usb_id(dev_path: str) -> str:
        """'vvvv:pppp' (lowercase) USB VID:PID for a /dev/videoN (following symlinks)
        via sysfs, or '' if it can't be determined."""
        import os
        try:
            node = os.path.basename(os.path.realpath(dev_path))
            d = os.path.realpath(f"/sys/class/video4linux/{node}/device")
        except OSError:
            return ""
        for _ in range(6):            # walk interface dir up to the usb_device dir
            iv, ip = os.path.join(d, "idVendor"), os.path.join(d, "idProduct")
            if os.path.exists(iv) and os.path.exists(ip):
                try:
                    with open(iv) as f:
                        v = f.read().strip().lower()
                    with open(ip) as f:
                        p = f.read().strip().lower()
                    return f"{v}:{p}"
                except OSError:
                    return ""
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
        return ""

    def _find_video_by_usb_id(self, usb_id: str) -> Optional[str]:
        """Lowest-numbered /dev/video* whose USB identity matches `usb_id` AND that
        actually delivers a frame (skips a camera's metadata-only nodes). None if the
        camera isn't present."""
        import glob
        import os

        def _num(p: str) -> int:
            digits = "".join(ch for ch in os.path.basename(p) if ch.isdigit())
            return int(digits) if digits else 0

        matches = [d for d in sorted(glob.glob("/dev/video*"), key=_num)
                   if self._node_usb_id(d) == usb_id]
        for dev in matches:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            try:
                if cap.isOpened() and cap.read()[0]:
                    return dev
            finally:
                cap.release()
        return matches[0] if matches else None

    def _resolve_device(self) -> Optional[str]:
        """Device path to open. Prefer the configured path, but if a usb_id is set,
        verify it still points at THIS camera and otherwise rescan by identity — so a
        replug that renumbers the node is transparently recovered. None if not found."""
        import os
        if not self.usb_id:
            return self.device if os.path.exists(self.device) else None
        # usb_id known: trust the configured path only if it still IS this camera
        if os.path.exists(self.device) and self._node_usb_id(self.device) == self.usb_id:
            return self.device
        found = self._find_video_by_usb_id(self.usb_id)
        if found and found != self.device:
            logger.info(f"{self.camera_id}: {self.usb_id} is now {found} "
                        f"(configured {self.device})")
        return found

    def initialize_camera(self):
        dev = self._resolve_device()
        if dev is None:
            raise RuntimeError(
                f"camera not found (device={self.device} usb_id={self.usb_id or 'n/a'})")
        logger.info(
            f"Opening UVC camera: id={self.camera_id} dev={dev} "
            f"{self.capture_w}x{self.capture_h}@{self.fps} fourcc={self.fourcc}"
        )

        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"Failed to open camera at {dev}")
        self._active_device = dev

        # FOURCC must be set BEFORE width/height/fps for many UVC drivers.
        fourcc = _FOURCC.get(self.fourcc)
        if fourcc is None:
            raise ValueError(f"Unsupported fourcc {self.fourcc!r}; expected MJPG or YUYV")
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Smallest possible internal buffer — keep latency low.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        afps = cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"Camera negotiated: {aw}x{ah} @ {afps:.1f} fps")

        self.cap = cap
        self._apply_v4l2_controls()

    def _apply_v4l2_controls(self) -> None:
        """Apply each control in self.v4l2_controls via the v4l2-ctl CLI.

        We shell out instead of using cv2 props because (a) cv2's V4L2
        property mapping is incomplete and inconsistent across opencv
        builds, and (b) v4l2-ctl's error messages are far clearer when a
        control name/value is wrong. Failures are logged but non-fatal —
        capture should still come up at the camera's default exposure.
        """
        if not self.v4l2_controls:
            return
        for name, value in self.v4l2_controls.items():
            cmd = ["v4l2-ctl", "--device", self._active_device,
                   f"--set-ctrl={name}={value}"]
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                if r.returncode != 0:
                    logger.warning(f"v4l2-ctl set {name}={value} failed: "
                                   f"{(r.stderr or r.stdout).strip()}")
                else:
                    logger.info(f"v4l2-ctl set {name}={value}")
            except FileNotFoundError:
                logger.warning("v4l2-ctl not installed; skipping v4l2_controls "
                               "(install with: sudo apt install v4l-utils)")
                return
            except subprocess.TimeoutExpired:
                logger.warning(f"v4l2-ctl set {name}={value} timed out")

    def initialize_zmq(self):
        logger.info(f"Binding ZMQ PUB to {self.zmq_endpoint}")
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.setsockopt(zmq.SNDHWM, 2)
        self.zmq_socket.bind(self.zmq_endpoint)
        if self.zmq_ctrl_endpoint:
            logger.info(f"Binding ZMQ REP (camera-control) to {self.zmq_ctrl_endpoint}")
            self.ctrl_socket = self.zmq_context.socket(zmq.REP)
            self.ctrl_socket.setsockopt(zmq.LINGER, 0)
            # Drop stale REQ if a consumer reconnects mid-conversation;
            # without RCVTIMEO a broken peer could leave the REP wedged.
            self.ctrl_socket.setsockopt(zmq.RCVTIMEO, 100)
            self.ctrl_socket.bind(self.zmq_ctrl_endpoint)

    # ------------------------------------------------------------------
    # Runtime control channel
    # ------------------------------------------------------------------

    # Controls exposed to GUI sliders. Constrains what runtime callers
    # can change — prevents misclicks from disabling auto-WB or worse.
    _CTRL_ALLOWLIST = (
        "auto_exposure",
        "exposure_time_absolute",
        "gain",
        "brightness",
        "contrast",
        "saturation",
        "gamma",
        "sharpness",
        "backlight_compensation",
        "white_balance_automatic",
        "white_balance_temperature",
        "power_line_frequency",
    )

    def _v4l2_set(self, name: str, value) -> tuple[bool, str]:
        """Run `v4l2-ctl --set-ctrl name=value`. Returns (ok, message)."""
        cmd = ["v4l2-ctl", "--device", self._active_device, f"--set-ctrl={name}={value}"]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
        except FileNotFoundError:
            return False, "v4l2-ctl not installed on this host"
        except subprocess.TimeoutExpired:
            return False, "v4l2-ctl timed out"
        if r.returncode != 0:
            return False, (r.stderr or r.stdout).strip() or f"exit {r.returncode}"
        return True, "ok"

    def _v4l2_list_ctrls(self) -> dict:
        """Parse `v4l2-ctl --list-ctrls` into a dict for each allow-listed control.

        Returns: {name: {"value": int, "min": int, "max": int, "default": int,
                          "type": "int"|"bool"|"menu"}}.
        Only allow-listed controls are returned; unknown / missing controls
        are silently omitted (e.g. a different USB camera may expose fewer).
        """
        try:
            r = subprocess.run(
                ["v4l2-ctl", "--device", self._active_device, "--list-ctrls"],
                capture_output=True, text=True, timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            logger.warning(f"v4l2-ctl --list-ctrls failed: {e}")
            return {}
        if r.returncode != 0:
            logger.warning(f"v4l2-ctl --list-ctrls returned {r.returncode}: {r.stderr.strip()}")
            return {}

        out: dict = {}
        # Lines look like:
        # "                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32"
        # We only need the leading name, the type (parenthesised), and the
        # min/max/default/value fields. Other fields (flags=inactive) are
        # ignored.
        for line in r.stdout.splitlines():
            line = line.strip()
            if not line or ":" not in line:
                continue
            head, _, tail = line.partition(":")
            head_parts = head.split()
            if len(head_parts) < 3:
                continue
            name = head_parts[0]
            if name not in self._CTRL_ALLOWLIST:
                continue
            ctype = head_parts[-1].strip("()")
            info = {"type": ctype}
            for kv in tail.split():
                if "=" not in kv:
                    continue
                k, v = kv.split("=", 1)
                if k in ("min", "max", "default", "step", "value"):
                    try:
                        info[k] = int(v)
                    except ValueError:
                        info[k] = v
            out[name] = info
        return out

    def _handle_control_messages(self) -> None:
        """Drain pending REQ messages on the camera-control socket.

        Called once per capture loop iteration. Non-blocking — if no REQ
        is pending it returns immediately. Each REQ must be replied to
        (REP/REQ contract) before the next REQ can arrive, so we serve
        at most one per call but loop until the socket is drained.
        """
        if self.ctrl_socket is None:
            return
        for _ in range(8):  # cap loops to keep capture latency bounded
            try:
                raw = self.ctrl_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            except zmq.ZMQError:
                return

            reply: dict
            try:
                import msgpack
                req = msgpack.unpackb(raw, raw=False)
                if not isinstance(req, dict):
                    raise ValueError(f"expected dict, got {type(req).__name__}")
                op = req.get("op")
                if op == "get_ctrls":
                    reply = {"ok": True, "ctrls": self._v4l2_list_ctrls()}
                elif op == "set_ctrl":
                    name = req.get("name")
                    value = req.get("value")
                    if name not in self._CTRL_ALLOWLIST:
                        reply = {"ok": False, "error": f"control {name!r} not in allow-list"}
                    elif value is None:
                        reply = {"ok": False, "error": "missing 'value'"}
                    else:
                        ok, msg = self._v4l2_set(name, value)
                        reply = {"ok": ok, "name": name, "value": value, "msg": msg}
                        # Mirror into our local dict so the values survive
                        # subsequent stop/start cycles within the same node.
                        if ok:
                            self.v4l2_controls[name] = value
                else:
                    reply = {"ok": False, "error": f"unknown op {op!r}"}
            except Exception as e:
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}

            try:
                import msgpack
                self.ctrl_socket.send(msgpack.packb(reply, use_bin_type=True))
            except zmq.ZMQError as e:
                logger.warning(f"ctrl reply failed: {e}")
                return

    def _compress_color(self, bgr: np.ndarray) -> tuple[bytes, int, int]:
        """Apply flip / optional downscale, then JPEG-encode. Returns (bytes, w, h)."""
        if self.color_flip == "horizontal":
            bgr = cv2.flip(bgr, 1)
        elif self.color_flip == "vertical":
            bgr = cv2.flip(bgr, 0)
        elif self.color_flip == "180":
            bgr = cv2.rotate(bgr, cv2.ROTATE_180)

        if (self.output_w is not None and self.output_h is not None
                and (bgr.shape[1] != self.output_w or bgr.shape[0] != self.output_h)):
            bgr = cv2.resize(bgr, (int(self.output_w), int(self.output_h)),
                             interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes(), bgr.shape[1], bgr.shape[0]

    def process_frames(self):
        """Capture + publish until the device disappears (or we're told to stop).

        Returns to the supervisor (run) on device loss so it can reconnect — it does
        NOT loop forever on a dead handle or exit the process. ZMQ stays bound across
        reconnects, so the collector's subscriber just resumes when frames return."""
        logger.info(f"{self.camera_id}: capture loop started ({self._active_device})")

        # Warm up — first few frames are typically junk on UVC.
        for _ in range(5):
            self.cap.read()

        read_fails = 0
        while self.running:
            try:
                ok, frame_bgr = self.cap.read()
                if not ok or frame_bgr is None:
                    # A UVC unplug usually surfaces as steady read failures (NOT an
                    # exception), so count them and treat a run of them as device loss.
                    read_fails += 1
                    if read_fails >= self._READ_FAIL_LIMIT:
                        logger.warning(f"{self.camera_id}: {read_fails} consecutive read "
                                       f"failures — camera unplugged? reconnecting...")
                        self._release_capture()
                        return
                    time.sleep(0.02)
                    continue
                read_fails = 0

                timestamp = time.time()

                t_enc = time.perf_counter()
                color_jpeg, cw, ch = self._compress_color(frame_bgr)
                self._stat_encode_ms.append((time.perf_counter() - t_enc) * 1000.0)

                message: dict = {
                    "camera_id": self.camera_id,
                    "timestamp": timestamp,
                    "frame_number": self.frame_count,
                    "color": {
                        "format": "jpeg",
                        "width": cw,
                        "height": ch,
                    },
                }

                t_snd = time.perf_counter()
                self.zmq_socket.send_multipart(pack_camera(message, color_jpeg, None))
                self._stat_send_ms.append((time.perf_counter() - t_snd) * 1000.0)

                # Drain any pending camera-control REQs from the GUI.
                # Non-blocking; budget is ~one v4l2-ctl call (~20-50 ms) per
                # iteration if a slider event arrives — fine at 30 fps.
                self._handle_control_messages()

                self.frame_count += 1

                if timestamp - self.last_stats_time >= 5.0:
                    elapsed = timestamp - self.last_stats_time

                    def _s(arr: list) -> str:
                        if not arr:
                            return "n/a"
                        a = sorted(arr)
                        n = len(a)
                        return (f"mean={sum(a) / n:.1f} "
                                f"p99={a[min(n - 1, int(n * 0.99))]:.1f} "
                                f"max={a[-1]:.1f}")

                    logger.info(
                        f"{self.camera_id}: {self.frame_count} frames "
                        f"({self.frame_count / elapsed:.1f} fps)  "
                        f"encode_ms[{_s(self._stat_encode_ms)}]  "
                        f"send_ms[{_s(self._stat_send_ms)}]"
                    )
                    self.frame_count = 0
                    self.last_stats_time = timestamp
                    self._stat_encode_ms.clear()
                    self._stat_send_ms.clear()

            except KeyboardInterrupt:
                self.running = False
                return
            except Exception as e:
                # Some drivers DO raise on unplug — hand back to the supervisor to
                # reconnect rather than dying or busy-looping on a dead handle.
                logger.error(f"{self.camera_id}: capture error: {e}", exc_info=True)
                self._release_capture()
                return

    def _release_capture(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def cleanup(self):
        logger.info(f"Shutting down {self.camera_id}...")
        self._release_capture()
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.ctrl_socket:
            self.ctrl_socket.close()
        if self.zmq_context:
            self.zmq_context.term()

    def _open_with_retry(self) -> bool:
        """Block until the camera opens (returns True) or we're shutting down (False).
        Retries forever with backoff — this is what lets a replug recover on its own."""
        attempt = 0
        while self.running:
            try:
                self.initialize_camera()
                logger.info(f"{self.camera_id}: camera online ({self._active_device})")
                return True
            except Exception as e:
                self._release_capture()
                wait = self._REOPEN_BACKOFF_S[min(attempt, len(self._REOPEN_BACKOFF_S) - 1)]
                if attempt == 0:
                    logger.warning(f"{self.camera_id}: camera unavailable ({e}); "
                                   f"retrying until it's (re)plugged...")
                elif attempt % 15 == 0:
                    logger.info(f"{self.camera_id}: still waiting for camera ({e})")
                attempt += 1
                slept = 0.0
                while slept < wait and self.running:   # responsive to shutdown
                    time.sleep(0.1)
                    slept += 0.1
        return False

    def run(self):
        self.running = True
        try:
            self.initialize_zmq()          # bind ONCE; survives camera reconnects
            logger.info(f"Gripper camera node ready: {self.camera_id}")
            while self.running:
                if not self._open_with_retry():
                    break
                time.sleep(0.3)            # let the freshly-opened device settle
                self.process_frames()      # returns on device loss -> loop reopens
        except KeyboardInterrupt:
            logger.info("Interrupted")
        except Exception as e:
            logger.error(f"Fatal error: {e}", exc_info=True)
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description="AIZEE Gripper Camera Node (ELP UVC)")
    parser.add_argument(
        "--config",
        required=True,
        help="Path to hardware YAML (e.g. config/hardware_jetson_gripper_cam.yaml)",
    )
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    node = GripperCameraNode(config)

    def _stop(signum, _frame):
        logger.info(f"signal {signum} — shutting down {node.camera_id}")
        node.running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    node.run()


if __name__ == "__main__":
    main()
