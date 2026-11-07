#!/usr/bin/env python3
"""
AIZEE Camera Node - Intel RealSense D455 streaming via ZeroMQ

Captures RGB-D data and IMU from RealSense D455 camera and publishes
via ZeroMQ for consumption by the Rerun bridge and other nodes.

Usage:
    python camera_node.py --config config/hardware_jetson_scene_cam.yaml
"""

import argparse
import io
import json
import logging
import os
import signal
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import yaml
import zmq
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_camera


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Keeping the camera on USB 3 (Usb3Keeper).
RETRAIN_BACKOFF_S = (0.0, 30.0, 120.0, 600.0)  # Before a reset, by how many resets in a row did not hold.
RETRAIN_EXIT = 75  # EX_TEMPFAIL: the camera was reset and is re-attaching; systemd starts a fresh process.
SETTLE_S = 10.0  # A camera reset this recently may still be re-attaching: wait for it before resetting again.
POLL_S = 0.5
LOOK_S = 2.0  # While a reset waits, the camera is read again this often: a replug ends the wait within it.
LIVE_POLL_S = 5.0
NO_CAMERA_WAIT_S = 60.0  # With no camera at all, the process waits this long, then exits (systemd starts it again).
LIVE_RECEIVERS = ('guarded_controller', 'head_controller')  # Minerva's live receivers: no reset while one runs.
STOP_TIMEOUT_S = 2.0  # pipeline.stop() hangs inside librealsense once the camera is gone; the exit does not wait.


def _image_rotation(value: int) -> int:
    """Only whole half-turns preserve both native image dimensions."""
    if type(value) is not int or value not in (0, 180):
        raise ValueError('image_rotation_deg must be the integer 0 or 180')
    return value


def _rotate_rgbd(color: np.ndarray, depth: np.ndarray, rotation: int) -> tuple:
    """Rotate each native grid equally; this does not register depth to RGB."""
    if _image_rotation(rotation) == 0:
        return color, depth
    return (np.ascontiguousarray(color[::-1, ::-1]), np.ascontiguousarray(depth[::-1, ::-1]))


def _rotate_depth_intrinsics(intrinsics: dict, rotation: int) -> dict:
    """Return K'=A*K for the rotated raster, retaining sensor optical XYZ.

    At 180 degrees u'=width-1-u and v'=height-1-v. Signed focal lengths
    keep (u'-cx')/fx' == (u-cx)/fx, so existing depth backprojection and
    extrinsics remain in the original sensor frame. No mount calibration
    or optical-frame rotation is inferred from the image correction.
    """
    result = dict(intrinsics)
    if _image_rotation(rotation) == 180:
        result.update(
            fx=-intrinsics['fx'],
            fy=-intrinsics['fy'],
            cx=intrinsics['width'] - 1 - intrinsics['cx'],
            cy=intrinsics['height'] - 1 - intrinsics['cy'],
        )
    return result


def _color_registration(profile, depth_stream, rotation: int) -> tuple:
    """The color camera's intrinsics for the published raster (rotated as the depth's are) and the depth sensor's
    pose in the color camera's optical frame ({'rotation': 3x3 row-major, 'translation': metres}): p_color =
    R p_depth + t, both in the sensors' original optical frames, as the signed focal lengths keep them. (None, None)
    where the SDK gives either not: the stream goes on without them."""
    try:
        color_stream = profile.get_stream(rs.stream.color)
        ci = color_stream.as_video_stream_profile().get_intrinsics()
        intrinsics = _rotate_depth_intrinsics(
            {
                'fx': ci.fx,
                'fy': ci.fy,
                'cx': ci.ppx,
                'cy': ci.ppy,
                'width': ci.width,
                'height': ci.height,
            },
            rotation,
        )
        extrinsics = depth_stream.get_extrinsics_to(color_stream)
        rotation_matrix = [[float(extrinsics.rotation[col * 3 + row]) for col in range(3)] for row in range(3)]
        return intrinsics, {'rotation': rotation_matrix, 'translation': [float(v) for v in extrinsics.translation]}
    except (AttributeError, RuntimeError, TypeError, IndexError) as exc:
        logger.warning('No color registration from the SDK: %s', exc)
        return None, None


def _scaled_intrinsics(intrinsics: Optional[dict], width: int, height: int) -> Optional[dict]:
    """`intrinsics` for the raster scaled to width x height (pixel centres kept), or None."""
    if intrinsics is None:
        return None
    sx, sy = width / intrinsics['width'], height / intrinsics['height']
    return dict(
        intrinsics,
        fx=intrinsics['fx'] * sx,
        fy=intrinsics['fy'] * sy,
        cx=(intrinsics['cx'] + 0.5) * sx - 0.5,
        cy=(intrinsics['cy'] + 0.5) * sy - 0.5,
        width=width,
        height=height,
    )


def _usb_major(usb) -> Optional[int]:
    """The major USB version in a usb_type_descriptor ('3.2' -> 3, '2.1' -> 2), or None."""
    try:
        return int(str(usb).split('.')[0])
    except (TypeError, ValueError):
        return None


def _live_receiver_running(proc: Path = Path('/proc')) -> bool:
    """Whether one of Minerva's live receivers (LIVE_RECEIVERS) runs on this computer."""
    try:
        entries = list(proc.iterdir())
    except OSError:
        return False
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            argv0 = (entry / 'cmdline').read_bytes().split(b'\0', 1)[0].decode(errors='replace')
        except OSError:
            continue
        if Path(argv0).name in LIVE_RECEIVERS:
            return True
    return False


def _info(device, key) -> Optional[str]:
    """One of the camera's camera_info values, or None where the SDK cannot say."""
    try:
        return device.get_info(key)
    except RuntimeError:
        return None


def _first_camera():
    """The connected RealSense camera, or None."""
    try:
        devices = rs.context().query_devices()
        return devices[0] if len(devices) else None
    except RuntimeError as e:  # A camera mid re-attach can fail its enumeration.
        logger.debug(f'RealSense enumeration failed: {e}')
        return None


def _look_camera():
    """The connected camera and its USB link, (device, usb), or None."""
    device = _first_camera()
    return None if device is None else (device, _info(device, rs.camera_info.usb_type_descriptor))


class Usb3Keeper:
    """Keeps the camera streaming over USB 3 only.

    Its USB 3 link can drop mid-stream (Minerva's D435, 2026-10-24/25); the camera then re-attaches at USB 2 on the
    same port's USB 2 side, which on Minerva is the hub both CAN adapters share, so it never streams there. A camera
    found at USB 2 is reset: it reboots and re-attaches, which retrains USB 3 (a USB 2 port reset does not). Each
    recovery is a fresh process, so how many resets in a row did not hold, and when the last was sent, live in a
    small state file. The next reset waits RETRAIN_BACKOFF_S for that count, and none is sent while a live arm or
    head receiver runs: the reset re-attaches the camera on the CAN adapters' hub. A camera found back at USB 3 while
    a reset waits (replugged, or on a new cable) needs none: it streams at once.
    """

    def __init__(self, state_path, *, clock=time.time, sleep=time.sleep, live=_live_receiver_running):
        self.state_path = Path(state_path)
        self.clock, self.sleep, self.live = clock, sleep, live

    def _load(self) -> dict:
        try:
            state = json.loads(self.state_path.read_text())
            return {'failures': int(state['failures']), 'last_reset': float(state['last_reset'])}
        except (OSError, ValueError, TypeError, KeyError):
            return {'failures': 0, 'last_reset': 0.0}

    def _save(self, state: dict):
        try:
            self.state_path.write_text(json.dumps(state))
        except OSError as e:
            logger.warning(f"Could not record the camera's USB 3 recovery in {self.state_path}: {e}")

    def held(self, usb):
        """The camera is at USB 3: any reset before held."""
        state = self._load()
        if state['failures']:
            logger.info(f'Camera back at USB {usb} after {state["failures"]} reset(s)')
            self._save({'failures': 0, 'last_reset': state['last_reset']})

    def settling(self) -> float:
        """Seconds the last reset may still need to re-attach the camera."""
        return max(0.0, self._load()['last_reset'] + SETTLE_S - self.clock())

    def _wait(self, seconds, look):
        """Sleep `seconds`, reading the camera again every LOOK_S: True as soon as it needs no reset, back at USB 3
        or gone (unplugged)."""
        end = self.clock() + seconds
        while self.clock() < end:
            self.sleep(min(LOOK_S, end - self.clock()))
            found = look() if look is not None else ()
            if found is None:
                logger.info('Camera gone while its reset waited: looking for it again')
                return True
            if found and _usb_major(found[1]) != 2:
                logger.info(f'Camera back at USB {found[1]} by itself while its reset waited')
                return True
        return False

    def reset(self, device, usb, look=None):
        """The camera is at USB `usb` (2.x): wait out the backoff and any live receiver, then reset it; True once
        reset. `look()` reads the camera again ((device, usb), or None if there is none): a camera replugged or on a
        new cable meanwhile needs no reset, so the waits end as soon as it is back at USB 3 or gone (False). A blind
        backoff kept the head camera dark for 9 minutes after it went onto a new cable (2026-10-29)."""
        state = self._load()
        wait = (
            state['last_reset'] + RETRAIN_BACKOFF_S[min(state['failures'], len(RETRAIN_BACKOFF_S) - 1)] - self.clock()
        )
        if wait > 0:
            logger.warning(
                f'Camera at USB {usb}: {state["failures"]} reset(s) in a row did not hold it at USB 3; '
                f'the next in {wait:.0f} s, unless it comes back by itself'
            )
            if self._wait(wait, look):
                return False
        if self.live():
            logger.warning(
                f'Camera at USB {usb}: waiting for the live arm/head session to end before resetting it '
                "(the reset re-attaches it on the CAN adapters' hub)"
            )
            while self.live():
                if self._wait(LIVE_POLL_S, look):
                    return False
        if look is not None:
            found = look()  # The camera as it is now, not as it was before the waits.
            if found is None or _usb_major(found[1]) != 2:
                return False
            device, usb = found
        logger.warning(
            f'Camera at USB {usb} (its USB 3 link dropped): resetting it to re-attach at USB 3; it never '
            'streams over USB 2'
        )
        self._save({'failures': state['failures'] + 1, 'last_reset': self.clock()})
        try:
            device.hardware_reset()
        except RuntimeError as e:
            logger.error(f'Camera reset failed: {e}')
        return True


class CameraNode:
    """Intel RealSense D455 camera node with ZeroMQ publishing"""

    def __init__(
        self,
        camera_id: str = 'cam_front',
        zmq_endpoint: str = 'tcp://*:5557',
        color_resolution: tuple = (640, 480),
        depth_resolution: tuple = (640, 480),
        fps: int = 30,
        jpeg_quality: int = 85,
        output_resolution: Optional[tuple] = None,
        image_rotation_deg: int = 0,
        require_usb3: bool = True,
        usb3_keeper: Optional[Usb3Keeper] = None,
    ):
        """Initialize camera node

        Args:
            camera_id: Unique identifier for this camera
            zmq_endpoint: ZeroMQ endpoint to publish data
            color_resolution: (width, height) for color stream
            depth_resolution: (width, height) for depth stream
            fps: Frame rate for both streams
            jpeg_quality: JPEG compression quality (1-100)
            output_resolution: optional (width, height) to downscale color
                to before JPEG encode.  None = publish at capture resolution.
            image_rotation_deg: 0 or 180, applied to color and native depth.
            require_usb3: stream only over USB 3, resetting a camera found at USB 2 (Usb3Keeper).
            usb3_keeper: its recovery state; by default a file in the temporary directory.
        """
        self.camera_id = camera_id
        self.zmq_endpoint = zmq_endpoint
        self.color_resolution = color_resolution
        self.depth_resolution = depth_resolution
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.output_resolution = output_resolution
        self.image_rotation_deg = _image_rotation(image_rotation_deg)
        self.require_usb3 = require_usb3
        self.usb3_keeper = usb3_keeper or Usb3Keeper(Path(tempfile.gettempdir()) / f'aizee-{camera_id}-usb3.json')
        self.device_serial: Optional[str] = None  # The camera ensure_usb3 found; the pipeline opens that one.

        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        self.running = False

        # Depth calibration (populated on pipeline start)
        self.depth_intrinsics: Optional[dict] = None
        self.depth_scale: float = 0.001  # metres per uint16 unit (D455 default)
        # The color camera's projection at capture size, and where the depth sensor sits relative to it: a consumer
        # textures each depth point with its color pixel (Helm's headset view). None where the SDK gave none.
        self.color_intrinsics: Optional[dict] = None
        self.depth_to_color: Optional[dict] = None

        # Statistics
        self.frame_count = 0
        self.last_stats_time = time.time()
        self.last_frame_time = 0
        self._stat_drain_total = 0
        self._stat_drain_max = 0
        self._stat_encode_ms: list = []
        self._stat_send_ms: list = []
        self._stat_age_ms: list = []
        self._rs_epoch_ms: Optional[float] = None

    def ensure_usb3(self) -> bool:
        """Find the camera and keep it on USB 3 (Usb3Keeper). True once it is at USB 3, to stream from; False after
        resetting it from USB 2 (the process exits, and systemd starts a fresh one once it re-attaches)."""
        keeper = self.usb3_keeper
        deadline = keeper.clock() + NO_CAMERA_WAIT_S
        while True:
            device = _first_camera()
            usb = _info(device, rs.camera_info.usb_type_descriptor) if device is not None else None
            if device is not None and _usb_major(usb) != 2:  # Streams when the SDK cannot tell, too.
                self.device_serial = _info(device, rs.camera_info.serial_number)
                logger.info(f'Camera {self.device_serial} at USB {usb}')
                keeper.held(usb)
                return True
            if device is not None and keeper.settling() <= 0:
                if keeper.reset(device, usb, look=_look_camera):
                    return False
                # Back at USB 3, or unplugged, while the reset waited: look again, a gone camera for as long anew.
                deadline = keeper.clock() + NO_CAMERA_WAIT_S
                continue
            if keeper.clock() >= deadline:
                raise RuntimeError(
                    'No RealSense camera connected'
                    if device is None
                    else f'The camera stayed at USB {usb} after its reset'
                )
            keeper.sleep(POLL_S)

    def initialize_camera(self):
        """Initialize RealSense pipeline and configure streams"""
        logger.info(f'Initializing RealSense camera: {self.camera_id}')

        # Create pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if self.device_serial is not None:
            self.config.enable_device(self.device_serial)

        # Configure streams
        self.config.enable_stream(
            rs.stream.color, self.color_resolution[0], self.color_resolution[1], rs.format.rgb8, self.fps
        )
        self.config.enable_stream(
            rs.stream.depth, self.depth_resolution[0], self.depth_resolution[1], rs.format.z16, self.fps
        )

        # Enable IMU streams - TEMPORARILY DISABLED FOR TESTING
        # try:
        #     self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        #     self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        #     logger.info("IMU streams enabled (200Hz)")
        # except RuntimeError as e:
        #     logger.warning(f"Could not enable IMU streams: {e}")
        logger.info('IMU streams disabled for testing')

        # Start pipeline
        try:
            profile = self.pipeline.start(self.config)
            logger.info('Camera pipeline started successfully')

            # Get depth intrinsics (used by rerun_bridge to compute pointclouds)
            depth_stream = profile.get_stream(rs.stream.depth)
            di = depth_stream.as_video_stream_profile().get_intrinsics()
            self.depth_intrinsics = _rotate_depth_intrinsics(
                {
                    'fx': di.fx,
                    'fy': di.fy,
                    'cx': di.ppx,
                    'cy': di.ppy,
                    'width': di.width,
                    'height': di.height,
                },
                self.image_rotation_deg,
            )
            logger.info(
                f'Depth intrinsics: {di.width}x{di.height}, '
                f'fx={di.fx:.2f}, fy={di.fy:.2f}, '
                f'cx={di.ppx:.2f}, cy={di.ppy:.2f}'
            )

            self.color_intrinsics, self.depth_to_color = _color_registration(
                profile, depth_stream, self.image_rotation_deg
            )

            # Get depth scale (metres per uint16 unit; typically 0.001 for D455)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            logger.info(f'Depth scale: {self.depth_scale:.6f} m/unit')
            logger.info(
                'Published image rotation: %d degrees; depth XYZ remains in sensor optical frame',
                self.image_rotation_deg,
            )

        except RuntimeError as e:
            logger.error(f'Failed to start camera pipeline: {e}')
            raise

    def initialize_zmq(self):
        """Initialize ZeroMQ publisher socket"""
        logger.info(f'Initializing ZeroMQ publisher on {self.zmq_endpoint}')

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.setsockopt(zmq.SNDHWM, 10)  # High water mark
        self.zmq_socket.bind(self.zmq_endpoint)

        logger.info('ZeroMQ publisher initialized')

    def compress_color_image(self, color_image: np.ndarray) -> tuple:
        """Compress RGB image to JPEG.

        Returns (bytes, width, height) — width/height reflect the encoded
        size after the optional output_resolution downscale.
        """
        img = Image.fromarray(color_image)
        if self.output_resolution is not None and (
            self.output_resolution[0] != img.width or self.output_resolution[1] != img.height
        ):
            img = img.resize(self.output_resolution, Image.BILINEAR)
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG', quality=self.jpeg_quality)
        return buffer.getvalue(), img.width, img.height

    def process_frames(self):
        """Main processing loop - capture and publish frames"""
        logger.info('Starting frame processing loop')
        self.running = True

        while self.running:
            try:
                # Wait, then drain to the newest frame so latency cannot
                # accumulate when encoding/network momentarily falls behind.
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                drained = 0
                while True:
                    try:
                        newer = self.pipeline.poll_for_frames()
                    except Exception:
                        break
                    if not newer:
                        break
                    frames = newer
                    drained += 1
                self._stat_drain_total += drained
                if drained > self._stat_drain_max:
                    self._stat_drain_max = drained

                timestamp = time.time()

                rs_ts_ms = float(frames.get_timestamp())
                if rs_ts_ms > 0 and self._rs_epoch_ms is None:
                    self._rs_epoch_ms = timestamp * 1000.0 - rs_ts_ms
                if self._rs_epoch_ms is not None:
                    self._stat_age_ms.append((timestamp * 1000.0) - (rs_ts_ms + self._rs_epoch_ms))

                # Get color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    logger.warning('No color frame received')
                    continue

                # Get depth frame
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    logger.warning('No depth frame received')
                    continue

                t_enc = time.perf_counter()
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image, depth_image = _rotate_rgbd(color_image, depth_image, self.image_rotation_deg)

                # Compress color image
                color_jpeg, _cw, _ch = self.compress_color_image(color_image)
                depth_bytes = depth_image.tobytes()

                # Prepare message header (raw bytes go in separate ZMQ frames)
                message = {
                    'camera_id': self.camera_id,
                    'timestamp': timestamp,
                    'frame_number': self.frame_count,
                    'image_rotation_deg': self.image_rotation_deg,
                    'color': {
                        'format': 'jpeg',
                        'width': _cw,
                        'height': _ch,
                    },
                    'depth': {
                        'format': 'uint16',
                        'width': depth_image.shape[1],
                        'height': depth_image.shape[0],
                        'intrinsics': self.depth_intrinsics,
                        'intrinsics_frame': 'sensor_optical',
                        'scale': self.depth_scale,
                    },
                }
                # Registration, where the SDK gave it: the color raster's intrinsics as encoded, and the depth
                # sensor's pose in the color camera's optical frame.
                color_k = _scaled_intrinsics(self.color_intrinsics, _cw, _ch)
                if color_k is not None and self.depth_to_color is not None:
                    message['color']['intrinsics'] = color_k
                    message['depth']['to_color'] = self.depth_to_color

                # Try to get IMU data (non-blocking)
                try:
                    accel_frame = frames.first_or_default(rs.stream.accel)
                    gyro_frame = frames.first_or_default(rs.stream.gyro)

                    if accel_frame:
                        accel_data = accel_frame.as_motion_frame().get_motion_data()
                        message['imu'] = {'accel': [accel_data.x, accel_data.y, accel_data.z]}

                    if gyro_frame:
                        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                        if 'imu' not in message:
                            message['imu'] = {}
                        message['imu']['gyro'] = [gyro_data.x, gyro_data.y, gyro_data.z]

                except Exception as e:
                    logger.debug(f'IMU data not available: {e}')
                self._stat_encode_ms.append((time.perf_counter() - t_enc) * 1000.0)

                # Publish via ZeroMQ multipart (header + raw JPEG + raw depth)
                t_snd = time.perf_counter()
                self.zmq_socket.send_multipart(pack_camera(message, color_jpeg, depth_bytes))
                self._stat_send_ms.append((time.perf_counter() - t_snd) * 1000.0)

                self.frame_count += 1
                self.last_frame_time = timestamp

                # Print statistics every 5 seconds
                if timestamp - self.last_stats_time >= 5.0:
                    elapsed = timestamp - self.last_stats_time
                    current_fps = self.frame_count / elapsed

                    def _s(arr: list) -> str:
                        if not arr:
                            return 'n/a'
                        a = sorted(arr)
                        n = len(a)
                        return f'mean={sum(a) / n:.1f} p99={a[min(n - 1, int(n * 0.99))]:.1f} max={a[-1]:.1f}'

                    try:
                        import resource as _resource

                        rss_kb = _resource.getrusage(_resource.RUSAGE_SELF).ru_maxrss
                    except Exception:
                        rss_kb = -1

                    logger.info(
                        f'Published {self.frame_count} frames in {elapsed:.1f}s '
                        f'({current_fps:.1f} fps)  '
                        f'drained={self._stat_drain_total} (max {self._stat_drain_max})  '
                        f'encode_ms[{_s(self._stat_encode_ms)}]  '
                        f'send_ms[{_s(self._stat_send_ms)}]  '
                        f'age_ms[{_s(self._stat_age_ms)}]  '
                        f'rss={rss_kb}kB'
                    )
                    self.frame_count = 0
                    self.last_stats_time = timestamp
                    self._stat_drain_total = 0
                    self._stat_drain_max = 0
                    self._stat_encode_ms.clear()
                    self._stat_send_ms.clear()
                    self._stat_age_ms.clear()

            except RuntimeError as e:
                # IMPORTANT: do NOT try to reinit in-process. librealsense's
                # RSUSB backend leaks worker threads holding libusb handles
                # on every `pipeline.stop() + pipeline.start()` cycle, and
                # those zombie threads keep poking the USB device with
                # "usbfs: did not claim interface 0 before use", which then
                # starves the *next* publisher of frames. Verified on the
                # Jetson Orin Nano with D455 firmware 5.17.0.10 — each
                # in-process reinit shortens the next stable streaming
                # window until the timeout becomes permanent.
                #
                # Instead: exit non-zero so systemd's Restart=on-failure
                # gets us a fresh process with zero zombie state.
                logger.error(f'Frame capture error: {e}')
                logger.error('Exiting so systemd restarts cleanly (avoids librealsense zombie-thread leak).')
                self.running = False
                raise SystemExit(1)

            except Exception as e:
                logger.error(f'Unexpected error in processing loop: {e}')
                self.running = False
                raise SystemExit(1)

    def cleanup(self):
        """Clean up resources. pipeline.stop() hangs inside librealsense once the camera is gone (unplugged, or its
        USB 3 link dropped), so it gets STOP_TIMEOUT_S; main then leaves with os._exit, which no SDK thread holds up."""
        logger.info('Cleaning up camera node...')

        if self.pipeline:
            outcome = []

            def stop():
                try:
                    self.pipeline.stop()
                    outcome.append(None)
                except Exception as e:
                    outcome.append(e)

            stopper = threading.Thread(target=stop, name='pipeline-stop', daemon=True)
            stopper.start()
            stopper.join(STOP_TIMEOUT_S)
            if not outcome:
                logger.warning(
                    f'Camera pipeline did not stop within {STOP_TIMEOUT_S:.0f} s (the camera is gone?); '
                    'exiting without it'
                )
            elif outcome[0] is None:
                logger.info('Camera pipeline stopped')
            else:
                logger.error(f'Error stopping pipeline: {outcome[0]}')

        if self.zmq_socket:
            self.zmq_socket.close(linger=0)

        if self.zmq_context:
            self.zmq_context.term()

        logger.info('Cleanup complete')

    def run(self) -> int:
        """Initialize and stream until stopped. Returns the process's exit code: 0 stopped, 1 failed, RETRAIN_EXIT
        after resetting the camera from USB 2; systemd starts the node again after either of the last two."""
        code = 0
        try:
            if self.require_usb3 and not self.ensure_usb3():
                return RETRAIN_EXIT
            self.initialize_camera()
            self.initialize_zmq()

            # Give ZeroMQ time to establish connections
            time.sleep(1)

            logger.info('Camera node running. Press Ctrl+C to stop.')
            self.process_frames()

        except KeyboardInterrupt:
            logger.info('Received interrupt signal')
        except SystemExit as e:  # SIGTERM (0), or a capture error (1): process_frames never reinitializes in-process.
            code = e.code if isinstance(e.code, int) else 1
        except Exception as e:
            logger.error(f'Fatal error: {e}', exc_info=True)
            code = 1
        finally:
            self.cleanup()
        return code


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    logger.info(f'Received signal {signum}, shutting down...')
    sys.exit(0)


def _kwargs_from_config(config: dict) -> dict:
    """Translate a hardware_jetson_*_cam.yaml dict into CameraNode kwargs.

    Mirrors the shape used by gripper_camera_node.py so the two configs
    look consistent on disk. Only fields the SDK pipeline actually
    consumes are pulled out — IMU / spatial / runtime-control hints are
    parsed by the wrapping service or future consumers, not here.
    """
    cam_cfg = config.get('camera', {}) or {}
    streams = cam_cfg.get('streams', {}) or {}
    color = streams.get('color', {}) or {}
    depth = streams.get('depth', {}) or {}
    zmq_cfg = (config.get('network', {}).get('device', {}).get('zmq', {})) or {}

    kwargs = {
        'camera_id': cam_cfg.get('id', 'scene_cam'),
        'zmq_endpoint': zmq_cfg.get('camera_pub', 'tcp://*:5564'),
        'color_resolution': (int(color.get('width', 640)), int(color.get('height', 480))),
        'depth_resolution': (int(depth.get('width', 640)), int(depth.get('height', 480))),
        'fps': int(color.get('fps', 30)),
        'jpeg_quality': int(color.get('quality', 85)),
        'image_rotation_deg': cam_cfg.get('image_rotation_deg', 0),
        'require_usb3': cam_cfg.get('require_usb3', True) is not False,
    }

    out_w = color.get('output_width')
    out_h = color.get('output_height')
    if out_w is not None and out_h is not None:
        kwargs['output_resolution'] = (int(out_w), int(out_h))

    return kwargs


def main() -> int:
    """Main entry point; returns the process's exit code (CameraNode.run)."""
    parser = argparse.ArgumentParser(description='AIZEE Camera Node')
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to a hardware_jetson_*_cam.yaml (camera.id / streams / '
        'network.device.zmq.camera_pub). Overrides individual flags '
        'below.',
    )
    parser.add_argument('--camera-id', type=str, default='cam_front', help='Unique camera identifier')
    parser.add_argument('--zmq-endpoint', type=str, default='tcp://*:5557', help='ZeroMQ endpoint to publish data')
    parser.add_argument('--fps', type=int, default=30, help='Frame rate for camera streams')
    parser.add_argument('--jpeg-quality', type=int, default=85, help='JPEG compression quality (1-100)')
    parser.add_argument(
        '--image-rotation-deg',
        type=int,
        choices=(0, 180),
        default=0,
        help='Rotate both RGB and native depth by 0 or 180 degrees',
    )
    parser.add_argument(
        '--allow-usb2',
        action='store_true',
        help='Stream over USB 2 too (by default a camera found at USB 2 is reset back to USB 3)',
    )
    parser.add_argument(
        '--output-size',
        type=str,
        default=None,
        help='Optional WxH to downscale color before JPEG encode (e.g. 320x240)',
    )

    args = parser.parse_args()
    output_resolution = None
    if args.output_size:
        try:
            _w, _h = args.output_size.lower().split('x')
            output_resolution = (int(_w), int(_h))
        except ValueError:
            parser.error('--output-size must be WxH (e.g. 320x240)')

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # YAML config wins over individual flags when provided — the systemd
    # unit ships a config so all tuning lives in one file on the Jetson.
    if args.config:
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
        log_level = (cfg.get('logging', {}) or {}).get('level', 'INFO')
        logger.setLevel(getattr(logging, log_level, logging.INFO))
        kwargs = _kwargs_from_config(cfg)
    else:
        kwargs = {
            'camera_id': args.camera_id,
            'zmq_endpoint': args.zmq_endpoint,
            'fps': args.fps,
            'jpeg_quality': args.jpeg_quality,
            'output_resolution': output_resolution,
            'image_rotation_deg': args.image_rotation_deg,
            'require_usb3': not args.allow_usb2,
        }

    node = CameraNode(**kwargs)
    return node.run()


if __name__ == '__main__':
    exit_code = main()
    logging.shutdown()
    # Not sys.exit: interpreter shutdown can hang in librealsense's destructors once the camera is gone.
    os._exit(exit_code if isinstance(exit_code, int) else 1)
