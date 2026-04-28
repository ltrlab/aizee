#!/usr/bin/env python3
"""
AIZEE Camera Node - Intel RealSense D455 streaming via ZeroMQ

Captures RGB-D data and IMU from RealSense D455 camera and publishes
via ZeroMQ for consumption by the Rerun bridge and other nodes.

Usage:
    python camera_node.py --config config/hardware.yaml
"""

import argparse
import base64
import io
import json
import logging
import signal
import sys
import time
from datetime import datetime
from typing import Optional

import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq
from PIL import Image


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CameraNode:
    """Intel RealSense D455 camera node with ZeroMQ publishing"""

    def __init__(
        self,
        camera_id: str = "cam_front",
        zmq_endpoint: str = "tcp://*:5557",
        color_resolution: tuple = (640, 480),
        depth_resolution: tuple = (640, 480),
        fps: int = 30,
        jpeg_quality: int = 85,
    ):
        """Initialize camera node

        Args:
            camera_id: Unique identifier for this camera
            zmq_endpoint: ZeroMQ endpoint to publish data
            color_resolution: (width, height) for color stream
            depth_resolution: (width, height) for depth stream
            fps: Frame rate for both streams
            jpeg_quality: JPEG compression quality (1-100)
        """
        self.camera_id = camera_id
        self.zmq_endpoint = zmq_endpoint
        self.color_resolution = color_resolution
        self.depth_resolution = depth_resolution
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        self.running = False

        # Depth calibration (populated on pipeline start)
        self.depth_intrinsics: Optional[dict] = None
        self.depth_scale: float = 0.001  # metres per uint16 unit (D455 default)

        # Statistics
        self.frame_count = 0
        self.last_stats_time = time.time()
        self.last_frame_time = 0
        self._stat_drain_total = 0
        self._stat_drain_max   = 0
        self._stat_encode_ms: list = []
        self._stat_send_ms:   list = []
        self._stat_age_ms:    list = []
        self._rs_epoch_ms: Optional[float] = None

    def initialize_camera(self):
        """Initialize RealSense pipeline and configure streams"""
        logger.info(f"Initializing RealSense camera: {self.camera_id}")

        # Create pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams
        self.config.enable_stream(
            rs.stream.color,
            self.color_resolution[0],
            self.color_resolution[1],
            rs.format.rgb8,
            self.fps
        )
        self.config.enable_stream(
            rs.stream.depth,
            self.depth_resolution[0],
            self.depth_resolution[1],
            rs.format.z16,
            self.fps
        )

        # Enable IMU streams - TEMPORARILY DISABLED FOR TESTING
        # try:
        #     self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        #     self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        #     logger.info("IMU streams enabled (200Hz)")
        # except RuntimeError as e:
        #     logger.warning(f"Could not enable IMU streams: {e}")
        logger.info("IMU streams disabled for testing")

        # Start pipeline
        try:
            profile = self.pipeline.start(self.config)
            logger.info("Camera pipeline started successfully")

            # Get depth intrinsics (used by rerun_bridge to compute pointclouds)
            depth_stream = profile.get_stream(rs.stream.depth)
            di = depth_stream.as_video_stream_profile().get_intrinsics()
            self.depth_intrinsics = {
                "fx": di.fx, "fy": di.fy,
                "cx": di.ppx, "cy": di.ppy,
                "width": di.width, "height": di.height,
            }
            logger.info(
                f"Depth intrinsics: {di.width}x{di.height}, "
                f"fx={di.fx:.2f}, fy={di.fy:.2f}, "
                f"cx={di.ppx:.2f}, cy={di.ppy:.2f}"
            )

            # Get depth scale (metres per uint16 unit; typically 0.001 for D455)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            logger.info(f"Depth scale: {self.depth_scale:.6f} m/unit")

        except RuntimeError as e:
            logger.error(f"Failed to start camera pipeline: {e}")
            raise

    def initialize_zmq(self):
        """Initialize ZeroMQ publisher socket"""
        logger.info(f"Initializing ZeroMQ publisher on {self.zmq_endpoint}")

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.setsockopt(zmq.SNDHWM, 10)  # High water mark
        self.zmq_socket.bind(self.zmq_endpoint)

        logger.info("ZeroMQ publisher initialized")

    def compress_color_image(self, color_image: np.ndarray) -> bytes:
        """Compress RGB image to JPEG

        Args:
            color_image: RGB numpy array (H, W, 3)

        Returns:
            JPEG compressed bytes
        """
        img = Image.fromarray(color_image)
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG', quality=self.jpeg_quality)
        return buffer.getvalue()

    def process_frames(self):
        """Main processing loop - capture and publish frames"""
        logger.info("Starting frame processing loop")
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
                    self._stat_age_ms.append(
                        (timestamp * 1000.0) - (rs_ts_ms + self._rs_epoch_ms)
                    )

                # Get color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    logger.warning("No color frame received")
                    continue

                # Get depth frame
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    logger.warning("No depth frame received")
                    continue

                t_enc = time.perf_counter()
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # Compress color image
                color_jpeg = self.compress_color_image(color_image)

                # Prepare message
                message = {
                    'camera_id': self.camera_id,
                    'timestamp': timestamp,
                    'frame_number': self.frame_count,
                    'color': {
                        'data': base64.b64encode(color_jpeg).decode('ascii'),
                        'format': 'jpeg',
                        'width': self.color_resolution[0],
                        'height': self.color_resolution[1],
                    },
                    'depth': {
                        'data': base64.b64encode(depth_image.tobytes()).decode('ascii'),
                        'format': 'uint16',
                        'width': self.depth_resolution[0],
                        'height': self.depth_resolution[1],
                        'intrinsics': self.depth_intrinsics,
                        'scale': self.depth_scale,
                    }
                }

                # Try to get IMU data (non-blocking)
                try:
                    accel_frame = frames.first_or_default(rs.stream.accel)
                    gyro_frame = frames.first_or_default(rs.stream.gyro)

                    if accel_frame:
                        accel_data = accel_frame.as_motion_frame().get_motion_data()
                        message['imu'] = {
                            'accel': [accel_data.x, accel_data.y, accel_data.z]
                        }

                    if gyro_frame:
                        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                        if 'imu' not in message:
                            message['imu'] = {}
                        message['imu']['gyro'] = [gyro_data.x, gyro_data.y, gyro_data.z]

                except Exception as e:
                    logger.debug(f"IMU data not available: {e}")
                self._stat_encode_ms.append((time.perf_counter() - t_enc) * 1000.0)

                # Publish via ZeroMQ
                t_snd = time.perf_counter()
                self.zmq_socket.send_json(message)
                self._stat_send_ms.append((time.perf_counter() - t_snd) * 1000.0)

                self.frame_count += 1
                self.last_frame_time = timestamp

                # Print statistics every 5 seconds
                if timestamp - self.last_stats_time >= 5.0:
                    elapsed = timestamp - self.last_stats_time
                    current_fps = self.frame_count / elapsed

                    def _s(arr: list) -> str:
                        if not arr:
                            return "n/a"
                        a = sorted(arr)
                        n = len(a)
                        return (f"mean={sum(a) / n:.1f} "
                                f"p99={a[min(n - 1, int(n * 0.99))]:.1f} "
                                f"max={a[-1]:.1f}")

                    try:
                        import resource as _resource
                        rss_kb = _resource.getrusage(_resource.RUSAGE_SELF).ru_maxrss
                    except Exception:
                        rss_kb = -1

                    logger.info(
                        f"Published {self.frame_count} frames in {elapsed:.1f}s "
                        f"({current_fps:.1f} fps)  "
                        f"drained={self._stat_drain_total} (max {self._stat_drain_max})  "
                        f"encode_ms[{_s(self._stat_encode_ms)}]  "
                        f"send_ms[{_s(self._stat_send_ms)}]  "
                        f"age_ms[{_s(self._stat_age_ms)}]  "
                        f"rss={rss_kb}kB"
                    )
                    self.frame_count = 0
                    self.last_stats_time = timestamp
                    self._stat_drain_total = 0
                    self._stat_drain_max   = 0
                    self._stat_encode_ms.clear()
                    self._stat_send_ms.clear()
                    self._stat_age_ms.clear()

            except RuntimeError as e:
                logger.error(f"Frame capture error: {e}")
                logger.info("Attempting to reinitialize camera...")
                self.cleanup()
                time.sleep(2)
                try:
                    self.initialize_camera()
                    self.initialize_zmq()
                except Exception as reinit_error:
                    logger.error(f"Failed to reinitialize: {reinit_error}")
                    self.running = False
                    break

            except Exception as e:
                logger.error(f"Unexpected error in processing loop: {e}")
                self.running = False
                break

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up camera node...")

        if self.pipeline:
            try:
                self.pipeline.stop()
                logger.info("Camera pipeline stopped")
            except Exception as e:
                logger.error(f"Error stopping pipeline: {e}")

        if self.zmq_socket:
            self.zmq_socket.close()

        if self.zmq_context:
            self.zmq_context.term()

        logger.info("Cleanup complete")

    def run(self):
        """Main run method - initialize and start processing"""
        try:
            self.initialize_camera()
            self.initialize_zmq()

            # Give ZeroMQ time to establish connections
            time.sleep(1)

            logger.info("Camera node running. Press Ctrl+C to stop.")
            self.process_frames()

        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error(f"Fatal error: {e}", exc_info=True)
        finally:
            self.cleanup()


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    logger.info(f"Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='AIZEE Camera Node')
    parser.add_argument(
        '--camera-id',
        type=str,
        default='cam_front',
        help='Unique camera identifier'
    )
    parser.add_argument(
        '--zmq-endpoint',
        type=str,
        default='tcp://*:5557',
        help='ZeroMQ endpoint to publish data'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=30,
        help='Frame rate for camera streams'
    )
    parser.add_argument(
        '--jpeg-quality',
        type=int,
        default=85,
        help='JPEG compression quality (1-100)'
    )

    args = parser.parse_args()

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create and run camera node
    node = CameraNode(
        camera_id=args.camera_id,
        zmq_endpoint=args.zmq_endpoint,
        fps=args.fps,
        jpeg_quality=args.jpeg_quality
    )

    node.run()


if __name__ == '__main__':
    main()
