#!/usr/bin/env python3
"""
AIZEE Arm Camera Node - Intel RealSense D435 streaming via ZeroMQ

Captures RGB and/or depth from D435 cameras USB-connected directly to the Jetson.
Both cameras run as separate processes (one per YAML config). No relay required —
the Jetson publishes directly on its WiFi interface.

Usage:
    python arm_camera_node.py --config config/hardware_jetson_arm_cam_left.yaml
    python arm_camera_node.py --config config/hardware_jetson_arm_cam_right.yaml
"""

import argparse
import base64
import io
import json
import logging
import signal
import sys
import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import yaml
import zmq
from PIL import Image


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ArmCameraNode:
    """Intel RealSense D435 node for arm-mounted cameras on the Jetson."""

    def __init__(self, config: dict):
        cam_cfg = config.get("camera", {})
        streams = cam_cfg.get("streams", {})
        color_cfg = streams.get("color", {})
        depth_cfg = streams.get("depth", {})
        zmq_cfg = config.get("network", {}).get("device", {}).get("zmq", {})

        self.camera_id: str = cam_cfg.get("id", "arm_cam_left")
        self.serial: Optional[str] = cam_cfg.get("serial") or None  # None = first available D435

        self.enable_color: bool = color_cfg.get("enabled", True)
        self.enable_depth: bool = depth_cfg.get("enabled", True)

        self.color_w: int = color_cfg.get("width", 640)
        self.color_h: int = color_cfg.get("height", 480)
        self.depth_w: int = depth_cfg.get("width", 640)
        self.depth_h: int = depth_cfg.get("height", 480)
        self.fps: int = color_cfg.get("fps", depth_cfg.get("fps", 30))
        self.jpeg_quality: int = color_cfg.get("quality", 85)

        self.zmq_endpoint: str = zmq_cfg.get("camera_pub", "tcp://*:5563")

        log_level = config.get("logging", {}).get("level", "INFO")
        logger.setLevel(getattr(logging, log_level, logging.INFO))

        self.pipeline: Optional[rs.pipeline] = None
        self.rs_config: Optional[rs.config] = None
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        self.running = False

        self.depth_intrinsics: Optional[dict] = None
        self.depth_scale: float = 0.001

        self.frame_count = 0
        self.last_stats_time = time.time()

    def initialize_camera(self):
        """Start the RealSense pipeline, selecting camera by serial if configured."""
        logger.info(
            f"Initializing D435: {self.camera_id} "
            f"(serial={self.serial or 'first available'}, "
            f"color={self.enable_color}, depth={self.enable_depth})"
        )

        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        if self.serial:
            self.rs_config.enable_device(self.serial)

        if self.enable_color:
            self.rs_config.enable_stream(
                rs.stream.color,
                self.color_w, self.color_h,
                rs.format.rgb8,
                self.fps,
            )

        if self.enable_depth:
            self.rs_config.enable_stream(
                rs.stream.depth,
                self.depth_w, self.depth_h,
                rs.format.z16,
                self.fps,
            )

        if not self.enable_color and not self.enable_depth:
            raise ValueError("At least one of color or depth must be enabled in config.")

        try:
            profile = self.pipeline.start(self.rs_config)
            device = profile.get_device()
            logger.info(f"Camera started: {device.get_info(rs.camera_info.name)} "
                        f"S/N {device.get_info(rs.camera_info.serial_number)}")

            if self.enable_depth:
                depth_stream = profile.get_stream(rs.stream.depth)
                di = depth_stream.as_video_stream_profile().get_intrinsics()
                self.depth_intrinsics = {
                    "fx": di.fx, "fy": di.fy,
                    "cx": di.ppx, "cy": di.ppy,
                    "width": di.width, "height": di.height,
                }
                self.depth_scale = device.first_depth_sensor().get_depth_scale()
                logger.info(
                    f"Depth: {di.width}x{di.height}, "
                    f"fx={di.fx:.2f}, fy={di.fy:.2f}, "
                    f"cx={di.ppx:.2f}, cy={di.ppy:.2f}, "
                    f"scale={self.depth_scale:.6f} m/unit"
                )

        except RuntimeError as e:
            logger.error(f"Failed to start camera pipeline: {e}")
            raise

    def initialize_zmq(self):
        """Bind ZeroMQ PUB socket."""
        logger.info(f"Binding ZMQ PUB to {self.zmq_endpoint}")
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.setsockopt(zmq.SNDHWM, 10)
        self.zmq_socket.bind(self.zmq_endpoint)

    def _compress_color(self, rgb: np.ndarray) -> bytes:
        img = Image.fromarray(rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=self.jpeg_quality, optimize=True)
        return buf.getvalue()

    def process_frames(self):
        """Main capture-and-publish loop."""
        logger.info("Starting frame capture loop")
        self.running = True

        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                timestamp = time.time()

                message: dict = {
                    "camera_id": self.camera_id,
                    "timestamp": timestamp,
                    "frame_number": self.frame_count,
                }

                if self.enable_color:
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        color_np = np.asanyarray(color_frame.get_data())
                        color_jpeg = self._compress_color(color_np)
                        message["color"] = {
                            "data": base64.b64encode(color_jpeg).decode("ascii"),
                            "format": "jpeg",
                            "width": self.color_w,
                            "height": self.color_h,
                        }

                if self.enable_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_np = np.asanyarray(depth_frame.get_data())
                        message["depth"] = {
                            "data": base64.b64encode(depth_np.tobytes()).decode("ascii"),
                            "format": "uint16",
                            "width": self.depth_w,
                            "height": self.depth_h,
                            "intrinsics": self.depth_intrinsics,
                            "scale": self.depth_scale,
                        }

                # Only publish if we have at least one stream's data
                if "color" in message or "depth" in message:
                    self.zmq_socket.send_json(message)

                self.frame_count += 1

                if timestamp - self.last_stats_time >= 5.0:
                    elapsed = timestamp - self.last_stats_time
                    logger.info(
                        f"{self.camera_id}: {self.frame_count} frames "
                        f"({self.frame_count / elapsed:.1f} fps)"
                    )
                    self.frame_count = 0
                    self.last_stats_time = timestamp

            except RuntimeError as e:
                logger.error(f"Frame capture error: {e}")
                logger.info("Attempting to reinitialize camera...")
                self._stop_pipeline()
                time.sleep(2)
                try:
                    self.initialize_camera()
                except Exception as reinit_err:
                    logger.error(f"Reinitialization failed: {reinit_err}")
                    self.running = False
                    break

            except Exception as e:
                logger.error(f"Unexpected error: {e}", exc_info=True)
                self.running = False
                break

    def _stop_pipeline(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass

    def cleanup(self):
        logger.info(f"Shutting down {self.camera_id}...")
        self._stop_pipeline()
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.zmq_context:
            self.zmq_context.term()

    def run(self):
        try:
            self.initialize_camera()
            self.initialize_zmq()
            time.sleep(1)  # Let ZMQ subscriber connections establish
            logger.info(f"Arm camera node ready: {self.camera_id}")
            self.process_frames()
        except KeyboardInterrupt:
            logger.info("Interrupted")
        except Exception as e:
            logger.error(f"Fatal error: {e}", exc_info=True)
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description="AIZEE Arm Camera Node (D435)")
    parser.add_argument(
        "--config",
        required=True,
        help="Path to hardware YAML config (e.g. config/hardware_jetson_arm_cam_left.yaml)",
    )
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))

    ArmCameraNode(config).run()


if __name__ == "__main__":
    main()
