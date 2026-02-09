#!/usr/bin/env python3
"""
AIZEE Camera Node - Intel RealSense D455 via OpenCV/V4L2

Alternative implementation using OpenCV for cameras where librealsense2
has compatibility issues. Accesses RGB-D streams directly via V4L2.

Usage:
    python camera_node_opencv.py --camera-id cam_front
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

import cv2
import msgpack
import numpy as np
import zmq
from PIL import Image


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CameraNodeOpenCV:
    """Intel RealSense D455 camera node using OpenCV/V4L2 backend"""

    def __init__(
        self,
        camera_id: str = "cam_front",
        zmq_endpoint: str = "tcp://*:5557",
        color_device: int = 4,  # /dev/video4 for RealSense RGB
        depth_device: int = 2,  # /dev/video2 for RealSense infrared (grayscale depth proxy)
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        jpeg_quality: int = 50,  # Lower quality for faster compression and transmission
        flip_vertical: bool = False,
        flip_horizontal: bool = False,
    ):
        """Initialize camera node

        Args:
            camera_id: Unique identifier for this camera
            zmq_endpoint: ZeroMQ endpoint to publish data
            color_device: V4L2 device number for color stream
            depth_device: V4L2 device number for depth stream
            width: Frame width
            height: Frame height
            fps: Frame rate
            jpeg_quality: JPEG compression quality (1-100)
        """
        self.camera_id = camera_id
        self.zmq_endpoint = zmq_endpoint
        self.color_device = color_device
        self.depth_device = depth_device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.flip_vertical = flip_vertical
        self.flip_horizontal = flip_horizontal

        self.color_cap: Optional[cv2.VideoCapture] = None
        self.depth_cap: Optional[cv2.VideoCapture] = None
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        self.running = False

        # Statistics
        self.frame_count = 0
        self.last_stats_time = time.time()

    def initialize_cameras(self):
        """Initialize OpenCV video captures for color and depth"""
        logger.info(f"Initializing cameras: {self.camera_id}")

        # Open color camera (YUYV format)
        logger.info(f"Opening color camera: /dev/video{self.color_device}")
        self.color_cap = cv2.VideoCapture(self.color_device, cv2.CAP_V4L2)

        if not self.color_cap.isOpened():
            raise RuntimeError(f"Failed to open color camera /dev/video{self.color_device}")

        # Set color camera properties
        self.color_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self.color_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.color_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.color_cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_width = self.color_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.color_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.color_cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"Color camera: {actual_width}x{actual_height} @ {actual_fps} fps")

        # Open depth camera (Z16 format - 16-bit depth)
        logger.info(f"Opening depth camera: /dev/video{self.depth_device}")
        self.depth_cap = cv2.VideoCapture(self.depth_device, cv2.CAP_V4L2)

        if not self.depth_cap.isOpened():
            raise RuntimeError(f"Failed to open depth camera /dev/video{self.depth_device}")

        # Set depth camera properties
        # Note: Z16 format support varies, OpenCV may use GREY instead
        self.depth_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.depth_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.depth_cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_width = self.depth_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.depth_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.depth_cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"Depth camera: {actual_width}x{actual_height} @ {actual_fps} fps")

        logger.info("Cameras initialized successfully")

    def initialize_zmq(self):
        """Initialize ZeroMQ publisher socket"""
        logger.info(f"Initializing ZeroMQ publisher on {self.zmq_endpoint}")

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)

        # Optimize for low latency
        self.zmq_socket.setsockopt(zmq.SNDHWM, 2)  # Low high water mark to drop old frames
        self.zmq_socket.setsockopt(zmq.SNDBUF, 1024 * 1024)  # 1MB send buffer

        self.zmq_socket.bind(self.zmq_endpoint)

        logger.info("ZeroMQ publisher initialized (optimized for low latency)")

    def compress_color_image(self, color_image: np.ndarray) -> bytes:
        """Compress RGB image to JPEG

        Args:
            color_image: RGB numpy array (H, W, 3)

        Returns:
            JPEG compressed bytes
        """
        # Use OpenCV for faster JPEG encoding
        ret, buffer = cv2.imencode('.jpg', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return buffer.tobytes()

    def process_frames(self):
        """Main processing loop - capture and publish frames"""
        logger.info("Starting frame processing loop")
        self.running = True

        # Warm up cameras
        for _ in range(10):
            self.color_cap.read()
            self.depth_cap.read()

        while self.running:
            try:
                timestamp = time.time()

                # Read color frame
                ret_color, color_frame = self.color_cap.read()
                if not ret_color or color_frame is None:
                    logger.warning("Failed to read color frame")
                    time.sleep(0.01)
                    continue

                # OpenCV automatically converts YUYV to BGR, so just convert BGR to RGB
                color_rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)

                # Apply orientation corrections if needed
                if self.flip_vertical and self.flip_horizontal:
                    # Both flips = 180 degree rotation
                    color_rgb = cv2.rotate(color_rgb, cv2.ROTATE_180)
                elif self.flip_vertical:
                    color_rgb = cv2.flip(color_rgb, 0)  # Flip around x-axis
                elif self.flip_horizontal:
                    color_rgb = cv2.flip(color_rgb, 1)  # Flip around y-axis

                # Read depth frame
                ret_depth, depth_frame = self.depth_cap.read()
                if not ret_depth or depth_frame is None:
                    logger.warning("Failed to read depth frame")
                    time.sleep(0.01)
                    continue

                # Infrared frame is GREY format (8-bit grayscale), convert to uint16 for consistency
                if len(depth_frame.shape) == 3:
                    # If BGR, convert to grayscale first
                    depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)

                # Keep as uint8 for infrared, or scale to uint16 for depth-like representation
                # For now, keep as uint8 to reduce bandwidth
                depth_frame = depth_frame.astype(np.uint8)

                # Apply same orientation corrections to infrared
                if self.flip_vertical and self.flip_horizontal:
                    depth_frame = cv2.rotate(depth_frame, cv2.ROTATE_180)
                elif self.flip_vertical:
                    depth_frame = cv2.flip(depth_frame, 0)
                elif self.flip_horizontal:
                    depth_frame = cv2.flip(depth_frame, 1)

                # Compress color image
                color_jpeg = self.compress_color_image(color_rgb)

                # Prepare message
                message = {
                    'camera_id': self.camera_id,
                    'timestamp': timestamp,
                    'frame_number': self.frame_count,
                    'color': {
                        'data': base64.b64encode(color_jpeg).decode('ascii'),
                        'format': 'jpeg',
                        'width': self.width,
                        'height': self.height,
                    },
                    'infrared': {
                        'data': base64.b64encode(depth_frame.tobytes()).decode('ascii'),
                        'format': 'uint8',
                        'width': depth_frame.shape[1],
                        'height': depth_frame.shape[0],
                    },
                    'note': 'Using infrared stream as depth proxy due to SDK compatibility issues'
                }

                # Publish via ZeroMQ
                self.zmq_socket.send_json(message)

                self.frame_count += 1

                # Print statistics every 5 seconds
                if timestamp - self.last_stats_time >= 5.0:
                    elapsed = timestamp - self.last_stats_time
                    current_fps = self.frame_count / elapsed
                    logger.info(f"Published {self.frame_count} frames in {elapsed:.1f}s "
                               f"({current_fps:.1f} fps)")
                    self.frame_count = 0
                    self.last_stats_time = timestamp

            except KeyboardInterrupt:
                logger.info("Received interrupt signal")
                break
            except Exception as e:
                logger.error(f"Error processing frames: {e}", exc_info=True)
                time.sleep(0.1)

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up camera node...")

        self.running = False

        if self.color_cap:
            self.color_cap.release()
            logger.info("Color camera released")

        if self.depth_cap:
            self.depth_cap.release()
            logger.info("Depth camera released")

        if self.zmq_socket:
            self.zmq_socket.close()

        if self.zmq_context:
            self.zmq_context.term()

        logger.info("Cleanup complete")

    def run(self):
        """Main run method - initialize and start processing"""
        try:
            self.initialize_cameras()
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
    parser = argparse.ArgumentParser(description='AIZEE Camera Node (OpenCV)')
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
        '--color-device',
        type=int,
        default=4,
        help='V4L2 device number for color (/dev/videoN)'
    )
    parser.add_argument(
        '--depth-device',
        type=int,
        default=2,
        help='V4L2 device number for depth/infrared (/dev/videoN)'
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
        default=50,
        help='JPEG compression quality (1-100, lower = faster)'
    )
    parser.add_argument(
        '--flip-vertical',
        action='store_true',
        help='Flip image vertically (for upside-down mounting)'
    )
    parser.add_argument(
        '--flip-horizontal',
        action='store_true',
        help='Flip image horizontally (mirror)'
    )
    parser.add_argument(
        '--rotate-180',
        action='store_true',
        help='Rotate image 180 degrees (same as --flip-vertical --flip-horizontal)'
    )

    args = parser.parse_args()

    # Handle --rotate-180 shortcut
    if args.rotate_180:
        args.flip_vertical = True
        args.flip_horizontal = True

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create and run camera node
    node = CameraNodeOpenCV(
        camera_id=args.camera_id,
        zmq_endpoint=args.zmq_endpoint,
        color_device=args.color_device,
        depth_device=args.depth_device,
        fps=args.fps,
        jpeg_quality=args.jpeg_quality,
        flip_vertical=args.flip_vertical,
        flip_horizontal=args.flip_horizontal
    )

    node.run()


if __name__ == '__main__':
    main()
