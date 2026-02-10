#!/usr/bin/env python3
"""
AIZEE Rerun Bridge Node

Subscribes to all ZeroMQ data streams and visualizes them in Rerun.
Supports:
- Camera streams (RGB + Infrared from multiple cameras)
- LiDAR scans (RPLiDAR A1M8 point clouds)
- Motor telemetry (future)
- Command logging (future)

Usage:
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557 tcp://192.168.0.3:5558 --lidar tcp://192.168.0.27:5561
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557 --save logs/session_001.mcap
"""

import argparse
import base64
import gc
import io
import json
import logging
import signal
import sys
import time
from datetime import datetime
from io import BytesIO
from typing import List, Optional

import cv2
import numpy as np
import rerun as rr
import zmq


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RerunBridge:
    """Bridge node that subscribes to ZMQ streams and logs to Rerun"""

    def __init__(
        self,
        camera_endpoints: List[str],
        lidar_endpoints: List[str] = None,
        ups_endpoints: List[str] = None,
        save_path: Optional[str] = None,
        application_id: str = "aizee"
    ):
        """Initialize Rerun bridge

        Args:
            camera_endpoints: List of ZMQ endpoints for camera streams
            lidar_endpoints: List of ZMQ endpoints for LiDAR streams
            ups_endpoints: List of ZMQ endpoints for UPS power streams
            save_path: Optional MCAP file path to save recording
            application_id: Rerun application ID
        """
        self.camera_endpoints = camera_endpoints
        self.lidar_endpoints = lidar_endpoints or []
        self.ups_endpoints = ups_endpoints or []
        self.save_path = save_path
        self.application_id = application_id

        self.zmq_context: Optional[zmq.Context] = None
        self.camera_sockets: List[zmq.Socket] = []
        self.lidar_sockets: List[zmq.Socket] = []
        self.ups_sockets: List[zmq.Socket] = []
        self.running = False

        # Statistics
        self.frame_counts = {}
        self.scan_counts = {}
        self.scan_sequences = {}  # Track sequence numbers per sensor
        self.ups_message_counts = {}
        self.last_stats_time = time.time()

    def initialize_rerun(self):
        """Initialize Rerun recording session"""
        logger.info(f"Initializing Rerun application: {self.application_id}")

        # Initialize Rerun with automatic viewer spawn
        rr.init(self.application_id, spawn=True)

        # Set up recording to MCAP if requested
        if self.save_path:
            logger.info(f"Recording to: {self.save_path}")
            rr.save(self.save_path)

        # Log application metadata
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        logger.info("Rerun initialized successfully")

    def initialize_zmq(self):
        """Initialize ZeroMQ subscribers for all camera and LiDAR streams"""
        logger.info("Initializing ZeroMQ subscribers...")

        self.zmq_context = zmq.Context()

        # Subscribe to camera streams
        for endpoint in self.camera_endpoints:
            logger.info(f"Subscribing to camera at {endpoint}")
            socket = self.zmq_context.socket(zmq.SUB)

            # Optimize for low latency
            socket.setsockopt(zmq.RCVHWM, 2)  # Keep only latest frames
            socket.setsockopt(zmq.RCVBUF, 2 * 1024 * 1024)  # 2MB receive buffer

            socket.connect(endpoint)
            socket.subscribe("")  # Subscribe to all messages
            self.camera_sockets.append(socket)

        # Subscribe to LiDAR streams
        for endpoint in self.lidar_endpoints:
            logger.info(f"Subscribing to LiDAR at {endpoint}")
            socket = self.zmq_context.socket(zmq.SUB)

            # Optimize for low latency
            socket.setsockopt(zmq.RCVHWM, 2)  # Keep only latest scans
            socket.setsockopt(zmq.RCVBUF, 1 * 1024 * 1024)  # 1MB receive buffer

            socket.connect(endpoint)
            socket.subscribe("")  # Subscribe to all messages
            self.lidar_sockets.append(socket)

        # Subscribe to UPS power streams
        for endpoint in self.ups_endpoints:
            logger.info(f"Subscribing to UPS at {endpoint}")
            socket = self.zmq_context.socket(zmq.SUB)

            # Optimize for low latency
            socket.setsockopt(zmq.RCVHWM, 2)  # Keep only latest readings
            socket.setsockopt(zmq.RCVBUF, 512 * 1024)  # 512KB receive buffer

            socket.connect(endpoint)
            socket.subscribe("")  # Subscribe to all messages
            self.ups_sockets.append(socket)

        # Give subscriptions time to propagate
        time.sleep(0.5)

        logger.info(f"Subscribed to {len(self.camera_sockets)} camera stream(s), {len(self.lidar_sockets)} LiDAR stream(s), and {len(self.ups_sockets)} UPS stream(s)")

    def process_camera_message(self, message: dict):
        """Process and log camera data to Rerun

        Args:
            message: Camera message dictionary with color and infrared data
        """
        camera_id = message.get('camera_id', 'unknown')
        timestamp = message.get('timestamp', time.time())
        frame_number = message.get('frame_number', 0)

        # Update statistics
        if camera_id not in self.frame_counts:
            self.frame_counts[camera_id] = 0
        self.frame_counts[camera_id] += 1

        # Set Rerun timeline - only set sequence to reduce overhead
        rr.set_time("frame", sequence=frame_number)

        # Process color image
        if 'color' in message:
            try:
                color_data = base64.b64decode(message['color']['data'])
                # Decode JPEG directly to numpy array (faster than PIL)
                color_np = cv2.imdecode(np.frombuffer(color_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                color_rgb = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)

                # Log to Rerun
                rr.log(f"cameras/{camera_id}/color", rr.Image(color_rgb))
            except Exception as e:
                logger.error(f"Error processing color image from {camera_id}: {e}")

        # Skip infrared for now to reduce latency (can enable later)
        # if 'infrared' in message:
        #     try:
        #         ir_data = base64.b64decode(message['infrared']['data'])
        #         width = message['infrared']['width']
        #         height = message['infrared']['height']
        #         ir_np = np.frombuffer(ir_data, dtype=np.uint8).reshape((height, width))
        #         rr.log(f"cameras/{camera_id}/infrared", rr.Image(ir_np))
        #     except Exception as e:
        #         logger.error(f"Error processing infrared image from {camera_id}: {e}")

        # Skip text updates for lower overhead (shown in console instead)

    def process_lidar_message(self, message: dict):
        """Process and log LiDAR scan data to Rerun as 3D point clouds

        Args:
            message: LiDAR telemetry message with scan data
        """
        timestamp = message.get('timestamp', time.time())
        lidar_scans = message.get('lidar_scans', [])

        if not lidar_scans:
            return

        # Set Rerun timeline with sequence number for each sensor
        # This ensures scans update in the viewer

        for scan in lidar_scans:
            sensor_id = scan.get('sensor_id', 'unknown')
            ranges = np.array(scan.get('ranges', []))
            intensities = np.array(scan.get('intensities', []))
            angle_min = scan.get('angle_min', 0.0)
            angle_max = scan.get('angle_max', 2 * np.pi)

            # Update statistics
            if sensor_id not in self.scan_counts:
                self.scan_counts[sensor_id] = 0
            self.scan_counts[sensor_id] += 1

            # For live streams, don't set explicit timeline
            # Let Rerun use wall-clock time for continuous playback

            if len(ranges) == 0:
                continue

            # Generate angles for each point
            angles = np.linspace(angle_min, angle_max, len(ranges))

            # Convert polar coordinates to Cartesian (x, y, z)
            # For 2D LiDAR on horizontal plane: z=0
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            z = np.zeros_like(x)

            # Stack into point cloud
            points = np.column_stack([x, y, z])

            # Filter out invalid points (zero range)
            valid_mask = ranges > 0.0
            points = points[valid_mask]
            intensities_filtered = intensities[valid_mask] if len(intensities) > 0 else None

            if len(points) == 0:
                continue

            # Normalize intensities to 0-255 for coloring
            if intensities_filtered is not None and len(intensities_filtered) > 0:
                colors = np.stack([intensities_filtered] * 3, axis=-1)  # Grayscale
            else:
                # Default color: cyan for front, magenta for back
                if 'front' in sensor_id:
                    colors = np.array([[0, 255, 255]] * len(points), dtype=np.uint8)
                else:
                    colors = np.array([[255, 0, 255]] * len(points), dtype=np.uint8)

            # Log to Rerun as 3D point cloud
            rr.log(
                f"world/sensors/{sensor_id}/scan",
                rr.Points3D(points, radii=0.02, colors=colors)
            )

    def process_ups_message(self, message: dict):
        """Process and log UPS power telemetry to Rerun

        Args:
            message: UPS telemetry message with power data
        """
        timestamp = message.get('timestamp', time.time())
        ups_data = message.get('ups', {})

        if not ups_data:
            return

        # Update statistics
        ups_id = "ups_module"
        if ups_id not in self.ups_message_counts:
            self.ups_message_counts[ups_id] = 0
        self.ups_message_counts[ups_id] += 1

        # For live streams, don't set explicit timeline
        # Let Rerun use wall-clock time for continuous playback

        # Log voltage as scalar
        voltage = ups_data.get('voltage', 0.0)
        rr.log("power/ups/voltage", rr.Scalars(voltage))

        # Log current as scalar
        current = ups_data.get('current', 0.0)
        rr.log("power/ups/current", rr.Scalars(current))

        # Log power as scalar
        power = ups_data.get('power', 0.0)
        rr.log("power/ups/power", rr.Scalars(power))

        # Log battery percentage as scalar
        percentage = ups_data.get('percentage', 0.0)
        rr.log("power/ups/battery_percentage", rr.Scalars(percentage))

        # Log power stats as text box (for easy viewing)
        rr.log(
            "power/ups/status",
            rr.TextDocument(
                f"**UPS Power Status**\n\n"
                f"- Voltage: {voltage:.2f}V\n"
                f"- Current: {current:.3f}A\n"
                f"- Power: {power:.2f}W\n"
                f"- Battery: {percentage:.0f}%",
                media_type=rr.MediaType.MARKDOWN
            )
        )

    def process_streams(self):
        """Main processing loop - receive and log all data streams"""
        logger.info("Starting stream processing loop...")
        self.running = True

        # Make garbage collection less aggressive to reduce pauses
        # Default thresholds are (700, 10, 10). Increase 10x to reduce frequency
        gc.set_threshold(7000, 100, 100)

        # Use zmq.Poller for efficient multi-socket handling
        poller = zmq.Poller()
        for socket in self.camera_sockets:
            poller.register(socket, zmq.POLLIN)
        for socket in self.lidar_sockets:
            poller.register(socket, zmq.POLLIN)
        for socket in self.ups_sockets:
            poller.register(socket, zmq.POLLIN)

        logger.info("Rerun bridge running. Open the Rerun viewer to see streams.")

        while self.running:
            try:
                # Poll for messages with shorter timeout for better responsiveness
                socks = dict(poller.poll(timeout=100))

                # Process camera messages
                for socket in self.camera_sockets:
                    if socket in socks and socks[socket] == zmq.POLLIN:
                        # Drain all pending messages, only process the latest
                        latest_message = None
                        while True:
                            try:
                                # Non-blocking receive to drain queue
                                message_json = socket.recv_string(zmq.NOBLOCK)
                                latest_message = json.loads(message_json)
                            except zmq.Again:
                                # No more messages available
                                break

                        # Process only the latest message
                        if latest_message:
                            self.process_camera_message(latest_message)

                # Process LiDAR messages
                for socket in self.lidar_sockets:
                    if socket in socks and socks[socket] == zmq.POLLIN:
                        # Drain all pending messages, only process the latest
                        latest_message = None
                        while True:
                            try:
                                # Non-blocking receive to drain queue
                                message_json = socket.recv_string(zmq.NOBLOCK)
                                latest_message = json.loads(message_json)
                            except zmq.Again:
                                # No more messages available
                                break

                        # Process only the latest message
                        if latest_message:
                            self.process_lidar_message(latest_message)

                # Process UPS messages
                for socket in self.ups_sockets:
                    if socket in socks and socks[socket] == zmq.POLLIN:
                        # Drain all pending messages, only process the latest
                        latest_message = None
                        while True:
                            try:
                                # Non-blocking receive to drain queue
                                message_json = socket.recv_string(zmq.NOBLOCK)
                                latest_message = json.loads(message_json)
                            except zmq.Again:
                                # No more messages available
                                break

                        # Process only the latest message
                        if latest_message:
                            self.process_ups_message(latest_message)

                # Print statistics every 5 seconds
                current_time = time.time()
                if current_time - self.last_stats_time >= 5.0:
                    elapsed = current_time - self.last_stats_time
                    logger.info("Stream statistics:")
                    for camera_id, count in self.frame_counts.items():
                        fps = count / elapsed
                        logger.info(f"  Camera {camera_id}: {count} frames ({fps:.1f} fps)")
                    for sensor_id, count in self.scan_counts.items():
                        scan_rate = count / elapsed
                        logger.info(f"  LiDAR {sensor_id}: {count} scans ({scan_rate:.1f} Hz)")
                    for ups_id, count in self.ups_message_counts.items():
                        ups_rate = count / elapsed
                        logger.info(f"  UPS {ups_id}: {count} messages ({ups_rate:.1f} Hz)")

                    # Reset counters
                    self.frame_counts = {}
                    self.scan_counts = {}
                    self.ups_message_counts = {}
                    self.last_stats_time = current_time

            except KeyboardInterrupt:
                logger.info("Received interrupt signal")
                break
            except Exception as e:
                logger.error(f"Error in processing loop: {e}", exc_info=True)
                time.sleep(0.1)

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up Rerun bridge...")

        self.running = False

        # Restore default garbage collection thresholds
        gc.set_threshold(700, 10, 10)

        # Close ZMQ sockets
        for socket in self.camera_sockets:
            socket.close()
        for socket in self.lidar_sockets:
            socket.close()
        for socket in self.ups_sockets:
            socket.close()

        if self.zmq_context:
            self.zmq_context.term()

        logger.info("Cleanup complete")

    def run(self):
        """Main run method - initialize and start processing"""
        try:
            self.initialize_rerun()
            self.initialize_zmq()

            logger.info("=" * 60)
            logger.info("Rerun bridge ready!")
            logger.info(f"Viewing {len(self.camera_endpoints)} camera stream(s)")
            logger.info(f"Viewing {len(self.lidar_endpoints)} LiDAR stream(s)")
            logger.info(f"Viewing {len(self.ups_endpoints)} UPS stream(s)")
            if self.save_path:
                logger.info(f"Recording to: {self.save_path}")
            logger.info("Open the Rerun viewer in your browser or app")
            logger.info("=" * 60)

            self.process_streams()

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
    parser = argparse.ArgumentParser(
        description='AIZEE Rerun Bridge - Visualize robot data streams'
    )
    parser.add_argument(
        '--cameras',
        nargs='+',
        default=[],
        help='ZMQ endpoints for camera streams (space-separated)'
    )
    parser.add_argument(
        '--lidar',
        nargs='+',
        default=[],
        help='ZMQ endpoints for LiDAR streams (space-separated)'
    )
    parser.add_argument(
        '--ups',
        nargs='+',
        default=[],
        help='ZMQ endpoints for UPS power streams (space-separated)'
    )
    parser.add_argument(
        '--save',
        type=str,
        help='Save recording to MCAP file (e.g., logs/session_001.mcap)'
    )
    parser.add_argument(
        '--app-id',
        type=str,
        default='aizee',
        help='Rerun application ID'
    )

    args = parser.parse_args()

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create and run bridge
    bridge = RerunBridge(
        camera_endpoints=args.cameras,
        lidar_endpoints=args.lidar,
        ups_endpoints=args.ups,
        save_path=args.save,
        application_id=args.app_id
    )

    bridge.run()


if __name__ == '__main__':
    main()
