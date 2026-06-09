#!/usr/bin/env python3
"""
AIZEE Rerun Bridge Node

Subscribes to all ZeroMQ data streams and visualizes them in Rerun.
Supports:
- Camera streams (RGB + Infrared from multiple cameras)
- LiDAR scans (RPLiDAR A1M8 point clouds)
- Motor telemetry (position, velocity, torque, temperature @ 50Hz)
- Gantry arm FK (3D transform hierarchy)
- Rover odometry (dead-reckoning path trail)
- Battery voltage (from motor telemetry message)
- UPS power monitoring

Usage:
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557 tcp://192.168.0.3:5558 --lidar tcp://192.168.0.27:5561
    python rerun_bridge.py --cameras tcp://192.168.0.2:5557 --save logs/session_001.mcap
    python rerun_bridge.py --telemetry tcp://192.168.0.27:5556 --lidar tcp://192.168.0.27:5561 --ups tcp://192.168.0.27:5562
"""

import argparse
import gc
import io
import json
import logging
import math
import signal
import sys
import time
from datetime import datetime
from io import BytesIO
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common.wire import unpack_camera, unpack_msg


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RoverOdometry:
    """Dead-reckoning odometry from differential-drive wheel velocities."""

    def __init__(self, wheel_radius=0.150, wheelbase=0.354):
        self.r = wheel_radius
        self.L = wheelbase
        self.x = self.y = self.theta = 0.0
        self.last_ts = None
        self.path: list[list[float]] = [[0.0, 0.0, 0.0]]

    def update(self, left_vel: float, right_vel: float, timestamp: float):
        if self.last_ts is None:
            self.last_ts = timestamp
            return
        dt = timestamp - self.last_ts
        self.last_ts = timestamp
        if dt <= 0 or dt > 1.0:
            return
        vL = left_vel * self.r
        vR = right_vel * self.r
        v = (vL + vR) / 2.0
        omega = (vR - vL) / self.L
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.path.append([self.x, self.y, 0.0])
        if len(self.path) > 2000:  # cap trail length
            self.path = self.path[-2000:]


def build_blueprint() -> rrb.Blueprint:
    """Build the programmatic Rerun panel layout sent on startup."""
    return rrb.Blueprint(
        rrb.Horizontal(
            # Single 3D view: rover body, arm FK, LiDAR, odometry path.
            rrb.Spatial3DView(
                name="World",
                origin="/",
                contents=["world/**"],
            ),
            rrb.Vertical(
                # Gripper + scene camera image feeds.
                rrb.Spatial2DView(
                    name="Cameras",
                    contents=["cameras/**"],
                ),
                rrb.TextDocumentView(
                    name="Motor Status",
                    origin="motors/status",
                ),
                rrb.TimeSeriesView(
                    name="Base Positions",
                    contents=[
                        "motors/left_wheel/position",
                        "motors/right_wheel/position",
                    ],
                ),
                rrb.TimeSeriesView(
                    name="Gantry Positions",
                    # Swivel is part of the arm post-unification.  Show it
                    # alongside the rest of the gantry chain (joint 0).
                    contents=[
                        "motors/swivel/position",
                        "motors/gantry_base/position",
                        "motors/gantry_mid/position",
                        "motors/gantry_end/position",
                    ],
                ),
                rrb.TimeSeriesView(
                    name="Wrist Positions",
                    contents=[
                        "motors/wrist_pitch/position",
                        "motors/wrist_roll/position",
                        "motors/gripper/position",
                    ],
                ),
                rrb.TimeSeriesView(
                    name="Velocity",
                    contents=["motors/*/velocity"],
                ),
                rrb.TimeSeriesView(
                    name="Torque",
                    contents=["motors/*/torque"],
                ),
                rrb.TimeSeriesView(
                    name="Temperature",
                    contents=["motors/*/temperature"],
                ),
                rrb.TimeSeriesView(
                    name="Power",
                    origin="power",
                ),
            ),
            column_shares=[3, 2],
        )
    )


class RerunBridge:
    """Bridge node that subscribes to ZMQ streams and logs to Rerun"""

    # Arm link lengths (metres)
    L0 = 0.5906  # base → mid
    L1 = 0.5649  # mid → end
    L2 = 0.100   # end → wrist_pitch pivot
    L3 = 0.1063  # wrist_pitch pivot → wrist_roll pivot
    L5 = 0.132   # wrist_roll pivot → gripper tip
    ARM_MOUNT_Z = 0.200  # arm mount height above rover base frame

    def __init__(
        self,
        camera_endpoints: List[str],
        lidar_endpoints: List[str] = None,
        ups_endpoints: List[str] = None,
        telemetry_endpoints: List[str] = None,
        save_path: Optional[str] = None,
        application_id: str = "aizee"
    ):
        """Initialize Rerun bridge

        Args:
            camera_endpoints: List of ZMQ endpoints for camera streams
            lidar_endpoints: List of ZMQ endpoints for LiDAR streams
            ups_endpoints: List of ZMQ endpoints for UPS power streams
            telemetry_endpoints: List of ZMQ endpoints for motor telemetry streams
            save_path: Optional MCAP file path to save recording
            application_id: Rerun application ID
        """
        self.camera_endpoints = camera_endpoints
        self.lidar_endpoints = lidar_endpoints or []
        self.ups_endpoints = ups_endpoints or []
        self.telemetry_endpoints = telemetry_endpoints or ["tcp://192.168.0.27:5556"]
        self.save_path = save_path
        self.application_id = application_id

        self.zmq_context: Optional[zmq.Context] = None
        self.camera_sockets: List[zmq.Socket] = []
        self.lidar_sockets: List[zmq.Socket] = []
        self.ups_sockets: List[zmq.Socket] = []
        self.telemetry_sockets: List[zmq.Socket] = []
        self.running = False

        # Statistics
        self.frame_counts = {}
        self.scan_counts = {}
        self.scan_sequences = {}  # Track sequence numbers per sensor
        self.ups_message_counts = {}
        self.telemetry_message_counts = {}
        self.last_stats_time = time.time()

        # Odometry
        self.odometry = RoverOdometry()

    def initialize_rerun(self):
        """Initialize Rerun recording session"""
        logger.info(f"Initializing Rerun application: {self.application_id}")

        # Initialize Rerun with automatic viewer spawn
        rr.init(self.application_id, spawn=True)

        # Send blueprint for automatic panel layout
        rr.send_blueprint(build_blueprint())

        # Set up recording to MCAP if requested
        if self.save_path:
            logger.info(f"Recording to: {self.save_path}")
            rr.save(self.save_path)

        # World coordinate system
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        # --- Static geometry ---

        # Rover body outline
        rr.log(
            "world/rover/body",
            rr.Boxes3D(
                half_sizes=[[0.25, 0.175, 0.125]],
                centers=[[0.0, 0.0, 0.125]],
            ),
            static=True,
        )

        # Arm mount offset (fixed relative to rover body)
        rr.log(
            "world/rover/arm",
            rr.Transform3D(translation=[0.0, 0.0, self.ARM_MOUNT_Z]),
            static=True,
        )

        # Arm link visualisations (in their respective joint frames)
        _jb  = "world/rover/arm/joint_base"
        _jm  = f"{_jb}/joint_mid"
        _je  = f"{_jm}/joint_end"
        _jwp = f"{_je}/joint_wrist_pitch"
        _jwr = f"{_jwp}/joint_wrist_roll"
        rr.log(f"{_jb}/link_0",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [self.L0, 0.0, 0.0]]], colors=[[255, 180, 0]]),
            static=True)
        rr.log(f"{_jm}/link_1",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [self.L1, 0.0, 0.0]]], colors=[[255, 140, 0]]),
            static=True)
        rr.log(f"{_je}/link_2",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [self.L2, 0.0, 0.0]]], colors=[[255, 100, 0]]),
            static=True)
        rr.log(f"{_jwp}/link_3",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [self.L3, 0.0, 0.0]]], colors=[[255, 60, 0]]),
            static=True)
        rr.log(f"{_jwr}/link_5",
            rr.LineStrips3D([[[0.0, 0.0, 0.0], [self.L5, 0.0, 0.0]]], colors=[[255, 0, 50]]),
            static=True)

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

        # Subscribe to motor telemetry streams (50 Hz)
        for endpoint in self.telemetry_endpoints:
            logger.info(f"Subscribing to motor telemetry at {endpoint}")
            socket = self.zmq_context.socket(zmq.SUB)

            socket.setsockopt(zmq.RCVHWM, 2)          # Keep only latest telemetry
            socket.setsockopt(zmq.RCVBUF, 512 * 1024)  # 512KB receive buffer

            socket.connect(endpoint)
            socket.subscribe("")  # Subscribe to all messages
            self.telemetry_sockets.append(socket)

        # Give subscriptions time to propagate
        time.sleep(0.5)

        logger.info(
            f"Subscribed to {len(self.camera_sockets)} camera stream(s), "
            f"{len(self.lidar_sockets)} LiDAR stream(s), "
            f"{len(self.ups_sockets)} UPS stream(s), "
            f"{len(self.telemetry_sockets)} telemetry stream(s)"
        )

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

        # Set Rerun timeline — use local receive time (not Pi timestamp) to avoid
        # clock skew between Pis causing out-of-order entries on the timeline.
        rr.set_time("time", timestamp=time.time())

        # --- Color image (gripper / scene cameras) ---
        # Logged as a 2-D image feed; orientation/flip is handled at the camera
        # node (per-camera config), not here.
        if 'color' in message:
            try:
                color_data = message['color']['data_bytes']
                # Decode JPEG directly to numpy array (faster than PIL)
                color_np = cv2.imdecode(np.frombuffer(color_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                color_rgb = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)
                rr.log(f"cameras/{camera_id}", rr.Image(color_rgb))
            except Exception as e:
                logger.error(f"Error processing color image from {camera_id}: {e}")

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

    def process_telemetry_message(self, message: dict):
        """Process and log motor telemetry to Rerun.

        Args:
            message: TelemetryMessage dict with timestamp, motors dict, and
                     optional battery_voltage field.
        """
        timestamp = message.get("timestamp", time.time())
        motors: dict = message.get("motors", {})

        if not motors:
            return

        # Set timeline to telemetry timestamp (seconds)
        rr.set_time("time", timestamp=timestamp)

        # --- Per-motor scalar plots ---
        for motor_id, m in motors.items():
            rr.log(f"motors/{motor_id}/position",    rr.Scalars(m.get("position",    0.0)))
            rr.log(f"motors/{motor_id}/velocity",    rr.Scalars(m.get("velocity",    0.0)))
            rr.log(f"motors/{motor_id}/torque",      rr.Scalars(m.get("torque",      0.0)))
            rr.log(f"motors/{motor_id}/temperature", rr.Scalars(m.get("temperature", 0.0)))

        # --- Motor status Markdown table ---
        motor_order = [
            "left_wheel", "right_wheel", "swivel",
            "gantry_base", "gantry_mid", "gantry_end",
            "wrist_pitch", "wrist_roll", "gripper",
        ]
        header = "| ID | State | Pos (rad) | Vel (rad/s) | Torque (Nm) | Temp (°C) | Error |\n"
        header += "|---|---|---|---|---|---|---|\n"
        rows = []
        for mid in motor_order:
            m = motors.get(mid)
            if m is None:
                rows.append(f"| {mid} | — | — | — | — | — | — |")
            else:
                state = m.get("state", "—")
                pos   = m.get("position",    0.0)
                vel   = m.get("velocity",    0.0)
                torq  = m.get("torque",      0.0)
                temp  = m.get("temperature", 0.0)
                error = m.get("error") or "—"
                rows.append(
                    f"| {mid} | {state} | {pos:.3f} | {vel:.3f} | {torq:.3f} | {temp:.1f} | {error} |"
                )
        # Also append any motors not in the canonical order
        for mid, m in motors.items():
            if mid not in motor_order:
                state = m.get("state", "—")
                pos   = m.get("position",    0.0)
                vel   = m.get("velocity",    0.0)
                torq  = m.get("torque",      0.0)
                temp  = m.get("temperature", 0.0)
                error = m.get("error") or "—"
                rows.append(
                    f"| {mid} | {state} | {pos:.3f} | {vel:.3f} | {torq:.3f} | {temp:.1f} | {error} |"
                )
        rr.log(
            "motors/status",
            rr.TextDocument(
                "**Motor Status**\n\n" + header + "\n".join(rows),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )

        # --- Rover odometry ---
        left_vel  = motors.get("left_wheel",  {}).get("velocity", 0.0)
        right_vel = motors.get("right_wheel", {}).get("velocity", 0.0)
        self.odometry.update(left_vel, right_vel, timestamp)

        rr.log(
            "world/rover",
            rr.Transform3D(
                translation=[self.odometry.x, self.odometry.y, 0.0],
                rotation=rr.RotationAxisAngle([0, 0, 1], self.odometry.theta),
            ),
        )
        rr.log(
            "world/rover/path",
            rr.LineStrips3D([self.odometry.path]),
        )

        # --- Gantry arm FK ---
        base_pos = motors.get("gantry_base", {}).get("position", 0.0)
        mid_pos  = motors.get("gantry_mid",  {}).get("position", 0.0)
        end_pos  = motors.get("gantry_end",  {}).get("position", 0.0)

        rr.log(
            "world/rover/arm/joint_base",
            rr.Transform3D(rotation=rr.RotationAxisAngle([0, 0, 1], base_pos)),
        )
        rr.log(
            "world/rover/arm/joint_base/joint_mid",
            rr.Transform3D(
                translation=[self.L0, 0.0, 0.0],
                rotation=rr.RotationAxisAngle([0, 1, 0], mid_pos),
            ),
        )
        rr.log(
            "world/rover/arm/joint_base/joint_mid/joint_end",
            rr.Transform3D(
                translation=[self.L1, 0.0, 0.0],
                rotation=rr.RotationAxisAngle([0, 1, 0], end_pos),
            ),
        )

        wrist_pitch_pos = motors.get("wrist_pitch", {}).get("position", 0.0)
        wrist_roll_pos  = motors.get("wrist_roll",  {}).get("position", 0.0)
        gripper_pos     = motors.get("gripper",     {}).get("position", 0.0)

        _je = "world/rover/arm/joint_base/joint_mid/joint_end"
        rr.log(f"{_je}/joint_wrist_pitch",
            rr.Transform3D(
                translation=[self.L2, 0.0, 0.0],
                rotation=rr.RotationAxisAngle([0, 1, 0], wrist_pitch_pos),
            ),
        )
        rr.log(f"{_je}/joint_wrist_pitch/joint_wrist_roll",
            rr.Transform3D(
                translation=[self.L3, 0.0, 0.0],
                rotation=rr.RotationAxisAngle([1, 0, 0], wrist_roll_pos),
            ),
        )
        rr.log(f"{_je}/joint_wrist_pitch/joint_wrist_roll/joint_gripper",
            rr.Transform3D(
                translation=[self.L5, 0.0, 0.0],
                rotation=rr.RotationAxisAngle([0, 0, 1], gripper_pos),
            ),
        )

        # --- Battery voltage ---
        battery_voltage = message.get("battery_voltage")
        if battery_voltage is not None:
            rr.log("power/battery", rr.Scalars(battery_voltage))

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
        for socket in self.telemetry_sockets:
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
                                frames = socket.recv_multipart(zmq.NOBLOCK)
                                latest_message = unpack_camera(frames)
                            except zmq.Again:
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

                # Process UPS messages (msgpack)
                for socket in self.ups_sockets:
                    if socket in socks and socks[socket] == zmq.POLLIN:
                        latest_message = None
                        while True:
                            try:
                                latest_message = unpack_msg(socket.recv(zmq.NOBLOCK))
                            except zmq.Again:
                                break

                        if latest_message:
                            self.process_ups_message(latest_message)

                # Process motor telemetry messages (50 Hz, msgpack — drain to latest)
                for socket in self.telemetry_sockets:
                    if socket in socks and socks[socket] == zmq.POLLIN:
                        latest_message = None
                        while True:
                            try:
                                latest_message = unpack_msg(socket.recv(zmq.NOBLOCK))
                            except zmq.Again:
                                break

                        if latest_message:
                            endpoint = self.telemetry_endpoints[self.telemetry_sockets.index(socket)]
                            if endpoint not in self.telemetry_message_counts:
                                self.telemetry_message_counts[endpoint] = 0
                            self.telemetry_message_counts[endpoint] += 1
                            self.process_telemetry_message(latest_message)

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
                    for ep, count in self.telemetry_message_counts.items():
                        tel_rate = count / elapsed
                        logger.info(f"  Telemetry {ep}: {count} messages ({tel_rate:.1f} Hz)")

                    # Reset counters
                    self.frame_counts = {}
                    self.scan_counts = {}
                    self.ups_message_counts = {}
                    self.telemetry_message_counts = {}
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
        for socket in self.telemetry_sockets:
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
            logger.info(f"Viewing {len(self.telemetry_endpoints)} telemetry stream(s)")
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
        '--telemetry',
        nargs='+',
        default=["tcp://192.168.0.27:5556"],
        help='ZMQ endpoints for motor telemetry streams (space-separated)'
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
        telemetry_endpoints=args.telemetry,
        save_path=args.save,
        application_id=args.app_id
    )

    bridge.run()


if __name__ == '__main__':
    main()
