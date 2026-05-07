#!/usr/bin/env python3
"""
AIZEE Tufty2040 Status Display Node

Subscribes to motor telemetry (ZMQ :5556, 50 Hz) and UPS telemetry
(ZMQ :5562, 1 Hz), assembles compact status packets, and forwards them
to a Pimoroni Tufty2040 LCD display via USB CDC serial at 2 Hz.

The Tufty2040 runs tufty2040/main.py (MicroPython) and renders a live
robot-health dashboard on its 320×240 IPS screen.

Usage:
    python display_node.py --config config/hardware_jetson_rover.yaml
    python display_node.py --serial-port /dev/tufty_display --rate 2.0
"""

import argparse
import json
import logging
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

import serial
import serial.serialutil
import yaml
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import unpack_msg

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Motor full-name → abbreviation used in the display packet
MOTOR_ABBREV = {
    "left_wheel":  "lw",
    "right_wheel": "rw",
    "swivel":      "sw",
    "gantry_base": "gb",
    "gantry_mid":  "gm",
    "gantry_end":  "ge",
    "wrist_pitch": "wp",
    "wrist_roll":  "wr",
    "gripper":     "gr",
}

# Motor states from Rust motor_control → single display char
STATE_CHAR = {
    "running":  "r",
    "enabling": "e",
    "enabled":  "e",  # treat enabled (awaiting first feedback) same as enabling
    "disabled": "d",
    "idle":     "d",  # idle == not yet enabled
    "error":    "x",
}

BASE_MOTOR_ABBREVS = {"lw", "rw", "sw"}

MOTOR_STALE_TIMEOUT = 10.0  # seconds
UPS_STALE_TIMEOUT   = 15.0  # seconds

# Jetson services to monitor — (systemd unit name, display abbreviation).
# Abbreviations are chosen short (≤6 chars) so the Tufty firmware can fit
# them in its services row without truncation.
SERVICES = [
    ("aizee-motor-control-rover", "motors"),
    ("aizee-lidar-control",       "lidar"),
    ("aizee-ups-monitor",         "ups"),
    ("aizee-camera-relay",        "relay"),
    ("aizee-display",             "disp"),
    ("aizee-arm-cam-left",        "armL"),
    ("aizee-arm-cam-right",       "armR"),
]
SERVICE_CHECK_INTERVAL = 5.0  # seconds — rate-limit systemctl calls

CAMERA_PIS = [
    ("pi1", "10.42.0.11"),
    ("pi2", "10.42.0.12"),
    ("pi3", "10.42.0.13"),
    ("pi4", "10.42.0.14"),
]
MOTOR_FULL_V = 25.2   # 6S LiPo @ 4.2 V/cell
MOTOR_MIN_V  = 19.8   # 6S LiPo @ 3.3 V/cell (practical empty)
IP_CHECK_INTERVAL = 30.0   # seconds — refresh Jetson IP


def _bind_to_connect(endpoint: str) -> str:
    """Convert a bind endpoint (tcp://*:PORT) to a connect endpoint (tcp://localhost:PORT)."""
    return endpoint.replace("tcp://*:", "tcp://localhost:")


class DisplayNode:
    """Bridges ZMQ telemetry streams to the Tufty2040 status display via serial."""

    def __init__(
        self,
        motor_endpoint: str = "tcp://localhost:5556",
        ups_endpoint: str = "tcp://localhost:5562",
        serial_port: str = "/dev/tufty_display",
        baud_rate: int = 115200,
        update_rate: float = 2.0,
    ):
        self.motor_endpoint = motor_endpoint
        self.ups_endpoint = ups_endpoint
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.update_interval = 1.0 / update_rate

        # ZMQ
        self.zmq_context: Optional[zmq.Context] = None
        self.motor_sub: Optional[zmq.Socket] = None
        self.ups_sub: Optional[zmq.Socket] = None
        self.poller: Optional[zmq.Poller] = None

        # Serial
        self.serial: Optional[serial.Serial] = None
        self._reconnect_after: float = 0.0

        # Cached telemetry
        self.latest_motor: Optional[dict] = None
        self.last_motor_time: float = 0.0
        self.latest_ups: Optional[dict] = None
        self.last_ups_time: float = 0.0

        # Cached service states
        self.service_states: dict = {}
        self.last_service_check: float = 0.0

        # Cached Pi reachability and Jetson IP
        self.pi_states: dict = {}
        self.last_pi_check: float = 0.0
        self.jetson_ip: str = ""
        self.last_ip_check: float = 0.0

        self.running = False

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize_zmq(self):
        """Create SUB sockets and poller for motor + UPS telemetry."""
        logger.info("Initializing ZMQ subscribers")
        self.zmq_context = zmq.Context()

        self.motor_sub = self.zmq_context.socket(zmq.SUB)
        self.motor_sub.setsockopt(zmq.RCVHWM, 10)
        self.motor_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.motor_sub.connect(self.motor_endpoint)
        logger.info(f"Motor sub connected to {self.motor_endpoint}")

        self.ups_sub = self.zmq_context.socket(zmq.SUB)
        self.ups_sub.setsockopt(zmq.RCVHWM, 10)
        self.ups_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.ups_sub.connect(self.ups_endpoint)
        logger.info(f"UPS sub connected to {self.ups_endpoint}")

        self.poller = zmq.Poller()
        self.poller.register(self.motor_sub, zmq.POLLIN)
        self.poller.register(self.ups_sub, zmq.POLLIN)

    def initialize_serial(self):
        """Open the serial port to the Tufty2040."""
        logger.info(f"Opening serial port {self.serial_port} at {self.baud_rate} baud")
        try:
            self.serial = serial.Serial(
                self.serial_port,
                baudrate=self.baud_rate,
                timeout=0,
            )
            logger.info("Serial port opened successfully")
        except (serial.serialutil.SerialException, OSError) as exc:
            logger.warning(f"Could not open serial port: {exc} — will retry in 5 s")
            self.serial = None
            self._reconnect_after = time.time() + 5.0

    # ------------------------------------------------------------------
    # ZMQ draining
    # ------------------------------------------------------------------

    def _drain_zmq(self):
        """Non-blocking drain of both ZMQ sockets; cache the latest messages."""
        if self.poller is None:
            return

        socks = dict(self.poller.poll(timeout=0))

        if self.motor_sub in socks and socks[self.motor_sub] == zmq.POLLIN:
            latest = None
            while True:
                try:
                    latest = unpack_msg(self.motor_sub.recv(zmq.NOBLOCK))
                except zmq.Again:
                    break
                except Exception:
                    pass
            if latest is not None:
                self.latest_motor = latest
                self.last_motor_time = time.time()

        if self.ups_sub in socks and socks[self.ups_sub] == zmq.POLLIN:
            latest = None
            while True:
                try:
                    latest = unpack_msg(self.ups_sub.recv(zmq.NOBLOCK))
                except zmq.Again:
                    break
                except Exception:
                    pass
            if latest is not None:
                self.latest_ups = latest
                self.last_ups_time = time.time()

    # ------------------------------------------------------------------
    # Service status
    # ------------------------------------------------------------------

    def _check_services(self) -> dict:
        """Query systemctl is-active for each monitored service.

        Returns {abbrev: status_char} where status_char is one of:
          "a" = active, "f" = failed, "i" = inactive, "e" = activating, "?" = unknown
        """
        names = [s[0] for s in SERVICES]
        states: dict = {}
        try:
            result = subprocess.run(
                ["systemctl", "is-active"] + names,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
            lines = result.stdout.strip().splitlines()
            for (_, abbrev), line in zip(SERVICES, lines):
                status = line.strip()
                if status == "active":
                    states[abbrev] = "a"
                elif status == "failed":
                    states[abbrev] = "f"
                elif status == "inactive":
                    states[abbrev] = "i"
                elif status == "activating":
                    states[abbrev] = "e"
                else:
                    states[abbrev] = "?"
        except Exception as exc:
            logger.warning(f"Service status check failed: {exc}")
            for _, abbrev in SERVICES:
                states[abbrev] = "?"
        return states

    def _ping_host(self, ip: str) -> bool:
        try:
            r = subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                               capture_output=True, timeout=2.5)
            return r.returncode == 0
        except Exception:
            return False

    def _check_pis(self) -> dict:
        try:
            with ThreadPoolExecutor(max_workers=4) as ex:
                futures = {k: ex.submit(self._ping_host, ip) for k, ip in CAMERA_PIS}
                return {k: ("u" if f.result(timeout=3.0) else "d")
                        for k, f in futures.items()}
        except Exception:
            return {k: "?" for k, _ in CAMERA_PIS}

    def _get_jetson_ip(self) -> str:
        try:
            r = subprocess.run(["hostname", "-I"], capture_output=True,
                               text=True, timeout=2.0)
            for ip in r.stdout.strip().split():
                if ip.startswith("192.168."):
                    return ip
            for ip in r.stdout.strip().split():
                if not ip.startswith("127."):
                    return ip
        except Exception:
            pass
        return ""

    # ------------------------------------------------------------------
    # Packet assembly
    # ------------------------------------------------------------------

    def _build_packet(self) -> dict:
        """Assemble the compact JSON dict to send to the Tufty2040."""
        now = time.time()

        # --- Motor telemetry ---
        motor_stale = (self.last_motor_time == 0.0 or
                       now - self.last_motor_time > MOTOR_STALE_TIMEOUT)
        if motor_stale or self.latest_motor is None:
            mv = None
            me = False
            ms = {}
            mpos = {}
        else:
            motors: dict = self.latest_motor.get("motors", {})

            # Bus voltage (top-level field published by motor_control)
            mv = self.latest_motor.get("battery_voltage")
            if mv is not None:
                mv = round(float(mv), 2)

            # Per-motor state abbreviations and positions (radians)
            ms = {}
            mpos = {}
            for full_name, abbrev in MOTOR_ABBREV.items():
                m = motors.get(full_name)
                if m is None:
                    char = "?"
                else:
                    state_str = m.get("state", "")
                    char = STATE_CHAR.get(state_str, "?")
                    pos = m.get("position")
                    if pos is not None:
                        mpos[abbrev] = round(float(pos), 2)
                ms[abbrev] = char

            # Motors-enabled: all three base motors must be "running"
            me = all(ms.get(b) == "r" for b in BASE_MOTOR_ABBREVS)

        # Motor battery percentage
        if mv is not None:
            mp = int(max(0, min(100,
                    (mv - MOTOR_MIN_V) / (MOTOR_FULL_V - MOTOR_MIN_V) * 100)))
        else:
            mp = None

        # --- UPS telemetry ---
        ups_stale = (self.last_ups_time == 0.0 or
                     now - self.last_ups_time > UPS_STALE_TIMEOUT)
        if ups_stale or self.latest_ups is None:
            up = None
            ub = None
        else:
            ups_data = self.latest_ups.get("ups", {})
            voltage = ups_data.get("voltage")
            up = round(float(voltage), 2) if voltage is not None else None
            pct = ups_data.get("percentage")
            ub = int(round(float(pct))) if pct is not None else None
            if ub is not None:
                ub = max(0, min(100, ub))

        return {
            "mv": mv,
            "mp": mp,
            "up": up,
            "ub": ub,
            "me": me,
            "ms": ms,
            "mpos": mpos,
            "sv": self.service_states,
            "ip": self.jetson_ip,
            "pi": self.pi_states,
            "t":  round(now, 1),
        }

    # ------------------------------------------------------------------
    # Serial output
    # ------------------------------------------------------------------

    def _send_display_packet(self, packet: dict):
        """Serialize packet to JSON + newline and write to serial."""
        if self.serial is None:
            return

        line = json.dumps(packet, separators=(",", ":")) + "\n"
        try:
            self.serial.write(line.encode("ascii"))
        except (serial.serialutil.SerialException, OSError) as exc:
            logger.warning(f"Serial write failed: {exc} — scheduling reconnect in 5 s")
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self._reconnect_after = time.time() + 5.0

    def _try_reconnect_serial(self):
        """Attempt to re-open the serial port."""
        if time.time() < self._reconnect_after:
            return
        logger.info(f"Attempting serial reconnect on {self.serial_port}")
        try:
            self.serial = serial.Serial(
                self.serial_port,
                baudrate=self.baud_rate,
                timeout=0,
            )
            logger.info("Serial reconnected successfully")
        except (serial.serialutil.SerialException, OSError) as exc:
            logger.debug(f"Serial reconnect failed: {exc} — will retry in 5 s")
            self.serial = None
            self._reconnect_after = time.time() + 5.0

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run_loop(self):
        """Main loop: drain ZMQ, send display packet at update_rate."""
        logger.info(f"Starting display loop at {1.0/self.update_interval:.1f} Hz")
        self.running = True
        next_send_time = time.time()

        while self.running:
            try:
                current_time = time.time()

                # Reconnect serial if needed
                if self.serial is None:
                    self._try_reconnect_serial()

                # Check service states periodically (rate-limited to avoid systemctl overhead)
                if current_time - self.last_service_check >= SERVICE_CHECK_INTERVAL:
                    self.service_states = self._check_services()
                    self.pi_states      = self._check_pis()
                    self.last_service_check = current_time

                if current_time - self.last_ip_check >= IP_CHECK_INTERVAL:
                    self.jetson_ip = self._get_jetson_ip()
                    self.last_ip_check = current_time

                # Always drain ZMQ (non-blocking)
                self._drain_zmq()

                # Send packet at configured rate
                if current_time >= next_send_time:
                    packet = self._build_packet()
                    self._send_display_packet(packet)

                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug(f"Sent: {packet}")

                    next_send_time += self.update_interval
                    if next_send_time < current_time:
                        next_send_time = current_time + self.update_interval

                # Sleep up to 50 ms — keeps the loop responsive
                sleep_to_next = next_send_time - time.time()
                time.sleep(min(max(sleep_to_next, 0), 0.05))

            except KeyboardInterrupt:
                logger.info("Received interrupt signal")
                break
            except Exception as exc:
                logger.error(f"Unexpected error in display loop: {exc}", exc_info=True)
                time.sleep(1.0)

    def cleanup(self):
        """Release ZMQ and serial resources."""
        logger.info("Cleaning up display node")
        self.running = False

        if self.motor_sub:
            self.motor_sub.close()
        if self.ups_sub:
            self.ups_sub.close()
        if self.zmq_context:
            self.zmq_context.term()

        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass

        logger.info("Cleanup complete")

    def run(self):
        """Top-level entry point."""
        try:
            self.initialize_zmq()
            self.initialize_serial()

            logger.info("=" * 60)
            logger.info("AIZEE Display Node Ready")
            logger.info(f"Motor sub : {self.motor_endpoint}")
            logger.info(f"UPS sub   : {self.ups_endpoint}")
            logger.info(f"Serial    : {self.serial_port} @ {self.baud_rate}")
            logger.info(f"Rate      : {1.0/self.update_interval:.1f} Hz")
            logger.info("=" * 60)

            self.run_loop()

        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as exc:
            logger.error(f"Fatal error: {exc}", exc_info=True)
        finally:
            self.cleanup()


# ---------------------------------------------------------------------------
# Signal handlers
# ---------------------------------------------------------------------------

_node: Optional[DisplayNode] = None


def _signal_handler(signum, frame):
    logger.info(f"Received signal {signum}, shutting down")
    if _node is not None:
        _node.running = False
    sys.exit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    global _node

    parser = argparse.ArgumentParser(
        description="AIZEE Tufty2040 Status Display Node"
    )
    parser.add_argument("--config", type=str, help="Path to YAML config file")
    parser.add_argument(
        "--serial-port",
        type=str,
        default="/dev/tufty_display",
        help="Serial device for Tufty2040 (default: /dev/tufty_display)",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=2.0,
        help="Display update rate in Hz (default: 2.0)",
    )
    args = parser.parse_args()

    # --- Load YAML config ---
    config: dict = {}
    if args.config:
        config = load_config(args.config)
        logger.info(f"Loaded config from {args.config}")

    display_cfg = config.get("display", {})
    zmq_cfg = config.get("network", {}).get("device", {}).get("zmq", {})

    # Serial settings: CLI arg > YAML display block > default
    serial_port = display_cfg.get("serial_port", args.serial_port)
    baud_rate   = display_cfg.get("baud_rate",   args.baud_rate)
    update_rate = display_cfg.get("update_rate", args.rate)

    # ZMQ endpoints: convert bind addresses to connect addresses
    raw_motor = zmq_cfg.get("telemetry_pub", "tcp://*:5556")
    raw_ups   = zmq_cfg.get("ups_pub",       "tcp://*:5562")
    motor_endpoint = zmq_cfg.get("motor_telem_sub", _bind_to_connect(raw_motor))
    ups_endpoint   = _bind_to_connect(raw_ups)

    # --- Signal handlers ---
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # --- Create and run node ---
    _node = DisplayNode(
        motor_endpoint=motor_endpoint,
        ups_endpoint=ups_endpoint,
        serial_port=serial_port,
        baud_rate=baud_rate,
        update_rate=update_rate,
    )
    _node.run()


if __name__ == "__main__":
    main()
