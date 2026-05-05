#!/usr/bin/env python3
"""
AIZEE UPS Power Monitoring Node

Monitors the Waveshare UPS Power Module (C) via I2C and publishes
power telemetry over ZeroMQ for visualization in Rerun and teleop display.

Hardware: Waveshare UPS Power Module (C) with INA219
I2C Address: 0x41 (default)
I2C Bus: 7 (Jetson Orin Nano)

Usage:
    python ups_node.py --config ../config/hardware_jetson_rover.yaml
    python ups_node.py --i2c-bus 7 --i2c-addr 0x41 --publish tcp://*:5562
"""

import argparse
import json
import logging
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import zmq
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg

# Import INA219 library
try:
    from .ina219 import INA219
except ImportError:
    # Fallback if running as script
    from ina219 import INA219


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class UPSTelemetry:
    """UPS power telemetry data"""
    timestamp: float
    voltage: float          # Volts
    current: float          # Amperes
    power: float            # Watts
    percentage: float       # Battery percentage (0-100)
    shunt_voltage: float    # Shunt voltage in V (for diagnostics)


class UPSMonitor:
    """Monitors Waveshare UPS Power Module and publishes telemetry"""

    def __init__(
        self,
        i2c_bus: int = 7,
        i2c_addr: int = 0x41,
        publish_endpoint: str = "tcp://*:5562",
        update_rate: float = 1.0,  # Hz
    ):
        """Initialize UPS monitor

        Args:
            i2c_bus: I2C bus number (7 for Jetson Orin Nano)
            i2c_addr: INA219 I2C address (0x41 default for UPS module)
            publish_endpoint: ZeroMQ PUB endpoint for telemetry
            update_rate: Publishing rate in Hz
        """
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr
        self.publish_endpoint = publish_endpoint
        self.update_interval = 1.0 / update_rate

        self.ina219: Optional[INA219] = None
        self.zmq_context: Optional[zmq.Context] = None
        self.telemetry_pub: Optional[zmq.Socket] = None
        self.running = False

        # Statistics
        self.message_count = 0
        self.last_stats_time = time.time()

    def initialize_hardware(self):
        """Initialize INA219 sensor"""
        logger.info(f"Initializing INA219 on I2C bus {self.i2c_bus}, address 0x{self.i2c_addr:02x}")

        try:
            self.ina219 = INA219(i2c_bus=self.i2c_bus, addr=self.i2c_addr)
            logger.info("INA219 initialized successfully")

            # Test read
            voltage = self.ina219.getBusVoltage_V()
            logger.info(f"Initial voltage reading: {voltage:.2f}V")

        except Exception as e:
            logger.error(f"Failed to initialize INA219: {e}")
            raise

    def initialize_zmq(self):
        """Initialize ZeroMQ publisher"""
        logger.info(f"Initializing ZeroMQ publisher at {self.publish_endpoint}")

        self.zmq_context = zmq.Context()
        self.telemetry_pub = self.zmq_context.socket(zmq.PUB)

        # Set socket options for low latency
        self.telemetry_pub.setsockopt(zmq.SNDHWM, 10)
        self.telemetry_pub.setsockopt(zmq.SNDBUF, 512 * 1024)

        self.telemetry_pub.bind(self.publish_endpoint)

        # Give time for subscribers to connect
        time.sleep(0.5)

        logger.info("ZeroMQ publisher initialized")

    def read_telemetry(self) -> UPSTelemetry:
        """Read current telemetry from UPS module

        Returns:
            UPSTelemetry object with current readings
        """
        timestamp = time.time()

        # Read all parameters from INA219
        bus_voltage = self.ina219.getBusVoltage_V()
        shunt_voltage_mv = self.ina219.getShuntVoltage_mV()
        current_ma = self.ina219.getCurrent_mA()
        power = self.ina219.getPower_W()

        # Convert to standard units
        shunt_voltage = shunt_voltage_mv / 1000.0  # mV to V
        current = current_ma / 1000.0  # mA to A

        # Calculate battery percentage (9V = 0%, 12.6V = 100%)
        # Adjust these values based on your battery chemistry
        voltage_min = 9.0
        voltage_max = 12.6
        percentage = ((bus_voltage - voltage_min) / (voltage_max - voltage_min)) * 100.0
        percentage = max(0.0, min(percentage, 100.0))

        return UPSTelemetry(
            timestamp=timestamp,
            voltage=bus_voltage,
            current=current,
            power=power,
            percentage=percentage,
            shunt_voltage=shunt_voltage
        )

    def publish_telemetry(self, telemetry: UPSTelemetry):
        """Publish telemetry over ZeroMQ

        Args:
            telemetry: UPSTelemetry object to publish
        """
        message = {
            "timestamp": telemetry.timestamp,
            "ups": {
                "voltage": telemetry.voltage,
                "current": telemetry.current,
                "power": telemetry.power,
                "percentage": telemetry.percentage,
                "shunt_voltage": telemetry.shunt_voltage,
            }
        }

        self.telemetry_pub.send(pack_msg(message))
        self.message_count += 1

    def run_loop(self):
        """Main monitoring loop"""
        logger.info("Starting UPS monitoring loop...")
        logger.info(f"Publishing at {1.0/self.update_interval:.1f} Hz")
        self.running = True

        next_publish_time = time.time()

        while self.running:
            try:
                current_time = time.time()

                # Check if it's time to publish
                if current_time >= next_publish_time:
                    # Read telemetry
                    telemetry = self.read_telemetry()

                    # Publish to ZeroMQ
                    self.publish_telemetry(telemetry)

                    # Log periodically
                    if current_time - self.last_stats_time >= 5.0:
                        elapsed = current_time - self.last_stats_time
                        rate = self.message_count / elapsed
                        logger.info(
                            f"UPS Status: {telemetry.voltage:.2f}V, "
                            f"{telemetry.current:.3f}A, "
                            f"{telemetry.power:.2f}W, "
                            f"{telemetry.percentage:.0f}% "
                            f"(Publishing at {rate:.1f} Hz)"
                        )
                        self.message_count = 0
                        self.last_stats_time = current_time

                    # Schedule next publish
                    next_publish_time += self.update_interval

                    # If we're behind schedule, catch up
                    if next_publish_time < current_time:
                        next_publish_time = current_time + self.update_interval

                # Sleep until next publish time
                sleep_time = next_publish_time - time.time()
                if sleep_time > 0:
                    time.sleep(min(sleep_time, 0.1))  # Wake up periodically to check running flag

            except KeyboardInterrupt:
                logger.info("Received interrupt signal")
                break
            except Exception as e:
                logger.error(f"Error in monitoring loop: {e}", exc_info=True)
                time.sleep(1.0)  # Backoff on error

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up UPS monitor...")

        self.running = False

        if self.telemetry_pub:
            self.telemetry_pub.close()

        if self.zmq_context:
            self.zmq_context.term()

        logger.info("Cleanup complete")

    def run(self):
        """Main run method"""
        try:
            self.initialize_hardware()
            self.initialize_zmq()

            logger.info("=" * 60)
            logger.info("UPS Power Monitor Ready")
            logger.info(f"I2C: Bus {self.i2c_bus}, Address 0x{self.i2c_addr:02x}")
            logger.info(f"Publishing to: {self.publish_endpoint}")
            logger.info("=" * 60)

            self.run_loop()

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


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file

    Args:
        config_path: Path to YAML config file

    Returns:
        Configuration dictionary
    """
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='AIZEE UPS Power Monitor - Waveshare UPS Module (C)'
    )
    parser.add_argument(
        '--config',
        type=str,
        help='Path to YAML config file'
    )
    parser.add_argument(
        '--i2c-bus',
        type=int,
        default=7,
        help='I2C bus number (default: 7 for Jetson)'
    )
    parser.add_argument(
        '--i2c-addr',
        type=lambda x: int(x, 0),  # Support hex notation like 0x41
        default=0x41,
        help='INA219 I2C address (default: 0x41)'
    )
    parser.add_argument(
        '--publish',
        type=str,
        default='tcp://*:5562',
        help='ZeroMQ publish endpoint (default: tcp://*:5562)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='Publishing rate in Hz (default: 1.0)'
    )

    args = parser.parse_args()

    # Load config if provided
    config = {}
    if args.config:
        config = load_config(args.config)
        logger.info(f"Loaded config from {args.config}")

    # Override with config values if present
    i2c_bus = config.get('ups', {}).get('i2c_bus', args.i2c_bus)
    i2c_addr = config.get('ups', {}).get('i2c_addr', args.i2c_addr)
    publish_endpoint = config.get('network', {}).get('device', {}).get('zmq', {}).get('ups_pub', args.publish)

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Create and run monitor
    monitor = UPSMonitor(
        i2c_bus=i2c_bus,
        i2c_addr=i2c_addr,
        publish_endpoint=publish_endpoint,
        update_rate=args.rate
    )

    monitor.run()


if __name__ == '__main__':
    main()
