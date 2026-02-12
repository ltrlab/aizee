#!/usr/bin/env python3
"""
Test script for RPLiDAR telemetry over ZMQ.

Subscribes to LiDAR telemetry on port 5561 and displays scan statistics.

Usage:
    python test_lidar_telemetry.py [--host 192.168.0.27] [--port 5561]
"""

import argparse
import json
import sys
import time
from datetime import datetime

try:
    import zmq
except ImportError:
    print("ERROR: zmq module not found. Install with: pip install pyzmq")
    sys.exit(1)


def format_timestamp(ts):
    """Convert Unix timestamp to readable format."""
    return datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]


def analyze_scan(scan):
    """Analyze a single LiDAR scan and return statistics."""
    ranges = scan['ranges']
    intensities = scan['intensities']

    # Filter out invalid ranges (0.0 or beyond max range)
    valid_ranges = [r for r in ranges if 0.15 <= r <= 12.0]

    stats = {
        'sensor_id': scan['sensor_id'],
        'total_points': len(ranges),
        'valid_points': len(valid_ranges),
        'min_range': min(valid_ranges) if valid_ranges else 0.0,
        'max_range': max(valid_ranges) if valid_ranges else 0.0,
        'avg_range': sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0.0,
        'avg_intensity': sum(intensities) / len(intensities) if intensities else 0.0,
        'angle_increment_deg': scan['angle_increment'] * 180.0 / 3.14159,
    }

    return stats


def main():
    parser = argparse.ArgumentParser(description='Test RPLiDAR telemetry over ZMQ')
    parser.add_argument('--host', default='192.168.0.27', help='Jetson IP address')
    parser.add_argument('--port', type=int, default=5561, help='ZMQ telemetry port')
    parser.add_argument('--verbose', '-v', action='store_true', help='Show detailed scan info')
    args = parser.parse_args()

    endpoint = f"tcp://{args.host}:{args.port}"
    print(f"Connecting to LiDAR telemetry at {endpoint}...")

    # Create ZMQ context and subscriber
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.subscribe(b"")  # Subscribe to all messages

    # Set receive timeout
    sub.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout

    print("Waiting for telemetry messages... (Ctrl+C to exit)\n")

    message_count = 0
    last_print_time = time.time()

    try:
        while True:
            try:
                # Receive JSON message
                msg_str = sub.recv_string()
                msg = json.loads(msg_str)
                message_count += 1

                # Check if message contains LiDAR scans
                if 'lidar_scans' not in msg or not msg['lidar_scans']:
                    print(f"[WARNING] Message {message_count} has no lidar_scans")
                    continue

                timestamp = msg.get('timestamp', 0)
                scans = msg['lidar_scans']

                # Analyze each scan
                scan_stats = [analyze_scan(scan) for scan in scans]

                # Print summary
                current_time = time.time()
                if current_time - last_print_time >= 0.5 or args.verbose:  # Throttle output
                    print(f"\n[{format_timestamp(timestamp)}] Message #{message_count}")
                    print(f"  Received {len(scans)} scan(s):")

                    for stats in scan_stats:
                        print(f"    {stats['sensor_id']:15s}: "
                              f"{stats['valid_points']:3d}/{stats['total_points']:3d} points, "
                              f"range: {stats['min_range']:5.2f}m - {stats['max_range']:5.2f}m "
                              f"(avg: {stats['avg_range']:5.2f}m), "
                              f"intensity: {stats['avg_intensity']:5.1f}/255")

                        if args.verbose:
                            print(f"      Angle increment: {stats['angle_increment_deg']:.2f}°")

                    last_print_time = current_time

            except zmq.Again:
                print("[ERROR] Timeout waiting for message. Is lidar_control running?")
                print(f"  Check with: ssh ltr@{args.host} 'systemctl status aizee-lidar-control'")
                break

            except json.JSONDecodeError as e:
                print(f"[ERROR] Failed to decode JSON: {e}")
                continue

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        sub.close()
        ctx.term()
        print(f"\nReceived {message_count} messages total")


if __name__ == '__main__':
    main()
