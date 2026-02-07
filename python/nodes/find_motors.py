#!/usr/bin/env python3
"""
Scan CAN bus for ROBSTRIDE motors and display their IDs
Useful for initial setup and debugging
"""

import can
import time
import sys
from typing import Set, Dict

def scan_motors(interface='can0', timeout=5.0):
    """
    Scan CAN bus for ROBSTRIDE motor responses

    Args:
        interface: CAN interface name (default: can0)
        timeout: How long to listen for responses (seconds)

    Returns:
        Set of motor CAN IDs found
    """
    print(f"Scanning for motors on {interface}...")
    print(f"Listening for {timeout} seconds...")
    print()

    try:
        bus = can.interface.Bus(interface=interface, bustype='socketcan')
    except Exception as e:
        print(f"Error: Failed to open CAN interface {interface}")
        print(f"  {e}")
        print()
        print("Make sure:")
        print("  1. CAN interface is up: sudo ip link set can0 up")
        print("  2. You have permission: sudo usermod -a -G dialout $USER")
        return set()

    motors_found: Set[int] = set()
    motor_data: Dict[int, Dict] = {}

    start_time = time.time()

    try:
        while time.time() - start_time < timeout:
            msg = bus.recv(timeout=0.1)

            if msg is None:
                continue

            # Check for extended ID (ROBSTRIDE uses 29-bit extended)
            if not msg.is_extended_id:
                continue

            # Extract motor ID from arbitration ID (bits 0-7)
            motor_id = msg.arbitration_id & 0xFF

            # Extract message type (bits 24-28)
            msg_type = (msg.arbitration_id >> 24) & 0xFF

            # Common ROBSTRIDE message types: 2 (feedback), 19 (param response)
            if msg_type in [2, 19] and motor_id > 0:
                if motor_id not in motors_found:
                    motors_found.add(motor_id)
                    motor_data[motor_id] = {
                        'first_seen': time.time() - start_time,
                        'message_count': 0
                    }

                motor_data[motor_id]['message_count'] += 1
                motor_data[motor_id]['last_msg_type'] = msg_type
                motor_data[motor_id]['last_data'] = msg.data.hex()

    except KeyboardInterrupt:
        print("\nScan interrupted")

    finally:
        bus.shutdown()

    return motors_found, motor_data


def print_results(motors_found: Set[int], motor_data: Dict):
    """Print scan results in a nice format"""
    print()
    print("=" * 60)
    print("  Scan Results")
    print("=" * 60)
    print()

    if not motors_found:
        print("❌ No motors found!")
        print()
        print("Troubleshooting:")
        print("  1. Check motor power (40V supply connected)")
        print("  2. Verify CAN wiring (CAN-H, CAN-L, termination)")
        print("  3. Check if motors are transmitting:")
        print("     candump can0")
        print("  4. Try enabling a motor manually:")
        print("     cansend can0 03000100AA#0000000000000000")
        return

    print(f"✓ Found {len(motors_found)} motor(s):")
    print()

    for motor_id in sorted(motors_found):
        data = motor_data[motor_id]
        print(f"  Motor CAN ID: 0x{motor_id:02X} ({motor_id})")
        print(f"    First seen: {data['first_seen']:.2f}s")
        print(f"    Messages:   {data['message_count']}")
        print(f"    Last type:  {data['last_msg_type']}")
        print(f"    Last data:  {data['last_data']}")
        print()

    print("=" * 60)
    print()
    print("Add these to config/hardware.yaml:")
    print()

    for motor_id in sorted(motors_found):
        print(f"  - id: motor_{motor_id}")
        print(f"    can_id: 0x{motor_id:02X}")
        print(f"    type: ROBSTRIDE03  # Update based on your motor model")
        print(f"    max_velocity: 5.0")
        print(f"    max_torque: 8.0")
        print()


def main():
    """Main entry point"""
    interface = 'can0'
    timeout = 5.0

    # Parse command line arguments
    if len(sys.argv) > 1:
        interface = sys.argv[1]
    if len(sys.argv) > 2:
        timeout = float(sys.argv[2])

    print()
    print("=" * 60)
    print("  AIZEE Motor Scanner")
    print("=" * 60)
    print()
    print(f"Interface: {interface}")
    print(f"Timeout:   {timeout}s")
    print()
    print("This tool listens for motor feedback messages on the CAN bus.")
    print("Motors must be powered and enabled to be detected.")
    print()

    motors_found, motor_data = scan_motors(interface, timeout)
    print_results(motors_found, motor_data)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
