#!/usr/bin/env python3
"""
Actively query specific motor IDs by sending enable commands
and checking for responses
"""

import can
import time
import sys

def query_motor(bus, motor_id, timeout=0.5):
    """
    Send enable command to a motor and check for response

    Returns True if motor responds, False otherwise
    """
    # ROBSTRIDE enable command format:
    # CAN ID: motor_id | (0xAA << 8) | (0x03 << 24)
    # 0x03 = enable command
    can_id = motor_id | (0xAA << 8) | (0x03 << 24)

    # Enable command data (all zeros)
    data = [0x00] * 8

    # Send enable command
    msg = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=data
    )

    try:
        bus.send(msg)
        print(f"  Sent enable to motor 0x{motor_id:02X}... ", end='', flush=True)
    except Exception as e:
        print(f"ERROR sending: {e}")
        return False

    # Wait for response
    start_time = time.time()
    while time.time() - start_time < timeout:
        response = bus.recv(timeout=0.1)
        if response is None:
            continue

        # Check if response is from this motor
        resp_motor_id = response.arbitration_id & 0xFF
        if resp_motor_id == motor_id:
            print(f"✓ RESPONDED")
            return True

    print(f"✗ No response")
    return False

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else 'can1'
    motor_ids = [int(x, 0) for x in sys.argv[2:]] if len(sys.argv) > 2 else [0x02, 0x03, 0x04, 0x05]

    print(f"\nQuerying motors on {interface}:")
    print(f"Motor IDs to test: {[f'0x{m:02X}' for m in motor_ids]}\n")

    try:
        bus = can.Bus(interface='socketcan', channel=interface)
    except Exception as e:
        print(f"ERROR: Failed to open {interface}: {e}")
        return 1

    found_motors = []

    for motor_id in motor_ids:
        if query_motor(bus, motor_id):
            found_motors.append(motor_id)
        time.sleep(0.2)  # Small delay between queries

    bus.shutdown()

    print(f"\nResults:")
    print(f"  Found: {len(found_motors)} motor(s)")
    if found_motors:
        for mid in found_motors:
            print(f"    - Motor 0x{mid:02X} ({mid})")
    else:
        print(f"    No motors responded")

    return 0 if found_motors else 1

if __name__ == "__main__":
    sys.exit(main())
