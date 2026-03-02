#!/usr/bin/env python3
"""Test motor enable and check for feedback"""
import can
import time

HOST_CAN_ID = 0xAA
MSG_ENABLE = 3
MSG_CONTROL = 1

def build_can_id(motor_id, msg_type):
    return motor_id | (HOST_CAN_ID << 8) | (msg_type << 24)

def scan_for_motor():
    """Scan for motor responses on IDs 1-10"""
    bus = can.Bus(interface="socketcan", channel="can1", bitrate=1000000)

    print("Scanning for ROBSTRIDE motors (IDs 1-10)...")
    print("=" * 50)

    for motor_id in range(1, 11):
        can_id = build_can_id(motor_id, MSG_ENABLE)
        msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=bytes([0]*8))

        print(f"\nTrying motor ID {motor_id} (CAN ID: 0x{can_id:08X})...")
        bus.send(msg)

        # Wait for response
        start = time.time()
        while time.time() - start < 0.5:
            response = bus.recv(timeout=0.1)
            if response:
                resp_id = response.arbitration_id
                # Check if response is from this motor (motor ID in bits 8-15)
                resp_motor_id = (resp_id >> 8) & 0xFF
                if resp_motor_id == motor_id:
                    print(f"  ✓ FOUND! Motor {motor_id} responded:")
                    print(f"    Response CAN ID: 0x{resp_id:08X}")
                    print(f"    Data: {response.data.hex()}")
                    bus.shutdown()
                    return motor_id

        print(f"  ✗ No response")

    bus.shutdown()
    print("\n" + "=" * 50)
    print("No motors found")
    return None

if __name__ == "__main__":
    found_id = scan_for_motor()
    if found_id:
        print(f"\n✓ Motor detected at ID {found_id}")
    else:
        print("\n✗ No motors detected")
        print("\nPossible issues:")
        print("  1. Motor not powered")
        print("  2. CAN wiring disconnected")
        print("  3. Motor in error state")
        print("  4. Wrong CAN interface (check if can0 instead of can1)")
