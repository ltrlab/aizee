#!/usr/bin/env python3
"""Test motor at different CAN bitrates"""
import can
import time
import subprocess
import sys

HOST_CAN_ID = 0xAA
MSG_ENABLE = 3
MOTOR_ID = 2

def test_bitrate(bitrate):
    """Test communication at specific bitrate"""
    print(f"\n{'='*60}")
    print(f"  Testing bitrate: {bitrate:,} bps")
    print('='*60)

    # Configure CAN interface
    subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 down", shell=True, capture_output=True)
    subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 type can bitrate {bitrate}", shell=True, capture_output=True)
    result = subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 up", shell=True, capture_output=True)

    if result.returncode != 0:
        print(f"Failed to set bitrate {bitrate}")
        return False

    time.sleep(0.5)

    try:
        bus = can.Bus(interface='socketcan', channel='can1', bitrate=bitrate)

        # Build enable command
        can_id = MOTOR_ID | (HOST_CAN_ID << 8) | (MSG_ENABLE << 24)

        # Send multiple enable commands
        print(f"Sending 5 enable commands to motor {MOTOR_ID}...")
        for i in range(5):
            msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=bytes([0]*8))
            bus.send(msg)
            time.sleep(0.05)

        # Listen for responses
        print("Listening for responses (2 seconds)...")
        start = time.time()
        response_count = 0

        while time.time() - start < 2.0:
            response = bus.recv(timeout=0.5)
            if response:
                response_count += 1
                print(f"\n✓✓✓ RESPONSE {response_count} DETECTED!")
                print(f"    CAN ID: 0x{response.arbitration_id:08X}")
                print(f"    Data: {response.data.hex()}")

                # Parse motor ID from response
                resp_motor_id = (response.arbitration_id >> 8) & 0xFF
                print(f"    Motor ID in response: {resp_motor_id}")

                bus.shutdown()
                return True

        if response_count == 0:
            print(f"✗ No response at {bitrate:,} bps")

        bus.shutdown()
        return False

    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("ROBSTRIDE Motor Bitrate Detection")
    print("Testing common CAN bitrates...")

    bitrates = [
        1000000,  # 1 Mbps (most common for ROBSTRIDE)
        500000,   # 500 kbps
        250000,   # 250 kbps
        125000,   # 125 kbps (less common)
    ]

    for bitrate in bitrates:
        if test_bitrate(bitrate):
            print(f"\n{'='*60}")
            print(f"✓✓✓ MOTOR FOUND AT {bitrate:,} bps!")
            print('='*60)

            # Restore this bitrate
            subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 down", shell=True, capture_output=True)
            subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 type can bitrate {bitrate}", shell=True, capture_output=True)
            subprocess.run(f"echo changeme\\!123 | sudo -S ip link set can1 up", shell=True, capture_output=True)

            print(f"\ncan1 configured to {bitrate:,} bps")
            return

        time.sleep(0.5)

    # No motor found, restore to 1 Mbps
    print(f"\n{'='*60}")
    print("✗ Motor not found at any tested bitrate")
    print('='*60)
    print("\nRestoring can1 to 1 Mbps...")
    subprocess.run("sudo ip link set can1 down", shell=True, capture_output=True)
    subprocess.run("sudo ip link set can1 type can bitrate 1000000", shell=True, capture_output=True)
    subprocess.run("sudo ip link set can1 up", shell=True, capture_output=True)

    print("\nPossible issues:")
    print("  1. Motor powered but CAN not connected")
    print("  2. CAN_H and CAN_L wires swapped")
    print("  3. Motor using non-standard bitrate")
    print("  4. Motor in fault state - try power cycling")
    print("  5. Motor on can0 instead of can1")

if __name__ == "__main__":
    main()
