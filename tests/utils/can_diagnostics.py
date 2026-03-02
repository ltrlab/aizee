#!/usr/bin/env python3
"""Comprehensive CAN bus diagnostics for ROBSTRIDE motor debugging"""
import subprocess
import time
import sys
import can

def run_cmd(cmd):
    """Run shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return "Command timed out"
    except Exception as e:
        return f"Error: {e}"

def section(title):
    """Print section header"""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)

def test_can_interface(interface):
    """Test specific CAN interface"""
    print(f"\n--- Testing {interface} ---")

    # Interface details
    print(f"\n1. Interface Status:")
    print(run_cmd(f"ip -details link show {interface}"))

    # Statistics
    print(f"\n2. Interface Statistics:")
    print(run_cmd(f"ip -s link show {interface}"))

    # Error counters
    print(f"\n3. Error Counters:")
    print(run_cmd(f"cat /sys/class/net/{interface}/statistics/tx_errors 2>/dev/null || echo 'N/A'"))
    print(run_cmd(f"cat /sys/class/net/{interface}/statistics/rx_errors 2>/dev/null || echo 'N/A'"))

def test_usb_can_adapter():
    """Check USB-CAN adapter hardware"""
    section("USB-CAN Adapter Hardware")

    print("\n1. USB Devices:")
    print(run_cmd("lsusb | grep -i 'can\\|serial\\|ch340\\|ftdi\\|peak\\|kvaser' || lsusb"))

    print("\n2. USB Serial Devices:")
    print(run_cmd("ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'No USB serial devices'"))

    print("\n3. CAN Device Info:")
    print(run_cmd("ls -la /sys/class/net/can* 2>/dev/null"))

    print("\n4. Kernel Messages (CAN related):")
    print(run_cmd("dmesg | grep -i can | tail -20"))

def test_can_configuration():
    """Test CAN bus configuration"""
    section("CAN Configuration")

    for interface in ['can0', 'can1']:
        test_can_interface(interface)

def test_bitrates():
    """Test different bitrates on can1"""
    section("Testing Different Bitrates")

    interface = 'can1'
    bitrates = [1000000, 500000, 250000]

    for bitrate in bitrates:
        print(f"\n--- Testing {bitrate} bps ---")

        # Bring down interface
        run_cmd(f"sudo ip link set {interface} down")
        time.sleep(0.5)

        # Set new bitrate
        result = run_cmd(f"sudo ip link set {interface} type can bitrate {bitrate}")
        if result:
            print(f"Set bitrate result: {result}")

        # Bring up interface
        result = run_cmd(f"sudo ip link set {interface} up")
        if result:
            print(f"Bring up result: {result}")

        time.sleep(0.5)

        # Try to send enable command
        try:
            bus = can.Bus(interface='socketcan', channel=interface, bitrate=bitrate)

            # Motor ID 2, Enable command
            HOST_CAN_ID = 0xAA
            MSG_ENABLE = 3
            motor_id = 2
            can_id = motor_id | (HOST_CAN_ID << 8) | (MSG_ENABLE << 24)

            msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=bytes([0]*8))
            bus.send(msg)
            print(f"✓ Sent enable command at {bitrate} bps")

            # Wait for response
            response = bus.recv(timeout=0.5)
            if response:
                print(f"✓✓ RESPONSE! CAN ID: 0x{response.arbitration_id:08X}")
                print(f"   Data: {response.data.hex()}")
                bus.shutdown()
                return bitrate
            else:
                print(f"✗ No response at {bitrate} bps")

            bus.shutdown()
        except Exception as e:
            print(f"✗ Error at {bitrate} bps: {e}")

        time.sleep(0.5)

    # Restore to 1 Mbps
    run_cmd(f"sudo ip link set {interface} down")
    run_cmd(f"sudo ip link set {interface} type can bitrate 1000000")
    run_cmd(f"sudo ip link set {interface} up")

    return None

def test_loopback():
    """Test CAN loopback mode to verify adapter works"""
    section("CAN Loopback Test")

    interface = 'can1'

    print(f"\n1. Setting {interface} to loopback mode...")
    run_cmd(f"sudo ip link set {interface} down")
    run_cmd(f"sudo ip link set {interface} type can bitrate 1000000 loopback on")
    run_cmd(f"sudo ip link set {interface} up")
    time.sleep(0.5)

    print(f"2. Sending test frame and checking if we receive it back...")
    try:
        bus = can.Bus(interface='socketcan', channel=interface, bitrate=1000000)

        # Send test frame
        test_msg = can.Message(arbitration_id=0x123, is_extended_id=False, data=bytes([1,2,3,4,5,6,7,8]))
        bus.send(test_msg)
        print("✓ Sent test frame")

        # Try to receive it back
        response = bus.recv(timeout=1.0)
        if response:
            print(f"✓✓ LOOPBACK WORKS! Received: ID=0x{response.arbitration_id:03X}, Data={response.data.hex()}")
            result = True
        else:
            print("✗✗ LOOPBACK FAILED - No frame received back!")
            print("   This suggests the CAN adapter hardware is NOT working correctly")
            result = False

        bus.shutdown()
    except Exception as e:
        print(f"✗✗ Loopback test error: {e}")
        result = False

    # Restore normal mode
    print(f"\n3. Restoring {interface} to normal mode...")
    run_cmd(f"sudo ip link set {interface} down")
    run_cmd(f"sudo ip link set {interface} type can bitrate 1000000 loopback off")
    run_cmd(f"sudo ip link set {interface} up")

    return result

def test_raw_can_frames():
    """Monitor for ANY CAN activity"""
    section("Raw CAN Frame Monitoring")

    print("\nMonitoring can1 for ANY CAN traffic (10 seconds)...")
    print("If you see ANY frames here, the adapter is receiving data")
    print("Starting in 2 seconds... (manually move/power-cycle motor if possible)\n")
    time.sleep(2)

    result = run_cmd("timeout 10 candump can1 2>&1")
    if result.strip():
        print("CAN traffic detected:")
        print(result)
        return True
    else:
        print("✗ No CAN traffic detected in 10 seconds")
        return False

def test_motor_power_detection():
    """Try to detect if motor is sending spontaneous frames"""
    section("Motor Power-On Detection")

    print("\nSome motors send frames when powered on...")
    print("Power cycling the motor now (if possible) while monitoring...")
    print("\nListening for 15 seconds...")

    try:
        bus = can.Bus(interface='socketcan', channel='can1', bitrate=1000000)

        start_time = time.time()
        frame_count = 0

        while time.time() - start_time < 15:
            msg = bus.recv(timeout=1.0)
            if msg:
                frame_count += 1
                print(f"✓ Frame {frame_count}: ID=0x{msg.arbitration_id:08X}, Data={msg.data.hex()}")

        bus.shutdown()

        if frame_count > 0:
            print(f"\n✓ Detected {frame_count} frames from motor!")
            return True
        else:
            print("\n✗ No spontaneous frames from motor")
            return False

    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("""
╔════════════════════════════════════════════════════════════╗
║     ROBSTRIDE CAN Bus Diagnostics                          ║
║     Testing USB-CAN adapter and motor connectivity         ║
╚════════════════════════════════════════════════════════════╝
""")

    # Test 1: USB-CAN adapter hardware
    test_usb_can_adapter()

    # Test 2: CAN configuration
    test_can_configuration()

    # Test 3: Loopback test (critical - verifies adapter works)
    loopback_works = test_loopback()

    if not loopback_works:
        print("\n" + "!"*60)
        print("! CRITICAL: Loopback test FAILED!")
        print("! The USB-CAN adapter is not working correctly.")
        print("! This must be fixed before motor communication will work.")
        print("!"*60)
        return

    # Test 4: Try different bitrates
    working_bitrate = test_bitrates()

    if working_bitrate:
        print(f"\n✓✓✓ Motor responded at {working_bitrate} bps!")
        return

    # Test 5: Monitor for any CAN activity
    has_traffic = test_raw_can_frames()

    # Test 6: Try to detect motor power-on
    if not has_traffic:
        test_motor_power_detection()

    # Final recommendations
    section("Diagnostic Summary")

    if loopback_works:
        print("\n✓ CAN adapter hardware: WORKING")
    else:
        print("\n✗ CAN adapter hardware: FAILED")
        print("  → Check USB connection")
        print("  → Try different USB port")
        print("  → Check adapter driver")

    if working_bitrate:
        print(f"✓ Motor communication: WORKING at {working_bitrate} bps")
    else:
        print("\n✗ Motor communication: NO RESPONSE")
        print("\nPossible causes:")
        print("  1. Motor not powered (check LED on motor)")
        print("  2. CAN_H and CAN_L wires swapped")
        print("  3. Motor ID is not 2 (need to scan all IDs)")
        print("  4. Motor in fault/error state (needs power cycle)")
        print("  5. Missing CAN termination resistors (120Ω)")
        print("  6. Motor firmware issue")
        print("\nNext steps:")
        print("  1. Verify motor LED is on")
        print("  2. Power cycle the motor")
        print("  3. Check CAN wiring polarity")
        print("  4. Verify 120Ω termination at both ends of CAN bus")
        print("  5. Try with ROBSTRIDE's official software (if available)")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nDiagnostics interrupted by user")
        sys.exit(0)
