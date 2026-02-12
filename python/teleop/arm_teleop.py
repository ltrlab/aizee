#!/usr/bin/env python3
"""
AIZEE Arm Teleop - Direct CAN control for gantry/arm motors on can2

Controls motors 0x05, 0x06, 0x07 (RB04, RB03, RB02) via direct CAN commands.
Includes fault clearing and enable/disable functionality.

Usage:
    python arm_teleop.py

Controls:
    E - Enable all arm motors
    D - Disable all arm motors
    C - Clear faults on all arm motors
    R - Reset (clear faults + enable)
    Z - Zero position (set current position as zero)
    Q/ESC - Quit

    Arrow keys - Control arm positions (when enabled)
"""

import can
import sys
import time
import termios
import tty
import select

# Motor configuration for gantry/arm on can2
ARM_MOTORS = {
    'gantry_base': {'id': 0x05, 'model': 'RB04', 'name': 'Gantry Base'},
    'gantry_mid': {'id': 0x06, 'model': 'RB03', 'name': 'Gantry Mid'},
    'gantry_end': {'id': 0x07, 'model': 'RB02', 'name': 'Gantry End'},
}

CAN_INTERFACE = 'can2'
HOST_ID = 0xAA

# Message types
MSG_TYPE_CONTROL = 1
MSG_TYPE_ENABLE = 3
MSG_TYPE_DISABLE = 4
MSG_TYPE_ZERO_POS = 6


class ArmTeleop:
    def __init__(self):
        self.bus = can.Bus(interface='socketcan', channel=CAN_INTERFACE, bitrate=1000000)
        self.running = False
        self.motors_enabled = False

    def build_frame(self, motor_id, msg_type, data=None):
        """Build a CAN frame for ROBSTRIDE protocol."""
        if data is None:
            data = [0x00] * 8
        arb_id = HOST_ID | (motor_id << 8) | (msg_type << 24)
        return can.Message(arbitration_id=arb_id, is_extended_id=True, data=data)

    def send_disable(self, motor_id):
        """Send disable command to clear fault state."""
        frame = self.build_frame(motor_id, MSG_TYPE_DISABLE)
        self.bus.send(frame)

    def send_enable(self, motor_id):
        """Send enable command."""
        frame = self.build_frame(motor_id, MSG_TYPE_ENABLE)
        self.bus.send(frame)

    def send_zero_position(self, motor_id):
        """Send zero position command."""
        frame = self.build_frame(motor_id, MSG_TYPE_ZERO_POS)
        self.bus.send(frame)

    def clear_faults_all(self):
        """Clear faults on all arm motors."""
        print("\n🔧 Clearing faults on all motors...")
        for name, motor in ARM_MOTORS.items():
            motor_id = motor['id']
            print(f"  {motor['name']} (0x{motor_id:02X}): ", end='', flush=True)

            # Send disable to clear fault
            self.send_disable(motor_id)
            time.sleep(0.05)

            # Send enable
            self.send_enable(motor_id)
            time.sleep(0.05)

            # Check for response
            timeout = time.time() + 0.5
            responded = False
            while time.time() < timeout:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.is_extended_id:
                    resp_id = (msg.arbitration_id >> 8) & 0xFF
                    if resp_id == motor_id:
                        print("✓ RESPONDED")
                        responded = True
                        break

            if not responded:
                print("✗ No response")

        print()

    def enable_all(self):
        """Enable all arm motors."""
        print("\n✅ Enabling all motors...")
        for name, motor in ARM_MOTORS.items():
            motor_id = motor['id']
            print(f"  {motor['name']} (0x{motor_id:02X}): ", end='', flush=True)

            # Send enable multiple times
            for _ in range(5):
                self.send_enable(motor_id)
                time.sleep(0.02)

            # Check for response
            timeout = time.time() + 0.5
            responded = False
            while time.time() < timeout:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.is_extended_id:
                    resp_id = (msg.arbitration_id >> 8) & 0xFF
                    if resp_id == motor_id:
                        print("✓ Enabled")
                        responded = True
                        self.motors_enabled = True
                        break

            if not responded:
                print("✗ Failed to enable")

        print()

    def disable_all(self):
        """Disable all arm motors."""
        print("\n❌ Disabling all motors...")
        for name, motor in ARM_MOTORS.items():
            motor_id = motor['id']
            self.send_disable(motor_id)
            print(f"  {motor['name']} (0x{motor_id:02X}): Disabled")
            time.sleep(0.05)

        self.motors_enabled = False
        print()

    def zero_position_all(self):
        """Zero position on all arm motors."""
        print("\n🎯 Setting zero position on all motors...")
        for name, motor in ARM_MOTORS.items():
            motor_id = motor['id']
            self.send_zero_position(motor_id)
            print(f"  {motor['name']} (0x{motor_id:02X}): Position zeroed")
            time.sleep(0.05)

        print()

    def reset_all(self):
        """Reset all motors (clear faults + enable)."""
        print("\n♻️  Resetting all motors (clear faults + enable)...")
        self.clear_faults_all()
        time.sleep(0.2)
        self.enable_all()

    def print_help(self):
        """Print control help."""
        print("\n" + "="*60)
        print(" AIZEE Arm Teleop - Gantry/Arm Motor Control")
        print("="*60)
        print("\nMotors on CAN2:")
        for name, motor in ARM_MOTORS.items():
            print(f"  0x{motor['id']:02X} - {motor['name']} ({motor['model']})")
        print("\nControls:")
        print("  E - Enable all motors")
        print("  D - Disable all motors")
        print("  C - Clear faults")
        print("  R - Reset (clear faults + enable)")
        print("  Z - Zero position")
        print("  Q / ESC - Quit")
        print("\nStatus:")
        print(f"  Motors enabled: {self.motors_enabled}")
        print("="*60)
        print()

    def run(self):
        """Main teleop loop."""
        self.running = True

        # Setup terminal for raw input
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            self.print_help()
            print("Ready! Press a key to start...")

            while self.running:
                # Check for keyboard input (non-blocking)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'e':
                        self.enable_all()
                    elif key == 'd':
                        self.disable_all()
                    elif key == 'c':
                        self.clear_faults_all()
                    elif key == 'r':
                        self.reset_all()
                    elif key == 'z':
                        self.zero_position_all()
                    elif key == 'q' or ord(key) == 27:  # Q or ESC
                        print("\nQuitting...")
                        self.running = False
                    elif key == 'h' or key == '?':
                        self.print_help()
                    else:
                        print(f"Unknown command: {key} (press H for help)")

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

            # Cleanup
            print("\nCleaning up...")
            self.bus.shutdown()
            print("Goodbye!")


def main():
    try:
        teleop = ArmTeleop()
        teleop.run()
    except KeyboardInterrupt:
        print("\nInterrupted!")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
