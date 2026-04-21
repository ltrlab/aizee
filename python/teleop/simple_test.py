#!/usr/bin/env python3
"""
Simple teleop test script for AIZEE motor control
Sends basic commands and displays telemetry
"""

import zmq
import json
import time
import sys
from typing import Dict, Any

class SimpleTeleop:
    def __init__(self, command_addr="tcp://localhost:5555", telemetry_addr="tcp://localhost:5556"):
        self.context = zmq.Context()

        # PUSH socket for commands (PUSH-PULL pattern)
        self.cmd_pub = self.context.socket(zmq.PUSH)
        self.cmd_pub.connect(command_addr)
        print(f"Connected to command endpoint: {command_addr}")

        # Subscriber for telemetry
        self.telem_sub = self.context.socket(zmq.SUB)
        self.telem_sub.connect(telemetry_addr)
        self.telem_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.telem_sub.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        print(f"Connected to telemetry endpoint: {telemetry_addr}")

        # Allow time for ZMQ to establish connections
        time.sleep(0.5)

    def send_command(self, cmd: Dict[str, Any]):
        """Send a command to the motor controller"""
        msg = json.dumps(cmd)
        self.cmd_pub.send_string(msg)
        print(f"Sent: {cmd}")

    def enable_motors(self, motor_ids: list):
        """Enable specified motors"""
        self.send_command({
            "type": "enable",
            "motor_ids": motor_ids
        })

    def disable_motors(self, motor_ids: list):
        """Disable specified motors"""
        self.send_command({
            "type": "disable",
            "motor_ids": motor_ids
        })

    def emergency_stop(self):
        """Send emergency stop command"""
        self.send_command({"type": "emergency_stop"})

    def drive(self, linear: float, angular: float):
        """Send drive command"""
        self.send_command({
            "type": "drive",
            "linear": linear,
            "angular": angular
        })

    def move_arm(self, positions: list, velocities: list = None, kp: list = None, kd: list = None, torques: list = None):
        """Send arm joint command"""
        cmd = {
            "type": "arm_joints",
            "positions": positions,
            "velocities": velocities or [0.0] * len(positions),
            "torques": torques or [0.0] * len(positions),
        }
        if kp:
            cmd["kp"] = kp
        if kd:
            cmd["kd"] = kd
        self.send_command(cmd)

    def zero_position(self, motor_ids: list):
        """Zero motor positions"""
        self.send_command({
            "type": "zero_position",
            "motor_ids": motor_ids
        })

    def read_telemetry(self) -> Dict[str, Any]:
        """Read latest telemetry (non-blocking)"""
        try:
            msg = self.telem_sub.recv_string()
            return json.loads(msg)
        except zmq.Again:
            return None

    def print_telemetry(self):
        """Print latest telemetry in human-readable format"""
        telem = self.read_telemetry()
        if telem:
            print(f"\n=== Telemetry (t={telem['timestamp']:.3f}) ===")
            for motor_id, data in telem['motors'].items():
                error_str = f" ERROR: {data['error']}" if data.get('error') else ""
                print(f"  {motor_id:15s}: "
                      f"pos={data['position']:7.3f} rad, "
                      f"vel={data['velocity']:7.3f} rad/s, "
                      f"torque={data['torque']:6.2f} Nm, "
                      f"temp={data['temperature']:5.1f}°C"
                      f"{error_str}")

    def close(self):
        """Close ZMQ sockets"""
        self.cmd_pub.close()
        self.telem_sub.close()
        self.context.term()


def interactive_test():
    """Interactive testing mode"""
    teleop = SimpleTeleop()

    print("\n" + "="*60)
    print("AIZEE Simple Teleop Test")
    print("="*60)
    print("\nCommands:")
    print("  e <motor_id> [motor_id...]  - Enable motor(s)")
    print("  d <motor_id> [motor_id...]  - Disable motor(s)")
    print("  z <motor_id> [motor_id...]  - Zero position")
    print("  drive <linear> <angular>    - Drive base (e.g., drive 0.5 0.2)")
    print("  arm <pos1> <pos2> <pos3>    - Move arm joints (radians)")
    print("  stop                        - Emergency stop")
    print("  t                           - Print telemetry")
    print("  q                           - Quit")
    print()

    try:
        while True:
            # Print telemetry periodically
            teleop.print_telemetry()

            cmd = input("\n> ").strip().split()
            if not cmd:
                continue

            action = cmd[0].lower()

            if action == 'q':
                break
            elif action == 'e':
                teleop.enable_motors(cmd[1:])
            elif action == 'd':
                teleop.disable_motors(cmd[1:])
            elif action == 'z':
                teleop.zero_position(cmd[1:])
            elif action == 'drive' and len(cmd) == 3:
                teleop.drive(float(cmd[1]), float(cmd[2]))
            elif action == 'arm' and len(cmd) == 4:
                positions = [float(x) for x in cmd[1:]]
                teleop.move_arm(positions)
            elif action == 'stop':
                teleop.emergency_stop()
            elif action == 't':
                pass  # Telemetry already printed above
            else:
                print("Invalid command")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        teleop.close()


def automated_test():
    """Automated test sequence"""
    teleop = SimpleTeleop()

    print("\n" + "="*60)
    print("AIZEE Automated Test Sequence")
    print("="*60)

    try:
        # Test 1: Enable all motors
        print("\nTest 1: Enabling all motors...")
        teleop.enable_motors(["left_wheel", "right_wheel", "swivel",
                             "shoulder_pitch", "elbow", "wrist_gripper"])
        time.sleep(1)
        teleop.print_telemetry()

        # Test 2: Zero arm positions
        print("\nTest 2: Zeroing arm positions...")
        teleop.zero_position(["shoulder_pitch", "elbow", "wrist_gripper"])
        time.sleep(0.5)

        # Test 3: Move arm to home position
        print("\nTest 3: Moving arm to home position...")
        teleop.move_arm([0.0, 0.5, 0.0])
        time.sleep(2)
        teleop.print_telemetry()

        # Test 4: Drive forward
        print("\nTest 4: Driving forward...")
        teleop.drive(0.5, 0.0)
        time.sleep(1)
        teleop.print_telemetry()

        # Test 5: Stop
        print("\nTest 5: Stopping...")
        teleop.drive(0.0, 0.0)
        time.sleep(0.5)

        # Test 6: Disable all motors
        print("\nTest 6: Disabling all motors...")
        teleop.disable_motors(["left_wheel", "right_wheel", "swivel",
                              "shoulder_pitch", "elbow", "wrist_gripper"])
        time.sleep(0.5)
        teleop.print_telemetry()

        print("\n" + "="*60)
        print("Test sequence complete!")
        print("="*60)

    except Exception as e:
        print(f"Error during test: {e}")
        teleop.emergency_stop()

    finally:
        teleop.close()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "auto":
        automated_test()
    else:
        interactive_test()
