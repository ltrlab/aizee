#!/usr/bin/env python3
"""
Test script for RPi4 arm module
Sends basic commands and monitors telemetry

Usage:
    python test_arm_module.py                    # default: 192.168.0.28
    python test_arm_module.py --host 192.168.0.28
"""

import argparse
import json
import time
import zmq


def main():
    parser = argparse.ArgumentParser(description="Test AIZEE arm module")
    parser.add_argument("--host", default="192.168.0.28", help="RPi4 IP address")
    args = parser.parse_args()

    cmd_addr = f"tcp://{args.host}:5557"
    telem_addr = f"tcp://{args.host}:5558"

    print(f"=== AIZEE Arm Module Test ===")
    print(f"Command:   {cmd_addr}")
    print(f"Telemetry: {telem_addr}")
    print()

    ctx = zmq.Context()

    # Command socket
    cmd = ctx.socket(zmq.PUSH)
    cmd.connect(cmd_addr)

    # Telemetry socket
    telem = ctx.socket(zmq.SUB)
    telem.connect(telem_addr)
    telem.setsockopt_string(zmq.SUBSCRIBE, "")
    telem.setsockopt(zmq.RCVTIMEO, 1000)

    arm_motors = ["shoulder_pitch", "elbow", "wrist"]

    try:
        # Wait for telemetry
        print("Waiting for telemetry...")
        for _ in range(5):
            try:
                msg = telem.recv_string()
                data = json.loads(msg)
                print(f"✓ Telemetry received: {len(data.get('motors', {}))} motors")
                break
            except zmq.Again:
                time.sleep(0.5)
        else:
            print("⚠ No telemetry received (motor_control may not be running)")
            return

        # Enable arm motors
        print(f"\nEnabling motors: {arm_motors}")
        cmd.send_string(json.dumps({
            "type": "enable",
            "motor_ids": arm_motors
        }))
        time.sleep(3)

        # Check telemetry
        try:
            msg = telem.recv_string(zmq.NOBLOCK)
            data = json.loads(msg)
            motors = data.get("motors", {})
            for motor_id in arm_motors:
                if motor_id in motors:
                    state = motors[motor_id].get("state", "unknown")
                    print(f"  {motor_id}: {state}")
        except zmq.Again:
            pass

        # Send test positions
        print("\nSending test arm positions...")
        positions = [0.1, 0.5, -0.3]
        cmd.send_string(json.dumps({
            "type": "arm_joints",
            "positions": positions,
            "velocities": [0.0, 0.0, 0.0]
        }))
        print(f"  Target positions: {positions}")
        time.sleep(3)

        # Monitor telemetry
        print("\nMonitoring telemetry (5 seconds)...")
        start = time.time()
        while time.time() - start < 5:
            try:
                msg = telem.recv_string(zmq.NOBLOCK)
                data = json.loads(msg)
                motors = data.get("motors", {})
                for motor_id in arm_motors:
                    if motor_id in motors:
                        m = motors[motor_id]
                        pos = m.get("position", 0.0)
                        vel = m.get("velocity", 0.0)
                        temp = m.get("temperature", 0.0)
                        print(f"  {motor_id}: pos={pos:+.3f} vel={vel:+.3f} T={temp:.0f}°C")
            except zmq.Again:
                time.sleep(0.1)

        # Disable motors
        print("\nDisabling motors...")
        cmd.send_string(json.dumps({
            "type": "disable",
            "motor_ids": arm_motors
        }))
        time.sleep(1)

        print("\n✓ Test complete")

    except KeyboardInterrupt:
        print("\n\nInterrupted - disabling motors...")
        cmd.send_string(json.dumps({
            "type": "disable",
            "motor_ids": arm_motors
        }))
        time.sleep(0.5)

    finally:
        cmd.close()
        telem.close()
        ctx.term()


if __name__ == "__main__":
    main()
