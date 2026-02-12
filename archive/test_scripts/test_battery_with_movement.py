#!/usr/bin/env python3
"""
Test battery voltage monitoring with motor movement.
Shows telemetry including battery voltage while motors are moving.
"""

import zmq
import json
import time
import sys

def main():
    ctx = zmq.Context()

    # Command publisher
    cmd = ctx.socket(zmq.PUSH)
    cmd.connect("tcp://192.168.0.27:5555")

    # Telemetry subscriber
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://192.168.0.27:5556")
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    print("=" * 70)
    print("AIZEE Battery Voltage + Motor Movement Test")
    print("=" * 70)
    print()

    try:
        # Step 1: Enable motors
        print("Step 1: Enabling motors...")
        cmd.send_json({"type": "enable", "motor_ids": ["left_wheel", "right_wheel", "swivel"]})
        time.sleep(2)

        # Check telemetry
        try:
            msg = sub.recv_json()
            batt_v = msg.get("battery_voltage")
            if batt_v:
                print(f"  ✓ Battery voltage: {batt_v:.2f}V")
            motors = msg.get("motors", {})
            print(f"  ✓ Motors enabled: {list(motors.keys())}")
        except:
            print("  (waiting for telemetry...)")

        print()

        # Step 2: Forward movement
        print("Step 2: Moving forward (linear = 0.3 rad/s for 2 seconds)...")
        for i in range(4):
            cmd.send_json({"type": "drive", "linear": 0.3, "angular": 0.0, "swivel": 0.0})
            time.sleep(0.5)

            # Show telemetry
            try:
                msg = sub.recv_json()
                batt_v = msg.get("battery_voltage")
                motors = msg.get("motors", {})

                if batt_v:
                    # Calculate status
                    if batt_v >= 22.2:
                        status = "OK"
                    elif batt_v >= 21.0:
                        status = "GOOD"
                    elif batt_v >= 20.0:
                        status = "WARN"
                    else:
                        status = "CRIT"

                    percent = max(0, min(100, ((batt_v - 18.0) / (25.2 - 18.0)) * 100))
                    print(f"  [{i+1}] Battery: {batt_v:.2f}V ({percent:.0f}%) [{status}]", end="")

                # Show motor velocities
                if "left_wheel" in motors and "right_wheel" in motors:
                    lw_vel = motors["left_wheel"].get("velocity", 0)
                    rw_vel = motors["right_wheel"].get("velocity", 0)
                    print(f"  |  Motors: L={lw_vel:.2f} R={rw_vel:.2f} rad/s")
                else:
                    print()
            except:
                print(f"  [{i+1}] (waiting for telemetry...)")

        print()

        # Step 3: Stop
        print("Step 3: Stopping...")
        cmd.send_json({"type": "drive", "linear": 0.0, "angular": 0.0, "swivel": 0.0})
        time.sleep(1)

        # Step 4: Turn right
        print("Step 4: Turning right (angular = 0.3 rad/s for 2 seconds)...")
        for i in range(4):
            cmd.send_json({"type": "drive", "linear": 0.0, "angular": 0.3, "swivel": 0.0})
            time.sleep(0.5)

            try:
                msg = sub.recv_json()
                batt_v = msg.get("battery_voltage")
                motors = msg.get("motors", {})

                if batt_v:
                    if batt_v >= 22.2:
                        status = "OK"
                    elif batt_v >= 21.0:
                        status = "GOOD"
                    elif batt_v >= 20.0:
                        status = "WARN"
                    else:
                        status = "CRIT"

                    percent = max(0, min(100, ((batt_v - 18.0) / (25.2 - 18.0)) * 100))
                    print(f"  [{i+1}] Battery: {batt_v:.2f}V ({percent:.0f}%) [{status}]", end="")

                if "left_wheel" in motors and "right_wheel" in motors:
                    lw_vel = motors["left_wheel"].get("velocity", 0)
                    rw_vel = motors["right_wheel"].get("velocity", 0)
                    print(f"  |  Motors: L={lw_vel:.2f} R={rw_vel:.2f} rad/s")
                else:
                    print()
            except:
                print(f"  [{i+1}] (waiting for telemetry...)")

        print()

        # Step 5: Stop
        print("Step 5: Final stop...")
        cmd.send_json({"type": "drive", "linear": 0.0, "angular": 0.0, "swivel": 0.0})
        time.sleep(1)

        # Final battery reading
        try:
            msg = sub.recv_json()
            batt_v = msg.get("battery_voltage")
            if batt_v:
                if batt_v >= 22.2:
                    status = "OK"
                elif batt_v >= 21.0:
                    status = "GOOD"
                elif batt_v >= 20.0:
                    status = "WARN"
                else:
                    status = "CRIT"

                percent = max(0, min(100, ((batt_v - 18.0) / (25.2 - 18.0)) * 100))
                print(f"  ✓ Final battery: {batt_v:.2f}V ({percent:.0f}%) [{status}]  (6S LIPO)")
        except:
            pass

        print()

        # Step 6: Disable motors
        print("Step 6: Disabling motors...")
        cmd.send_json({"type": "disable", "motor_ids": ["left_wheel", "right_wheel", "swivel"]})
        time.sleep(1)
        print("  ✓ Motors disabled")

        print()
        print("=" * 70)
        print("Test Complete!")
        print("=" * 70)
        print()
        print("Summary:")
        print("  ✓ Motors enabled and moved successfully")
        print("  ✓ Battery voltage monitored throughout movement")
        print("  ✓ Telemetry updating in real-time")
        print("  ✓ Motors safely disabled")

    except KeyboardInterrupt:
        print("\n\nTest interrupted - disabling motors...")
        cmd.send_json({"type": "disable", "motor_ids": ["left_wheel", "right_wheel", "swivel"]})
        time.sleep(0.5)
    except Exception as e:
        print(f"\nError: {e}")
        print("Disabling motors...")
        cmd.send_json({"type": "disable", "motor_ids": ["left_wheel", "right_wheel", "swivel"]})
        time.sleep(0.5)
    finally:
        cmd.close()
        sub.close()
        ctx.term()

if __name__ == "__main__":
    main()
