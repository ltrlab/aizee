#!/usr/bin/env python3
"""Send ZeroMQ commands to motor_control"""
import zmq
import time
import json
import sys

def send_command(cmd_dict, endpoint="tcp://localhost:5555"):
    """Send a command to the motor control system"""
    context = zmq.Context()
    # Use PUSH to match motor_control's PULL socket
    sock = context.socket(zmq.PUSH)

    # Connect to motor_control's PULL socket
    sock.connect(endpoint)
    print(f"Connected PUSH to {endpoint}")

    # Allow socket to establish
    time.sleep(0.5)

    # Send command
    json_msg = json.dumps(cmd_dict)
    sock.send_string(json_msg)
    print(f"Sent: {json_msg}")

    # Allow message to be delivered
    time.sleep(0.1)

    sock.close()
    context.term()

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  send_zmq_command.py enable <motor_id> [motor_id...]")
        print("  send_zmq_command.py disable <motor_id> [motor_id...]")
        print("  send_zmq_command.py drive <linear> <angular>")
        print("  send_zmq_command.py zero <motor_id> [motor_id...]")
        print("  send_zmq_command.py estop")
        print("\nExamples:")
        print("  send_zmq_command.py enable left_wheel right_wheel")
        print("  send_zmq_command.py drive 0.5 0.0")
        print("  send_zmq_command.py disable left_wheel right_wheel")
        sys.exit(1)

    cmd_type = sys.argv[1].lower()

    if cmd_type == "enable":
        if len(sys.argv) < 3:
            print("Error: enable requires motor IDs")
            sys.exit(1)
        motor_ids = sys.argv[2:]
        cmd = {"type": "enable", "motor_ids": motor_ids}

    elif cmd_type == "disable":
        if len(sys.argv) < 3:
            print("Error: disable requires motor IDs")
            sys.exit(1)
        motor_ids = sys.argv[2:]
        cmd = {"type": "disable", "motor_ids": motor_ids}

    elif cmd_type == "drive":
        if len(sys.argv) < 4:
            print("Error: drive requires <linear> <angular>")
            sys.exit(1)
        linear = float(sys.argv[2])
        angular = float(sys.argv[3])
        cmd = {"type": "drive", "linear": linear, "angular": angular}

    elif cmd_type == "zero":
        if len(sys.argv) < 3:
            print("Error: zero requires motor IDs")
            sys.exit(1)
        motor_ids = sys.argv[2:]
        cmd = {"type": "zero_position", "motor_ids": motor_ids}

    elif cmd_type == "estop":
        cmd = {"type": "emergency_stop"}

    else:
        print(f"Error: unknown command type '{cmd_type}'")
        sys.exit(1)

    send_command(cmd)
    print("✓ Command sent successfully")

if __name__ == "__main__":
    main()
