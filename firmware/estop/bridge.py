#!/usr/bin/env python3
"""E-Stop bridge: reads JSON lines from the receiver ESP32 over serial
and forwards state changes to motor_control via ZMQ PUSH on port 5555.

Usage:
    python bridge.py [--port /dev/ttyACM0] [--zmq tcp://localhost:5555]
"""

import argparse
import json
import serial
import zmq


def main():
    parser = argparse.ArgumentParser(description="E-Stop serial→ZMQ bridge")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port of receiver ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--zmq", default="tcp://localhost:5555", help="ZMQ endpoint for motor_control commands")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.connect(args.zmq)

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"[ESTOP-BRIDGE] serial={args.port} zmq={args.zmq}")

    prev_estop = None
    for raw in ser:
        line = raw.decode(errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        estop = data.get("estop")
        if estop is None:
            continue

        if estop != prev_estop:
            cmd = {"type": "emergency_stop"} if estop else {"type": "clear_emergency_stop"}
            sock.send_string(json.dumps(cmd))
            print(f"[ESTOP-BRIDGE] sent {cmd['type']}")
            prev_estop = estop


if __name__ == "__main__":
    main()
