#!/usr/bin/env python3
"""E-Stop bridge: reads JSON lines from the receiver ESP32 over serial
and forwards state changes to motor_control via ZMQ PUSH on port 5555.

Usage:
    python bridge.py [--port /dev/estop-receiver] [--zmq tcp://localhost:5555]
"""

import argparse
import json
import time

import serial
import zmq


def main():
    parser = argparse.ArgumentParser(description="E-Stop serial to ZMQ bridge")
    parser.add_argument("--port", default="/dev/estop-receiver", help="Serial port of receiver ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--zmq", default="tcp://localhost:5555", help="ZMQ endpoint for motor_control commands")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.connect(args.zmq)

    print(f"[ESTOP-BRIDGE] serial={args.port} zmq={args.zmq}")

    prev_estop = None
    ser = None

    while True:
        # (Re)open serial if needed
        if ser is None:
            try:
                ser = serial.Serial(args.port, args.baud, timeout=2)
                print(f"[ESTOP-BRIDGE] opened {args.port}")
            except serial.SerialException:
                time.sleep(2)
                continue

        # Read a line (blocks up to timeout=2s, then returns empty)
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("[ESTOP-BRIDGE] serial error, reconnecting...")
            ser = None
            time.sleep(1)
            continue

        if not raw:
            continue

        line = raw.decode(errors="replace").strip()
        if not line:
            continue

        estop = None

        if line.startswith("#"):
            # Diagnostic: # nc=0 no=0 seq=1234 age=50
            # nc_raw is the raw G8 pin: 0 = pressed (e-stop active), 1 = released
            for part in line[2:].split():
                if part.startswith("nc="):
                    try:
                        estop = int(part[3:]) == 0
                    except ValueError:
                        pass
                    break
        elif line.startswith("{"):
            # JSON state change: {"estop": true}
            try:
                data = json.loads(line)
                estop = data.get("estop")
            except json.JSONDecodeError:
                pass

        if estop is None:
            continue

        if estop != prev_estop:
            cmd = {"type": "emergency_stop"} if estop else {"type": "clear_emergency_stop"}
            sock.send_string(json.dumps(cmd))
            print(f"[ESTOP-BRIDGE] sent {cmd['type']}")
            prev_estop = estop


if __name__ == "__main__":
    main()
