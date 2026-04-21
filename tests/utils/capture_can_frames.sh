#!/bin/bash
# Capture CAN frames while motor_control runs
candump can1 -n 50 > /tmp/can_capture.txt &
CANDUMP_PID=

sleep 1

# Run motor control briefly
cd ~/aizee
timeout 2 ./rust/target/release/motor_control --config config/hardware_single_motor.yaml > /dev/null 2>&1 &
MOTOR_PID=

sleep 2

# Send a simple command via Python
python3 << 'PYEOF'
import zmq
import json
import time

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5555")
time.sleep(0.5)

# Enable motor
socket.send_json({"type": "enable", "motor_ids": ["right_wheel"]})
time.sleep(0.5)

# Drive briefly
socket.send_json({"type": "drive", "linear": 0.3, "angular": 0.0})
time.sleep(1)

# Stop
socket.send_json({"type": "drive", "linear": 0.0, "angular": 0.0})
time.sleep(0.3)

# Disable
socket.send_json({"type": "disable", "motor_ids": ["right_wheel"]})

socket.close()
context.term()
PYEOF

sleep 1

kill  2>/dev/null
kill  2>/dev/null
pkill -9 motor_control 2>/dev/null

cat /tmp/can_capture.txt
