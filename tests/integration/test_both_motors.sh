#!/bin/bash
cd ~/aizee

pkill -9 motor_control
sleep 1

echo 'Starting motor_control...'
AIZEE_CONFIG=config/hardware_two_motors.yaml ./rust/target/release/motor_control > /tmp/zeroed_test.log 2>&1 &
MOTOR_PID=$!

sleep 3

echo 'Enabling motors...'
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "enable", "motor_ids": ["left_wheel", "right_wheel"]}); s.close(); c.term()'

sleep 1

echo 'Zeroing positions...'
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "zero_position", "motor_ids": ["left_wheel", "right_wheel"]}); s.close(); c.term()'

sleep 1

echo 'Driving forward slowly (0.3 rad/s) for 4 seconds...'
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "drive", "linear": 0.3, "angular": 0.0}); s.close(); c.term()'

sleep 4

echo 'Stopping...'
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "drive", "linear": 0.0, "angular": 0.0}); s.close(); c.term()'

sleep 1

echo 'Disabling...'
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "disable", "motor_ids": ["left_wheel", "right_wheel"]}); s.close(); c.term()'

sleep 1
kill $MOTOR_PID 2>/dev/null
pkill -9 motor_control

echo ''
echo 'Test complete! Should have been smoother with position zeroing.'
