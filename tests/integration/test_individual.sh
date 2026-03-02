#!/bin/bash
cd ~/aizee
pkill -9 motor_control; sleep 1

AIZEE_CONFIG=config/hardware_two_motors.yaml ./rust/target/release/motor_control > /tmp/individual_test.log 2>&1 &
MOTOR_PID=$!
sleep 3

echo '=== Testing ROBSTRIDE03 (ID 2, right_wheel) alone ==='
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "enable", "motor_ids": ["right_wheel"]}); s.close(); c.term()'
sleep 1
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "zero_position", "motor_ids": ["right_wheel"]}); s.close(); c.term()'
sleep 1
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "drive", "linear": 0.5, "angular": 0.0}); s.close(); c.term()'
echo 'Only right_wheel should be spinning for 3 seconds...'
sleep 3
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "disable", "motor_ids": ["right_wheel"]}); s.close(); c.term()'
sleep 2

echo ''
echo '=== Testing ROBSTRIDE04 (ID 3, left_wheel) alone ==='
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "enable", "motor_ids": ["left_wheel"]}); s.close(); c.term()'
sleep 1
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "zero_position", "motor_ids": ["left_wheel"]}); s.close(); c.term()'
sleep 1
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "drive", "linear": 0.5, "angular": 0.0}); s.close(); c.term()'
echo 'Only left_wheel should be spinning for 3 seconds...'
sleep 3
python3 -c 'import zmq, json, time; c = zmq.Context(); s = c.socket(zmq.PUSH); s.connect("tcp://localhost:5555"); time.sleep(0.2); s.send_json({"type": "disable", "motor_ids": ["left_wheel"]}); s.close(); c.term()'

sleep 1
kill $MOTOR_PID 2>/dev/null; pkill -9 motor_control
echo ''
echo 'Tests complete!'
