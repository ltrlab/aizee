#!/bin/bash
# Test rover motors on can2 adapter

echo "Stopping motor_control..."
pkill -f motor_control
sleep 1

echo "Starting motor_control on can2..."
cd /home/ltr/aizee
export AIZEE_CONFIG=/home/ltr/aizee/config/hardware_test_rover_on_can2.yaml
./rust/target/release/motor_control
