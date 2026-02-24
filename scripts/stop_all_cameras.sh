#!/bin/bash
# Stop all 4 camera services in parallel
# Usage: ./stop_all_cameras.sh

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"
PI_SSH="ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no"

echo "=== Stopping all AIZEE camera services ==="
echo ""

echo "Stopping cam_front (10.42.0.11)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.11 'sudo systemctl stop aizee-camera-cam_front'" &
PID1=$!

echo "Stopping cam_rear (10.42.0.12)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.12 'sudo systemctl stop aizee-camera-cam_rear'" &
PID2=$!

echo "Stopping cam_left (10.42.0.13)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.13 'sudo systemctl stop aizee-camera-cam_left'" &
PID3=$!

echo "Stopping cam_right (10.42.0.14)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.14 'sudo systemctl stop aizee-camera-cam_right'" &
PID4=$!

echo ""
echo "Waiting for all services to stop..."
wait $PID1 $PID2 $PID3 $PID4

echo ""
echo "=== All camera services stopped! ==="
echo ""
