#!/bin/bash
# Start all 4 camera services in parallel
# Usage: ./start_all_cameras.sh

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"
PI_SSH="ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no"

echo "=== Starting all AIZEE camera services ==="
echo ""

echo "Starting cam_front (10.42.0.11)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.11 'sudo systemctl start aizee-camera-cam_front'" &
PID1=$!

echo "Starting cam_rear (10.42.0.12)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.12 'sudo systemctl start aizee-camera-cam_rear'" &
PID2=$!

echo "Starting cam_left (10.42.0.13)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.13 'sudo systemctl start aizee-camera-cam_left'" &
PID3=$!

echo "Starting cam_right (10.42.0.14)..."
$JETSON_SSH "$PI_SSH ltr@10.42.0.14 'sudo systemctl start aizee-camera-cam_right'" &
PID4=$!

echo ""
echo "Waiting for all services to start..."
wait $PID1 $PID2 $PID3 $PID4

echo ""
echo "=== All camera services started! ==="
echo ""
echo "To check status:"
JCMD="ssh -i $SSH_KEY ltr@$JETSON_IP"
echo "  $JCMD \"ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo systemctl status aizee-camera-cam_front'\""
echo "  $JCMD \"ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.12 'sudo systemctl status aizee-camera-cam_rear'\""
echo ""
echo "To view live logs:"
echo "  $JCMD \"ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo journalctl -u aizee-camera-cam_front -f'\""
echo ""
