#!/bin/bash
# Stop all 4 camera services in parallel
# Usage: ./stop_all_cameras.sh

set -e

echo "=== Stopping all AIZEE camera services ==="
echo ""

echo "Stopping cam_front (192.168.0.22)..."
ssh pi@192.168.0.22 "sudo systemctl stop aizee-camera-cam_front" &
PID1=$!

echo "Stopping cam_rear (192.168.0.23)..."
ssh pi@192.168.0.23 "sudo systemctl stop aizee-camera-cam_rear" &
PID2=$!

echo "Stopping cam_left (192.168.0.24)..."
ssh pi@192.168.0.24 "sudo systemctl stop aizee-camera-cam_left" &
PID3=$!

echo "Stopping cam_right (192.168.0.25)..."
ssh pi@192.168.0.25 "sudo systemctl stop aizee-camera-cam_right" &
PID4=$!

echo ""
echo "Waiting for all services to stop..."
wait $PID1 $PID2 $PID3 $PID4

echo ""
echo "=== All camera services stopped! ==="
echo ""
