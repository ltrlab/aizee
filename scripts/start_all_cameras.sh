#!/bin/bash
# Start all 4 camera services in parallel
# Usage: ./start_all_cameras.sh

set -e

echo "=== Starting all AIZEE camera services ==="
echo ""

echo "Starting cam_front (192.168.0.22)..."
ssh pi@192.168.0.22 "sudo systemctl start aizee-camera-cam_front" &
PID1=$!

echo "Starting cam_rear (192.168.0.23)..."
ssh pi@192.168.0.23 "sudo systemctl start aizee-camera-cam_rear" &
PID2=$!

echo "Starting cam_left (192.168.0.24)..."
ssh pi@192.168.0.24 "sudo systemctl start aizee-camera-cam_left" &
PID3=$!

echo "Starting cam_right (192.168.0.25)..."
ssh pi@192.168.0.25 "sudo systemctl start aizee-camera-cam_right" &
PID4=$!

echo ""
echo "Waiting for all services to start..."
wait $PID1 $PID2 $PID3 $PID4

echo ""
echo "=== All camera services started! ==="
echo ""
echo "To check status:"
echo "  ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front"
echo "  ssh pi@192.168.0.23 sudo systemctl status aizee-camera-cam_rear"
echo "  ssh pi@192.168.0.24 sudo systemctl status aizee-camera-cam_left"
echo "  ssh pi@192.168.0.25 sudo systemctl status aizee-camera-cam_right"
echo ""
echo "To view live logs (run in separate terminals):"
echo "  ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -f"
echo "  ssh pi@192.168.0.23 sudo journalctl -u aizee-camera-cam_rear -f"
echo "  ssh pi@192.168.0.24 sudo journalctl -u aizee-camera-cam_left -f"
echo "  ssh pi@192.168.0.25 sudo journalctl -u aizee-camera-cam_right -f"
echo ""
