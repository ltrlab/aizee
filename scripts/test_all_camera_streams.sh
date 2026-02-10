#!/bin/bash
# Test all 4 camera streams simultaneously from dev machine
# Usage: ./test_all_camera_streams.sh [duration_seconds]

DURATION="${1:-30}"  # Default 30 seconds

echo "=== Testing all AIZEE camera streams ==="
echo "Duration: ${DURATION} seconds"
echo ""

# Check if Python script exists
if [ ! -f "python/test_camera_subscriber.py" ]; then
    echo "ERROR: python/test_camera_subscriber.py not found"
    exit 1
fi

# Start test subscribers for all 4 cameras in background
echo "Starting stream tests..."

python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557 &
PID1=$!
echo "  cam_front (192.168.0.22:5557) - PID $PID1"

python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.23:5558 &
PID2=$!
echo "  cam_rear (192.168.0.23:5558) - PID $PID2"

python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.24:5559 &
PID3=$!
echo "  cam_left (192.168.0.24:5559) - PID $PID3"

python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.25:5560 &
PID4=$!
echo "  cam_right (192.168.0.25:5560) - PID $PID4"

echo ""
echo "Waiting ${DURATION} seconds..."
sleep "$DURATION"

echo ""
echo "Stopping all test subscribers..."
kill $PID1 $PID2 $PID3 $PID4 2>/dev/null
wait 2>/dev/null

echo ""
echo "=== Stream test complete ==="
echo ""
echo "Next steps:"
echo "1. Check output above for FPS and latency statistics"
echo "2. Verify all 4 cameras achieved ≥25 fps"
echo "3. If issues found, check individual camera logs:"
echo "   ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -n 50"
echo ""
