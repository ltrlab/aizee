#!/bin/bash
# Deploy AIZEE camera node to Raspberry Pi 4
# Usage: ./deploy_rpi4_camera.sh [cam_front|cam_rear|cam_left|cam_right]

set -e

CAMERA_ID="${1:-cam_front}"

# Map camera ID to IP and hostname
case "$CAMERA_ID" in
    cam_front)
        TARGET_IP="192.168.0.22"
        HOSTNAME="AIZEE-ROVER-PI-1"
        ZMQ_PORT="5557"
        ;;
    cam_rear)
        TARGET_IP="192.168.0.23"
        HOSTNAME="AIZEE-ROVER-PI-2"
        ZMQ_PORT="5558"
        ;;
    cam_left)
        TARGET_IP="192.168.0.24"
        HOSTNAME="AIZEE-ROVER-PI-3"
        ZMQ_PORT="5559"
        ;;
    cam_right)
        TARGET_IP="192.168.0.25"
        HOSTNAME="AIZEE-ROVER-PI-4"
        ZMQ_PORT="5560"
        ;;
    *)
        echo "ERROR: Invalid camera ID: $CAMERA_ID"
        echo "Usage: $0 [cam_front|cam_rear|cam_left|cam_right]"
        exit 1
        ;;
esac

TARGET="pi@${TARGET_IP}"
REMOTE_DIR="aizee"

echo "=== AIZEE RPi4 Camera Module Deployment ==="
echo "Camera ID: $CAMERA_ID"
echo "Target: $TARGET ($HOSTNAME)"
echo "ZMQ Port: $ZMQ_PORT"
echo "Remote directory: ~/$REMOTE_DIR"
echo ""

# Check if target is reachable
if ! ping -c 1 -W 2 "$TARGET_IP" > /dev/null 2>&1; then
    echo "ERROR: Cannot reach $TARGET_IP"
    exit 1
fi

echo "1. Syncing Python codebase..."
rsync -av --delete \
    --exclude 'rust/' \
    --exclude 'target/' \
    --exclude '.git/' \
    --exclude '*.pyc' \
    --exclude '__pycache__/' \
    --exclude 'logs/' \
    --exclude '*.mcap' \
    ./ "$TARGET:~/$REMOTE_DIR/"

echo ""
echo "2. Installing Python dependencies..."
ssh "$TARGET" "cd ~/$REMOTE_DIR && pip3 install --user -r requirements.txt"

echo ""
echo "3. Installing systemd service..."
ssh "$TARGET" "sudo cp ~/$REMOTE_DIR/config/systemd/aizee-camera-${CAMERA_ID}.service /etc/systemd/system/"
ssh "$TARGET" "sudo systemctl daemon-reload"

echo ""
echo "4. Testing camera connectivity..."
ssh "$TARGET" "rs-enumerate-devices 2>/dev/null || echo 'Warning: RealSense SDK not configured (run installation steps first)'"

echo ""
echo "=== Deployment complete! ==="
echo ""
echo "To discover camera serial number:"
echo "  ssh $TARGET rs-enumerate-devices | grep Serial"
echo ""
echo "To start the service:"
echo "  ssh $TARGET sudo systemctl start aizee-camera-${CAMERA_ID}"
echo ""
echo "To enable on boot:"
echo "  ssh $TARGET sudo systemctl enable aizee-camera-${CAMERA_ID}"
echo ""
echo "To view logs:"
echo "  ssh $TARGET sudo journalctl -u aizee-camera-${CAMERA_ID} -f"
echo ""
echo "To test stream reception (from dev machine):"
echo "  python python/test_camera_subscriber.py --zmq-endpoint tcp://${TARGET_IP}:${ZMQ_PORT}"
echo ""
