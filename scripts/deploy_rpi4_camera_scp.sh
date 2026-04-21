#!/bin/bash
# Deploy AIZEE camera node to Raspberry Pi 4 using tar+ssh (no rsync needed)
# Usage: ./deploy_rpi4_camera_scp.sh [cam_front|cam_rear|cam_left|cam_right]

set -e

CAMERA_ID="${1:-cam_front}"
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"
PI_SSH="ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no"

# Map camera ID to IP and hostname
case "$CAMERA_ID" in
    cam_front)
        TARGET_IP="10.42.0.11"
        HOSTNAME="AIZEE-ROVER-PI-1"
        ZMQ_PORT="5557"
        ;;
    cam_rear)
        TARGET_IP="10.42.0.12"
        HOSTNAME="AIZEE-ROVER-PI-2"
        ZMQ_PORT="5558"
        ;;
    cam_left)
        TARGET_IP="10.42.0.13"
        HOSTNAME="AIZEE-ROVER-PI-3"
        ZMQ_PORT="5559"
        ;;
    cam_right)
        TARGET_IP="10.42.0.14"
        HOSTNAME="AIZEE-ROVER-PI-4"
        ZMQ_PORT="5560"
        ;;
    *)
        echo "ERROR: Invalid camera ID: $CAMERA_ID"
        echo "Usage: $0 [cam_front|cam_rear|cam_left|cam_right]"
        exit 1
        ;;
esac

TARGET="ltr@${TARGET_IP}"
REMOTE_DIR="aizee"
# Pi SSH helper: run a command on a Pi (routes dev→Jetson→Pi)
pi_ssh() { $JETSON_SSH "$PI_SSH $TARGET \"$*\""; }

echo "=== AIZEE RPi4 Camera Module Deployment ==="
echo "Camera ID: $CAMERA_ID"
echo "Target: $TARGET ($HOSTNAME)"
echo "ZMQ Port: $ZMQ_PORT"
echo "Remote directory: ~/$REMOTE_DIR"
echo ""

# Check if target is reachable via SSH (dev→Jetson→Pi)
if ! $JETSON_SSH "$PI_SSH -o ConnectTimeout=5 $TARGET 'echo ok'" > /dev/null 2>&1; then
    echo "ERROR: Cannot reach $TARGET_IP via Jetson ($JETSON_IP)"
    exit 1
fi

echo "1. Syncing Python codebase via tar through Jetson..."
# Pack, send to Jetson /tmp, then forward to Pi
tar czf - \
    --exclude='rust' \
    --exclude='target' \
    --exclude='.git' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='logs' \
    --exclude='*.mcap' \
    . | $JETSON_SSH "cat > /tmp/_pi_deploy.tar.gz && $PI_SSH $TARGET 'mkdir -p ~/$REMOTE_DIR && cd ~/$REMOTE_DIR && tar xzf -' < /tmp/_pi_deploy.tar.gz; rm -f /tmp/_pi_deploy.tar.gz"

echo ""
echo "2. Installing Python dependencies..."
$JETSON_SSH "$PI_SSH $TARGET 'cd ~/$REMOTE_DIR && pip3 install --break-system-packages -r requirements.txt 2>&1 | grep -E \"(Successfully installed|Requirement already satisfied)\" || echo Dependencies installed'"

echo ""
echo "3. Installing systemd service..."
$JETSON_SSH "$PI_SSH $TARGET 'sudo cp ~/$REMOTE_DIR/config/systemd/aizee-camera-${CAMERA_ID}.service /etc/systemd/system/'"
$JETSON_SSH "$PI_SSH $TARGET 'sudo systemctl daemon-reload'"

echo ""
echo "4. Testing camera connectivity..."
$JETSON_SSH "$PI_SSH $TARGET 'rs-enumerate-devices 2>/dev/null || echo Warning: RealSense SDK not configured'"

echo ""
echo "=== Deployment complete! ==="
echo ""
echo "To discover camera serial number:"
echo "  ssh -i $SSH_KEY ltr@$JETSON_IP \"ssh -i ~/.ssh/aizee_rover_id $TARGET rs-enumerate-devices | grep Serial\""
echo ""
echo "To start the service:"
echo "  ssh -i $SSH_KEY ltr@$JETSON_IP \"ssh -i ~/.ssh/aizee_rover_id $TARGET sudo systemctl start aizee-camera-${CAMERA_ID}\""
echo ""
echo "To enable on boot:"
echo "  ssh -i $SSH_KEY ltr@$JETSON_IP \"ssh -i ~/.ssh/aizee_rover_id $TARGET sudo systemctl enable aizee-camera-${CAMERA_ID}\""
echo ""
echo "To view logs:"
echo "  ssh -i $SSH_KEY ltr@$JETSON_IP \"ssh -i ~/.ssh/aizee_rover_id $TARGET sudo journalctl -u aizee-camera-${CAMERA_ID} -f\""
echo ""
echo "To test stream reception (from dev machine, via Jetson relay):"
echo "  python python/test_camera_subscriber.py --zmq-endpoint tcp://${JETSON_IP}:${ZMQ_PORT}"
echo ""
