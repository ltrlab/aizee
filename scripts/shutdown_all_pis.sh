#!/bin/bash
# Cleanly shutdown all Raspberry Pi camera nodes
# Usage: ./shutdown_all_pis.sh

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"
PI_SSH="ssh -i ~/.ssh/aizee_rover_id -o ConnectTimeout=5 -o StrictHostKeyChecking=no"

echo "=== AIZEE Raspberry Pi Shutdown Script ==="
echo ""
echo "This will cleanly shutdown all 4 camera Raspberry Pis:"
echo "  - Pi 1 (Front): 10.42.0.11"
echo "  - Pi 2 (Rear):  10.42.0.12"
echo "  - Pi 3 (Left):  10.42.0.13"
echo "  - Pi 4 (Right): 10.42.0.14"
echo ""

# Ask for confirmation (can be skipped with -y flag)
if [[ "$1" != "-y" ]]; then
    read -p "Are you sure you want to shutdown all Pis? (y/N): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo "Shutdown cancelled."
        exit 0
    fi
fi

echo ""
echo "=== Stopping camera services first ==="

declare -A CAM_MAP
CAM_MAP["10.42.0.11"]="cam_front"
CAM_MAP["10.42.0.12"]="cam_rear"
CAM_MAP["10.42.0.13"]="cam_left"
CAM_MAP["10.42.0.14"]="cam_right"

for ip in 10.42.0.11 10.42.0.12 10.42.0.13 10.42.0.14; do
    cam="${CAM_MAP[$ip]}"
    echo "Stopping $ip ($cam)..."
    $JETSON_SSH "$PI_SSH ltr@$ip 'sudo systemctl stop aizee-camera-$cam'" 2>/dev/null \
        || echo "  Warning: Could not stop $cam (may already be stopped)"
done

echo ""
echo "=== Initiating shutdown sequence ==="
echo ""

for ip in 10.42.0.11 10.42.0.12 10.42.0.13 10.42.0.14; do
    echo "Shutting down $ip..."
    $JETSON_SSH "$PI_SSH ltr@$ip 'sudo shutdown -h now'" 2>/dev/null &
done

echo ""
echo "Waiting for shutdown commands to be sent..."
sleep 2

echo ""
echo "=== Shutdown commands sent to all Pis ==="
echo ""
echo "The Raspberry Pis are now shutting down. This will take about 30 seconds."
echo ""
echo "Verify from Jetson:"
echo "  ssh -i $SSH_KEY ltr@$JETSON_IP 'ping -c 3 10.42.0.11'"
echo ""
