#!/bin/bash
# Deploy all 4 camera nodes to their respective Raspberry Pi 4 devices
# Usage: ./deploy_all_cameras.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== AIZEE Multi-Camera Deployment ==="
echo "Deploying to 4 Raspberry Pi 4 camera nodes"
echo ""

CAMERAS=("cam_front" "cam_rear" "cam_left" "cam_right")

for camera in "${CAMERAS[@]}"; do
    echo "========================================="
    echo "Deploying $camera..."
    echo "========================================="
    "$SCRIPT_DIR/deploy_rpi4_camera_scp.sh" "$camera"
    echo ""
done

echo "=== All cameras deployed successfully! ==="
echo ""
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
SSH="ssh -i $SSH_KEY -J ltr@$JETSON_IP"

echo "Next steps:"
echo "1. Discover serial numbers on each Pi:"
echo "   $SSH ltr@10.42.0.11 rs-enumerate-devices | grep Serial  # cam_front"
echo "   $SSH ltr@10.42.0.12 rs-enumerate-devices | grep Serial  # cam_rear"
echo "   $SSH ltr@10.42.0.13 rs-enumerate-devices | grep Serial  # cam_left"
echo "   $SSH ltr@10.42.0.14 rs-enumerate-devices | grep Serial  # cam_right"
echo ""
echo "2. Update serial numbers in config files:"
echo "   config/hardware_rpi4_cam_*.yaml"
echo ""
echo "3. Start all camera services:"
echo "   ./scripts/start_all_cameras.sh"
echo ""
echo "4. Test Rerun bridge with all cameras (streams via Jetson relay):"
echo "   python python/rerun_bridge.py \\"
echo "       --cameras tcp://${JETSON_IP}:5557 tcp://${JETSON_IP}:5558 \\"
echo "                 tcp://${JETSON_IP}:5559 tcp://${JETSON_IP}:5560 \\"
echo "       --save logs/cameras_test.mcap"
echo ""
