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
    "$SCRIPT_DIR/deploy_rpi4_camera.sh" "$camera"
    echo ""
done

echo "=== All cameras deployed successfully! ==="
echo ""
echo "Next steps:"
echo "1. Discover serial numbers on each Pi:"
echo "   ssh pi@192.168.0.22 rs-enumerate-devices | grep Serial  # cam_front"
echo "   ssh pi@192.168.0.23 rs-enumerate-devices | grep Serial  # cam_rear"
echo "   ssh pi@192.168.0.24 rs-enumerate-devices | grep Serial  # cam_left"
echo "   ssh pi@192.168.0.25 rs-enumerate-devices | grep Serial  # cam_right"
echo ""
echo "2. Update serial numbers in config files:"
echo "   config/hardware_rpi4_cam_*.yaml"
echo ""
echo "3. Start all camera services:"
echo "   ./scripts/start_all_cameras.sh"
echo ""
echo "4. Test Rerun bridge with all cameras:"
echo "   python python/rerun_bridge.py \\"
echo "       --cameras tcp://192.168.0.22:5557 tcp://192.168.0.23:5558 \\"
echo "                 tcp://192.168.0.24:5559 tcp://192.168.0.25:5560 \\"
echo "       --save logs/cameras_test.mcap"
echo ""
