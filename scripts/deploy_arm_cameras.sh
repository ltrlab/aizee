#!/bin/bash
# Deploy and install AIZEE arm camera nodes (D435) on the Jetson.
#
# What this does:
#   1. Copy python/nodes/arm_camera_node.py and configs to Jetson
#   2. Install udev rules  -> cameras get stable /dev/realsense_arm_{left,right} symlinks
#   3. Install / reload systemd services bound to those device units
#   4. Enable services (creates .device.wants/ symlinks for auto-start on plug-in)
#   5. Trigger udev to activate already-connected cameras immediately
#
# Usage: ./scripts/deploy_arm_cameras.sh [ltr@192.168.0.27]

set -e

TARGET="${1:-ltr@192.168.0.27}"
REMOTE_DIR="aizee"
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"
TARBALL="/tmp/aizee_arm_cam_deploy.tar.gz"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Arm Camera Deployment ==="
echo "Target: $TARGET"
echo ""

echo "Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: Cannot reach $TARGET"
    exit 1
fi
echo "Connected."
echo ""

echo "1. Packing files..."
tar czf "$TARBALL" \
    python/nodes/arm_camera_node.py \
    config/hardware_jetson_arm_cam_left.yaml \
    config/hardware_jetson_arm_cam_right.yaml \
    config/systemd/aizee-arm-cam-left.service \
    config/systemd/aizee-arm-cam-right.service \
    config/udev/99-aizee-realsense.rules
echo "   $(du -h $TARBALL | cut -f1) packed."
echo ""

echo "2. Copying to Jetson..."
$SCP "$TARBALL" "$TARGET:/tmp/aizee_arm_cam_deploy.tar.gz"
rm -f "$TARBALL"
echo ""

echo "3. Extracting on Jetson..."
$SSH "$TARGET" "cd ~/$REMOTE_DIR && tar xzf /tmp/aizee_arm_cam_deploy.tar.gz && rm /tmp/aizee_arm_cam_deploy.tar.gz"
echo ""

echo "4. Installing udev rules, systemd services, and enabling..."
$SSH "$TARGET" "echo '$JETSON_PASS' | sudo -S bash -c '
# Udev rules
cp /home/ltr/aizee/config/udev/99-aizee-realsense.rules /etc/udev/rules.d/
echo \"Udev rules installed.\"

# Service files
cp /home/ltr/aizee/config/systemd/aizee-arm-cam-left.service  /etc/systemd/system/
cp /home/ltr/aizee/config/systemd/aizee-arm-cam-right.service /etc/systemd/system/
systemctl daemon-reload

# Remove old multi-user.target symlinks if they exist
rm -f /etc/systemd/system/multi-user.target.wants/aizee-arm-cam-left.service
rm -f /etc/systemd/system/multi-user.target.wants/aizee-arm-cam-right.service

# Stop currently running instances (harmless if not running)
systemctl stop aizee-arm-cam-left aizee-arm-cam-right 2>/dev/null || true

# Enable against the device units (creates .device.wants/ symlinks)
systemctl enable aizee-arm-cam-left aizee-arm-cam-right

# Reload udev rules and trigger — activates cameras if already connected
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add
sleep 3

echo ""
echo "=== Service status ==="
systemctl status aizee-arm-cam-left aizee-arm-cam-right --no-pager -l
' 2>&1"
echo ""

echo "=== Deploy complete! ==="
echo ""
echo "Services start/stop automatically with camera plug/unplug."
echo "Logs: ssh -i $SSH_KEY $TARGET 'journalctl -u aizee-arm-cam-left -u aizee-arm-cam-right -f'"
