#!/bin/bash
# Deploy and install the AIZEE gripper camera node (ELP-USBFHD01M-L21) on the Jetson.
#
# What this does:
#   1. Copy python/nodes/gripper_camera_node.py and configs to Jetson
#   2. Install udev rule  -> /dev/aizee_gripper_cam symlink + systemd device unit
#   3. Disable + mask the old arm-cam-left/right RealSense services so the
#      stereo D435 pipeline cannot come back on a reboot or hot-plug
#   4. Install / reload systemd service bound to the device unit
#   5. Enable service (auto-start on plug-in)
#   6. Trigger udev to activate the already-connected camera immediately
#
# Usage: ./scripts/deploy_gripper_camera.sh [user@host]   (default: auto-detected via deploy_common.sh)

set -e

source "$(dirname "$0")/deploy_common.sh"
TARGET="${1:-$AIZEE_TARGET}"
REMOTE_DIR="aizee"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"
TARBALL="/tmp/aizee_gripper_cam_deploy.tar.gz"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Gripper Camera Deployment ==="
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
    python/nodes/gripper_camera_node.py \
    config/hardware_jetson_gripper_cam.yaml \
    config/systemd/aizee-gripper-cam.service \
    config/udev/99-aizee-gripper-cam.rules
echo "   $(du -h $TARBALL | cut -f1) packed."
echo ""

echo "2. Copying to Jetson..."
$SCP "$TARBALL" "$TARGET:/tmp/aizee_gripper_cam_deploy.tar.gz"
rm -f "$TARBALL"
echo ""

echo "3. Extracting on Jetson..."
$SSH "$TARGET" "cd ~/$REMOTE_DIR && tar xzf /tmp/aizee_gripper_cam_deploy.tar.gz && rm /tmp/aizee_gripper_cam_deploy.tar.gz"
echo ""

echo "4. Disabling old RealSense arm-cam services, installing new udev + systemd..."
$SSH "$TARGET" "echo '$JETSON_PASS' | sudo -S bash -c '
set -e

# --- Retire old stereo D435 arm-cam pipeline -------------------------------
# Stop running instances, disable from auto-start on device events, mask so
# they cannot be (re)started manually until explicitly unmasked. We do NOT
# delete the unit files or the python node — they remain in the repo and can
# be restored with: systemctl unmask aizee-arm-cam-left aizee-arm-cam-right.
for svc in aizee-arm-cam-left aizee-arm-cam-right; do
    if systemctl list-unit-files | grep -q \"^\${svc}.service\"; then
        echo \"  -> retiring \${svc}\"
        systemctl stop \${svc}.service 2>/dev/null || true
        systemctl disable \${svc}.service 2>/dev/null || true
        systemctl mask \${svc}.service 2>/dev/null || true
    fi
done

# Remove old RealSense udev rule so plugging in a D435 will not synthesize
# the realsense_arm_{left,right} device units anymore.
rm -f /etc/udev/rules.d/99-aizee-realsense.rules

# --- Install new gripper camera --------------------------------------------
cp /home/ltr/aizee/config/udev/99-aizee-gripper-cam.rules /etc/udev/rules.d/
cp /home/ltr/aizee/config/systemd/aizee-gripper-cam.service /etc/systemd/system/
systemctl daemon-reload

# Drop any leftover wants/symlinks
rm -f /etc/systemd/system/multi-user.target.wants/aizee-gripper-cam.service

systemctl stop aizee-gripper-cam 2>/dev/null || true
systemctl enable aizee-gripper-cam

# Reload + retrigger udev so the symlink + device unit appear right now.
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux --action=add
sleep 3

echo \"\"
echo \"=== Service status ===\"
systemctl status aizee-gripper-cam --no-pager -l || true
echo \"\"
echo \"=== /dev/aizee_gripper_cam ===\"
ls -l /dev/aizee_gripper_cam 2>&1 || echo \"(symlink not present yet — replug the camera if needed)\"
' 2>&1"
echo ""

echo "=== Deploy complete! ==="
echo ""
echo "Service starts/stops automatically with camera plug/unplug."
echo "Logs: ssh -i $SSH_KEY $TARGET 'journalctl -u aizee-gripper-cam -f'"
