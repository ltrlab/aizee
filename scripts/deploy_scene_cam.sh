#!/bin/bash
# Deploy and install the AIZEE scene-cam node (Intel RealSense, RGB-D)
# on the Jetson, then run the self-test.
#
# What this does:
#   1. Copy python/nodes/camera_node.py, the scene-cam config, the udev
#      rule, the systemd unit, and scripts/test_realsense.py to the Jetson
#   2. Install the udev rule + systemd unit
#   3. Reload + retrigger udev so the symlink + device unit appear now
#   4. Run the self-test (SDK enumeration + short stream + ZMQ frame)
#
# The gripper-cam pipeline is untouched — both cameras run side by side
# (gripper on tcp://*:5563, scene on tcp://*:5564).
#
# Usage: ./scripts/deploy_scene_cam.sh [ltr@192.168.0.27]

set -e

TARGET="${1:-ltr@192.168.0.27}"
REMOTE_DIR="aizee"
SSH_KEY="${SSH_KEY:-/p/Workspace/ssh-keys/aizee_rover_id}"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"
TARBALL="/tmp/aizee_scene_cam_deploy.tar.gz"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Scene Camera Deployment ==="
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
    python/nodes/camera_node.py \
    config/hardware_jetson_scene_cam.yaml \
    config/systemd/aizee-scene-cam.service \
    config/udev/99-aizee-scene-cam.rules \
    scripts/test_realsense.py
echo "   $(du -h $TARBALL | cut -f1) packed."
echo ""

echo "2. Copying to Jetson..."
$SCP "$TARBALL" "$TARGET:/tmp/aizee_scene_cam_deploy.tar.gz"
rm -f "$TARBALL"
echo ""

echo "3. Extracting on Jetson..."
$SSH "$TARGET" "cd ~/$REMOTE_DIR && tar xzf /tmp/aizee_scene_cam_deploy.tar.gz && rm /tmp/aizee_scene_cam_deploy.tar.gz"
echo ""

echo "4. Installing udev rule + systemd unit..."
$SSH "$TARGET" "echo '$JETSON_PASS' | sudo -S bash -c '
set -e

cp /home/ltr/aizee/config/udev/99-aizee-scene-cam.rules /etc/udev/rules.d/
cp /home/ltr/aizee/config/systemd/aizee-scene-cam.service /etc/systemd/system/
systemctl daemon-reload

# Drop any leftover wants/symlinks before re-enabling.
rm -f /etc/systemd/system/multi-user.target.wants/aizee-scene-cam.service

systemctl stop aizee-scene-cam 2>/dev/null || true
systemctl enable aizee-scene-cam

# Reload + retrigger udev so the symlink + device unit appear right now
# for an already-connected camera.
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add
sleep 3

echo \"\"
echo \"=== Service status ===\"
systemctl status aizee-scene-cam --no-pager -l || true
echo \"\"
echo \"=== /dev/aizee_scene_cam ===\"
ls -l /dev/aizee_scene_cam 2>&1 || echo \"(symlink not present yet — replug the camera if needed)\"
' 2>&1"
echo ""

echo "5. Running self-test on the Jetson..."
echo "   (SDK enumeration + 15-frame stream + ZMQ subscribe to 127.0.0.1:5564)"
echo ""
$SSH "$TARGET" "cd ~/$REMOTE_DIR && python3 scripts/test_realsense.py \
    --config config/hardware_jetson_scene_cam.yaml \
    --zmq tcp://127.0.0.1:5564 \
    --zmq-timeout 15"
TEST_RC=$?

echo ""
if [[ $TEST_RC -eq 0 ]]; then
    echo "=== Deploy + self-test complete (PASS) ==="
else
    echo "=== Deploy complete, self-test FAILED (rc=$TEST_RC) ==="
    echo "Inspect journalctl on the Jetson:"
    echo "  ssh -i $SSH_KEY $TARGET 'journalctl -u aizee-scene-cam -n 80 --no-pager'"
fi
echo ""
echo "Tail logs:  ssh -i $SSH_KEY $TARGET 'journalctl -u aizee-scene-cam -f'"
exit $TEST_RC
