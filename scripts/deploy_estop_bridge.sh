#!/bin/bash
# Deploy e-stop bridge to Jetson Orin Nano
# Usage: ./deploy_estop_bridge.sh [user@host]   (default: auto-detected via deploy_common.sh)
#
# Copies bridge.py, installs udev rule for persistent /dev/estop-receiver
# symlink, installs and starts the systemd service.

set -e

source "$(dirname "$0")/deploy_common.sh"
TARGET="${1:-$AIZEE_TARGET}"
REMOTE_DIR="aizee"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found. Create it with the Jetson sudo password."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE E-Stop Bridge Deployment ==="
echo "Target: $TARGET"
echo ""

# Check SSH connectivity
echo "Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: Cannot reach $TARGET via SSH"
    exit 1
fi
echo "Connected."
echo ""

# Copy files
echo "1. Copying bridge files..."
$SSH "$TARGET" "mkdir -p ~/$REMOTE_DIR/firmware/estop ~/$REMOTE_DIR/config/udev ~/$REMOTE_DIR/config/systemd"
$SCP firmware/estop/bridge.py "$TARGET:~/$REMOTE_DIR/firmware/estop/bridge.py"
$SCP config/udev/99-aizee-estop.rules "$TARGET:~/$REMOTE_DIR/config/udev/99-aizee-estop.rules"
$SCP config/systemd/aizee-estop-bridge.service "$TARGET:~/$REMOTE_DIR/config/systemd/aizee-estop-bridge.service"
echo ""

# Detect the ESP32 serial port
echo "2. Detecting ESP32 serial port..."
$SSH "$TARGET" "ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo '   No serial devices found — plug in the receiver and re-run'"
echo ""

# Install udev rule, service file, reload, and start
echo "3. Installing udev rule, service, and starting..."
{
    echo "$JETSON_PASS"
    cat << 'SUDO_CMDS'
# Install udev rule for persistent /dev/estop-receiver symlink
cp /home/ltr/aizee/config/udev/99-aizee-estop.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

# Install service file
cp /home/ltr/aizee/config/systemd/aizee-estop-bridge.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable aizee-estop-bridge
systemctl restart aizee-estop-bridge
sleep 2
systemctl status aizee-estop-bridge --no-pager -l
SUDO_CMDS
} | $SSH "$TARGET" "sudo -S bash -s 2>/dev/null"
echo ""

echo "=== Deploy complete! ==="
echo ""
echo "Check symlink:  ssh -i $SSH_KEY $TARGET ls -l /dev/estop-receiver"
echo "Follow logs:    ssh -i $SSH_KEY $TARGET sudo journalctl -u aizee-estop-bridge -f"
