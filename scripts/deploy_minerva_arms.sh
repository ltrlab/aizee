#!/bin/bash
# Deploy the Minerva dual-arm motor-control stack to the Jetson Orin Nano.
# Same Jetson as the AIZEE rover — this RETIRES the single-arm rover service and
# installs two instances (one motor_control per arm, one CAN bus each):
#
#     aizee-minerva-left   AIZEE_CONFIG=hardware_minerva_left.yaml   can1  :5555/:5556
#     aizee-minerva-right  AIZEE_CONFIG=hardware_minerva_right.yaml  can2  :5557/:5558
#
# IMPORTANT: the services are installed but NOT started/enabled. motor_control
# wedges at init if its configured motors aren't on the bus, so start the arms
# only AFTER the motor bus is powered (this script prints how).
#
# Usage: ./deploy_minerva_arms.sh [user@host]   (default: auto-detected)

set -e

source "$(dirname "$0")/deploy_common.sh"
TARGET="${1:-$AIZEE_TARGET}"
REMOTE_DIR="aizee"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"
TARBALL="/tmp/aizee_minerva_deploy.tar.gz"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found. Create it with the Jetson sudo password."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== Minerva Dual-Arm Deployment ==="
echo "Target: $TARGET"
echo ""

echo "Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: Cannot reach $TARGET via SSH"
    exit 1
fi
echo "Connected."
echo ""

echo "1. Packing rust/, config/, and the USB-CAN reset helper ..."
tar czf "$TARBALL" \
    --exclude='rust/target' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='logs' \
    rust/ config/ scripts/aizee-reset-usb-can scripts/aizee-can-up scripts/aizee-can-reset
echo "   $(du -h $TARBALL | cut -f1) packed."
echo ""

echo "2. Copying tarball to Jetson..."
$SCP "$TARBALL" "$TARGET:/tmp/aizee_minerva_deploy.tar.gz"
rm -f "$TARBALL"
echo ""

echo "3. Extracting on Jetson..."
$SSH "$TARGET" "mkdir -p ~/$REMOTE_DIR && cd ~/$REMOTE_DIR && tar xzf /tmp/aizee_minerva_deploy.tar.gz && rm /tmp/aizee_minerva_deploy.tar.gz"
echo ""

echo "4. Building motor_control on Jetson (cached — usually a no-op)..."
$SSH "$TARGET" "cd ~/$REMOTE_DIR/rust/motor_control && source ~/.cargo/env && cargo build --release 2>&1 | tail -3"
echo ""

echo "5. Installing helper, sudoers, and BOTH arm services (not started)..."
{
    echo "$JETSON_PASS"
    cat << 'SUDO_CMDS'
# CAN helpers (strip CRLF from the Windows checkout)
sed 's/\r$//' /home/ltr/aizee/scripts/aizee-can-up > /usr/local/bin/aizee-can-up
sed 's/\r$//' /home/ltr/aizee/scripts/aizee-reset-usb-can > /usr/local/bin/aizee-reset-usb-can
sed 's/\r$//' /home/ltr/aizee/scripts/aizee-can-reset > /usr/local/bin/aizee-can-reset
chmod 755 /usr/local/bin/aizee-can-up /usr/local/bin/aizee-reset-usb-can /usr/local/bin/aizee-can-reset

# Pin the two identical USB-CAN adapters to canL/canR by physical USB port so
# left/right can never swap across reboots/resets.
sed 's/\r$//' /home/ltr/aizee/config/udev/80-minerva-can.rules > /etc/udev/rules.d/80-minerva-can.rules
udevadm control --reload-rules
# Apply USB-autosuspend-off to any already-connected adapters now (the rule itself
# handles future hotplugs/boots). Resume-from-autosuspend is a known gs_usb wedge cause.
for _p in 1-2.1.2 1-2.1.4; do
    [ -e /sys/bus/usb/devices/$_p/power/control ] && echo on > /sys/bus/usb/devices/$_p/power/control || true
done

# Passwordless CAN management for both arm buses (canL, canR) + helpers.
cat > /etc/sudoers.d/aizee-can << 'SUDOERS'
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-up
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-reset-usb-can
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-reset canL
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-reset canR
ltr ALL=(ALL) NOPASSWD: /usr/sbin/ip link set canL *
ltr ALL=(ALL) NOPASSWD: /usr/sbin/ip link set canR *
SUDOERS
chmod 440 /etc/sudoers.d/aizee-can

# Retire the single-arm rover motor-control service (it holds can1).
systemctl stop aizee-motor-control-rover 2>/dev/null || true
systemctl disable aizee-motor-control-rover 2>/dev/null || true

# Install the two Minerva arm services (strip CRLF).
for svc in aizee-minerva-left aizee-minerva-right; do
    sed 's/\r$//' /home/ltr/aizee/config/systemd/${svc}.service > /etc/systemd/system/${svc}.service
done
systemctl daemon-reload
# Enable boot-start. Safe now: motor_control degrades (stays alive) instead of
# wedging/flapping if it boots with the motor bus unpowered.
systemctl enable aizee-minerva-left aizee-minerva-right

echo "--- installed units ---"
systemctl list-unit-files | grep -E 'aizee-minerva|aizee-motor-control-rover' || true
SUDO_CMDS
} | $SSH "$TARGET" "sudo -S bash -s 2>/dev/null"
echo ""

echo "=== Deploy complete (services installed + enabled for boot; NOT started now) ==="
echo ""
echo "They auto-start on boot. To start now without rebooting (motor bus POWERED):"
echo "  $SSH $TARGET 'sudo systemctl start aizee-minerva-left aizee-minerva-right'"
echo "Watch a bus:"
echo "  $SSH $TARGET 'journalctl -u aizee-minerva-left -f'"
echo "To stop auto-start on boot:"
echo "  $SSH $TARGET 'sudo systemctl disable aizee-minerva-left aizee-minerva-right'"
