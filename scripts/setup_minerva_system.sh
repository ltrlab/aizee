#!/bin/bash
# One-time system setup on Minerva's Jetson (needs sudo; run it yourself).
#
#   - CAN helpers in /usr/local/bin (aizee-can-up, aizee-reset-usb-can, aizee-can-reset)
#   - udev rule pinning canL/canR to the verified USB adapter serials, USB autosuspend off
#   - sudoers entries so the robot runtime can bring canL/canR up and down
#   - the retired single-arm rover motor service stopped and disabled
#
# Deploying code is separate and needs no sudo: scripts/deploy_minerva_arms.sh.
# This script installs no motor services and starts nothing.
#
# Usage: scripts/setup_minerva_system.sh [user@host]   (default: auto-detected)
# Reads the Jetson sudo password from scripts/.jetson_password (git-ignored).
set -euo pipefail

source "$(dirname "$0")/deploy_common.sh"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="${1:-$AIZEE_TARGET}"
SSH="ssh -i $SSH_KEY"
STAGE="/tmp/minerva-system-setup"

PASS_FILE="$REPO_DIR/scripts/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found. Create it with the Jetson sudo password."
    exit 1
fi

echo "=== Minerva system setup on $TARGET ==="
$SSH "$TARGET" "rm -rf $STAGE && mkdir -p $STAGE"
scp -i "$SSH_KEY" -q "$REPO_DIR/scripts/aizee-can-up" "$REPO_DIR/scripts/aizee-reset-usb-can" \
    "$REPO_DIR/scripts/aizee-can-reset" "$REPO_DIR/config/udev/80-minerva-can.rules" "$TARGET:$STAGE/"

{
    cat "$PASS_FILE"
    echo
    cat << SUDO_CMDS
set -e
# CAN helpers (strip CRLF from the Windows checkout).
for helper in aizee-can-up aizee-reset-usb-can aizee-can-reset; do
    sed 's/\r\$//' $STAGE/\$helper > /usr/local/bin/\$helper
    chmod 755 /usr/local/bin/\$helper
done

# Pin the two USB-CAN adapters to canL/canR by verified unique serial.
sed 's/\r\$//' $STAGE/80-minerva-can.rules > /etc/udev/rules.d/80-minerva-can.rules
udevadm control --reload-rules
# Autosuspend off for already-connected adapters (the rule covers hotplug and boot).
for _d in /sys/bus/usb/devices/*; do
    [ -f "\$_d/serial" ] || continue
    case "\$(cat "\$_d/serial")" in
        004D002C4730511520303650|0021002F4D53530920303937)
            [ -e "\$_d/power/control" ] && echo on > "\$_d/power/control" ;;
    esac
done

cat > /etc/sudoers.d/aizee-can << 'SUDOERS'
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-up
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-reset-usb-can
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-reset canL
ltr ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-can-reset canR
ltr ALL=(ALL) NOPASSWD: /usr/sbin/ip link set canL *
ltr ALL=(ALL) NOPASSWD: /usr/sbin/ip link set canR *
SUDOERS
chmod 440 /etc/sudoers.d/aizee-can
visudo -cf /etc/sudoers.d/aizee-can

# The retired single-arm rover service must not hold a CAN bus.
systemctl stop aizee-motor-control-rover 2>/dev/null || true
systemctl disable aizee-motor-control-rover 2>/dev/null || true
rm -rf $STAGE
echo "System setup complete."
SUDO_CMDS
} | $SSH "$TARGET" "sudo -S -p '' bash -s"
