#!/usr/bin/env bash
# Reset USB CAN adapter on the Jetson and bring up can1 interface.
# Run from the dev machine — SSHes into the Jetson and executes the reset remotely.

set -euo pipefail

HOST="ltr@192.168.0.27"
KEY="/p/Workspace/ssh-keys/aizee_rover_id"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password")"

echo "==> Uploading reset script to $HOST ..."
# First SSH (no PTY): stdin is the heredoc, piped cleanly into cat on the remote side
ssh -i "$KEY" "$HOST" "cat > /tmp/reset_can_adapter.sh" <<'REMOTE'
#!/bin/bash
set -e

# Dynamically resolve the USB sysfs device ID from the can1 network interface.
# The path looks like: .../usb1/1-2.3.1/1-2.3.1:1.0/net/can1
# We need the device component (e.g. "1-2.3.1") before the colon-separated interface.
if [ ! -d /sys/class/net/can1 ]; then
    echo "ERROR: can1 interface not found — is the CAN adapter connected?"
    exit 1
fi
SYSPATH=$(readlink -f /sys/class/net/can1)
USB_ID=$(echo "$SYSPATH" | grep -oP '(?<=/usb\d/)\d+-[\d.]+(?=/)' | head -1)
if [ -z "$USB_ID" ]; then
    echo "ERROR: could not parse USB device path from: $SYSPATH"
    exit 1
fi
echo "CAN adapter USB path: $USB_ID"

echo "Resetting USB CAN adapter..."
echo "$USB_ID" > /sys/bus/usb/drivers/usb/unbind
sleep 2
echo "$USB_ID" > /sys/bus/usb/drivers/usb/bind

# Wait for the kernel to re-enumerate the USB device and create can1
echo "Waiting for can1 to appear..."
for i in $(seq 1 15); do
    ip link show can1 >/dev/null 2>&1 && break
    sleep 1
done
if ! ip link show can1 >/dev/null 2>&1; then
    echo "ERROR: can1 did not appear after USB reset"
    exit 1
fi

echo "Configuring can1 interface..."
ip link set can1 down 2>/dev/null || true
ip link set can1 type can bitrate 1000000 restart-ms 100
ip link set can1 txqueuelen 1000
ip link set can1 up

echo ""
echo "✓ CAN adapter reset complete"
echo ""
echo "Interface status:"
ip link show can1
REMOTE

echo "==> Running reset on $HOST ..."
# Second SSH (with PTY): same sudo pattern as restart_motor_service.sh
ssh -tt -i "$KEY" "$HOST" "printf '%s\n' '${PASS}' | sudo -S bash /tmp/reset_can_adapter.sh; rm -f /tmp/reset_can_adapter.sh"
