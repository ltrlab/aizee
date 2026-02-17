#!/bin/bash
# Reset USB CAN adapter and bring up can1 interface

set -e

echo "Resetting USB CAN adapter..."
echo 1-2.3.1 > /sys/bus/usb/drivers/usb/unbind
sleep 2
echo 1-2.3.1 > /sys/bus/usb/drivers/usb/bind
sleep 1

echo "Configuring can1 interface..."
ip link set can1 down 2>/dev/null || true
ip link set can1 type can bitrate 1000000
ip link set can1 txqueuelen 1000
ip link set can1 up

echo "✓ CAN adapter reset complete"
echo ""
echo "Interface status:"
ip link show can1
