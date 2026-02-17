#!/bin/bash
# CAN-USB adapter watchdog
# Monitors CAN TX traffic and resets the USB adapter when it freezes.
# The CANable gs_usb firmware has an echo ID bug with extended CAN frames
# that causes it to lock up periodically. This script detects the lockup
# and performs a USB device reset to restore communication.
#
# Usage: sudo ./scripts/can_watchdog.sh [interface] [check_interval]
#   interface: CAN interface to monitor (default: can1)
#   check_interval: seconds between checks (default: 3)

CAN_IF="${1:-can1}"
CHECK_INTERVAL="${2:-3}"
USB_VID_PID="1d50:606f"
STALE_THRESHOLD=2  # Number of consecutive stale checks before reset

get_tx_packets() {
    ip -statistics link show "$CAN_IF" 2>/dev/null | grep -A1 "TX:" | tail -1 | xargs | cut -d' ' -f2
}

echo "CAN watchdog started: monitoring $CAN_IF every ${CHECK_INTERVAL}s"

LAST_TX=$(get_tx_packets)
STALE_COUNT=0

while true; do
    sleep "$CHECK_INTERVAL"

    CURRENT_TX=$(get_tx_packets)

    if [ -z "$CURRENT_TX" ]; then
        echo "$(date '+%H:%M:%S') WARNING: $CAN_IF interface not found"
        STALE_COUNT=$((STALE_COUNT + 1))
    elif [ "$CURRENT_TX" = "$LAST_TX" ]; then
        STALE_COUNT=$((STALE_COUNT + 1))
        if [ "$STALE_COUNT" -ge "$STALE_THRESHOLD" ]; then
            echo "$(date '+%H:%M:%S') TX frozen at $CURRENT_TX for ${STALE_COUNT} checks - resetting USB adapter"

            ip link set "$CAN_IF" down 2>/dev/null
            usbreset "$USB_VID_PID" 2>/dev/null
            sleep 2
            ip link set "$CAN_IF" type can bitrate 1000000 restart-ms 100 2>/dev/null
            ip link set "$CAN_IF" txqueuelen 1000
            ip link set "$CAN_IF" up 2>/dev/null

            echo "$(date '+%H:%M:%S') USB reset complete, $CAN_IF reconfigured"
            STALE_COUNT=0
            LAST_TX=$(get_tx_packets)
        fi
    else
        if [ "$STALE_COUNT" -gt 0 ]; then
            echo "$(date '+%H:%M:%S') TX recovered: $LAST_TX -> $CURRENT_TX"
        fi
        STALE_COUNT=0
        LAST_TX="$CURRENT_TX"
    fi
done
