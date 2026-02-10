#!/bin/bash
# Cleanly shutdown all Raspberry Pi camera nodes
# Usage: ./shutdown_all_pis.sh

set -e

echo "=== AIZEE Raspberry Pi Shutdown Script ==="
echo ""
echo "This will cleanly shutdown all 4 camera Raspberry Pis:"
echo "  - Pi 1 (Front): 192.168.0.22"
echo "  - Pi 2 (Rear):  192.168.0.23"
echo "  - Pi 3 (Left):  192.168.0.24"
echo "  - Pi 4 (Right): 192.168.0.25"
echo ""

# Ask for confirmation (can be skipped with -y flag)
if [[ "$1" != "-y" ]]; then
    read -p "Are you sure you want to shutdown all Pis? (y/N): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo "Shutdown cancelled."
        exit 0
    fi
fi

echo ""
echo "=== Stopping camera services first ==="

for ip in 22 23 24 25; do
    case $ip in
        22) cam="cam_front"; name="Pi 1 (Front)" ;;
        23) cam="cam_rear";  name="Pi 2 (Rear)" ;;
        24) cam="cam_left";  name="Pi 3 (Left)" ;;
        25) cam="cam_right"; name="Pi 4 (Right)" ;;
    esac

    echo "Stopping $name camera service..."
    ssh -o ConnectTimeout=5 ltr@192.168.0.$ip "sudo systemctl stop aizee-camera-$cam" 2>/dev/null || echo "  Warning: Could not stop service (may already be stopped)"
done

echo ""
echo "=== Initiating shutdown sequence ==="
echo ""

for ip in 22 23 24 25; do
    case $ip in
        22) name="Pi 1 (Front)" ;;
        23) name="Pi 2 (Rear)" ;;
        24) name="Pi 3 (Left)" ;;
        25) name="Pi 4 (Right)" ;;
    esac

    echo "Shutting down $name (192.168.0.$ip)..."
    ssh -o ConnectTimeout=5 ltr@192.168.0.$ip "sudo shutdown -h now" 2>/dev/null &
done

echo ""
echo "Waiting for shutdown commands to be sent..."
sleep 2

echo ""
echo "=== Shutdown commands sent to all Pis ==="
echo ""
echo "The Raspberry Pis are now shutting down. This will take about 30 seconds."
echo ""
echo "You can verify shutdown completion by checking:"
echo "  ping 192.168.0.22  # Should timeout when fully shutdown"
echo "  ping 192.168.0.23"
echo "  ping 192.168.0.24"
echo "  ping 192.168.0.25"
echo ""
echo "To power them back on, you'll need to:"
echo "  - Physically cycle power (unplug/replug) if no remote power management"
echo "  - Or use Wake-on-LAN if configured"
echo "  - Or use PoE management if using managed PoE switch"
echo ""
