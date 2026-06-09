#!/bin/bash
# AIZEE system shutdown — stops Jetson services cleanly, then powers it down.
#
# Usage:
#   ./shutdown_all.sh        # with confirmation prompt
#   ./shutdown_all.sh -y     # skip prompt

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found (needed for Jetson sudo)"
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Shutdown ==="
echo ""
echo "Will shut down in order:"
echo "  1. Jetson services (motor-control, cameras, lidar, ups)"
echo "  2. Jetson (192.168.0.27)"
echo ""

if [[ "$1" != "-y" ]]; then
    read -p "Proceed with shutdown? (y/N): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo "Shutdown cancelled."
        exit 0
    fi
fi

# ── Step 1: Stop Jetson services ─────────────────────────────────────────────
echo ""
echo "=== Stopping Jetson services ==="

JETSON_SERVICES=(
    aizee-motor-control-rover
    aizee-gripper-cam
    aizee-scene-cam
    aizee-lidar-control
    aizee-ups-monitor
)

for svc in "${JETSON_SERVICES[@]}"; do
    echo "  Stopping $svc..."
    echo "$JETSON_PASS" | $JETSON_SSH "sudo -S systemctl stop $svc" 2>/dev/null \
        || echo "  Warning: $svc may already be stopped"
done

# ── Step 2: Shutdown Jetson ──────────────────────────────────────────────────
echo ""
echo "=== Shutting down Jetson ==="
echo "$JETSON_PASS" | $JETSON_SSH "sudo -S shutdown -h now" 2>/dev/null || true

echo ""
echo "=== Shutdown command sent ==="
echo ""
echo "The Jetson will be offline in ~30-60 seconds."
echo "Safe to cut power after that."
echo ""
