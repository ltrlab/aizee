#!/bin/bash
# Full AIZEE system shutdown — Pis then Jetson
# Stops all services cleanly before powering down each node.
#
# Usage:
#   ./shutdown_all.sh        # with confirmation prompt
#   ./shutdown_all.sh -y     # skip prompt

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
JETSON_IP="192.168.0.27"
JETSON_SSH="ssh -i $SSH_KEY ltr@$JETSON_IP"
PI_SSH="ssh -i ~/.ssh/aizee_rover_id -o ConnectTimeout=5 -o StrictHostKeyChecking=no"

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found (needed for Jetson sudo)"
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Full System Shutdown ==="
echo ""
echo "Will shut down in order:"
echo "  1. Camera services on all Pis"
echo "  2. Raspberry Pi nodes (10.42.0.11-14)"
echo "  3. Jetson services (motor-control, camera-relay, lidar, ups)"
echo "  4. Jetson (192.168.0.27)"
echo ""

if [[ "$1" != "-y" ]]; then
    read -p "Proceed with full system shutdown? (y/N): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo "Shutdown cancelled."
        exit 0
    fi
fi

# ── Step 1: Stop Pi camera services ──────────────────────────────────────────
echo ""
echo "=== Stopping Pi camera services ==="

declare -A CAM_MAP
CAM_MAP["10.42.0.11"]="cam_front"
CAM_MAP["10.42.0.12"]="cam_rear"
CAM_MAP["10.42.0.13"]="cam_left"
CAM_MAP["10.42.0.14"]="cam_right"

for ip in 10.42.0.11 10.42.0.12 10.42.0.13 10.42.0.14; do
    cam="${CAM_MAP[$ip]}"
    echo "  Stopping $cam ($ip)..."
    $JETSON_SSH "$PI_SSH ltr@$ip 'sudo systemctl stop aizee-camera-$cam'" 2>/dev/null \
        || echo "  Warning: $cam may already be stopped or unreachable"
done

# ── Step 2: Shutdown Pis ─────────────────────────────────────────────────────
echo ""
echo "=== Shutting down Raspberry Pis ==="

for ip in 10.42.0.11 10.42.0.12 10.42.0.13 10.42.0.14; do
    echo "  Shutting down $ip..."
    $JETSON_SSH "$PI_SSH ltr@$ip 'sudo shutdown -h now'" 2>/dev/null &
done
wait
echo "  Shutdown commands sent — Pis will power off in ~30s"

# ── Step 3: Stop Jetson services ─────────────────────────────────────────────
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

# ── Step 4: Shutdown Jetson ──────────────────────────────────────────────────
echo ""
echo "=== Shutting down Jetson ==="
echo "$JETSON_PASS" | $JETSON_SSH "sudo -S shutdown -h now" 2>/dev/null || true

echo ""
echo "=== Shutdown commands sent to all nodes ==="
echo ""
echo "All nodes will be offline in ~30-60 seconds."
echo "Safe to cut power after that."
echo ""
