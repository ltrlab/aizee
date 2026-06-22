#!/bin/bash
# Deploy the AIZEE heartbeat dashboard (python/tools/heartbeat_server.py) to
# the Jetson and restart the aizee-heartbeat service.
#
# The dashboard now subscribes (read-only) to the robot's ZMQ telemetry
# streams and renders motors, batteries, and cameras at a glance:
#   motor telemetry  tcp://localhost:5556   (E-stop, per-motor state, 6S pack)
#   UPS telemetry    tcp://localhost:5562   (logic battery)
#   gripper camera   tcp://localhost:5563
#   scene camera     tcp://localhost:5564
#
# The systemd unit is unchanged (endpoints default to localhost), so this just
# copies the script and restarts the service.  pyzmq + msgpack are already
# present under /usr/bin/python3 (the ups/display nodes use them); if they were
# missing the telemetry section would degrade gracefully.
#
# Usage: ./scripts/deploy_heartbeat.sh [ltr@10.42.0.1]

set -e

TARGET="${1:-ltr@10.42.0.1}"
REMOTE_DIR="aizee"
SSH_KEY="${SSH_KEY:-/c/Users/ltr/Workspace/ssh-keys/aizee_rover_id}"
SSH="ssh -i $SSH_KEY -o StrictHostKeyChecking=accept-new"
SCP="scp -i $SSH_KEY -o StrictHostKeyChecking=accept-new"
PORT=8088

PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Heartbeat Deployment ==="
echo "Target: $TARGET"
echo ""

echo "Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 "$TARGET" true 2>/dev/null; then
    echo "ERROR: Cannot reach $TARGET"
    exit 1
fi
echo "Connected."
echo ""

echo "1. Copying heartbeat_server.py to Jetson..."
$SCP python/tools/heartbeat_server.py \
    "$TARGET:/home/ltr/$REMOTE_DIR/python/tools/heartbeat_server.py"
echo ""

echo "2. Restarting aizee-heartbeat service..."
$SSH "$TARGET" "echo '$JETSON_PASS' | sudo -S systemctl restart aizee-heartbeat" 2>&1
sleep 2
echo ""

echo "=== Service status ==="
$SSH "$TARGET" "systemctl status aizee-heartbeat --no-pager -l | head -n 12" 2>&1 || true
echo ""

echo "3. Smoke-testing /api/status (telemetry block)..."
$SSH "$TARGET" "curl -s http://localhost:$PORT/api/status \
    | python3 -c 'import sys,json; t=json.load(sys.stdin).get(\"telemetry\",{}); \
print(\"telemetry.available =\", t.get(\"available\")); \
print(\"  estop   =\", t.get(\"estop\")); \
print(\"  motors  =\", len((t.get(\"motors\") or {}).get(\"list\") or []), \"reporting, stale=\", (t.get(\"motors\") or {}).get(\"stale\")); \
print(\"  cameras =\", [(c[\"name\"], c[\"online\"], c[\"fps\"]) for c in (t.get(\"cameras\") or [])])'" 2>&1 || true
echo ""

LAN_IP="${TARGET#*@}"
echo "=== Deploy complete ==="
echo "Open: http://$LAN_IP:$PORT/"
echo "Tail logs:  ssh -i $SSH_KEY $TARGET 'journalctl -u aizee-heartbeat -f'"
