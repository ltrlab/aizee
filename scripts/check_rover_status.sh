#!/bin/bash
# Check status of the AIZEE Jetson: network, CAN, motor service, telemetry.
# Usage: ./check_rover_status.sh [user@host]   (auto-detects the target otherwise)

set -e

source "$(dirname "$0")/deploy_common.sh"
ROVER="${1:-$AIZEE_TARGET}"
HOST="${ROVER#*@}"
SSH="ssh -i $SSH_KEY"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================"
echo "  AIZEE Rover Status"
echo "======================================"
echo ""

echo "--- Jetson ($HOST) ---"
if $SSH -o ConnectTimeout=3 -o BatchMode=yes "$ROVER" true 2>/dev/null; then
    echo -e "${GREEN}✓ Network: Online${NC}"

    # Check CAN interface (the motors live on can1; ExecStartPre renames strays)
    CAN_STATUS=$($SSH "$ROVER" "ip link show can1 2>/dev/null | grep 'state UP' || echo 'DOWN'" 2>/dev/null || echo "ERROR")
    if [[ "$CAN_STATUS" == *"UP"* ]]; then
        echo -e "${GREEN}✓ CAN Interface (can1): UP${NC}"
        $SSH "$ROVER" "ip link show can1 | grep -E 'can1|bitrate'"
    else
        echo -e "${RED}✗ CAN Interface (can1): DOWN${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER 'sudo /usr/local/bin/aizee-reset-usb-can can1'"
    fi

    # Check service
    SERVICE_STATUS=$($SSH "$ROVER" "systemctl is-active aizee-motor-control-rover 2>/dev/null || echo 'inactive'")
    if [ "$SERVICE_STATUS" == "active" ]; then
        echo -e "${GREEN}✓ Motor Control Service: Running${NC}"
    else
        echo -e "${RED}✗ Motor Control Service: Not Running${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER sudo systemctl start aizee-motor-control-rover"
    fi

    # Check if telemetry is publishing (msgpack wire format on :5556)
    echo ""
    echo "Testing telemetry (5 second timeout)..."
    if python3 -c "
import sys, zmq, msgpack
ctx = zmq.Context(); s = ctx.socket(zmq.SUB)
s.connect('tcp://$HOST:5556'); s.setsockopt(zmq.SUBSCRIBE, b''); s.setsockopt(zmq.RCVTIMEO, 5000)
data = msgpack.unpackb(s.recv(), raw=False)
print('✓ Telemetry OK: %d motors' % len(data.get('motors', {})))" 2>/dev/null; then
        :
    else
        echo -e "${RED}✗ No telemetry received on :5556${NC}"
    fi

    # Heartbeat dashboard
    if curl -fsS -m 3 "http://$HOST:8088/healthz" >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Heartbeat dashboard: http://$HOST:8088${NC}"
    else
        echo -e "${YELLOW}– Heartbeat dashboard not responding on :8088${NC}"
    fi
else
    echo -e "${RED}✗ Network: Offline (tried $HOST)${NC}"
fi
echo ""

echo "======================================"
echo "To view logs:"
echo "  ssh -i $SSH_KEY $ROVER sudo journalctl -u aizee-motor-control-rover -f"
echo ""
echo "Guided validation checklist:"
echo "  http://$HOST:8088/setup"
echo "======================================"
