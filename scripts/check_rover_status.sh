#!/bin/bash
# Check status of AIZEE rover module only

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
ROVER="ltr@192.168.0.27"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================"
echo "  AIZEE Rover Status"
echo "======================================"
echo ""

# Check Rover Module (Jetson)
echo "--- Rover Module (Jetson 192.168.0.27) ---"
if ping -c 1 -W 2 192.168.0.27 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Network: Online${NC}"

    # Check CAN interface
    CAN_STATUS=$(ssh -i "$SSH_KEY" "$ROVER" "ip link show can0 2>/dev/null | grep 'state UP' || echo 'DOWN'" 2>/dev/null || echo "ERROR")
    if [[ "$CAN_STATUS" == *"UP"* ]]; then
        echo -e "${GREEN}✓ CAN Interface: UP${NC}"
        ssh -i "$SSH_KEY" "$ROVER" "ip link show can0 | grep -E 'can0|bitrate'"
    else
        echo -e "${RED}✗ CAN Interface: DOWN${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER 'sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up'"
    fi

    # Check service
    SERVICE_STATUS=$(ssh -i "$SSH_KEY" "$ROVER" "sudo systemctl is-active aizee-motor-control-rover 2>/dev/null || echo 'inactive'")
    if [ "$SERVICE_STATUS" == "active" ]; then
        echo -e "${GREEN}✓ Motor Control Service: Running${NC}"
    else
        echo -e "${RED}✗ Motor Control Service: Not Running${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER sudo systemctl start aizee-motor-control-rover"
    fi

    # Check if telemetry is publishing
    echo ""
    echo "Testing telemetry (5 second timeout)..."
    if python3 -c "import zmq, json; ctx = zmq.Context(); s = ctx.socket(zmq.SUB); s.connect('tcp://192.168.0.27:5556'); s.setsockopt(zmq.SUBSCRIBE, b''); s.setsockopt(zmq.RCVTIMEO, 5000); data = json.loads(s.recv_string()); print(f\"✓ Telemetry OK: {len(data.get('motors', {}))} motors\")" 2>/dev/null; then
        :
    else
        echo -e "${RED}✗ No telemetry received${NC}"
    fi
else
    echo -e "${RED}✗ Network: Offline${NC}"
fi
echo ""

echo "======================================"
echo "To view logs:"
echo "  ssh -i $SSH_KEY $ROVER sudo journalctl -u aizee-motor-control-rover -f"
echo ""
echo "To start teleop once rover is ready:"
echo "  python python/teleop/teleop.py --config config/teleop_rover_only.yaml"
echo "======================================"
