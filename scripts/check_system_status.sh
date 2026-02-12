#!/bin/bash
# Check status of all AIZEE modules

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
ROVER="ltr@192.168.0.27"
ARM="pi@192.168.0.28"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================"
echo "  AIZEE System Status"
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
    else
        echo -e "${RED}✗ CAN Interface: DOWN${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER sudo ip link set can0 type can bitrate 1000000"
        echo "       ssh -i $SSH_KEY $ROVER sudo ip link set can0 up"
    fi

    # Check service
    SERVICE_STATUS=$(ssh -i "$SSH_KEY" "$ROVER" "sudo systemctl is-active aizee-motor-control-rover 2>/dev/null || echo 'inactive'")
    if [ "$SERVICE_STATUS" == "active" ]; then
        echo -e "${GREEN}✓ Motor Control Service: Running${NC}"
    else
        echo -e "${RED}✗ Motor Control Service: Not Running${NC}"
        echo "  Run: ssh -i $SSH_KEY $ROVER sudo systemctl start aizee-motor-control-rover"
    fi
else
    echo -e "${RED}✗ Network: Offline${NC}"
fi
echo ""

# Check Arm Module (RPi4)
echo "--- Arm Module (RPi4 192.168.0.28) ---"
if ping -c 1 -W 2 192.168.0.28 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Network: Online${NC}"

    # Check CAN interface
    CAN_STATUS=$(ssh "$ARM" "ip link show can0 2>/dev/null | grep 'state UP' || echo 'DOWN'" 2>/dev/null || echo "ERROR")
    if [[ "$CAN_STATUS" == *"UP"* ]]; then
        echo -e "${GREEN}✓ CAN Interface: UP${NC}"
    else
        echo -e "${RED}✗ CAN Interface: DOWN${NC}"
        echo "  Run: ssh $ARM sudo ip link set can0 type can bitrate 1000000"
        echo "       ssh $ARM sudo ip link set can0 up"
    fi

    # Check service
    SERVICE_STATUS=$(ssh "$ARM" "sudo systemctl is-active aizee-motor-control-arm 2>/dev/null || echo 'inactive'")
    if [ "$SERVICE_STATUS" == "active" ]; then
        echo -e "${GREEN}✓ Motor Control Service: Running${NC}"
    else
        echo -e "${RED}✗ Motor Control Service: Not Running${NC}"
        echo "  Run: ssh $ARM sudo systemctl start aizee-motor-control-arm"
    fi
else
    echo -e "${RED}✗ Network: Offline${NC}"
fi
echo ""

echo "======================================"
echo "To start teleop once both modules are ready:"
echo "  python python/teleop/teleop.py"
echo "======================================"
