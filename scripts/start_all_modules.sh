#!/bin/bash
# Start all AIZEE motor control modules

set -e

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
ROVER="ltr@192.168.0.27"
ARM="pi@192.168.0.28"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "======================================"
echo "  Starting AIZEE Modules"
echo "======================================"
echo ""

# Start Rover Module
echo -e "${YELLOW}Starting Rover Module (Jetson)...${NC}"
ssh -i "$SSH_KEY" "$ROVER" "sudo systemctl start aizee-motor-control-rover"
sleep 2
STATUS=$(ssh -i "$SSH_KEY" "$ROVER" "sudo systemctl is-active aizee-motor-control-rover")
if [ "$STATUS" == "active" ]; then
    echo -e "${GREEN}✓ Rover module started${NC}"
else
    echo -e "✗ Rover module failed to start"
    exit 1
fi
echo ""

# Start Arm Module
echo -e "${YELLOW}Starting Arm Module (RPi4)...${NC}"
ssh "$ARM" "sudo systemctl start aizee-motor-control-arm"
sleep 2
STATUS=$(ssh "$ARM" "sudo systemctl is-active aizee-motor-control-arm")
if [ "$STATUS" == "active" ]; then
    echo -e "${GREEN}✓ Arm module started${NC}"
else
    echo -e "✗ Arm module failed to start"
    exit 1
fi
echo ""

echo "======================================"
echo -e "${GREEN}All modules started successfully!${NC}"
echo ""
echo "To view logs:"
echo "  Rover: ssh -i $SSH_KEY $ROVER sudo journalctl -u aizee-motor-control-rover -f"
echo "  Arm:   ssh $ARM sudo journalctl -u aizee-motor-control-arm -f"
echo ""
echo "To start teleop:"
echo "  python python/teleop/teleop.py"
echo "======================================"
