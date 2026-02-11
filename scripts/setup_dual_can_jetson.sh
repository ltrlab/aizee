#!/usr/bin/env bash
# Dual CAN Interface Setup Script for AIZEE Jetson
# Sets up can1 (gantry) and can2 (rover base) with proper TX queue lengths

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "===================================="
echo " AIZEE Dual-CAN Setup (Jetson)"
echo "===================================="
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root (use sudo)${NC}"
    exit 1
fi

BITRATE=1000000
TXQLEN=1000  # Increased from default 10 to prevent "No buffer space" errors

echo "Setting up CAN interfaces:"
echo "  - Bitrate: ${BITRATE} (1 Mbps)"
echo "  - TX Queue Length: ${TXQLEN} frames"
echo

# Setup CAN1 (gantry/arm motors: 0x05, 0x06, 0x07)
echo -e "${YELLOW}Configuring can1 (gantry motors)...${NC}"
ip link set can1 down 2>/dev/null || true
ip link set can1 type can bitrate ${BITRATE}
ip link set can1 txqueuelen ${TXQLEN}
ip link set can1 up

if ip link show can1 | grep -q "UP"; then
    echo -e "${GREEN}✓ can1 is UP${NC}"
else
    echo -e "${RED}✗ can1 failed to come up${NC}"
    exit 1
fi

# Setup CAN2 (rover base motors: 0x02, 0x03, 0x04)
echo -e "${YELLOW}Configuring can2 (rover motors)...${NC}"
ip link set can2 down 2>/dev/null || true
ip link set can2 type can bitrate ${BITRATE}
ip link set can2 txqueuelen ${TXQLEN}
ip link set can2 up

if ip link show can2 | grep -q "UP"; then
    echo -e "${GREEN}✓ can2 is UP${NC}"
else
    echo -e "${RED}✗ can2 failed to come up${NC}"
    exit 1
fi

echo
echo "===================================="
echo " CAN Interfaces Ready"
echo "===================================="
echo
ip -brief link show can1 can2
echo
echo "Details:"
ip -details link show can1 | grep "qlen\|state\|bitrate"
echo
ip -details link show can2 | grep "qlen\|state\|bitrate"
echo
echo "To monitor traffic:"
echo "  candump can1  # Gantry motors"
echo "  candump can2  # Rover motors"
echo
echo "To check statistics:"
echo "  ip -statistics link show can1"
echo "  ip -statistics link show can2"
echo
