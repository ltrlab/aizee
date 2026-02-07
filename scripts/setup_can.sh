#!/usr/bin/env bash
# CAN Interface Setup Script for AIZEE

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "===================================="
echo " AIZEE CAN Interface Setup"
echo "===================================="
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root (use sudo)${NC}"
    exit 1
fi

# Detect adapter type
echo "Select CAN adapter type:"
echo "1) CANable/slcan (USB adapter, /dev/ttyACM0 or /dev/ttyUSB0)"
echo "2) Native SocketCAN (built-in CAN controller)"
echo "3) PEAK PCAN-USB"
read -p "Enter choice [1-3]: " adapter_choice

case $adapter_choice in
    1)
        echo -e "\n${GREEN}Setting up CANable/slcan adapter...${NC}"

        # Find USB serial device
        if [ -e /dev/ttyACM0 ]; then
            DEVICE="/dev/ttyACM0"
        elif [ -e /dev/ttyUSB0 ]; then
            DEVICE="/dev/ttyUSB0"
        else
            read -p "Enter device path (e.g., /dev/ttyACM0): " DEVICE
        fi

        echo "Using device: $DEVICE"

        # Load slcan module
        modprobe slcan || true

        # Stop any existing slcan instance
        killall slcand 2>/dev/null || true
        sleep 1

        # Attach slcan device
        # -o: open device
        # -c: close device on exit
        # -s8: 1 Mbps bitrate
        slcand -o -c -s8 $DEVICE can0

        # Bring up interface
        ip link set can0 up

        echo -e "${GREEN}✓ CANable adapter configured${NC}"
        ;;

    2)
        echo -e "\n${GREEN}Setting up native SocketCAN...${NC}"

        # Configure bitrate
        ip link set can0 down 2>/dev/null || true
        ip link set can0 type can bitrate 1000000
        ip link set can0 up

        echo -e "${GREEN}✓ Native CAN interface configured${NC}"
        ;;

    3)
        echo -e "\n${GREEN}Setting up PEAK PCAN-USB...${NC}"

        # PEAK adapters usually work with native SocketCAN
        ip link set can0 down 2>/dev/null || true
        ip link set can0 type can bitrate 1000000
        ip link set can0 up

        echo -e "${GREEN}✓ PCAN-USB configured${NC}"
        ;;

    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

# Verify interface is up
echo
echo "Verifying CAN interface..."
if ip link show can0 | grep -q "UP"; then
    echo -e "${GREEN}✓ can0 is UP${NC}"
    ip link show can0
else
    echo -e "${RED}✗ can0 failed to come up${NC}"
    exit 1
fi

echo
echo "===================================="
echo " CAN Interface Ready"
echo "===================================="
echo
echo "To monitor CAN traffic:"
echo "  candump can0"
echo
echo "To bring down the interface:"
echo "  sudo ip link set can0 down"
echo
echo "To send a test frame:"
echo "  cansend can0 001#1122334455667788"
echo
