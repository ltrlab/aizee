#!/usr/bin/env bash
# Launch script for AIZEE motor control system

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "===================================="
echo " AIZEE Motor Control Launcher"
echo "===================================="
echo

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Check if motor_control binary exists
MOTOR_CONTROL_BIN="$PROJECT_ROOT/rust/motor_control/target/release/motor_control"

if [ ! -f "$MOTOR_CONTROL_BIN" ]; then
    echo -e "${YELLOW}Motor control binary not found. Building...${NC}"
    cd "$PROJECT_ROOT/rust/motor_control"
    cargo build --release
    cd "$PROJECT_ROOT"
    echo -e "${GREEN}✓ Build complete${NC}"
    echo
fi

# Check if CAN interface is up
if ! ip link show can1 &>/dev/null; then
    echo -e "${RED}✗ CAN interface 'can1' not found${NC}"
    echo "Run: sudo $SCRIPT_DIR/setup_can.sh"
    exit 1
fi

if ! ip link show can1 | grep -q "UP"; then
    echo -e "${RED}✗ CAN interface 'can1' is DOWN${NC}"
    echo "Run: sudo ip link set can1 up"
    exit 1
fi

echo -e "${GREEN}✓ CAN interface ready${NC}"

# Set config path
export AIZEE_CONFIG="${AIZEE_CONFIG:-$PROJECT_ROOT/config/hardware.yaml}"

if [ ! -f "$AIZEE_CONFIG" ]; then
    echo -e "${RED}✗ Config file not found: $AIZEE_CONFIG${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Config loaded: $AIZEE_CONFIG${NC}"

# Set log level
export RUST_LOG="${RUST_LOG:-info}"

echo
echo "Starting motor control system..."
echo "  - Arm control: 1 kHz"
echo "  - Base control: 100 Hz"
echo "  - Telemetry: 50 Hz"
echo
echo "Commands:     tcp://*:5555"
echo "Telemetry:    tcp://*:5556"
echo
echo "Press Ctrl+C to stop"
echo
echo "===================================="
echo

# Run motor control
cd "$PROJECT_ROOT"
exec "$MOTOR_CONTROL_BIN"
