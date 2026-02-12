#!/usr/bin/env bash
# Reset dual CAN adapters and motor control service
# Run with: sudo ./scripts/reset_dual_can_and_motors.sh

set -e

echo "=== Resetting Dual CAN and Motor Control ==="

# Step 1: Kill motor_control if running
echo "1. Stopping motor_control..."
killall -9 motor_control 2>/dev/null || true
sleep 2
# Also kill any processes holding ZMQ ports
lsof -ti:5555,5556 2>/dev/null | xargs kill -9 2>/dev/null || true
sleep 2

# Step 2: Bring down CAN interfaces
echo "2. Bringing down CAN interfaces..."
ip link set can1 down 2>/dev/null || true
ip link set can2 down 2>/dev/null || true
sleep 1

# Step 3: Find and reset USB CAN adapters
echo "3. Resetting USB CAN adapters..."

# Find can1 USB device
CAN1_USB=$(basename $(readlink -f /sys/class/net/can1/device) 2>/dev/null) || CAN1_USB=""
if [ -n "$CAN1_USB" ]; then
    echo "   Resetting can1 USB device: $CAN1_USB"
    echo "$CAN1_USB" > /sys/bus/usb/drivers/gs_usb/unbind 2>/dev/null || true
    sleep 1
    echo "$CAN1_USB" > /sys/bus/usb/drivers/gs_usb/bind 2>/dev/null || true
    sleep 1
fi

# Find can2 USB device
CAN2_USB=$(basename $(readlink -f /sys/class/net/can2/device) 2>/dev/null) || CAN2_USB=""
if [ -n "$CAN2_USB" ]; then
    echo "   Resetting can2 USB device: $CAN2_USB"
    echo "$CAN2_USB" > /sys/bus/usb/drivers/gs_usb/unbind 2>/dev/null || true
    sleep 1
    echo "$CAN2_USB" > /sys/bus/usb/drivers/gs_usb/bind 2>/dev/null || true
    sleep 1
fi

# Step 4: Configure CAN interfaces
echo "4. Configuring CAN interfaces..."
BITRATE=1000000
TXQLEN=1000

sleep 2  # Wait for USB devices to stabilize after bind

# Setup CAN1 (gantry/arm motors: 0x05, 0x06, 0x07)
ip link set can1 type can bitrate ${BITRATE} 2>/dev/null || true
ip link set can1 txqueuelen ${TXQLEN}
ip link set can1 up
echo "   can1: UP, bitrate ${BITRATE}, txqlen ${TXQLEN}"

# Setup CAN2 (rover base motors: 0x02, 0x03, 0x04)
ip link set can2 type can bitrate ${BITRATE} 2>/dev/null || true
ip link set can2 txqueuelen ${TXQLEN}
ip link set can2 up
echo "   can2: UP, bitrate ${BITRATE}, txqlen ${TXQLEN}"

sleep 1

# Step 5: Restart motor_control
echo "5. Starting motor_control..."
# Run as the user who invoked sudo, not as root
su - $SUDO_USER -c "cd ~/aizee && AIZEE_CONFIG=config/hardware_jetson_dual_can.yaml RUST_LOG=info nohup ./rust/target/release/motor_control > motor_control.log 2>&1 &"
sleep 2
MOTOR_PID=$(pgrep -f "motor_control" | tail -1)

# Step 6: Verify
echo ""
echo "=== Status ==="
ip link show can1 | head -1
ip link show can2 | head -1
if ps -p $MOTOR_PID > /dev/null; then
    echo "motor_control: RUNNING (PID $MOTOR_PID)"
else
    echo "motor_control: FAILED TO START"
    exit 1
fi

echo ""
echo "=== Recent motor_control log ==="
tail -10 /home/$SUDO_USER/aizee/motor_control.log

echo ""
echo "=== Reset complete ==="
