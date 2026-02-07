#!/bin/bash
# Comprehensive ROBSTRIDE motor debugging script
# Tests CAN interface, motor connectivity, and control

set -e

MOTOR_ID=2
CAN_INTERFACE="can1"

echo "========================================="
echo "ROBSTRIDE Motor Debugging Script"
echo "Motor ID: $MOTOR_ID"
echo "CAN Interface: $CAN_INTERFACE"
echo "========================================="
echo ""

# Task 1: Check CAN interface
echo "[1/5] Checking CAN interface status..."
if ip link show $CAN_INTERFACE > /dev/null 2>&1; then
    echo "✓ CAN interface $CAN_INTERFACE exists"
    ip link show $CAN_INTERFACE | grep -E "UP|DOWN|state"

    # Check if UP
    if ip link show $CAN_INTERFACE | grep -q "UP"; then
        echo "✓ CAN interface is UP"
    else
        echo "✗ CAN interface is DOWN - bringing it up..."
        sudo ip link set $CAN_INTERFACE up type can bitrate 1000000
        sleep 1
        echo "✓ CAN interface brought up"
    fi
else
    echo "✗ CAN interface $CAN_INTERFACE not found!"
    echo "Available interfaces:"
    ip link show | grep can
    exit 1
fi
echo ""

# Task 2: Scan for motor on CAN bus
echo "[2/5] Scanning for motor ID $MOTOR_ID..."
cd ~/aizee/tests/utils
timeout 5 python3 scan_all_motors.py || echo "Scan completed (or timed out)"
echo ""

# Task 3: Test with direct Python CAN
echo "[3/5] Testing motor with direct Python CAN commands..."
echo "This will enable the motor and send a position command for 3 seconds..."
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd ~/aizee/tests/direct_can
    timeout 5 python3 motor_control_test.py || echo "Test completed (or interrupted)"
else
    echo "Skipped"
fi
echo ""

# Task 4: Check Rust binary
echo "[4/5] Checking Rust motor_control binary..."
if [ -f ~/aizee/rust/target/release/motor_control ]; then
    echo "✓ Binary exists"
    ls -lh ~/aizee/rust/target/release/motor_control
else
    echo "✗ Binary not found - building..."
    cd ~/aizee/rust/motor_control
    cargo build --release
fi
echo ""

# Task 5: Test with Rust motor_control
echo "[5/5] Testing with Rust motor_control + ZeroMQ..."
echo "This will:"
echo "  1. Start motor_control in background"
echo "  2. Enable motor via ZeroMQ"
echo "  3. Send drive commands"
echo "  4. Monitor for 5 seconds"
echo "  5. Disable motor and stop"
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    # Kill any existing motor_control
    pkill -9 motor_control || true
    sleep 1

    # Start motor_control
    echo "Starting motor_control..."
    cd ~/aizee
    RUST_LOG=info AIZEE_CONFIG=config/hardware_two_motors.yaml \
        ./rust/target/release/motor_control > /tmp/motor_debug.log 2>&1 &
    MOTOR_PID=$!
    echo "Motor control started (PID: $MOTOR_PID)"
    sleep 2

    # Enable motor
    echo "Enabling motor..."
    python3 ~/aizee/tests/utils/send_zmq_command.py enable right_wheel
    sleep 1

    # Send drive command
    echo "Sending drive command (0.2 rad/s)..."
    python3 ~/aizee/tests/utils/send_zmq_command.py drive 0.2 0.0

    # Monitor
    echo "Monitoring for 5 seconds..."
    echo "Check logs: tail -f /tmp/motor_debug.log"
    sleep 5

    # Stop
    echo "Stopping motor..."
    python3 ~/aizee/tests/utils/send_zmq_command.py drive 0.0 0.0
    sleep 1
    python3 ~/aizee/tests/utils/send_zmq_command.py disable right_wheel

    # Show logs
    echo ""
    echo "=== Last 20 lines of motor_control log ==="
    tail -20 /tmp/motor_debug.log

    # Stop motor_control
    kill $MOTOR_PID || true
else
    echo "Skipped"
fi
echo ""

echo "========================================="
echo "Debugging complete!"
echo "========================================="
