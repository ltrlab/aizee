#!/bin/bash
# Fix CAN1 configuration on Jetson - run this ON the Jetson via SSH

set -e

echo "========================================"
echo "Fixing CAN1 Configuration on Jetson"
echo "========================================"
echo ""

# Check if service file was copied
if [ ! -f ~/aizee-motor-control-rover.service ]; then
    echo "ERROR: Service file not found at ~/aizee-motor-control-rover.service"
    echo "Run this from dev machine first:"
    echo "  scp -i P:/Workspace/ssh-keys/aizee_rover_id P:/Workspace/aizee/config/systemd/aizee-motor-control-rover.service ltr@192.168.0.27:~/"
    exit 1
fi

echo "[1/5] Stopping current service..."
sudo systemctl stop aizee-motor-control-rover

echo "[2/5] Installing updated service file..."
sudo cp ~/aizee-motor-control-rover.service /etc/systemd/system/aizee-motor-control-rover.service
sudo chmod 644 /etc/systemd/system/aizee-motor-control-rover.service

echo "[3/5] Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "[4/5] Starting service with new config..."
sudo systemctl start aizee-motor-control-rover

echo "[5/5] Checking status..."
sleep 2
sudo systemctl status aizee-motor-control-rover --no-pager -l

echo ""
echo "========================================"
echo "Checking CAN interface status..."
echo "========================================"
ip link show can1

echo ""
echo "========================================"
echo "Checking for CAN traffic..."
echo "========================================"
echo "Monitoring can1 for 3 seconds..."
timeout 3 candump can1 || echo "(No CAN traffic detected - motors may be unpowered)"

echo ""
echo "========================================"
echo "Service updated successfully!"
echo "Config now uses can1 instead of can0"
echo "========================================"
