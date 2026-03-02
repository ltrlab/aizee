#!/bin/bash
# Install AIZEE LiDAR control systemd service

set -e

echo "Installing AIZEE LiDAR Control service..."

# Install systemd service
sudo cp ~/aizee/config/systemd/aizee-lidar-control.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable service to start on boot
sudo systemctl enable aizee-lidar-control

# Check if lidar_control binary exists
if [ ! -f ~/aizee/rust/target/release/lidar_control ]; then
    echo "ERROR: lidar_control binary not found!"
    echo "Please build it first: cd ~/aizee/rust/lidar_control && cargo build --release"
    exit 1
fi

echo ""
echo "✅ LiDAR service installed and enabled!"
echo ""
echo "To start the service now:"
echo "  sudo systemctl start aizee-lidar-control"
echo ""
echo "To check status:"
echo "  sudo systemctl status aizee-lidar-control"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u aizee-lidar-control -f"
echo ""
