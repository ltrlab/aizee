#!/bin/bash
# Install RPLiDAR udev rules on Jetson
# Run this script ON THE JETSON with: sudo bash install_lidar_udev.sh

set -e

echo "Installing RPLiDAR udev rules..."

# Create rules file
cat > /etc/udev/rules.d/99-rplidar.rules << 'RULES'
# Udev rules for RPLiDAR A1M8 sensors
# Uses USB port location since serial numbers are identical

# Front LiDAR (USB port 1-2.2)
SUBSYSTEM=="tty", KERNELS=="1-2.2", SYMLINK+="rplidar_front", MODE="0666", GROUP="dialout"

# Back LiDAR (USB port 1-2.4)
SUBSYSTEM=="tty", KERNELS=="1-2.4", SYMLINK+="rplidar_back", MODE="0666", GROUP="dialout"
RULES

echo "Udev rules installed to /etc/udev/rules.d/99-rplidar.rules"

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "Done! Checking symlinks..."
ls -la /dev/rplidar_* 2>/dev/null || echo "Symlinks not found yet. Try unplugging and replugging the USB devices."
