#!/bin/bash
# Deploy lidar_control to Jetson Orin Nano

set -e

# Configuration
source "$(dirname "$0")/deploy_common.sh"
JETSON_IP="${AIZEE_TARGET#*@}"
JETSON_USER="${AIZEE_TARGET%%@*}"
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "========================================"
echo "AIZEE LiDAR Control Deployment"
echo "========================================"
echo ""
echo "Target: ${JETSON_USER}@${JETSON_IP}"
echo ""

# Check SSH key exists
if [ ! -f "$SSH_KEY" ]; then
    echo "ERROR: SSH key not found at $SSH_KEY"
    exit 1
fi

# Function to run SSH commands
ssh_exec() {
    ssh -i "$SSH_KEY" "${JETSON_USER}@${JETSON_IP}" "$@"
}

# Function to copy files
scp_copy() {
    scp -i "$SSH_KEY" "$@"
}

echo "Step 1: Deploying code..."
cd "$WORKSPACE_DIR"

# Cargo resolves every member's manifest, even when building only lidar_control.
# Copy the whole shared workspace without build artifacts.
echo "  - Deploying the shared Rust workspace..."
WORKSPACE_FILES=(Cargo.toml .cargo)
[[ -f Cargo.lock ]] && WORKSPACE_FILES+=(Cargo.lock)
TARBALL="$(mktemp "${TMPDIR:-/tmp}/aizee_lidar_deploy.XXXXXX.tar.gz")"
trap 'rm -f "$TARBALL"' EXIT
tar czf "$TARBALL" --exclude='rust/target' "${WORKSPACE_FILES[@]}" rust/
scp_copy "$TARBALL" "${JETSON_USER}@${JETSON_IP}:/tmp/aizee_lidar_deploy.tar.gz"
ssh_exec "mkdir -p ~/aizee/config/systemd ~/aizee/python && cd ~/aizee && tar xzf /tmp/aizee_lidar_deploy.tar.gz && rm -f rust/Cargo.toml && rm /tmp/aizee_lidar_deploy.tar.gz"

# Deploy config
echo "  - Deploying config files..."
scp_copy config/hardware_jetson_rover.yaml "${JETSON_USER}@${JETSON_IP}:~/aizee/config/"
scp_copy config/systemd/aizee-lidar-control.service "${JETSON_USER}@${JETSON_IP}:~/aizee/config/systemd/"

# Create udev directory and deploy rules
ssh_exec "mkdir -p ~/aizee/config/udev"
scp_copy config/udev/99-rplidar.rules "${JETSON_USER}@${JETSON_IP}:~/aizee/config/udev/"

echo ""
echo "Step 2: Building on Jetson..."
ssh_exec "source ~/.cargo/env && cd ~/aizee && cargo build --release -p lidar_control"

echo ""
echo "Step 3: Checking USB devices..."
echo "Connected USB devices:"
ssh_exec "lsusb | grep 'Silicon Labs' || echo '  No Silicon Labs devices found'"
echo ""
echo "Available ttyUSB devices:"
ssh_exec "ls -la /dev/ttyUSB* 2>/dev/null || echo '  No ttyUSB devices found'"
echo ""

echo "========================================"
echo "Deployment complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo ""
echo "1. Configure udev rules (if not done):"
echo "   ssh -i $SSH_KEY ${JETSON_USER}@${JETSON_IP}"
echo "   # Find serial numbers:"
echo "   sudo udevadm info -a -n /dev/ttyUSB0 | grep serial"
echo "   sudo udevadm info -a -n /dev/ttyUSB1 | grep serial"
echo "   # Edit udev rules:"
echo "   sudo nano /etc/udev/rules.d/99-rplidar.rules"
echo "   # (Copy from ~/aizee/config/udev/99-rplidar.rules and update serials)"
echo "   # Reload:"
echo "   sudo udevadm control --reload-rules && sudo udevadm trigger"
echo ""
echo "2. Test manually:"
echo "   cd ~/aizee"
echo "   AIZEE_CONFIG=~/aizee/config/hardware_jetson_rover.yaml RUST_LOG=info ./rust/target/release/lidar_control"
echo ""
echo "3. Install systemd service:"
echo "   sudo cp ~/aizee/config/systemd/aizee-lidar-control.service /etc/systemd/system/"
echo "   sudo systemctl daemon-reload"
echo "   sudo systemctl enable aizee-lidar-control"
echo "   sudo systemctl start aizee-lidar-control"
echo ""
echo "4. Inspect service output:"
echo "   journalctl -u aizee-lidar-control -n 50 --no-pager"
echo ""
