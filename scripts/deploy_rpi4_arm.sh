#!/bin/bash
# Deploy AIZEE arm module to Raspberry Pi 4
# Usage: ./deploy_rpi4_arm.sh [pi@192.168.0.28]

set -e

TARGET="${1:-pi@192.168.0.28}"
REMOTE_DIR="aizee"

echo "=== AIZEE RPi4 Arm Module Deployment ==="
echo "Target: $TARGET"
echo "Remote directory: ~/$REMOTE_DIR"
echo ""

# Check if target is reachable
HOST=$(echo $TARGET | cut -d'@' -f2)
if ! ping -c 1 -W 2 "$HOST" > /dev/null 2>&1; then
    echo "ERROR: Cannot reach $HOST"
    exit 1
fi

echo "1. Syncing codebase..."
rsync -av --delete \
    --exclude 'target/' \
    --exclude '.git/' \
    --exclude '*.pyc' \
    --exclude '__pycache__/' \
    --exclude 'logs/' \
    ./ "$TARGET:~/$REMOTE_DIR/"

echo ""
echo "2. Building motor_control on RPi4..."
ssh "$TARGET" "cd ~/$REMOTE_DIR/rust/motor_control && cargo build --release"

echo ""
echo "3. Installing systemd service..."
ssh "$TARGET" "sudo cp ~/$REMOTE_DIR/config/systemd/aizee-motor-control-arm.service /etc/systemd/system/"
ssh "$TARGET" "sudo systemctl daemon-reload"

echo ""
echo "4. Setup complete!"
echo ""
echo "To start the service:"
echo "  ssh $TARGET sudo systemctl start aizee-motor-control-arm"
echo ""
echo "To enable on boot:"
echo "  ssh $TARGET sudo systemctl enable aizee-motor-control-arm"
echo ""
echo "To view logs:"
echo "  ssh $TARGET sudo journalctl -u aizee-motor-control-arm -f"
