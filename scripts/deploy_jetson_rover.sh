#!/bin/bash
# Deploy AIZEE rover module to Jetson Orin Nano
# Usage: ./deploy_jetson_rover.sh [ltr@192.168.0.27]

set -e

TARGET="${1:-ltr@192.168.0.27}"
REMOTE_DIR="aizee"
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"

echo "=== AIZEE Jetson Rover Module Deployment ==="
echo "Target: $TARGET"
echo "Remote directory: ~/$REMOTE_DIR"
echo ""

# Check if target is reachable (cross-platform: Windows uses -n/-w, Linux uses -c/-W)
HOST=$(echo $TARGET | cut -d'@' -f2)
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || "$OS" == "Windows_NT" ]]; then
    PING_OK=$(ping -n 1 -w 2000 "$HOST" 2>&1 | grep -c "TTL=")
else
    PING_OK=$(ping -c 1 -W 2 "$HOST" > /dev/null 2>&1 && echo 1 || echo 0)
fi
if [[ "$PING_OK" == "0" ]]; then
    echo "ERROR: Cannot reach $HOST"
    exit 1
fi

echo "1. Syncing codebase..."
rsync -av --delete \
    -e "ssh -i $SSH_KEY" \
    --exclude 'target/' \
    --exclude '.git/' \
    --exclude '*.pyc' \
    --exclude '__pycache__/' \
    --exclude 'logs/' \
    ./ "$TARGET:~/$REMOTE_DIR/"

echo ""
echo "2. Building motor_control on Jetson..."
ssh -i "$SSH_KEY" "$TARGET" "cd ~/$REMOTE_DIR/rust/motor_control && source ~/.cargo/env && cargo build --release"

echo ""
echo "3. Installing systemd service..."
ssh -i "$SSH_KEY" "$TARGET" "sudo cp ~/$REMOTE_DIR/config/systemd/aizee-motor-control-rover.service /etc/systemd/system/"
ssh -i "$SSH_KEY" "$TARGET" "sudo systemctl daemon-reload"

echo ""
echo "4. Setup complete!"
echo ""
echo "To start the service:"
echo "  ssh -i $SSH_KEY $TARGET sudo systemctl start aizee-motor-control-rover"
echo ""
echo "To enable on boot:"
echo "  ssh -i $SSH_KEY $TARGET sudo systemctl enable aizee-motor-control-rover"
echo ""
echo "To view logs:"
echo "  ssh -i $SSH_KEY $TARGET sudo journalctl -u aizee-motor-control-rover -f"
