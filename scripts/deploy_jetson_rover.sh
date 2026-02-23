#!/bin/bash
# Deploy AIZEE rover module to Jetson Orin Nano
# Usage: ./deploy_jetson_rover.sh [ltr@192.168.0.27]
#
# Transfer method: tar + scp (rsync not required)
#   1. Pack rust/ and config/ into a local tarball (excluding build artifacts)
#   2. scp tarball to Jetson /tmp/
#   3. Extract into ~/aizee/ on Jetson
#   4. Build motor_control (cargo)
#   5. Install service file, daemon-reload, restart, show status
#      (uses ssh -tt so sudo can prompt for password interactively)

set -e

TARGET="${1:-ltr@192.168.0.27}"
REMOTE_DIR="aizee"
SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
SSH="ssh -i $SSH_KEY"
SCP="scp -i $SSH_KEY"
TARBALL="/tmp/aizee_deploy.tar.gz"

# Password for sudo on the Jetson — stored in scripts/.jetson_password (gitignored)
PASS_FILE="$(dirname "$0")/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found. Create it with the Jetson sudo password."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

echo "=== AIZEE Jetson Rover Module Deployment ==="
echo "Target: $TARGET"
echo ""

# Check SSH connectivity (more reliable than ping across platforms)
echo "Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: Cannot reach $TARGET via SSH"
    echo "Check: ssh -i $SSH_KEY $TARGET"
    exit 1
fi
echo "Connected."
echo ""

# Pack sources into a tarball, excluding build artifacts and repo metadata
echo "1. Packing rust/ and config/ ..."
tar czf "$TARBALL" \
    --exclude='rust/target' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='logs' \
    rust/ config/
echo "   $(du -h $TARBALL | cut -f1) packed."
echo ""

# Copy tarball to Jetson
echo "2. Copying tarball to Jetson..."
$SCP "$TARBALL" "$TARGET:/tmp/aizee_deploy.tar.gz"
rm -f "$TARBALL"
echo ""

# Extract on Jetson (preserves existing files not in the archive, e.g. logs/)
echo "3. Extracting on Jetson..."
$SSH "$TARGET" "mkdir -p ~/$REMOTE_DIR && cd ~/$REMOTE_DIR && tar xzf /tmp/aizee_deploy.tar.gz && rm /tmp/aizee_deploy.tar.gz"
echo ""

# Build motor_control on Jetson
echo "4. Building motor_control on Jetson (this takes a minute)..."
$SSH "$TARGET" "cd ~/$REMOTE_DIR/rust/motor_control && source ~/.cargo/env && cargo build --release 2>&1"
echo ""

# Install service file, reload, restart, and show status.
# sudo -S reads the password from the first line of stdin; bash -s reads the
# subsequent lines as commands — pipe both together in one group.
echo "5. Installing service, restarting, and checking status..."
{
    echo "$JETSON_PASS"
    cat << 'SUDO_CMDS'
cp ~/aizee/config/systemd/aizee-motor-control-rover.service /etc/systemd/system/
systemctl daemon-reload
systemctl restart aizee-motor-control-rover
sleep 1
systemctl status aizee-motor-control-rover --no-pager -l
SUDO_CMDS
} | $SSH "$TARGET" "sudo -S bash -s 2>/dev/null"
echo ""

echo "=== Deploy complete! ==="
echo ""
echo "Follow logs:  ssh -i $SSH_KEY $TARGET sudo journalctl -u aizee-motor-control-rover -f"
