#!/bin/bash
# AIZEE fresh-Jetson bootstrap — run on the DEV machine (git-bash on Windows).
#
# Takes a freshly flashed Jetson Orin Nano (JetPack 6.x, user created during
# oem-config, reachable over the network) all the way to a running robot:
#   1. Installs your SSH public key (one password prompt on the first run)
#   2. Syncs the repo (tar+scp: rust/, config/, scripts/, python/, firmware/)
#   3. Runs scripts/setup_jetson.sh on the device (apt, rustup, pip, udev,
#      systemd, cargo build, WiFi AP) — sudo will prompt once via the TTY
#
# Usage:
#   ./scripts/bootstrap_jetson.sh [user@host] [-- setup_jetson.sh options...]
#
# Examples:
#   ./scripts/bootstrap_jetson.sh ltr@192.168.55.1 -- --ap-pass 'mypsk' --hostname aizee-jetson
#       (192.168.55.1 is the JetPack default USB-C device-mode address)
#   ./scripts/bootstrap_jetson.sh                # re-sync + re-run setup on 10.42.0.1
#
# Environment:
#   SSH_KEY  — private key path. Auto-detected from the usual locations if unset.
#
# NOTE: if the device was re-flashed, its host key changed; clear the old one:
#   ssh-keygen -R <host>

set -euo pipefail

TARGET="${1:-ltr@10.42.0.1}"
[[ "${TARGET}" == "--" ]] && TARGET="ltr@10.42.0.1"
shift $(( $# > 0 ? 1 : 0 )) || true
[[ "${1:-}" == "--" ]] && shift
SETUP_ARGS=("$@")

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TARBALL="/tmp/aizee_bootstrap.tar.gz"

# --- resolve SSH key (paths differ between git-bash /c and the P: mapped drive)
if [[ -z "${SSH_KEY:-}" ]]; then
    for cand in /c/Users/ltr/Workspace/ssh-keys/aizee_rover_id \
                /p/Workspace/ssh-keys/aizee_rover_id \
                "$HOME/.ssh/aizee_rover_id"; do
        [[ -f "$cand" ]] && SSH_KEY="$cand" && break
    done
fi
if [[ -z "${SSH_KEY:-}" ]]; then
    echo "ERROR: no SSH key found. Set SSH_KEY=<path to aizee_rover_id>."
    exit 1
fi
SSH_OPTS=(-i "$SSH_KEY" -o StrictHostKeyChecking=accept-new)

echo "=== AIZEE Jetson bootstrap (host side) ==="
echo "Target: $TARGET   Key: $SSH_KEY"
echo ""

# --- 1. key-based auth (falls back to one password prompt on a fresh device)
if ssh "${SSH_OPTS[@]}" -o BatchMode=yes -o ConnectTimeout=5 "$TARGET" true 2>/dev/null; then
    echo "1. SSH key auth OK."
else
    echo "1. Installing SSH public key (enter the Jetson password when prompted)..."
    ssh -o StrictHostKeyChecking=accept-new "$TARGET" \
        "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys" \
        < "${SSH_KEY}.pub"
    ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$TARGET" true
    echo "   Key installed."
fi
echo ""

# --- 2. sync the repo subset the device needs
echo "2. Packing and syncing repo..."
tar czf "$TARBALL" -C "$REPO_DIR" \
    --exclude='rust/target' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='logs' \
    --exclude='data' \
    --exclude='checkpoints' \
    rust config scripts python firmware requirements_jetson.txt
echo "   $(du -h "$TARBALL" | cut -f1) packed."
scp "${SSH_OPTS[@]}" "$TARBALL" "$TARGET:/tmp/aizee_bootstrap.tar.gz"
rm -f "$TARBALL"
ssh "${SSH_OPTS[@]}" "$TARGET" \
    "mkdir -p ~/aizee && cd ~/aizee && tar xzf /tmp/aizee_bootstrap.tar.gz && rm /tmp/aizee_bootstrap.tar.gz \
     && sed -i 's/\r$//' scripts/setup_jetson.sh && chmod +x scripts/setup_jetson.sh"
echo "   Synced to ~/aizee."
echo ""

# --- 3. run the on-device setup (interactive TTY so sudo can prompt)
echo "3. Running setup_jetson.sh on the device..."
ssh -t "${SSH_OPTS[@]}" "$TARGET" "cd ~/aizee && ./scripts/setup_jetson.sh ${SETUP_ARGS[*]+${SETUP_ARGS[@]@Q}}"
echo ""
echo "=== Bootstrap complete ==="
echo "Validation checklist:  http://${TARGET#*@}:8088/setup"
