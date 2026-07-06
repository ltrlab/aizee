#!/usr/bin/env bash
# Deploy tufty2040/main.py to the Pimoroni Tufty2040 via the Jetson.
#
# Steps:
#   1. Copy main.py to /tmp on the Jetson
#   2. Stop aizee-display (releases the serial port)
#   3. Flash via mpremote and soft-reset the board
#   4. Restart aizee-display
#
# Requirements: mpremote must be installed on the Jetson
#   pip install mpremote       (as ltr, or system-wide)
#
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/deploy_common.sh"
HOST="${1:-$AIZEE_TARGET}"
KEY="$SSH_KEY"
SERVICE="aizee-display"
DEVICE="/dev/tufty_display"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password")"
LOCAL_SCRIPT="$(dirname "${BASH_SOURCE[0]}")/../firmware/tufty2040/main.py"

echo "=== Tufty2040 Deploy ==="

echo "1. Copying tufty2040/main.py to Jetson /tmp ..."
scp -i "$KEY" "$LOCAL_SCRIPT" "${HOST}:/tmp/tufty_main.py"

echo "2. Stopping $SERVICE ..."
ssh -tt -i "$KEY" "$HOST" "printf '%s\n' '${PASS}' | sudo -S systemctl stop ${SERVICE}"

echo "3. Flashing main.py via mpremote ..."
ssh -i "$KEY" "$HOST" "PATH=\$PATH:\$HOME/.local/bin mpremote connect ${DEVICE} cp /tmp/tufty_main.py :main.py + reset"

echo "   Waiting for board to re-enumerate ..."
sleep 3

echo "4. Restarting $SERVICE ..."
ssh -tt -i "$KEY" "$HOST" "printf '%s\n' '${PASS}' | sudo -S systemctl restart ${SERVICE} && sudo systemctl status ${SERVICE} --no-pager -l"

echo "=== Deploy complete! ==="
