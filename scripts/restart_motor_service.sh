#!/usr/bin/env bash
# Restart the motor control service on the Jetson and tail status.
set -euo pipefail

HOST="ltr@192.168.0.27"
KEY="/p/Workspace/ssh-keys/aizee_rover_id"
SERVICE="aizee-motor-control-rover"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password")"

echo "==> Restarting $SERVICE on $HOST ..."
ssh -tt -i "$KEY" "$HOST" "printf '%s\n' '${PASS}' | sudo -S systemctl restart ${SERVICE} && sudo systemctl status ${SERVICE} --no-pager -l"
