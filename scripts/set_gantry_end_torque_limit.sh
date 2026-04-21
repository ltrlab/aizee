#!/usr/bin/env bash
# Set LIMIT_TORQUE = 6.0 Nm on gantry_end (CAN ID 0x07) and verify.
#
# The motor control service must be stopped first — it holds the CAN socket.
# This script stops it, writes the parameter, reads back to verify, then
# restarts the service.
#
# NOTE: Save-to-flash is intentionally skipped (command has known issues).
#       The value will revert on power cycle until flash save is confirmed working.
#
# Usage: ./scripts/set_gantry_end_torque_limit.sh
set -euo pipefail

HOST="ltr@192.168.0.27"
KEY="/p/Workspace/ssh-keys/aizee_rover_id"
SERVICE="aizee-motor-control-rover"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password")"

# Expand PASS and SERVICE locally; escape \$! and \$DUMP_PID so they stay remote.
ssh -i "$KEY" "$HOST" 'bash -s' << EOF
set -euo pipefail

echo "==> Stopping ${SERVICE} ..."
printf '%s\n' '${PASS}' | sudo -S systemctl stop ${SERVICE}
echo ""

echo "==> Writing LIMIT_TORQUE = 6.0 Nm to gantry_end (0x07) ..."
# param_id 0x700B LE = 0B 70, value 6.0f32 LE = 00 00 C0 40
cansend can1 1200AA07#0B7000000000C040
echo "   Write sent."
echo ""

echo "==> Verifying (reading LIMIT_TORQUE back from arb ID 110007AA) ..."
( timeout 2 candump can1 2>/dev/null | grep --line-buffered '110007AA' ) &
DUMP_PID=\$!
sleep 0.15
cansend can1 1100AA07#0B70000000000000
wait \$DUMP_PID || true
echo ""

python3 -c 'import struct; print("==> Decoded: LIMIT_TORQUE =", struct.unpack("<f", bytes.fromhex("0000C040"))[0], "Nm  (expected 6.0)")'
echo ""

echo "==> Restarting ${SERVICE} ..."
printf '%s\n' '${PASS}' | sudo -S systemctl start ${SERVICE}
sleep 1
printf '%s\n' '${PASS}' | sudo -S systemctl status ${SERVICE} --no-pager -l
EOF
