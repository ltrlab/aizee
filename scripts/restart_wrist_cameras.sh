#!/usr/bin/env bash
# Restart stale wrist camera services on the Jetson.
#
# With no arguments: restarts both arm camera services.
# With "left" or "right": restarts only that camera.
# With "--check": prints service status and exits (no restart).
#
# Usage:
#   ./restart_wrist_cameras.sh              # restart both
#   ./restart_wrist_cameras.sh left         # restart left only
#   ./restart_wrist_cameras.sh right        # restart right only
#   ./restart_wrist_cameras.sh --check      # status only, no restart
set -euo pipefail

HOST="ltr@192.168.0.27"
KEY="/p/Workspace/ssh-keys/aizee_rover_id"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password")"
SVC_LEFT="aizee-arm-cam-left"
SVC_RIGHT="aizee-arm-cam-right"

ssh_sudo() {
    ssh -tt -i "$KEY" "$HOST" "printf '%s\n' '${PASS}' | sudo -S $*"
}

print_status() {
    local svc="$1"
    echo ""
    echo "--- $svc ---"
    ssh_sudo systemctl status "$svc" --no-pager -l || true
}

restart_svc() {
    local svc="$1"
    echo "==> Restarting $svc on $HOST ..."
    ssh_sudo systemctl restart "$svc"
    print_status "$svc"
}

TARGET="${1:-both}"

case "$TARGET" in
    --check)
        print_status "$SVC_LEFT"
        print_status "$SVC_RIGHT"
        ;;
    left)
        restart_svc "$SVC_LEFT"
        ;;
    right)
        restart_svc "$SVC_RIGHT"
        ;;
    both|"")
        restart_svc "$SVC_LEFT"
        restart_svc "$SVC_RIGHT"
        ;;
    *)
        echo "Usage: $0 [left|right|both|--check]" >&2
        exit 1
        ;;
esac

echo ""
echo "Done. To tail logs:"
echo "  ssh -i $KEY $HOST 'journalctl -u $SVC_LEFT -f'"
echo "  ssh -i $KEY $HOST 'journalctl -u $SVC_RIGHT -f'"
