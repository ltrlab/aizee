#!/usr/bin/env bash
# Diagnose wrist camera failures on the Jetson.
#
# Checks, in order:
#   1. Network connectivity
#   2. Udev rules installed and camera device units active
#   3. Current USB port topology vs rules (catches port-swap failures)
#   4. Service status and recent logs
#   5. ZMQ port binding (:5563 / :5564)
#   6. Python dependency: pyrealsense2
#
# Usage: ./scripts/diagnose_wrist_cameras.sh
set -uo pipefail

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
HOST="ltr@192.168.0.27"
SSH="ssh -i $SSH_KEY -o ConnectTimeout=6 -o BatchMode=yes"
PASS="$(cat "$(dirname "${BASH_SOURCE[0]}")/.jetson_password" 2>/dev/null || true)"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}OK${NC}   $*"; }
fail() { echo -e "  ${RED}FAIL${NC} $*"; }
warn() { echo -e "  ${YELLOW}WARN${NC} $*"; }
hdr()  { echo ""; echo "--- $* ---"; }

# ---------------------------------------------------------------------------
# 1. Network
# ---------------------------------------------------------------------------
hdr "1. Network"
if $SSH "$HOST" true 2>/dev/null; then
    ok "SSH to $HOST working"
else
    fail "SSH failed — check WiFi / key at $SSH_KEY"
    exit 1
fi

# ---------------------------------------------------------------------------
# Helper: run a sudo command on the Jetson
# ---------------------------------------------------------------------------
jsudo() {
    # Falls back to password auth if PASS is non-empty
    if [[ -n "$PASS" ]]; then
        $SSH "$HOST" "printf '%s\n' '${PASS}' | sudo -S $*"
    else
        $SSH "$HOST" "sudo $*"
    fi
}

# ---------------------------------------------------------------------------
# 2. Udev rules
# ---------------------------------------------------------------------------
hdr "2. Udev rules"
RULES_INSTALLED=$(jsudo test -f /etc/udev/rules.d/99-aizee-realsense.rules 2>/dev/null && echo yes || echo no)
if [[ "$RULES_INSTALLED" == "yes" ]]; then
    ok "/etc/udev/rules.d/99-aizee-realsense.rules installed"
else
    fail "Udev rules not installed — run scripts/deploy_arm_cameras.sh"
fi

# ---------------------------------------------------------------------------
# 3. USB topology vs udev rules
# ---------------------------------------------------------------------------
hdr "3. USB port topology (cameras vs rules)"

echo "  D435 cameras seen by lsusb:"
$SSH "$HOST" "lsusb -d 8086:0b07 2>/dev/null || echo '    (none found — cameras may not be plugged in)'"

echo ""
echo "  USB port paths for all D435 devices:"
# udevadm info gives DEVPATH; we extract the bus/port topology string (KERNELS)
$SSH "$HOST" "
for node in \$(find /dev/bus/usb -name '*' 2>/dev/null); do true; done
# Use udevadm to find D435 USB nodes
for syspath in \$(udevadm info --export-db 2>/dev/null | grep -B20 'idVendor=8086' | grep -B10 'idProduct=0b07' | grep 'DEVPATH=' | sed 's/.*DEVPATH=//'); do
    kernel=\$(basename \$syspath)
    serial=\$(udevadm info /sys\$syspath 2>/dev/null | grep 'ID_SERIAL_SHORT' | sed 's/.*=//' | head -1)
    echo \"    kernel=\$kernel  serial=\$serial  syspath=\$syspath\"
done
" 2>/dev/null || true

echo ""
echo "  Symlinks created by udev rules:"
$SSH "$HOST" "ls -la /dev/realsense_arm_left /dev/realsense_arm_right 2>/dev/null || echo '    (no symlinks — udev rules did not match any device)'"

echo ""
echo "  Device units active:"
for unit in dev-realsense_arm_left.device dev-realsense_arm_right.device; do
    state=$($SSH "$HOST" "systemctl is-active $unit 2>/dev/null || echo inactive")
    if [[ "$state" == "active" ]]; then
        ok "$unit  → active"
    else
        fail "$unit  → $state  (check USB port in udev rules)"
    fi
done

# ---------------------------------------------------------------------------
# 4. Service status + recent logs
# ---------------------------------------------------------------------------
hdr "4. Service status"
for svc in aizee-arm-cam-left aizee-arm-cam-right; do
    state=$($SSH "$HOST" "systemctl is-active $svc 2>/dev/null || echo inactive")
    case "$state" in
        active)  ok "$svc → running" ;;
        failed)  fail "$svc → FAILED" ;;
        inactive|activating|*)
            warn "$svc → $state" ;;
    esac
done

hdr "4b. Recent journal (last 40 lines, both services)"
$SSH "$HOST" "journalctl -u aizee-arm-cam-left -u aizee-arm-cam-right -n 40 --no-pager 2>/dev/null || true"

# ---------------------------------------------------------------------------
# 5. ZMQ ports
# ---------------------------------------------------------------------------
hdr "5. ZMQ port binding"
for port in 5563 5564; do
    bound=$($SSH "$HOST" "ss -tlnp 2>/dev/null | grep ':$port ' || echo ''")
    if [[ -n "$bound" ]]; then
        ok "Port $port is bound  ($bound)"
    else
        fail "Port $port not bound — camera node is not running or failed before bind"
    fi
done

# ---------------------------------------------------------------------------
# 6. Python dependency
# ---------------------------------------------------------------------------
hdr "6. Python dependencies on Jetson"
$SSH "$HOST" "/usr/bin/python3 -c 'import pyrealsense2 as rs; ctx=rs.context(); n=len(ctx.query_devices()); print(\"  pyrealsense2: OK (\"+str(n)+\" device(s) visible)\")' 2>&1 || echo '  FAIL: pyrealsense2 not importable'"
$SSH "$HOST" "/usr/bin/python3 -c 'import zmq; print(\"  pyzmq:\", zmq.__version__)' 2>&1 || echo '  FAIL: zmq not importable'"
$SSH "$HOST" "/usr/bin/python3 -c 'import PIL; print(\"  Pillow:\", PIL.__version__)' 2>&1 || echo '  FAIL: Pillow not importable'"

# ---------------------------------------------------------------------------
# 7. rs-enumerate-devices (quick sanity check)
# ---------------------------------------------------------------------------
hdr "7. rs-enumerate-devices (serials)"
$SSH "$HOST" "rs-enumerate-devices 2>/dev/null | grep -E 'Serial|Name|Firmware' || echo '  (rs-enumerate-devices not found or no cameras)'"

echo ""
echo "========================================"
echo " Diagnosis complete."
echo " Common fixes:"
echo "   Port mismatch:  update KERNEL== in config/udev/99-aizee-realsense.rules"
echo "                   then re-run: ./scripts/deploy_arm_cameras.sh"
echo "   Stale code:     ./scripts/deploy_arm_cameras.sh"
echo "   Restart only:   ./scripts/restart_wrist_cameras.sh"
echo "   Full logs:      ssh -i $SSH_KEY $HOST 'journalctl -u aizee-arm-cam-left -f'"
echo "========================================"
