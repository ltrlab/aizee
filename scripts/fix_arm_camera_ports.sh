#!/bin/bash
# Auto-detect D435 arm cameras on the Jetson and update udev rules to match
# their current USB ports.  Fixes "stale camera" issues caused by cameras
# being replugged into different USB ports.
#
# What this does:
#   1. SSH to Jetson, enumerate D435 cameras via pyrealsense2 (serial + USB port)
#   2. Map serials to left/right identity
#   3. Update config/udev/99-aizee-realsense.rules with correct KERNEL== values
#   4. Deploy updated rules to Jetson, reload udev, restart services
#
# Usage:
#   ./scripts/fix_arm_camera_ports.sh              # auto-fix and deploy
#   ./scripts/fix_arm_camera_ports.sh --dry-run    # detect only, don't deploy
#   ./scripts/fix_arm_camera_ports.sh ltr@1.2.3.4  # custom target

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
DRY_RUN=false
TARGET="ltr@192.168.0.27"
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        *)         TARGET="$arg" ;;
    esac
done

SSH_KEY="/p/Workspace/ssh-keys/aizee_rover_id"
SSH="ssh -i $SSH_KEY"

PASS_FILE="$SCRIPT_DIR/.jetson_password"
if [[ ! -f "$PASS_FILE" ]]; then
    echo "ERROR: $PASS_FILE not found. Create it with the Jetson sudo password."
    exit 1
fi
JETSON_PASS="$(cat "$PASS_FILE")"

# Known camera serials — source of truth for left/right identity.
# Update these if a camera is physically replaced.
LEFT_SERIAL="941322071864"
RIGHT_SERIAL="818312070515"

UDEV_RULES="$REPO_ROOT/config/udev/99-aizee-realsense.rules"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

echo "=== AIZEE Arm Camera Port Auto-Fix ==="
echo "Target: $TARGET"
if $DRY_RUN; then echo -e "${YELLOW}DRY RUN — will detect but not deploy${NC}"; fi
echo ""

# ---------------------------------------------------------------------------
# 1. Connectivity check
# ---------------------------------------------------------------------------
echo "1. Checking connectivity..."
if ! $SSH -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo -e "${RED}ERROR: Cannot reach $TARGET via SSH${NC}"
    exit 1
fi
echo -e "   ${GREEN}Connected.${NC}"
echo ""

# ---------------------------------------------------------------------------
# 2. Discover cameras via pyrealsense2
# ---------------------------------------------------------------------------
echo "2. Discovering D435 cameras on Jetson..."

# Run a small Python script on the Jetson that outputs: serial|physical_port|name
# one camera per line.
CAMERA_INFO=$($SSH "$TARGET" 'python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
for d in ctx.query_devices():
    sn   = d.get_info(rs.camera_info.serial_number)
    port = d.get_info(rs.camera_info.physical_port)
    name = d.get_info(rs.camera_info.name)
    print(f\"{sn}|{port}|{name}\")
" 2>/dev/null' || true)

if [[ -z "$CAMERA_INFO" ]]; then
    echo -e "${RED}ERROR: No RealSense cameras detected on Jetson.${NC}"
    echo "  - Are both cameras physically plugged in?"
    echo "  - Is pyrealsense2 installed?  (python3 -c 'import pyrealsense2')"
    exit 1
fi

echo "   Found cameras:"
echo "$CAMERA_INFO" | while IFS='|' read -r serial port name; do
    echo "     $name  serial=$serial  port=$port"
done
echo ""

# ---------------------------------------------------------------------------
# 3. Extract USB KERNEL port paths and map to left/right
# ---------------------------------------------------------------------------
echo "3. Mapping serials to USB ports..."

LEFT_PORT=""
RIGHT_PORT=""
HAS_UNKNOWN=false

while IFS='|' read -r serial port name; do
    # physical_port is a sysfs path like:
    #   /sys/devices/platform/.../usb2/2-1/2-1.4/2-1.4.1:1.0
    # The udev KERNEL== value is the bus-port topology component (e.g. 2-1.4.1).
    # Extract: last segment matching N-N[.N]* before any :interface suffix.
    kernel=$(echo "$port" | grep -oE '[0-9]+-[0-9]+(\.[0-9]+)+' | tail -1)

    if [[ -z "$kernel" ]]; then
        echo -e "   ${RED}WARNING: Could not parse USB port from: $port${NC}"
        continue
    fi

    if [[ "$serial" == "$LEFT_SERIAL" ]]; then
        LEFT_PORT="$kernel"
        echo -e "   ${GREEN}Left  camera${NC}  (S/N $serial) → USB port ${GREEN}$kernel${NC}"
    elif [[ "$serial" == "$RIGHT_SERIAL" ]]; then
        RIGHT_PORT="$kernel"
        echo -e "   ${GREEN}Right camera${NC}  (S/N $serial) → USB port ${GREEN}$kernel${NC}"
    else
        HAS_UNKNOWN=true
        echo -e "   ${YELLOW}UNKNOWN camera${NC}  (S/N $serial) → USB port $kernel  ($name)"
    fi
done <<< "$CAMERA_INFO"

echo ""

if $HAS_UNKNOWN; then
    echo -e "${YELLOW}WARNING: Unrecognized camera serial(s) detected.${NC}"
    echo "  If this is a replacement camera, update LEFT_SERIAL / RIGHT_SERIAL"
    echo "  at the top of this script and in the YAML configs."
    echo ""
fi

if [[ -z "$LEFT_PORT" ]]; then
    echo -e "${RED}ERROR: Left camera (S/N $LEFT_SERIAL) not found.${NC}"
    echo "  Is it plugged in?"
fi
if [[ -z "$RIGHT_PORT" ]]; then
    echo -e "${RED}ERROR: Right camera (S/N $RIGHT_SERIAL) not found.${NC}"
    echo "  Is it plugged in?"
fi
if [[ -z "$LEFT_PORT" || -z "$RIGHT_PORT" ]]; then
    exit 1
fi

# ---------------------------------------------------------------------------
# 4. Read current udev rules and check if update is needed
# ---------------------------------------------------------------------------
echo "4. Checking current udev rules..."

# Extract current KERNEL values from the rules file (avoid grep -P for Git Bash compat)
CUR_LEFT_PORT=$(grep -A2 'realsense_arm_left' "$UDEV_RULES" | sed -n 's/.*KERNEL=="\([^"]*\)".*/\1/p' | head -1)
CUR_RIGHT_PORT=$(grep -A2 'realsense_arm_right' "$UDEV_RULES" | sed -n 's/.*KERNEL=="\([^"]*\)".*/\1/p' | head -1)

echo "   Current:   left=$CUR_LEFT_PORT  right=$CUR_RIGHT_PORT"
echo "   Detected:  left=$LEFT_PORT  right=$RIGHT_PORT"

if [[ "$CUR_LEFT_PORT" == "$LEFT_PORT" && "$CUR_RIGHT_PORT" == "$RIGHT_PORT" ]]; then
    echo ""
    echo -e "${GREEN}Udev rules already match current USB ports — no update needed.${NC}"
    echo ""
    echo "If the right camera is still stale, the issue may be elsewhere. Run:"
    echo "  ./scripts/diagnose_wrist_cameras.sh"
    exit 0
fi

echo ""
CHANGES=""
if [[ "$CUR_LEFT_PORT" != "$LEFT_PORT" ]]; then
    echo -e "   ${YELLOW}Left port changed:  $CUR_LEFT_PORT → $LEFT_PORT${NC}"
    CHANGES="yes"
fi
if [[ "$CUR_RIGHT_PORT" != "$RIGHT_PORT" ]]; then
    echo -e "   ${YELLOW}Right port changed: $CUR_RIGHT_PORT → $RIGHT_PORT${NC}"
    CHANGES="yes"
fi
echo ""

# ---------------------------------------------------------------------------
# 5. Update the local udev rules file
# ---------------------------------------------------------------------------
echo "5. Updating udev rules file..."

cat > "$UDEV_RULES" << EOF
# Udev rules for AIZEE arm-mounted Intel RealSense D435 cameras
#
# Creates stable /dev symlinks and systemd device units so that
# aizee-arm-cam-left/right.service start on plug-in and stop on unplug.
#
# Cameras are identified by USB port location (the D435 does not expose
# its firmware serial number via the USB descriptor).
#
# SETUP: Run scripts/deploy_arm_cameras.sh from the dev machine, or manually:
#   sudo cp config/udev/99-aizee-realsense.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#
# If cameras are moved to different USB ports, run:
#   ./scripts/fix_arm_camera_ports.sh
#
# Camera port assignments (Jetson Orin Nano):
#   arm_cam_left  (S/N $LEFT_SERIAL): USB port $LEFT_PORT
#   arm_cam_right (S/N $RIGHT_SERIAL): USB port $RIGHT_PORT
# Auto-detected by fix_arm_camera_ports.sh on $(date +%Y-%m-%d)

SUBSYSTEM=="usb", \\
    ATTR{idVendor}=="8086", ATTR{idProduct}=="0b07", \\
    KERNEL=="$LEFT_PORT", \\
    SYMLINK+="realsense_arm_left", \\
    TAG+="systemd", ENV{SYSTEMD_ALIAS}+="/dev/realsense_arm_left"

SUBSYSTEM=="usb", \\
    ATTR{idVendor}=="8086", ATTR{idProduct}=="0b07", \\
    KERNEL=="$RIGHT_PORT", \\
    SYMLINK+="realsense_arm_right", \\
    TAG+="systemd", ENV{SYSTEMD_ALIAS}+="/dev/realsense_arm_right"
EOF

echo "   Updated: $UDEV_RULES"
echo ""

if $DRY_RUN; then
    echo "=== DRY RUN complete ==="
    echo "Updated local file: $UDEV_RULES"
    echo "Run without --dry-run to deploy to Jetson."
    exit 0
fi

# ---------------------------------------------------------------------------
# 6. Deploy updated udev rules to Jetson and reload
# ---------------------------------------------------------------------------
echo "6. Deploying to Jetson..."

# Copy the updated rules file
scp -i "$SSH_KEY" "$UDEV_RULES" "$TARGET:/tmp/99-aizee-realsense.rules"

$SSH "$TARGET" "echo '$JETSON_PASS' | sudo -S bash -c '
set -e

# Stop current camera services (harmless if not running)
systemctl stop aizee-arm-cam-left aizee-arm-cam-right 2>/dev/null || true

# Install updated udev rules
cp /tmp/99-aizee-realsense.rules /etc/udev/rules.d/
cp /tmp/99-aizee-realsense.rules /home/ltr/aizee/config/udev/
rm -f /tmp/99-aizee-realsense.rules
echo \"  Udev rules installed.\"

# Reload udev and trigger to create symlinks for already-connected cameras
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add
sleep 2

# Verify symlinks were created
echo \"\"
echo \"  Device symlinks:\"
ls -la /dev/realsense_arm_left /dev/realsense_arm_right 2>/dev/null || echo \"    (no symlinks — udev rules did not match)\"

echo \"\"
echo \"  Device units:\"
systemctl is-active dev-realsense_arm_left.device 2>/dev/null  || echo \"  left:  inactive\"
systemctl is-active dev-realsense_arm_right.device 2>/dev/null || echo \"  right: inactive\"

# Restart services (they are WantedBy the device units, so udev trigger
# should have started them — but restart to be safe)
systemctl restart aizee-arm-cam-left  2>/dev/null || true
systemctl restart aizee-arm-cam-right 2>/dev/null || true
sleep 3

echo \"\"
echo \"  Service status:\"
systemctl status aizee-arm-cam-left  --no-pager -l 2>&1 | head -5
echo \"\"
systemctl status aizee-arm-cam-right --no-pager -l 2>&1 | head -5
' 2>&1"

echo ""

# ---------------------------------------------------------------------------
# 7. Quick ZMQ port check
# ---------------------------------------------------------------------------
echo "7. Verifying ZMQ ports..."
for port in 5563 5564; do
    label="left"; [[ "$port" == "5564" ]] && label="right"
    bound=$($SSH "$TARGET" "ss -tlnp 2>/dev/null | grep ':$port '" 2>/dev/null || true)
    if [[ -n "$bound" ]]; then
        echo -e "   ${GREEN}Port $port ($label): bound${NC}"
    else
        echo -e "   ${YELLOW}Port $port ($label): not yet bound (may need a few seconds)${NC}"
    fi
done

echo ""
echo -e "${GREEN}=== Done! ===${NC}"
echo ""
echo "Tail logs:  $SSH $TARGET 'journalctl -u aizee-arm-cam-left -u aizee-arm-cam-right -f'"
echo "Full diag:  ./scripts/diagnose_wrist_cameras.sh"
