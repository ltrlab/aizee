#!/bin/bash
# AIZEE fresh-Jetson bootstrap — run ON the Jetson (Orin Nano, JetPack 6.x).
#
# Turns a freshly flashed device into a working AIZEE brain:
#   1. apt packages        (CAN tools, i2c, python, build deps for rust/zmq)
#   2. device groups       (ltr -> i2c, dialout, video)
#   3. Rust toolchain      (rustup, if cargo is missing)
#   4. Python packages     (requirements_jetson.txt into system python3 --user)
#   5. pyrealsense2        (pip attempt; warns + points at the source build on failure)
#   6. CAN helper + sudoers (/usr/local/bin/aizee-reset-usb-can)
#   7. udev rules          (all of config/udev/)
#   8. systemd units       (all of config/systemd/, enabled)
#   9. motor_control build (cargo build --release)
#  10. network             (WiFi AP "aizee" @ 192.168.50.1, optional USB-C share @ 10.42.0.1)
#
# Idempotent: safe to re-run after a partial failure or a config change.
#
# Usage (from ~/aizee after the repo has been synced, e.g. by scripts/bootstrap_jetson.sh):
#   ./scripts/setup_jetson.sh [options]
#
# Options:
#   --ap-pass <psk>     Create/refresh the WiFi access point "aizee" (192.168.50.1).
#                       Skipped (with instructions) when omitted and no AP exists.
#   --usb-eth <iface>   Also create a shared (10.42.0.1/24) connection on this
#                       ethernet interface (USB-C adapter). Skipped when omitted.
#   --hostname <name>   Set the system hostname (e.g. aizee-jetson).
#   --with-lidar        Build lidar_control and enable aizee-lidar-control.
#   --skip-build        Skip the cargo build (config-only refresh).
#
# The heartbeat dashboard (http://<jetson>:8088) is installed and enabled by
# this script; open /setup there for the guided validation checklist.

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AIZEE_USER="${SUDO_USER:-$USER}"
AP_PASS=""
USB_ETH=""
NEW_HOSTNAME=""
WITH_LIDAR=0
SKIP_BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ap-pass)   AP_PASS="$2"; shift 2 ;;
        --usb-eth)   USB_ETH="$2"; shift 2 ;;
        --hostname)  NEW_HOSTNAME="$2"; shift 2 ;;
        --with-lidar) WITH_LIDAR=1; shift ;;
        --skip-build) SKIP_BUILD=1; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

PASS=()
WARN=()
FAIL=()
step_ok()   { echo "  [ok] $1"; PASS+=("$1"); }
step_warn() { echo "  [WARN] $1"; WARN+=("$1"); }
step_fail() { echo "  [FAIL] $1"; FAIL+=("$1"); }

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "ERROR: this script runs ON the Jetson (aarch64), not the dev machine."
    echo "Use scripts/bootstrap_jetson.sh from the dev machine instead."
    exit 1
fi

SUDO="sudo"
if [[ $EUID -eq 0 ]]; then SUDO=""; fi

echo "=== AIZEE Jetson bootstrap ==="
echo "Repo: $REPO_DIR   User: $AIZEE_USER"
echo ""

# ---------------------------------------------------------------- 0. hygiene
# The repo may have been synced from a Windows checkout: strip CRLF from
# everything that gets executed or parsed on this machine.
echo "[0/10] Normalizing line endings on scripts/configs..."
find "$REPO_DIR/scripts" -maxdepth 1 -type f \( -name '*.sh' -o -name 'aizee-reset-usb-can' \) \
    -exec sed -i 's/\r$//' {} +
find "$REPO_DIR/config/systemd" "$REPO_DIR/config/udev" -type f -exec sed -i 's/\r$//' {} + 2>/dev/null
step_ok "line endings normalized"

# ------------------------------------------------------------------- 1. apt
echo "[1/10] Installing apt packages..."
APT_PKGS=(can-utils i2c-tools v4l-utils python3-pip python3-dev python3-opencv
          python3-smbus build-essential pkg-config libzmq3-dev curl git)
if $SUDO apt-get update -qq && $SUDO apt-get install -y -qq "${APT_PKGS[@]}"; then
    step_ok "apt packages (${APT_PKGS[*]})"
else
    step_fail "apt install — check network and re-run"
fi

# ----------------------------------------------------------- 2. device groups
echo "[2/10] Adding $AIZEE_USER to hardware groups (i2c, dialout, video)..."
if $SUDO usermod -aG i2c,dialout,video "$AIZEE_USER"; then
    step_ok "groups (takes effect on next login — services are unaffected)"
else
    step_warn "usermod failed; UPS/serial/camera nodes may hit permission errors"
fi

# ------------------------------------------------------------------- 3. rust
echo "[3/10] Rust toolchain..."
if [[ -f "$HOME/.cargo/env" ]]; then source "$HOME/.cargo/env"; fi
if command -v cargo >/dev/null 2>&1; then
    step_ok "cargo present ($(cargo --version))"
else
    echo "  installing rustup (minimal profile)..."
    if curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal \
        && source "$HOME/.cargo/env"; then
        step_ok "rustup installed ($(cargo --version))"
    else
        step_fail "rustup install — motor_control cannot be built"
    fi
fi

# ----------------------------------------------------------------- 4. python
echo "[4/10] Python packages (system python3, --user)..."
if python3 -m pip install --user -q -r "$REPO_DIR/requirements_jetson.txt"; then
    step_ok "requirements_jetson.txt"
else
    step_fail "pip install -r requirements_jetson.txt"
fi

# ----------------------------------------------------------- 5. pyrealsense2
echo "[5/10] pyrealsense2 (scene cam)..."
if python3 -c "import pyrealsense2" 2>/dev/null; then
    step_ok "pyrealsense2 already importable"
elif python3 -m pip install --user -q pyrealsense2 && python3 -c "import pyrealsense2" 2>/dev/null; then
    step_ok "pyrealsense2 via pip"
else
    step_warn "pyrealsense2 unavailable via pip — scene cam disabled until you run scripts/build_librealsense_rsusb.sh (gripper cam is unaffected)"
fi

# ------------------------------------------------- 6. CAN helper + sudoers
echo "[6/10] CAN reset helper + sudoers..."
if $SUDO install -m 755 "$REPO_DIR/scripts/aizee-reset-usb-can" /usr/local/bin/aizee-reset-usb-can; then
    step_ok "/usr/local/bin/aizee-reset-usb-can"
else
    step_fail "install aizee-reset-usb-can"
fi
$SUDO tee /etc/sudoers.d/aizee-can >/dev/null << SUDOERS
# Allow $AIZEE_USER to run USB-CAN reset (systemd ExecStartPre + runtime recovery)
$AIZEE_USER ALL=(ALL) NOPASSWD: /usr/local/bin/aizee-reset-usb-can
# Allow $AIZEE_USER to manage CAN interfaces (runtime ip link down/up recovery)
$AIZEE_USER ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can0 *
$AIZEE_USER ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can1 *
SUDOERS
$SUDO chmod 440 /etc/sudoers.d/aizee-can && step_ok "/etc/sudoers.d/aizee-can"

# ------------------------------------------------------------- 7. udev rules
echo "[7/10] udev rules..."
for rules in "$REPO_DIR"/config/udev/99-*.rules; do
    name="$(basename "$rules")"
    if [[ "$name" == "99-rplidar.rules" && $WITH_LIDAR -eq 0 ]]; then
        continue
    fi
    $SUDO cp "$rules" /etc/udev/rules.d/ && step_ok "udev: $name"
done
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger --subsystem-match=usb --subsystem-match=video4linux --subsystem-match=tty

# ---------------------------------------------------------- 8. systemd units
echo "[8/10] systemd units..."
# Boot services start unconditionally; device-bound units are started by udev
# (SYSTEMD_ALIAS + WantedBy=dev-*.device) when their hardware appears.
BOOT_UNITS=(aizee-motor-control-rover aizee-heartbeat aizee-ups-monitor aizee-estop-bridge)
DEVICE_UNITS=(aizee-gripper-cam aizee-scene-cam aizee-display)
[[ $WITH_LIDAR -eq 1 ]] && BOOT_UNITS+=(aizee-lidar-control)

for unit in "$REPO_DIR"/config/systemd/aizee-*.service; do
    $SUDO cp "$unit" /etc/systemd/system/
done
$SUDO systemctl daemon-reload
for svc in "${BOOT_UNITS[@]}" "${DEVICE_UNITS[@]}"; do
    if $SUDO systemctl enable "$svc.service" >/dev/null 2>&1; then
        step_ok "enabled $svc"
    else
        step_warn "could not enable $svc"
    fi
done
if [[ $WITH_LIDAR -eq 0 ]]; then
    $SUDO systemctl disable aizee-lidar-control.service >/dev/null 2>&1
fi

# --------------------------------------------------------------- 9. build
if [[ $SKIP_BUILD -eq 0 ]] && command -v cargo >/dev/null 2>&1; then
    echo "[9/10] Building motor_control (first build takes several minutes)..."
    if (cd "$REPO_DIR/rust/motor_control" && cargo build --release); then
        step_ok "motor_control built"
    else
        step_fail "cargo build motor_control"
    fi
    if [[ $WITH_LIDAR -eq 1 ]]; then
        if (cd "$REPO_DIR/rust" && cargo build --release -p lidar_control); then
            step_ok "lidar_control built"
        else
            step_fail "cargo build lidar_control"
        fi
    fi
else
    echo "[9/10] Skipping cargo build."
fi

# --------------------------------------------------------------- 10. network
echo "[10/10] Network (NetworkManager)..."
if [[ -n "$NEW_HOSTNAME" ]]; then
    $SUDO hostnamectl set-hostname "$NEW_HOSTNAME" && step_ok "hostname -> $NEW_HOSTNAME"
fi
if nmcli -t -f NAME con show 2>/dev/null | grep -qx "aizee-ap"; then
    step_ok "WiFi AP connection 'aizee-ap' already exists"
elif [[ -n "$AP_PASS" ]]; then
    if $SUDO nmcli con add type wifi ifname wlan0 con-name aizee-ap autoconnect yes ssid aizee \
            802-11-wireless.mode ap 802-11-wireless.band bg \
            ipv4.method shared ipv4.addresses 192.168.50.1/24 \
            wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$AP_PASS" \
        && $SUDO nmcli con up aizee-ap; then
        step_ok "WiFi AP 'aizee' up at 192.168.50.1"
    else
        step_warn "WiFi AP creation failed — check 'nmcli device' for wlan0"
    fi
else
    step_warn "WiFi AP not configured (re-run with --ap-pass <psk> to create SSID 'aizee')"
fi
if [[ -n "$USB_ETH" ]]; then
    if nmcli -t -f NAME con show 2>/dev/null | grep -qx "aizee-usb"; then
        step_ok "USB-C shared connection 'aizee-usb' already exists"
    elif $SUDO nmcli con add type ethernet ifname "$USB_ETH" con-name aizee-usb autoconnect yes \
            ipv4.method shared ipv4.addresses 10.42.0.1/24 \
        && $SUDO nmcli con up aizee-usb; then
        step_ok "USB-C share up at 10.42.0.1 ($USB_ETH)"
    else
        step_warn "USB-C shared connection failed on $USB_ETH"
    fi
fi

# ------------------------------------------------------------------ summary
echo ""
echo "=== Bootstrap summary ==="
echo "  ok: ${#PASS[@]}   warn: ${#WARN[@]}   fail: ${#FAIL[@]}"
for w in "${WARN[@]:-}"; do [[ -n "$w" ]] && echo "  WARN: $w"; done
for f in "${FAIL[@]:-}"; do [[ -n "$f" ]] && echo "  FAIL: $f"; done
echo ""
echo "Start everything now (or just reboot):"
echo "  sudo systemctl start ${BOOT_UNITS[*]}"
echo ""
echo "Then open the guided validation checklist:"
echo "  http://192.168.50.1:8088/setup   (WiFi AP)"
echo "  http://10.42.0.1:8088/setup     (USB-C)"
[[ ${#FAIL[@]} -gt 0 ]] && exit 1
exit 0
