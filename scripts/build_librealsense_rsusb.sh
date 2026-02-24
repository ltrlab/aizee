#!/bin/bash
# Build librealsense2 with RSUSB backend on Raspberry Pi 4 aarch64
#
# Fixes "bad optional access" error caused by missing hid_sensor_* kernel
# modules on the RPi custom kernel. FORCE_RSUSB_BACKEND uses libusb directly
# and has no kernel HID/IIO dependency.
#
# Takes ~40 min on Pi 4. Run as the ltr user (has passwordless sudo).
# Output logged to ~/librealsense_build.log
#
# Usage (from dev machine, routed through Jetson):
#   ssh -i KEY ltr@JETSON "ssh -i ~/.ssh/aizee_rover_id ltr@PI_IP 'nohup bash ~/build_librealsense_rsusb.sh > ~/librealsense_build.log 2>&1 &'"

set -eo pipefail
LOG="$HOME/librealsense_build.log"
VERSION="v2.56.2"   # Match the version currently installed

echo "[$(date)] Starting librealsense2 RSUSB backend build ($VERSION)" | tee -a $LOG

# 1. Dependencies
echo "[$(date)] Installing build deps..." | tee -a $LOG
sudo apt-get update -qq 2>&1 | tee -a $LOG
sudo apt-get install -y \
    libusb-1.0-0-dev \
    libssl-dev \
    cmake \
    git \
    build-essential \
    pkg-config \
    python3-dev \
    libatomic1 \
    2>&1 | tee -a $LOG
echo "[$(date)] Build deps installed." | tee -a $LOG

# 2. Clone source (shallow, specific tag)
# If directory exists but lacks CMakeLists.txt (incomplete/failed clone), remove and re-clone
if [ -d "$HOME/librealsense" ] && [ ! -f "$HOME/librealsense/CMakeLists.txt" ]; then
    echo "[$(date)] Stale/incomplete source tree found — removing and re-cloning..." | tee -a $LOG
    rm -rf "$HOME/librealsense"
fi

if [ ! -d "$HOME/librealsense" ]; then
    echo "[$(date)] Cloning librealsense $VERSION..." | tee -a $LOG
    git clone --depth 1 --branch $VERSION \
        https://github.com/IntelRealSense/librealsense.git \
        "$HOME/librealsense" 2>&1 | tee -a $LOG | tail -3
    echo "[$(date)] Clone complete." | tee -a $LOG
else
    echo "[$(date)] Using existing source tree at ~/librealsense" | tee -a $LOG
fi

# 3. Configure
echo "[$(date)] Configuring (FORCE_RSUSB_BACKEND=ON)..." | tee -a $LOG
mkdir -p "$HOME/librealsense/build"
cd "$HOME/librealsense/build"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DOTHER_LIBS="-latomic" \
    2>&1 | tee -a $LOG
echo "[$(date)] CMake exit: $?" | tee -a $LOG

echo "[$(date)] Backend configured — RSUSB=$(grep -o 'FORCE_RSUSB_BACKEND:.*' CMakeCache.txt)" | tee -a $LOG

# 4. Build (~40 min on Pi 4)
echo "[$(date)] Building with $(nproc) cores — this takes ~40 min..." | tee -a $LOG
make -j$(nproc) 2>&1 | tee -a $LOG
echo "[$(date)] Make exit: $?" | tee -a $LOG

# 5. Install
echo "[$(date)] Installing..." | tee -a $LOG
sudo make install 2>&1 | tee -a $LOG
sudo ldconfig

# 6. Install Python bindings to site-packages
echo "[$(date)] Installing Python bindings..." | tee -a $LOG
PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
SITE="/usr/local/lib/python${PYVER}/dist-packages"
sudo mkdir -p "$SITE/pyrealsense2"
sudo cp "$HOME/librealsense/build/wrappers/python/pyrealsense2"*.so "$SITE/pyrealsense2/" 2>/dev/null || \
sudo cp "$HOME/librealsense/build/Release/pyrealsense2"*.so "$SITE/" 2>/dev/null || true
# pyrealsense2 __init__.py
if [ ! -f "$SITE/pyrealsense2/__init__.py" ]; then
    echo "from .pyrealsense2 import *" | sudo tee "$SITE/pyrealsense2/__init__.py"
fi

# 7. Verify
echo "[$(date)] Testing pyrealsense2..." | tee -a $LOG
python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
devs = ctx.query_devices()
print(f'Found {len(devs)} device(s)')
if len(devs) > 0:
    d = devs[0]
    try:
        fw = d.get_info(rs.camera_info.firmware_version)
        sn = d.get_info(rs.camera_info.serial_number)
        print(f'  Serial: {sn}  Firmware: {fw}')
    except Exception as e:
        print(f'  info error: {e}')
" 2>&1 | tee -a $LOG

echo "[$(date)] Build complete! Check $LOG for details." | tee -a $LOG
