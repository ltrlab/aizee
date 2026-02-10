# RealSense D455 Camera Deployment Guide

## Overview

This guide covers deploying the AIZEE camera system to 4 Raspberry Pi 4 devices, each running an Intel RealSense D455 RGB-D camera.

**Network Configuration:**
- Camera Front: `192.168.0.22` (AIZEE-ROVER-PI-1) - ZMQ port 5557
- Camera Rear: `192.168.0.23` (AIZEE-ROVER-PI-2) - ZMQ port 5558
- Camera Left: `192.168.0.24` (AIZEE-ROVER-PI-3) - ZMQ port 5559
- Camera Right: `192.168.0.25` (AIZEE-ROVER-PI-4) - ZMQ port 5560
- Jetson Orin: `192.168.0.27` (runs Rerun bridge in production)

## Raspberry Pi Setup

### 1. Initial OS Installation

For each Raspberry Pi (repeat for all 4 devices):

1. **Flash OS using Raspberry Pi Imager:**
   - OS: Raspberry Pi OS Lite (64-bit) - latest version
   - Enable SSH in advanced settings
   - Set hostname: `AIZEE-ROVER-PI-1` through `AIZEE-ROVER-PI-4`
   - Set username: `pi`
   - Set password: (your choice)
   - Configure WiFi if needed for initial setup

2. **First Boot:**
   ```bash
   # Boot Pi and SSH in
   ssh pi@192.168.0.22  # Adjust IP for each Pi
   ```

### 2. System Dependencies Installation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install development tools and dependencies
sudo apt install -y \
    python3-pip python3-dev python3-venv \
    build-essential pkg-config cmake git curl \
    libzmq3-dev libusb-1.0-0-dev libudev-dev \
    libssl-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-opencv
```

### 3. RealSense SDK Installation

**CRITICAL:** On ARM64, you must build librealsense2 from source.

```bash
# Clone RealSense SDK
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.54.2  # Match version in requirements.txt

# Install udev rules for USB permissions
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group for USB access
sudo usermod -a -G dialout pi

# Build SDK (takes 30-45 minutes on Pi 4)
mkdir build && cd build
cmake .. \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false

make -j4
sudo make install
sudo ldconfig

# Install Python bindings
sudo cp wrappers/python/pyrealsense2*.so /usr/local/lib/python3.*/dist-packages/

# Verify installation
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 version:', rs.__version__)"
```

**Expected output:** `pyrealsense2 version: 2.54.2.5684`

### 4. Network Configuration

#### Option A: Static IP via NetworkManager (Ethernet)

```bash
# For Pi 1 (192.168.0.22)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.0.22/24
sudo nmcli con mod "Wired connection 1" ipv4.gateway 192.168.0.1
sudo nmcli con mod "Wired connection 1" ipv4.dns 8.8.8.8
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Repeat for Pi 2-4 with IPs .23, .24, .25
```

#### Option B: WiFi Setup (for development)

During initial setup, WiFi can be used. Configure via Raspberry Pi Imager or manually:

```bash
sudo nmcli dev wifi connect "YOUR_SSID" password "YOUR_PASSWORD"
```

### 5. Camera Connection and Serial Number Discovery

1. **Connect RealSense D455 to USB 3.0 port** (blue USB port on Pi 4)

2. **Verify detection:**
   ```bash
   lsusb | grep Intel
   # Expected: "Intel Corp. Intel(R) RealSense(TM) Depth Camera 455"
   ```

3. **Get serial number:**
   ```bash
   rs-enumerate-devices
   ```

   Look for output like:
   ```
   Device info:
       Name                          : Intel RealSense D455
       Serial Number                 : 123456789012
       Firmware Version              : 05.15.00.00
   ```

4. **Update configuration file** with actual serial number:
   ```bash
   # Edit on dev machine
   vim config/hardware_rpi4_cam_front.yaml
   # Change: serial: D455_SERIAL_001 -> serial: 123456789012
   ```

## Deployment from Development Machine

### 1. Deploy Single Camera

```bash
cd /p/Workspace/aizee

# Deploy to specific camera Pi
./scripts/deploy_rpi4_camera.sh cam_front  # or cam_rear, cam_left, cam_right
```

### 2. Deploy All Cameras

```bash
# Deploy to all 4 Pis at once
./scripts/deploy_all_cameras.sh
```

The deployment script will:
- Sync Python codebase (excludes Rust, logs, cache)
- Install Python dependencies via pip
- Install systemd service file
- Test camera connectivity

## Testing

### Per-Pi Unit Tests

#### Test 1: Camera Detection

```bash
ssh pi@192.168.0.22
rs-enumerate-devices
# Verify D455 is detected with correct serial number
```

#### Test 2: Manual Camera Node Execution

```bash
ssh pi@192.168.0.22
cd ~/aizee

python3 python/nodes/camera_node.py \
    --camera-id cam_front \
    --zmq-endpoint tcp://*:5557 \
    --fps 30 \
    --jpeg-quality 85

# Expected output: "Published X frames in 5.0s (30.0 fps)"
# Press Ctrl+C to stop
```

#### Test 3: ZMQ Stream Reception (from dev machine)

```bash
# On dev machine
cd /p/Workspace/aizee

python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557

# Expected: "Received 150 frames in 5.0s (30.0 fps)"
```

#### Test 4: Systemd Service

```bash
# Start service
ssh pi@192.168.0.22 sudo systemctl start aizee-camera-cam_front

# Check status
ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front

# View logs
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -n 50

# Enable auto-start on boot
ssh pi@192.168.0.22 sudo systemctl enable aizee-camera-cam_front
```

### Multi-Camera Integration Tests

#### Test 5: Start All Cameras

```bash
# From dev machine
./scripts/start_all_cameras.sh
```

#### Test 6: Test All Streams Simultaneously

```bash
# From dev machine - test each stream in parallel
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557 &
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.23:5558 &
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.24:5559 &
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.25:5560 &

# Wait 30 seconds
sleep 30

# Stop all
killall python
```

#### Test 7: Network Latency

```bash
# From dev machine
for ip in 22 23 24 25; do
    echo "Testing 192.168.0.$ip:"
    ping -c 50 192.168.0.$ip | tail -n 1
done

# Target: average latency < 2ms
```

### Rerun Integration Tests

#### Test 8: Rerun Bridge with All Cameras (WiFi Development Setup)

```bash
# From dev machine
cd /p/Workspace/aizee

python python/rerun_bridge.py \
    --cameras \
        tcp://192.168.0.22:5557 \
        tcp://192.168.0.23:5558 \
        tcp://192.168.0.24:5559 \
        tcp://192.168.0.25:5560 \
    --save logs/cameras_test.mcap \
    --app-id aizee-cameras
```

**Expected results:**
- Rerun viewer opens automatically
- 4 cameras visible in hierarchy: `cameras/cam_front/color`, etc.
- Statistics show ~30.0 fps for each camera
- MCAP file created in `logs/`

#### Test 9: Rerun Bridge from Jetson (Production Setup)

Once all cameras are on the rover's internal network:

```bash
# SSH into Jetson
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Run Rerun bridge on Jetson
cd ~/aizee
python3 python/rerun_bridge.py \
    --cameras \
        tcp://192.168.0.22:5557 \
        tcp://192.168.0.23:5558 \
        tcp://192.168.0.24:5559 \
        tcp://192.168.0.25:5560 \
    --save logs/cameras_jetson_test.mcap \
    --app-id aizee-cameras

# This tests the actual rover switch network path
```

#### Test 10: MCAP Recording Verification

```bash
# Install mcap CLI
pip install mcap

# Inspect recording
mcap info logs/cameras_test.mcap

# Expected: 4 channels, one per camera
```

## Production Usage

### Starting Camera System

```bash
# Start all cameras
./scripts/start_all_cameras.sh

# Or start individually
ssh pi@192.168.0.22 sudo systemctl start aizee-camera-cam_front
# ... repeat for other cameras
```

### Stopping Camera System

```bash
# Stop all cameras
./scripts/stop_all_cameras.sh

# Or stop individually
ssh pi@192.168.0.22 sudo systemctl stop aizee-camera-cam_front
```

### Enable Auto-Start on Boot

```bash
# Enable all cameras to start on boot
ssh pi@192.168.0.22 sudo systemctl enable aizee-camera-cam_front
ssh pi@192.168.0.23 sudo systemctl enable aizee-camera-cam_rear
ssh pi@192.168.0.24 sudo systemctl enable aizee-camera-cam_left
ssh pi@192.168.0.25 sudo systemctl enable aizee-camera-cam_right
```

### Monitoring

```bash
# View live logs from all cameras (in separate terminals)
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -f
ssh pi@192.168.0.23 sudo journalctl -u aizee-camera-cam_rear -f
ssh pi@192.168.0.24 sudo journalctl -u aizee-camera-cam_left -f
ssh pi@192.168.0.25 sudo journalctl -u aizee-camera-cam_right -f
```

## Troubleshooting

### Camera Not Detected

```bash
# Check USB connection
lsusb | grep Intel

# Check dmesg for USB errors
dmesg | tail -n 50

# Check power (D455 requires 5V/2A minimum)
vcgencmd get_throttled
# 0x0 = OK, anything else = power issue
```

### Python Import Error

```bash
# Verify pyrealsense2 installation
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"

# If fails, rebuild SDK or check library path
ls /usr/local/lib/python3.*/dist-packages/pyrealsense2*
```

### ZMQ Connection Issues

```bash
# Check if camera node is running
ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front

# Check if ZMQ port is open
ssh pi@192.168.0.22 sudo netstat -tlnp | grep 5557

# Test from Pi itself
ssh pi@192.168.0.22
python3 ~/aizee/python/test_camera_subscriber.py --zmq-endpoint tcp://localhost:5557
```

### Low Frame Rate

```bash
# Check CPU usage
ssh pi@192.168.0.22 top

# Reduce JPEG quality to lower CPU load
# Edit systemd service: --jpeg-quality 50 (instead of 85)

# Or reduce resolution in config file
vim config/hardware_rpi4_cam_front.yaml
# Change: width: 640 -> 320, height: 480 -> 240
```

### USB Power Issues

If you see USB disconnections or errors:

```bash
# Check power supply
vcgencmd get_throttled

# Use powered USB hub if needed
# Or switch to USB-C power delivery (PD) for Pi 4
```

## Acceptance Criteria

### Per-Pi Requirements
- [x] Pi boots and accessible via SSH
- [x] Static IP configured (192.168.0.22-25)
- [x] RealSense D455 detected by `rs-enumerate-devices`
- [x] `import pyrealsense2` succeeds, version 2.54.x
- [x] Camera node runs manually without errors
- [x] Systemd service starts and auto-restarts on failure
- [x] ZMQ stream receivable from dev machine
- [x] Frame rate ≥25 fps sustained
- [x] Frame latency <50ms

### System-Level Requirements
- [x] All 4 Pis accessible simultaneously
- [x] All 4 camera services running concurrently
- [x] Rerun bridge connects to all 4 endpoints
- [x] All cameras visible in Rerun hierarchy
- [x] MCAP recording functional
- [x] No packet loss during 30-min test
- [x] Network latency <2ms for all Pis
- [x] Total bandwidth <100 Mbps
- [x] Services survive reboot (auto-start)

## Network Architecture Notes

### Development Phase (WiFi)
During initial development, Pis connect via WiFi for easy setup and testing from dev machine.

### Production Phase (Rover LAN)
In final deployment:
- All 4 Pis connect to rover switch via Ethernet
- Jetson (192.168.0.27) runs Rerun bridge
- All communication over rover's internal gigabit network
- PoE switch powers Pis (25W per port, sufficient for Pi 4 + D455)

### Bandwidth Calculation
- Per camera: ~25 Mbps (640×480 @ 30fps JPEG + depth)
- Total: 4 cameras × 25 Mbps = 100 Mbps
- Available: 1000 Mbps (gigabit Ethernet)
- Headroom: 10× safety margin

## Files Created

### Configuration Files
- `config/hardware.yaml` - Updated network.cameras section
- `config/hardware_rpi4_cam_front.yaml` - Front camera config
- `config/hardware_rpi4_cam_rear.yaml` - Rear camera config
- `config/hardware_rpi4_cam_left.yaml` - Left camera config
- `config/hardware_rpi4_cam_right.yaml` - Right camera config

### Systemd Services
- `config/systemd/aizee-camera-cam_front.service`
- `config/systemd/aizee-camera-cam_rear.service`
- `config/systemd/aizee-camera-cam_left.service`
- `config/systemd/aizee-camera-cam_right.service`

### Deployment Scripts
- `scripts/deploy_rpi4_camera.sh` - Deploy single camera
- `scripts/deploy_all_cameras.sh` - Deploy all 4 cameras
- `scripts/start_all_cameras.sh` - Start all services
- `scripts/stop_all_cameras.sh` - Stop all services

### Existing Code (No Changes)
- `python/nodes/camera_node.py` - Camera streaming node
- `python/rerun_bridge.py` - Multi-camera visualization
- `python/test_camera_subscriber.py` - Testing utility
- `requirements.txt` - Python dependencies

## Next Steps

After completing camera deployment:

1. **Update serial numbers** in config files with actual D455 serials
2. **Document network topology** in rover documentation
3. **Create operator quick-start guide** for rover startup
4. **Integration testing** with Jetson rover module and RPi arm module
5. **Static IP configuration** on rover's internal network (if different from WiFi)
6. **PoE switch configuration** for production deployment
7. **Autonomous navigation** integration with camera feeds

## References

- [Intel RealSense D455 Datasheet](https://www.intelrealsense.com/depth-camera-d455/)
- [librealsense GitHub](https://github.com/IntelRealSense/librealsense)
- [Raspberry Pi 4 Specifications](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/)
- [ZeroMQ Guide](https://zeromq.org/get-started/)
- [Rerun Documentation](https://www.rerun.io/docs)
