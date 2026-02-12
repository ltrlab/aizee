# Camera Deployment Quick Start

## One-Time Setup Per Raspberry Pi

### 1. Flash SD Card
- OS: Raspberry Pi OS Lite 64-bit
- Enable SSH
- Set hostname: AIZEE-ROVER-PI-1/2/3/4
- Username: pi

### 2. SSH In and Install Dependencies

```bash
# SSH into Pi
ssh pi@192.168.0.22  # Change IP for each Pi

# System update and dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-dev build-essential pkg-config \
    cmake git curl libzmq3-dev libusb-1.0-0-dev libudev-dev libssl-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev python3-opencv
```

### 3. Build RealSense SDK (30-45 min)

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.54.2

# Install udev rules
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout pi

# Build
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=bool:true \
         -DPYTHON_EXECUTABLE=/usr/bin/python3 \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=false \
         -DBUILD_GRAPHICAL_EXAMPLES=false
make -j4
sudo make install
sudo ldconfig
sudo cp wrappers/python/pyrealsense2*.so /usr/local/lib/python3.*/dist-packages/

# Verify
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

### 4. Configure Static IP

```bash
# For each Pi, use appropriate IP (.22, .23, .24, .25)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.0.22/24
sudo nmcli con mod "Wired connection 1" ipv4.gateway 192.168.0.1
sudo nmcli con mod "Wired connection 1" ipv4.dns 8.8.8.8
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Reboot to apply
sudo reboot
```

### 5. Connect Camera and Get Serial Number

```bash
# Connect D455 to USB 3.0 port (blue)
# Then run:
rs-enumerate-devices | grep Serial

# Note the serial number, update in config files
```

## Deployment from Dev Machine

```bash
cd /p/Workspace/aizee

# Deploy all cameras
./scripts/deploy_all_cameras.sh

# Update serial numbers in config files
vim config/hardware_rpi4_cam_front.yaml  # Update serial field
vim config/hardware_rpi4_cam_rear.yaml
vim config/hardware_rpi4_cam_left.yaml
vim config/hardware_rpi4_cam_right.yaml
```

## Start Cameras

```bash
# Start all
./scripts/start_all_cameras.sh

# Enable auto-start on boot
ssh pi@192.168.0.22 sudo systemctl enable aizee-camera-cam_front
ssh pi@192.168.0.23 sudo systemctl enable aizee-camera-cam_rear
ssh pi@192.168.0.24 sudo systemctl enable aizee-camera-cam_left
ssh pi@192.168.0.25 sudo systemctl enable aizee-camera-cam_right
```

## Test with Rerun

```bash
# From dev machine
python python/rerun_bridge.py \
    --cameras tcp://192.168.0.22:5557 tcp://192.168.0.23:5558 \
              tcp://192.168.0.24:5559 tcp://192.168.0.25:5560 \
    --save logs/test.mcap
```

## Common Commands

```bash
# Check status
ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front

# View logs
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -f

# Stop all
./scripts/stop_all_cameras.sh

# Restart single camera
ssh pi@192.168.0.22 sudo systemctl restart aizee-camera-cam_front
```

## IP Assignment Reference

| Camera Position | IP Address    | Hostname          | ZMQ Port |
|----------------|---------------|-------------------|----------|
| Front          | 192.168.0.22  | AIZEE-ROVER-PI-1  | 5557     |
| Rear           | 192.168.0.23  | AIZEE-ROVER-PI-2  | 5558     |
| Left           | 192.168.0.24  | AIZEE-ROVER-PI-3  | 5559     |
| Right          | 192.168.0.25  | AIZEE-ROVER-PI-4  | 5560     |

## Troubleshooting

**Camera not detected:**
```bash
lsusb | grep Intel
dmesg | tail -n 30
```

**Service not starting:**
```bash
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -n 100
```

**Low FPS:**
- Reduce JPEG quality: Edit service file, change `--jpeg-quality 85` to `--jpeg-quality 50`
- Reduce resolution: Edit config file, use 320×240 instead of 640×480
