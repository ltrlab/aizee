# Camera Deployment Scripts

This directory contains scripts for deploying and managing the AIZEE camera system across 4 Raspberry Pi 4 devices.

## Deployment Scripts

### `deploy_rpi4_camera.sh`
Deploy camera software to a single Raspberry Pi.

**Usage:**
```bash
./deploy_rpi4_camera.sh [cam_front|cam_rear|cam_left|cam_right]
```

**Examples:**
```bash
./deploy_rpi4_camera.sh cam_front  # Deploy front camera
./deploy_rpi4_camera.sh cam_rear   # Deploy rear camera
```

**What it does:**
1. Verifies target Pi is reachable (ping test)
2. Syncs Python codebase via rsync (excludes Rust, logs, cache)
3. Installs Python dependencies remotely
4. Installs systemd service file
5. Tests camera connectivity

**Auto IP Mapping:**
- `cam_front` → 192.168.0.22 (AIZEE-ROVER-PI-1)
- `cam_rear` → 192.168.0.23 (AIZEE-ROVER-PI-2)
- `cam_left` → 192.168.0.24 (AIZEE-ROVER-PI-3)
- `cam_right` → 192.168.0.25 (AIZEE-ROVER-PI-4)

### `deploy_all_cameras.sh`
Deploy camera software to all 4 Raspberry Pi devices at once.

**Usage:**
```bash
./deploy_all_cameras.sh
```

**What it does:**
1. Calls `deploy_rpi4_camera.sh` for each camera
2. Provides comprehensive next-step instructions
3. Includes serial number discovery commands

**Time:** ~5-10 minutes (depending on network speed)

## Service Management Scripts

### `start_all_cameras.sh`
Start camera services on all 4 Raspberry Pis simultaneously.

**Usage:**
```bash
./start_all_cameras.sh
```

**What it does:**
1. SSH into each Pi in parallel
2. Starts systemd service for each camera
3. Provides status checking commands

**Equivalent manual commands:**
```bash
ssh pi@192.168.0.22 sudo systemctl start aizee-camera-cam_front
ssh pi@192.168.0.23 sudo systemctl start aizee-camera-cam_rear
ssh pi@192.168.0.24 sudo systemctl start aizee-camera-cam_left
ssh pi@192.168.0.25 sudo systemctl start aizee-camera-cam_right
```

### `stop_all_cameras.sh`
Stop camera services on all 4 Raspberry Pis simultaneously.

**Usage:**
```bash
./stop_all_cameras.sh
```

**What it does:**
1. SSH into each Pi in parallel
2. Stops systemd service for each camera

## Testing Scripts

### `test_all_camera_streams.sh`
Test all 4 camera streams simultaneously from dev machine.

**Usage:**
```bash
./test_all_camera_streams.sh [duration_seconds]
```

**Examples:**
```bash
./test_all_camera_streams.sh      # Test for 30 seconds (default)
./test_all_camera_streams.sh 60   # Test for 60 seconds
```

**What it does:**
1. Launches 4 `test_camera_subscriber.py` instances in parallel
2. Each instance connects to one camera's ZMQ endpoint
3. Collects FPS and latency statistics
4. Runs for specified duration, then stops

**Expected Output:**
```
Received 150 frames in 5.0s (30.0 fps)  # For each camera
```

## Common Workflows

### Initial Deployment
```bash
# 1. Deploy to all cameras
./deploy_all_cameras.sh

# 2. Start all cameras
./start_all_cameras.sh

# 3. Test all streams
./test_all_camera_streams.sh 30

# 4. Enable auto-start on boot (manual step)
ssh pi@192.168.0.22 sudo systemctl enable aizee-camera-cam_front
ssh pi@192.168.0.23 sudo systemctl enable aizee-camera-cam_rear
ssh pi@192.168.0.24 sudo systemctl enable aizee-camera-cam_left
ssh pi@192.168.0.25 sudo systemctl enable aizee-camera-cam_right
```

### Update After Code Changes
```bash
# Redeploy and restart
./deploy_all_cameras.sh
./stop_all_cameras.sh
./start_all_cameras.sh
```

### Troubleshooting Single Camera
```bash
# Deploy and restart specific camera
./deploy_rpi4_camera.sh cam_front
ssh pi@192.168.0.22 sudo systemctl restart aizee-camera-cam_front
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -f
```

## Script Requirements

### Prerequisites
- SSH access to all Raspberry Pi devices
- SSH key-based authentication configured (no password prompt)
- `rsync` installed on dev machine
- Python 3 installed on all Pis
- Network connectivity to 192.168.0.22-25

### File Structure Expected
```
aizee/
├── python/
│   ├── nodes/
│   │   └── camera_node.py
│   └── test_camera_subscriber.py
├── config/
│   ├── hardware_rpi4_cam_front.yaml
│   ├── hardware_rpi4_cam_rear.yaml
│   ├── hardware_rpi4_cam_left.yaml
│   ├── hardware_rpi4_cam_right.yaml
│   └── systemd/
│       ├── aizee-camera-cam_front.service
│       ├── aizee-camera-cam_rear.service
│       ├── aizee-camera-cam_left.service
│       └── aizee-camera-cam_right.service
├── requirements.txt
└── scripts/
    ├── deploy_rpi4_camera.sh
    ├── deploy_all_cameras.sh
    ├── start_all_cameras.sh
    ├── stop_all_cameras.sh
    └── test_all_camera_streams.sh
```

## Troubleshooting

### Script Fails: "Cannot reach host"
```bash
# Verify Pi is on network
ping 192.168.0.22

# Check SSH access
ssh pi@192.168.0.22 echo "Connection OK"

# Verify static IP is configured on Pi
```

### Deployment Fails: "rsync error"
```bash
# Check rsync is installed
which rsync

# Test rsync manually
rsync -av test_file pi@192.168.0.22:~/
```

### Service Fails to Start
```bash
# Check service status on Pi
ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front

# View detailed logs
ssh pi@192.168.0.22 sudo journalctl -u aizee-camera-cam_front -n 100

# Common issues:
# - Python dependencies not installed: pip3 install -r requirements.txt
# - RealSense SDK not built: See docs/CAMERA_DEPLOYMENT.md
# - Camera not connected: lsusb | grep Intel
```

### Test Stream Fails
```bash
# Test single camera
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557

# Check if service is running
ssh pi@192.168.0.22 sudo systemctl status aizee-camera-cam_front

# Check if ZMQ port is open
ssh pi@192.168.0.22 sudo netstat -tlnp | grep 5557

# Check firewall (should be disabled on Raspberry Pi OS by default)
ssh pi@192.168.0.22 sudo iptables -L
```

## Advanced Usage

### Deploy to Custom IP
Edit the script and modify the IP mapping in the `case` statement:

```bash
# In deploy_rpi4_camera.sh
case "$CAMERA_ID" in
    cam_front)
        TARGET_IP="192.168.0.22"  # Change this
        ...
```

### Run Deployment from Different Directory
```bash
cd /path/to/aizee
./scripts/deploy_rpi4_camera.sh cam_front
```

### Parallel Deployment to Speed Up
```bash
# Start all deployments in parallel
./scripts/deploy_rpi4_camera.sh cam_front &
./scripts/deploy_rpi4_camera.sh cam_rear &
./scripts/deploy_rpi4_camera.sh cam_left &
./scripts/deploy_rpi4_camera.sh cam_right &
wait
```

## See Also

- [Full Deployment Guide](../docs/CAMERA_DEPLOYMENT.md) - Comprehensive setup instructions
- [Quick Start Guide](../docs/CAMERA_QUICK_START.md) - Condensed reference
- [Implementation Summary](../docs/CAMERA_IMPLEMENTATION_SUMMARY.md) - Architecture overview
- [Deployment Checklist](../CAMERA_DEPLOYMENT_CHECKLIST.md) - Step-by-step checklist
