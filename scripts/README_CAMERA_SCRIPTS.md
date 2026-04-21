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

**Auto IP Mapping (PoE subnet, accessed via Jetson hop):**
- `cam_front` → 10.42.0.11 (cam_front / PI-1)
- `cam_rear` → 10.42.0.12 (cam_rear / PI-2)
- `cam_left` → 10.42.0.13 (cam_left / PI-3)
- `cam_right` → 10.42.0.14 (cam_right / PI-4)

> The Pis are on the PoE-only subnet and are not directly reachable from the dev machine. Deploy scripts use the Jetson (192.168.0.27) as a hop. The camera relay service on the Jetson re-publishes streams on 192.168.0.27:5557-5560.

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

**Equivalent manual commands (via Jetson hop):**
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo systemctl start aizee-camera-cam_front'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.12 'sudo systemctl start aizee-camera-cam_rear'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.13 'sudo systemctl start aizee-camera-cam_left'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.14 'sudo systemctl start aizee-camera-cam_right'"
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

# 4. Enable auto-start on boot (manual step, via Jetson hop)
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo systemctl enable aizee-camera-cam_front'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.12 'sudo systemctl enable aizee-camera-cam_rear'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.13 'sudo systemctl enable aizee-camera-cam_left'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.14 'sudo systemctl enable aizee-camera-cam_right'"
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
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo systemctl restart aizee-camera-cam_front'"
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'sudo journalctl -u aizee-camera-cam_front -f'"
```

## Script Requirements

### Prerequisites
- SSH access to all Raspberry Pi devices via Jetson hop (PoE subnet 10.42.0.0/24)
- SSH key at `/p/Workspace/ssh-keys/aizee_rover_id` authorized on all nodes
- `rsync` installed on dev machine
- Python 3 installed on all Pis
- Network connectivity to Jetson (192.168.0.27)

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
# Verify Jetson is reachable
ping 192.168.0.27

# Verify SSH key works
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 echo "OK"

# Verify Pi is reachable via Jetson hop
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 echo OK"
```

### Deployment Fails: "rsync error"
```bash
# Check rsync is installed
which rsync

# Test rsync manually
rsync -av test_file -e "ssh -i /p/Workspace/ssh-keys/aizee_rover_id" \
    ltr@192.168.0.27:~/
```

### Service Fails to Start
```bash
# Check service status on Pi (via Jetson hop)
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 \
     'sudo systemctl status aizee-camera-cam_front'"

# View detailed logs
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 \
     'sudo journalctl -u aizee-camera-cam_front -n 100'"

# Common issues:
# - Python dependencies not installed: pip3 install -r requirements.txt
# - Camera not connected: lsusb | grep Intel
```

### Test Stream Fails
```bash
# Test relay stream (connects via Jetson, no hop needed)
python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.27:5557

# Check relay service on Jetson
sudo journalctl -u aizee-camera-relay -f
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

- [Camera System Overview](../docs/subsystems/CAMERAS.md) - Current network config, relay, and operational reference
