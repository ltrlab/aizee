# RPLiDAR A1M8 Integration - Deployment Guide

This guide provides step-by-step instructions for deploying the RPLiDAR A1M8 integration on the AIZEE rover's Jetson Orin Nano.

## Overview

The AIZEE rover uses two RPLiDAR A1M8 sensors for obstacle detection and SLAM:
- **Front LiDAR**: `/dev/rplidar_front` (stable symlink via udev)
- **Back LiDAR**: `/dev/rplidar_back` (stable symlink via udev)

The integration follows the modular architecture pattern:
- **Separate Rust crate**: `rust/lidar_control`
- **Independent ZMQ port**: `tcp://*:5561` (dedicated LiDAR telemetry)
- **Systemd service**: `aizee-lidar-control.service`
- **Async control loop**: 5Hz publish rate matching natural scan rate (~5.5Hz)

## Architecture

```
                          ┌─────────────────────┐
                          │  Jetson Orin Nano   │
                          │   (192.168.0.27)    │
                          └──────────┬──────────┘
                                     │
                    ┌────────────────┼────────────────┐
                    │                │                │
              USB Serial        USB Serial      Network
                    │                │                │
           ┌────────▼────────┐ ┌────▼─────────┐ ┌───▼────────┐
           │  RPLiDAR Front  │ │ RPLiDAR Back │ │  Dev Machine│
           │    (A1M8)       │ │   (A1M8)     │ │             │
           │ /dev/ttyUSB0 ──>│ │/dev/ttyUSB1─>│ │ ZMQ Sub     │
           │ /dev/rplidar_   │ │/dev/rplidar_ │ │ :5561       │
           │     front       │ │    back      │ └─────────────┘
           └─────────────────┘ └──────────────┘
                    │                │
                    └────────┬───────┘
                             │
                      Scan Data (5Hz)
                             │
                             ▼
                   ┌──────────────────┐
                   │ lidar_control    │
                   │  (Rust Service)  │
                   │                  │
                   │ • spawn_blocking │
                   │ • Async tokio    │
                   │ • Auto-reconnect │
                   └────────┬─────────┘
                            │
                      ZMQ PUB :5561
                            │
                            ▼
                   TelemetryMessage {
                     lidar_scans: Vec<LidarScan>
                   }
```

## Prerequisites

- Jetson Orin Nano with Ubuntu 20.04/22.04
- Two RPLiDAR A1M8 sensors connected via USB
- Rust toolchain installed (`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`)
- SSH access to Jetson (key: `P:/Workspace/ssh-keys/aizee_rover_id`)

## Step 1: Discover USB Device Serial Numbers

Connect both LiDAR sensors to the Jetson and identify their serial numbers:

```bash
# SSH into Jetson
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Find RPLiDAR devices (Silicon Labs CP2102 USB-to-UART)
lsusb | grep "Silicon Labs"
# Expected output:
# Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
# Bus 001 Device 006: ID 10c4:ea60 Silicon Labs CP210x UART Bridge

# Check which ttyUSB devices exist
ls -la /dev/ttyUSB*
# Should show: /dev/ttyUSB0, /dev/ttyUSB1

# Extract serial numbers for each device
sudo udevadm info -a -n /dev/ttyUSB0 | grep serial
sudo udevadm info -a -n /dev/ttyUSB1 | grep serial

# Example output:
#   ATTRS{serial}=="0001"
#   ATTRS{serial}=="0002"
```

**Note the serial numbers** - you'll need them in Step 2.

## Step 2: Configure Udev Rules

Create stable device names using udev rules:

```bash
# On Jetson, edit the udev rules file
sudo nano /etc/udev/rules.d/99-rplidar.rules
```

Paste the following content, replacing `<FRONT_SERIAL>` and `<BACK_SERIAL>` with actual values from Step 1:

```
# Udev rules for RPLiDAR A1M8 sensors
# Front LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<FRONT_SERIAL>", SYMLINK+="rplidar_front", MODE="0666", GROUP="dialout"

# Back LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<BACK_SERIAL>", SYMLINK+="rplidar_back", MODE="0666", GROUP="dialout"
```

Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify the symlinks were created:

```bash
ls -la /dev/rplidar_*
# Expected output:
# lrwxrwxrwx 1 root root 7 Feb 10 12:00 /dev/rplidar_back -> ttyUSB1
# lrwxrwxrwx 1 root root 7 Feb 10 12:00 /dev/rplidar_front -> ttyUSB0
```

Test serial port access:

```bash
# Should show device info without errors
cat /dev/rplidar_front &
sleep 1
kill %1
```

## Step 3: Deploy Code to Jetson

From your development machine:

```bash
# Navigate to workspace
cd /p/Workspace/aizee

# Deploy entire rust directory (includes lidar_control)
scp -i /p/Workspace/ssh-keys/aizee_rover_id -r rust ltr@192.168.0.27:~/aizee/

# Deploy updated config
scp -i /p/Workspace/ssh-keys/aizee_rover_id config/hardware_jetson_rover.yaml ltr@192.168.0.27:~/aizee/config/

# Deploy systemd service
scp -i /p/Workspace/ssh-keys/aizee_rover_id config/systemd/aizee-lidar-control.service ltr@192.168.0.27:~/aizee/config/systemd/
```

## Step 4: Build on Jetson

SSH into the Jetson and build the lidar_control binary:

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Load Rust environment
source ~/.cargo/env

# Navigate to workspace
cd ~/aizee/rust

# Build lidar_control (release mode)
cargo build --release -p lidar_control

# Verify binary exists
ls -lh target/release/lidar_control
```

Expected build time: 2-5 minutes (first build with dependencies).

## Step 5: Test Manually

Before installing as a systemd service, test manually:

```bash
# On Jetson
cd ~/aizee

# Set environment variables
export AIZEE_CONFIG=~/aizee/config/hardware_jetson_rover.yaml
export RUST_LOG=info

# Run lidar_control
./rust/target/release/lidar_control
```

Expected output:

```
INFO lidar_control: AIZEE LiDAR Control starting...
INFO lidar_control: Loading config from: /home/ltr/aizee/config/hardware_jetson_rover.yaml
INFO lidar_control: LiDAR telemetry will publish on: tcp://*:5561
INFO lidar_control: Initializing LiDAR: lidar_front at /dev/rplidar_front
INFO lidar_control::scanner: Opening LiDAR device lidar_front at /dev/rplidar_front
INFO lidar_control::scanner: LiDAR lidar_front - Model: 24, Firmware: 1.29, Hardware: 7
INFO lidar_control::scanner: LiDAR lidar_front health status: Good
INFO lidar_control::scanner: LiDAR lidar_front scanning started
INFO lidar_control: LiDAR lidar_front initialized successfully
INFO lidar_control: Initializing LiDAR: lidar_back at /dev/rplidar_back
INFO lidar_control::scanner: Opening LiDAR device lidar_back at /dev/rplidar_back
INFO lidar_control::scanner: LiDAR lidar_back - Model: 24, Firmware: 1.29, Hardware: 7
INFO lidar_control::scanner: LiDAR lidar_back scanning started
INFO lidar_control: LiDAR lidar_back initialized successfully
INFO lidar_control: Initialized 2 LiDAR sensor(s)
INFO lidar_control: LiDAR control loop started
INFO lidar_control: Received scan from lidar_front: 362 points
INFO lidar_control: Received scan from lidar_back: 358 points
INFO lidar_control: Published 2 LiDAR scan(s)
```

Press `Ctrl+C` to stop. Verify graceful shutdown:

```
INFO lidar_control::scanner: Stopping scan for lidar_front
INFO lidar_control::scanner: Stopping scan for lidar_back
INFO lidar_control: LiDAR control shutting down
```

## Step 6: Test ZMQ Subscription (from Dev Machine)

Open a new terminal on your development machine and test receiving LiDAR data:

```python
# test_lidar_sub.py
import zmq
import json
import time

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://192.168.0.27:5561")
sub.subscribe(b"")

print("Waiting for LiDAR telemetry on tcp://192.168.0.27:5561...")

while True:
    try:
        msg = sub.recv_json()
        if 'lidar_scans' in msg:
            scans = msg['lidar_scans']
            print(f"\n[{time.strftime('%H:%M:%S')}] Received {len(scans)} scan(s):")
            for scan in scans:
                sensor_id = scan['sensor_id']
                num_points = len(scan['ranges'])
                min_range = min(scan['ranges']) if scan['ranges'] else 0
                max_range = max(scan['ranges']) if scan['ranges'] else 0
                print(f"  {sensor_id}: {num_points} points, range: {min_range:.2f}m - {max_range:.2f}m")
    except KeyboardInterrupt:
        print("\nExiting...")
        break
```

Run it:

```bash
python test_lidar_sub.py
```

Expected output:

```
Waiting for LiDAR telemetry on tcp://192.168.0.27:5561...

[12:34:56] Received 2 scan(s):
  lidar_front: 362 points, range: 0.18m - 8.45m
  lidar_back: 358 points, range: 0.16m - 11.23m

[12:34:56] Received 2 scan(s):
  lidar_front: 361 points, range: 0.19m - 8.42m
  lidar_back: 360 points, range: 0.15m - 11.28m
```

## Step 7: Install Systemd Service

If manual testing succeeded, install as a systemd service:

```bash
# On Jetson
sudo cp ~/aizee/config/systemd/aizee-lidar-control.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable aizee-lidar-control

# Start service
sudo systemctl start aizee-lidar-control

# Check status
sudo systemctl status aizee-lidar-control
```

Expected status output:

```
● aizee-lidar-control.service - AIZEE LiDAR Control - RPLiDAR A1M8 Scanners
     Loaded: loaded (/etc/systemd/system/aizee-lidar-control.service; enabled)
     Active: active (running) since Mon 2026-02-10 12:45:23 PST; 5s ago
   Main PID: 12345 (lidar_control)
      Tasks: 4 (limit: 4915)
     Memory: 8.2M
     CGroup: /system.slice/aizee-lidar-control.service
             └─12345 /home/ltr/aizee/rust/target/release/lidar_control

Feb 10 12:45:23 aizee-jetson systemd[1]: Started AIZEE LiDAR Control.
Feb 10 12:45:25 aizee-jetson lidar_control[12345]: INFO lidar_control: Initialized 2 LiDAR sensor(s)
Feb 10 12:45:25 aizee-jetson lidar_control[12345]: INFO lidar_control: LiDAR control loop started
```

## Step 8: Monitor Service Logs

Monitor real-time logs:

```bash
# Follow logs (Ctrl+C to exit)
sudo journalctl -u aizee-lidar-control -f

# View last 50 lines
sudo journalctl -u aizee-lidar-control -n 50

# View logs since boot
sudo journalctl -u aizee-lidar-control -b
```

## Verification Checklist

- [ ] Both LiDAR devices appear as `/dev/rplidar_front` and `/dev/rplidar_back`
- [ ] `lidar_control` builds successfully without errors
- [ ] Manual test shows scan data being received (~360 points per sensor)
- [ ] ZMQ subscription from dev machine receives telemetry on port 5561
- [ ] Systemd service starts and runs without errors
- [ ] Service restarts automatically after sensor disconnect/reconnect
- [ ] Service starts automatically on Jetson reboot
- [ ] Publish rate is ~5Hz (check logs)
- [ ] No error messages in journalctl logs

## Troubleshooting

### Issue: `/dev/rplidar_*` symlinks not created

**Cause**: Udev rules not applied or incorrect serial numbers

**Fix**:
```bash
# Verify serial numbers again
sudo udevadm info -a -n /dev/ttyUSB0 | grep serial
sudo udevadm info -a -n /dev/ttyUSB1 | grep serial

# Check udev rules syntax
cat /etc/udev/rules.d/99-rplidar.rules

# Reload and trigger
sudo udevadm control --reload-rules
sudo udevadm trigger

# Unplug/replug USB devices
```

### Issue: Permission denied opening serial port

**Cause**: User not in `dialout` group

**Fix**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout ltr

# Log out and back in for changes to take effect
exit
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Verify group membership
groups
```

### Issue: Service fails with "test -e /dev/rplidar_front" error

**Cause**: Udev symlinks not created (see first issue)

**Fix**: Follow steps in first troubleshooting section. Service `ExecStartPre` checks ensure devices exist before starting.

### Issue: Only one LiDAR works, other shows "Failed to open serial port"

**Cause**: Second sensor may be faulty, wrong serial number, or USB power issue

**Fix**:
```bash
# Test each sensor individually
cat /dev/rplidar_front &
sleep 1
kill %1

cat /dev/rplidar_back &
sleep 1
kill %1

# Check USB power
lsusb -v -s 001:005 | grep MaxPower  # Adjust bus:device numbers
```

The system is designed to continue operating with one sensor if the other fails.

### Issue: Scans have very few points (<100)

**Cause**: LiDAR motor not spinning, sensor obstruction, or hardware issue

**Fix**:
```bash
# Check device health
cd ~/aizee/rust/lidar_control
cargo run --example check_health  # If example exists

# Verify motor is spinning (listen for spinning sound)
# Check for obstructions on sensor lens
# Try power cycling the sensor (unplug/replug USB)
```

### Issue: ZMQ subscription receives no data

**Cause**: Firewall blocking port 5561, service not running, or wrong IP

**Fix**:
```bash
# On Jetson - verify service is publishing
sudo journalctl -u aizee-lidar-control -n 20 | grep "Published"

# Check if port is open
sudo netstat -tulpn | grep 5561

# Test locally on Jetson
python3 -c "import zmq; c=zmq.Context(); s=c.socket(zmq.SUB); s.connect('tcp://127.0.0.1:5561'); s.subscribe(b''); print('OK')"

# On dev machine - verify network connectivity
ping 192.168.0.27
telnet 192.168.0.27 5561
```

### Issue: Service keeps restarting

**Cause**: Persistent error in lidar_control

**Fix**:
```bash
# Check logs for error details
sudo journalctl -u aizee-lidar-control -n 100

# Stop service and run manually for debugging
sudo systemctl stop aizee-lidar-control
export RUST_LOG=debug
export AIZEE_CONFIG=~/aizee/config/hardware_jetson_rover.yaml
~/aizee/rust/target/release/lidar_control
```

## Testing USB Disconnect/Reconnect

The system is designed to auto-reconnect on USB disconnect:

```bash
# Test procedure:
# 1. Start service and verify it's working
sudo systemctl status aizee-lidar-control

# 2. Monitor logs in one terminal
sudo journalctl -u aizee-lidar-control -f

# 3. In another terminal, find USB bus/device
lsusb | grep "Silicon Labs"

# 4. Simulate disconnect (unbind USB device)
# Replace 001:005 with actual bus:device from lsusb
echo "1-2" | sudo tee /sys/bus/usb/drivers/usb/unbind

# 5. Watch logs - should show error and reconnect attempt
# Expected: WARN "Scan error for lidar_front: ..."
#           INFO "Attempting to reconnect LiDAR lidar_front"

# 6. Rebind device
echo "1-2" | sudo tee /sys/bus/usb/drivers/usb/bind

# 7. Verify reconnection in logs
# Expected: INFO "LiDAR lidar_front reconnected successfully"
```

## Performance Metrics

Expected performance on Jetson Orin Nano:

- **Scan rate**: ~5.5Hz per sensor (natural RPLiDAR A1M8 rate)
- **Publish rate**: 5Hz (200ms interval)
- **Points per scan**: 350-365 points (depending on rotation speed)
- **CPU usage**: <5% per sensor
- **Memory usage**: ~8-12 MB total
- **Network bandwidth**: ~50-100 KB/s on port 5561 (JSON telemetry)

## Network Topology

The complete AIZEE network topology with LiDAR:

```
Jetson Orin Nano (192.168.0.27):
  - tcp://*:5555 - Commands (motor_control subscriber)
  - tcp://*:5556 - Telemetry (motor + battery, motor_control publisher)
  - tcp://*:5561 - Telemetry (LiDAR scans, lidar_control publisher) ← NEW

Dev Machine / Teleop:
  - tcp://192.168.0.27:5555 - Command publisher
  - tcp://192.168.0.27:5556 - Telemetry subscriber (motor)
  - tcp://192.168.0.27:5561 - Telemetry subscriber (LiDAR) ← NEW

Raspberry Pi 4 - Arm (192.168.0.28):
  - tcp://*:5557 - Commands (arm motor_control subscriber)
  - tcp://*:5558 - Telemetry (arm motors, motor_control publisher)

Raspberry Pi 4 - Cameras (future):
  - tcp://*:5559+ - Camera streams
```

## Future Enhancements

Potential improvements not yet implemented:

1. **Scan merging**: Combine front/back scans into unified 360° view
2. **Point cloud filtering**: Remove floor, ceiling, or noise points
3. **ROS bridge**: Publish as ROS `sensor_msgs/LaserScan` for Nav2
4. **SLAM integration**: Connect to RTAB-Map or Cartographer
5. **Obstacle detection**: Simple 2D obstacle layer for navigation
6. **Dynamic reconfigure**: Change scan mode via ZMQ commands
7. **Rerun visualization**: Add point cloud logging to rerun_bridge.py

## References

- **RPLiDAR A1M8 Specs**: https://www.slamtec.com/en/Lidar/A1
- **rplidar_drv crate**: https://docs.rs/rplidar_drv/
- **AIZEE Architecture**: See `CLAUDE.md` in repository root
- **Multi-Device Deployment**: See `docs/MULTI_DEVICE_DEPLOYMENT.md`
