# AIZEE Camera Deployment Checklist

**Deployment Date:** _________________
**Deployed By:** _________________

## Pre-Deployment (Development Machine)

- [x] Configuration files created
- [x] Systemd service files created
- [x] Deployment scripts created and executable
- [x] Documentation complete
- [ ] All 4 Raspberry Pi 4 devices acquired
- [ ] All 4 Intel RealSense D455 cameras acquired
- [ ] SD cards prepared (32GB+ recommended)
- [ ] Network switch configured (gigabit + PoE optional)
- [ ] Ethernet cables prepared (4× for cameras)

## Raspberry Pi 1 - Front Camera (192.168.0.22)

### Initial Setup
- [ ] Flash Raspberry Pi OS Lite 64-bit to SD card
- [ ] Set hostname to `AIZEE-ROVER-PI-1`
- [ ] Enable SSH during imaging
- [ ] First boot successful
- [ ] SSH access confirmed: `ssh pi@192.168.0.22`

### System Configuration
- [ ] System updated: `sudo apt update && sudo apt upgrade -y`
- [ ] Development dependencies installed
- [ ] RealSense SDK v2.54.2 built from source (30-45 min)
- [ ] Python bindings installed
- [ ] Verified: `python3 -c "import pyrealsense2 as rs; print(rs.__version__)"`
- [ ] Static IP configured: `192.168.0.22/24`
- [ ] User added to dialout group: `sudo usermod -a -G dialout pi`

### Camera Setup
- [ ] D455 connected to USB 3.0 port (blue port)
- [ ] Camera detected: `lsusb | grep Intel`
- [ ] Serial number discovered: `rs-enumerate-devices | grep Serial`
- [ ] Serial: _________________ (record here)
- [ ] Config updated: `config/hardware_rpi4_cam_front.yaml`

### Deployment & Testing
- [ ] Codebase deployed: `./scripts/deploy_rpi4_camera.sh cam_front`
- [ ] Python dependencies installed
- [ ] Manual test passed: `python3 python/nodes/camera_node.py --camera-id cam_front ...`
- [ ] Systemd service installed
- [ ] Service started: `sudo systemctl start aizee-camera-cam_front`
- [ ] Service status OK: `sudo systemctl status aizee-camera-cam_front`
- [ ] Stream tested from dev machine: `python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557`
- [ ] FPS ≥25 fps confirmed
- [ ] Latency <50ms confirmed
- [ ] Auto-start enabled: `sudo systemctl enable aizee-camera-cam_front`

## Raspberry Pi 2 - Rear Camera (192.168.0.23)

### Initial Setup
- [ ] Flash Raspberry Pi OS Lite 64-bit to SD card
- [ ] Set hostname to `AIZEE-ROVER-PI-2`
- [ ] Enable SSH during imaging
- [ ] First boot successful
- [ ] SSH access confirmed: `ssh pi@192.168.0.23`

### System Configuration
- [ ] System updated
- [ ] Development dependencies installed
- [ ] RealSense SDK v2.54.2 built from source
- [ ] Python bindings installed
- [ ] Verified: `python3 -c "import pyrealsense2 as rs; print(rs.__version__)"`
- [ ] Static IP configured: `192.168.0.23/24`
- [ ] User added to dialout group

### Camera Setup
- [ ] D455 connected to USB 3.0 port
- [ ] Camera detected: `lsusb | grep Intel`
- [ ] Serial number discovered: `rs-enumerate-devices | grep Serial`
- [ ] Serial: _________________ (record here)
- [ ] Config updated: `config/hardware_rpi4_cam_rear.yaml`

### Deployment & Testing
- [ ] Codebase deployed: `./scripts/deploy_rpi4_camera.sh cam_rear`
- [ ] Python dependencies installed
- [ ] Manual test passed
- [ ] Systemd service installed
- [ ] Service started: `sudo systemctl start aizee-camera-cam_rear`
- [ ] Service status OK
- [ ] Stream tested from dev machine: `python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.23:5558`
- [ ] FPS ≥25 fps confirmed
- [ ] Latency <50ms confirmed
- [ ] Auto-start enabled: `sudo systemctl enable aizee-camera-cam_rear`

## Raspberry Pi 3 - Left Camera (192.168.0.24)

### Initial Setup
- [ ] Flash Raspberry Pi OS Lite 64-bit to SD card
- [ ] Set hostname to `AIZEE-ROVER-PI-3`
- [ ] Enable SSH during imaging
- [ ] First boot successful
- [ ] SSH access confirmed: `ssh pi@192.168.0.24`

### System Configuration
- [ ] System updated
- [ ] Development dependencies installed
- [ ] RealSense SDK v2.54.2 built from source
- [ ] Python bindings installed
- [ ] Verified: `python3 -c "import pyrealsense2 as rs; print(rs.__version__)"`
- [ ] Static IP configured: `192.168.0.24/24`
- [ ] User added to dialout group

### Camera Setup
- [ ] D455 connected to USB 3.0 port
- [ ] Camera detected: `lsusb | grep Intel`
- [ ] Serial number discovered: `rs-enumerate-devices | grep Serial`
- [ ] Serial: _________________ (record here)
- [ ] Config updated: `config/hardware_rpi4_cam_left.yaml`

### Deployment & Testing
- [ ] Codebase deployed: `./scripts/deploy_rpi4_camera.sh cam_left`
- [ ] Python dependencies installed
- [ ] Manual test passed
- [ ] Systemd service installed
- [ ] Service started: `sudo systemctl start aizee-camera-cam_left`
- [ ] Service status OK
- [ ] Stream tested from dev machine: `python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.24:5559`
- [ ] FPS ≥25 fps confirmed
- [ ] Latency <50ms confirmed
- [ ] Auto-start enabled: `sudo systemctl enable aizee-camera-cam_left`

## Raspberry Pi 4 - Right Camera (192.168.0.25)

### Initial Setup
- [ ] Flash Raspberry Pi OS Lite 64-bit to SD card
- [ ] Set hostname to `AIZEE-ROVER-PI-4`
- [ ] Enable SSH during imaging
- [ ] First boot successful
- [ ] SSH access confirmed: `ssh pi@192.168.0.25`

### System Configuration
- [ ] System updated
- [ ] Development dependencies installed
- [ ] RealSense SDK v2.54.2 built from source
- [ ] Python bindings installed
- [ ] Verified: `python3 -c "import pyrealsense2 as rs; print(rs.__version__)"`
- [ ] Static IP configured: `192.168.0.25/24`
- [ ] User added to dialout group

### Camera Setup
- [ ] D455 connected to USB 3.0 port
- [ ] Camera detected: `lsusb | grep Intel`
- [ ] Serial number discovered: `rs-enumerate-devices | grep Serial`
- [ ] Serial: _________________ (record here)
- [ ] Config updated: `config/hardware_rpi4_cam_right.yaml`

### Deployment & Testing
- [ ] Codebase deployed: `./scripts/deploy_rpi4_camera.sh cam_right`
- [ ] Python dependencies installed
- [ ] Manual test passed
- [ ] Systemd service installed
- [ ] Service started: `sudo systemctl start aizee-camera-cam_right`
- [ ] Service status OK
- [ ] Stream tested from dev machine: `python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.25:5560`
- [ ] FPS ≥25 fps confirmed
- [ ] Latency <50ms confirmed
- [ ] Auto-start enabled: `sudo systemctl enable aizee-camera-cam_right`

## Multi-Camera Integration Testing

### Network Tests
- [ ] All 4 Pis pingable simultaneously
- [ ] Latency test: `for ip in 22 23 24 25; do ping -c 50 192.168.0.$ip | tail -n 1; done`
- [ ] Average latency <2ms for all Pis
- [ ] No packet loss observed

### Concurrent Streaming Tests
- [ ] All 4 services started: `./scripts/start_all_cameras.sh`
- [ ] All services running: verify with `systemctl status` on each Pi
- [ ] All 4 streams tested simultaneously from dev machine
- [ ] No frame drops observed
- [ ] Total bandwidth <100 Mbps (measured with network monitor)

### Rerun Integration (WiFi Development Setup)
- [ ] Rerun bridge launched with all 4 cameras:
  ```bash
  python python/rerun_bridge.py \
      --cameras tcp://192.168.0.22:5557 tcp://192.168.0.23:5558 \
                tcp://192.168.0.24:5559 tcp://192.168.0.25:5560 \
      --save logs/cameras_test.mcap
  ```
- [ ] Rerun viewer opened automatically
- [ ] All 4 cameras visible in hierarchy
  - [ ] `cameras/cam_front/color`
  - [ ] `cameras/cam_rear/color`
  - [ ] `cameras/cam_left/color`
  - [ ] `cameras/cam_right/color`
- [ ] FPS statistics show ~30.0 fps for all cameras
- [ ] MCAP file created: `logs/cameras_test.mcap`
- [ ] MCAP verification: `mcap info logs/cameras_test.mcap`
- [ ] 4 channels present in MCAP (one per camera)

### Stress Testing
- [ ] 30-minute continuous streaming test completed
- [ ] No crashes or service restarts during stress test
- [ ] Frame rate stable throughout test
- [ ] No memory leaks observed (check with `top` on each Pi)

## Rover LAN Integration (Production Setup)

### Jetson Deployment
- [ ] AIZEE codebase deployed to Jetson (192.168.0.27)
- [ ] SSH access confirmed: `ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27`
- [ ] Python dependencies installed on Jetson

### Network Configuration
- [ ] All 4 Pis connected to rover switch via Ethernet
- [ ] Jetson connected to rover switch
- [ ] All devices pingable from Jetson
- [ ] Static IPs confirmed on rover network

### Jetson-Based Testing
- [ ] Camera streams accessible from Jetson:
  ```bash
  ssh ltr@192.168.0.27
  cd ~/aizee
  python3 python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557
  ```
- [ ] Rerun bridge tested from Jetson with all 4 cameras
- [ ] MCAP recording from Jetson successful
- [ ] Bandwidth verified on rover switch network

### System Integration
- [ ] Jetson rover module running (motor control)
- [ ] RPi arm module running (192.168.0.28)
- [ ] All 4 camera modules running
- [ ] Unified Rerun bridge showing all subsystems
- [ ] No port conflicts detected
- [ ] All systems functional concurrently

## Production Readiness

### Auto-Start Configuration
- [ ] Pi 1 cam_front auto-starts on boot
- [ ] Pi 2 cam_rear auto-starts on boot
- [ ] Pi 3 cam_left auto-starts on boot
- [ ] Pi 4 cam_right auto-starts on boot
- [ ] Reboot test: All cameras come up automatically

### Documentation
- [ ] Actual D455 serial numbers documented in configs
- [ ] Network topology documented
- [ ] IP address assignments documented
- [ ] Operator quick-start guide created
- [ ] Troubleshooting procedures verified

### Final Verification
- [ ] All acceptance criteria met (per-Pi requirements)
- [ ] All acceptance criteria met (system-level requirements)
- [ ] Deployment documentation updated with actual serial numbers
- [ ] System ready for production use

## Notes & Issues

### Raspberry Pi 1 - Front Camera
```
Issue/Note:


Resolution:


```

### Raspberry Pi 2 - Rear Camera
```
Issue/Note:


Resolution:


```

### Raspberry Pi 3 - Left Camera
```
Issue/Note:


Resolution:


```

### Raspberry Pi 4 - Right Camera
```
Issue/Note:


Resolution:


```

### System-Level Issues
```
Issue/Note:


Resolution:


```

## Sign-Off

- [ ] All 4 camera nodes deployed and operational
- [ ] All tests passed
- [ ] Documentation complete
- [ ] System ready for integration with rover

**Deployment Completed:** _________________ (date)

**Deployed By:** _________________ (name)

**Verified By:** _________________ (name)

**Notes:**
```


```
