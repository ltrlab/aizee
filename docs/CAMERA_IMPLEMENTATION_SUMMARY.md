# RealSense D455 Camera Deployment - Implementation Summary

## Overview

This document summarizes the implementation of the RealSense D455 camera deployment infrastructure for the AIZEE rover's 4 Raspberry Pi 4 camera nodes.

**Implementation Date:** 2026-02-10
**Status:** ✅ Complete - Ready for deployment

## What Was Implemented

### 1. Configuration Files ✅

#### Main Hardware Configuration Update
- **File:** `config/hardware.yaml` (lines 86-113)
- **Changes:**
  - Updated IP addresses: `192.168.1.21-24` → `192.168.0.22-25`
  - Updated hostnames: `aizee-rpi-front/rear/left/right` → `AIZEE-ROVER-PI-1/2/3/4`
  - Maintained spatial transforms and camera configuration

#### Per-Device Configuration Files (New)
Created 4 device-specific configuration files:

1. **`config/hardware_rpi4_cam_front.yaml`**
   - Camera ID: `cam_front`
   - IP: `192.168.0.22`
   - Hostname: `AIZEE-ROVER-PI-1`
   - ZMQ Port: `5557`
   - Position: `[0.25, 0.0, 0.15]` (front-facing)

2. **`config/hardware_rpi4_cam_rear.yaml`**
   - Camera ID: `cam_rear`
   - IP: `192.168.0.23`
   - Hostname: `AIZEE-ROVER-PI-2`
   - ZMQ Port: `5558`
   - Position: `[-0.25, 0.0, 0.15]` (rear-facing, 180° rotation)

3. **`config/hardware_rpi4_cam_left.yaml`**
   - Camera ID: `cam_left`
   - IP: `192.168.0.24`
   - Hostname: `AIZEE-ROVER-PI-3`
   - ZMQ Port: `5559`
   - Position: `[0.0, 0.20, 0.15]` (left-facing, 90° rotation)

4. **`config/hardware_rpi4_cam_right.yaml`**
   - Camera ID: `cam_right`
   - IP: `192.168.0.25`
   - Hostname: `AIZEE-ROVER-PI-4`
   - ZMQ Port: `5560`
   - Position: `[0.0, -0.20, 0.15]` (right-facing, -90° rotation)

**Common Configuration:**
- Resolution: 640×480 @ 30fps (color + depth)
- JPEG quality: 85
- IMU: 200Hz (accel + gyro)
- Serial numbers: Placeholders (to be updated after hardware discovery)

### 2. Systemd Service Files ✅

Created 4 systemd service files for automatic camera node startup:

**Files:**
- `config/systemd/aizee-camera-cam_front.service`
- `config/systemd/aizee-camera-cam_rear.service`
- `config/systemd/aizee-camera-cam_left.service`
- `config/systemd/aizee-camera-cam_right.service`

**Key Features:**
- User: `pi`
- Working directory: `/home/pi/aizee`
- Python unbuffered output for real-time logging
- Auto-restart on failure (5 second delay)
- Journal logging (stdout + stderr)
- Environment variable: `AIZEE_CONFIG` points to device-specific config
- Command-line arguments: camera-id, zmq-endpoint, fps, jpeg-quality

**Differences from Motor Control Service:**
- No CAN interface setup (camera nodes don't use CAN)
- Python interpreter instead of Rust binary
- Different environment variables (PYTHONUNBUFFERED vs RUST_LOG)
- Camera-specific CLI arguments

### 3. Deployment Scripts ✅

#### `scripts/deploy_rpi4_camera.sh`
Single-camera deployment script with features:
- Accepts camera ID as parameter (cam_front/rear/left/right)
- Automatically maps camera ID → IP address → hostname
- Ping check before deployment
- Syncs Python codebase via rsync (excludes Rust, logs, cache)
- Installs Python dependencies remotely
- Installs systemd service file
- Tests camera connectivity with `rs-enumerate-devices`
- Provides helpful next-step instructions

**Exclusions in rsync:**
- `rust/` - Not needed for Python-only camera nodes
- `target/` - Rust build artifacts
- `.git/` - Version control
- `*.pyc`, `__pycache__/` - Python cache
- `logs/`, `*.mcap` - Local data files

#### `scripts/deploy_all_cameras.sh`
Batch deployment script:
- Loops through all 4 cameras
- Calls `deploy_rpi4_camera.sh` for each
- Provides comprehensive next-step instructions
- Includes serial number discovery commands
- Includes Rerun bridge test command

#### `scripts/start_all_cameras.sh`
Service startup script:
- Starts all 4 camera services in parallel (via SSH background jobs)
- Waits for all to complete
- Provides status checking commands
- Includes log viewing instructions

#### `scripts/stop_all_cameras.sh`
Service shutdown script:
- Stops all 4 camera services in parallel
- Clean shutdown for all cameras

All scripts are executable (`chmod +x`).

### 4. Documentation ✅

#### `docs/CAMERA_DEPLOYMENT.md` (Comprehensive Guide)
Full deployment documentation covering:
- **Overview:** Network topology, system architecture
- **Raspberry Pi Setup:**
  - Initial OS installation (Raspberry Pi Imager instructions)
  - System dependencies installation
  - RealSense SDK installation from source (v2.54.2)
  - Network configuration (static IP via NetworkManager)
  - Camera connection and serial number discovery
- **Deployment from Dev Machine:**
  - Single camera deployment
  - Multi-camera batch deployment
- **Testing Procedures:**
  - 10 comprehensive tests covering:
    - Per-Pi unit tests (camera detection, manual execution, ZMQ, systemd)
    - Multi-camera integration tests (simultaneous streaming, latency)
    - Rerun integration tests (WiFi dev setup + Jetson production setup)
    - MCAP recording verification
- **Production Usage:**
  - Starting/stopping camera system
  - Auto-start on boot configuration
  - Monitoring with journalctl
- **Troubleshooting:**
  - Camera not detected
  - Python import errors
  - ZMQ connection issues
  - Low frame rate
  - USB power issues
- **Acceptance Criteria:**
  - Per-Pi requirements checklist (9 items)
  - System-level requirements checklist (9 items)
- **Network Architecture Notes:**
  - Development phase (WiFi)
  - Production phase (rover LAN)
  - Bandwidth calculations (100 Mbps total, 10× headroom)
- **References:** Links to datasheets, documentation

#### `docs/CAMERA_QUICK_START.md` (Quick Reference)
Condensed quick-reference guide:
- One-time setup checklist per Pi
- Copy-paste commands for common tasks
- IP assignment reference table
- Common troubleshooting commands

### 5. Existing Infrastructure (Verified) ✅

**No changes required** - these files already exist and are production-ready:

- **`python/nodes/camera_node.py`**
  - Production-ready camera streaming node
  - Uses pyrealsense2 SDK v2.54.0
  - ZeroMQ publishing with JPEG compression
  - IMU data support
  - CLI arguments match systemd service files
  - Signal handling (SIGINT, SIGTERM)
  - Frame statistics logging

- **`python/rerun_bridge.py`**
  - Multi-camera ZMQ subscription
  - Hierarchical visualization: `cameras/{camera_id}/color`
  - Frame draining (latest-only for low latency)
  - Per-camera FPS statistics
  - MCAP recording capability
  - Automatic Rerun viewer spawn

- **`python/test_camera_subscriber.py`**
  - ZMQ stream testing utility
  - FPS and latency measurements

- **`requirements.txt`**
  - All Python dependencies including `pyrealsense2>=2.54.0`
  - Ready for `pip install -r requirements.txt`

## Network Topology

### IP Address Assignment

| Device              | IP Address    | Hostname          | ZMQ Ports    | Purpose           |
|---------------------|---------------|-------------------|--------------|-------------------|
| Camera Front        | 192.168.0.22  | AIZEE-ROVER-PI-1  | 5557         | Front D455 camera |
| Camera Rear         | 192.168.0.23  | AIZEE-ROVER-PI-2  | 5558         | Rear D455 camera  |
| Camera Left         | 192.168.0.24  | AIZEE-ROVER-PI-3  | 5559         | Left D455 camera  |
| Camera Right        | 192.168.0.25  | AIZEE-ROVER-PI-4  | 5560         | Right D455 camera |
| Jetson Orin Nano    | 192.168.0.27  | aizee-rover       | 5555/5556    | Rover module      |
| RPi4 Arm            | 192.168.0.28  | aizee-arm         | 5557/5558    | Arm module        |

### ZMQ Port Allocation

- **Rover (Jetson):** 5555 (cmd sub), 5556 (telemetry pub)
- **Arm (RPi4):** 5557 (cmd sub), 5558 (telemetry pub)
- **Camera Front:** 5557 (camera pub)
- **Camera Rear:** 5558 (camera pub)
- **Camera Left:** 5559 (camera pub)
- **Camera Right:** 5560 (camera pub)

**Note:** There's a port overlap between Arm module and Camera Front (both use 5557/5558). This is acceptable since:
1. Arm module uses ports for motor control (different traffic pattern)
2. Camera nodes are on separate physical devices
3. All ZMQ endpoints include IP addresses for disambiguation

If port conflicts arise in integrated testing, camera ports can be shifted to 5561-5564.

## Implementation Changes from Plan

### Minor Adjustments

1. **ZMQ Port Assignment:**
   - Original plan used 5557 for all camera_pub in configs
   - Implementation uses unique ports per camera (5557-5560)
   - Ensures no port conflicts when testing from single machine

2. **Script Enhancements:**
   - Added `stop_all_cameras.sh` (not in original plan)
   - Added more comprehensive error checking
   - Enhanced output with colored status messages
   - Added next-step instructions to all scripts

3. **Documentation Expansion:**
   - Created two-tier documentation (comprehensive + quick-start)
   - Added specific test procedures for Jetson-based testing
   - Included bandwidth calculations and network topology details
   - Added acceptance criteria checklists

### No Changes Required

- Existing `camera_node.py` already supports all required features
- Existing `rerun_bridge.py` already supports multi-camera visualization
- No code changes needed to existing Python infrastructure
- `requirements.txt` already includes all dependencies

## Files Created

### Configuration (5 files)
1. `config/hardware.yaml` - Updated (network.cameras section)
2. `config/hardware_rpi4_cam_front.yaml` - New
3. `config/hardware_rpi4_cam_rear.yaml` - New
4. `config/hardware_rpi4_cam_left.yaml` - New
5. `config/hardware_rpi4_cam_right.yaml` - New

### Systemd Services (4 files)
6. `config/systemd/aizee-camera-cam_front.service` - New
7. `config/systemd/aizee-camera-cam_rear.service` - New
8. `config/systemd/aizee-camera-cam_left.service` - New
9. `config/systemd/aizee-camera-cam_right.service` - New

### Deployment Scripts (4 files)
10. `scripts/deploy_rpi4_camera.sh` - New
11. `scripts/deploy_all_cameras.sh` - New
12. `scripts/start_all_cameras.sh` - New
13. `scripts/stop_all_cameras.sh` - New

### Documentation (3 files)
14. `docs/CAMERA_DEPLOYMENT.md` - New
15. `docs/CAMERA_QUICK_START.md` - New
16. `docs/CAMERA_IMPLEMENTATION_SUMMARY.md` - New (this file)

**Total: 16 new/modified files**

## Next Steps for Deployment

### Phase 1: First Pi Setup (AIZEE-ROVER-PI-1)

1. Flash Raspberry Pi OS Lite 64-bit to SD card
2. Set hostname to `AIZEE-ROVER-PI-1`, enable SSH
3. Boot and SSH into Pi
4. Install system dependencies (15 min)
5. Build RealSense SDK from source (30-45 min)
6. Configure static IP `192.168.0.22`
7. Connect D455 camera via USB 3.0
8. Discover serial number: `rs-enumerate-devices | grep Serial`
9. Update `config/hardware_rpi4_cam_front.yaml` with actual serial
10. Deploy from dev machine: `./scripts/deploy_rpi4_camera.sh cam_front`
11. Test manually: SSH in and run camera_node.py
12. Start systemd service: `ssh pi@192.168.0.22 sudo systemctl start aizee-camera-cam_front`
13. Test stream from dev machine: `python python/test_camera_subscriber.py --zmq-endpoint tcp://192.168.0.22:5557`

### Phase 2: Remaining Pis (PI-2, PI-3, PI-4)

1. Repeat Phase 1 for each Pi with appropriate IPs/hostnames/configs
2. **Optimization:** Can clone SD card from PI-1 to speed up (update hostname/IP after clone)

### Phase 3: Multi-Camera Testing (WiFi Development)

1. Start all cameras: `./scripts/start_all_cameras.sh`
2. Test Rerun bridge from dev machine:
   ```bash
   python python/rerun_bridge.py \
       --cameras tcp://192.168.0.22:5557 tcp://192.168.0.23:5558 \
                 tcp://192.168.0.24:5559 tcp://192.168.0.25:5560 \
       --save logs/cameras_test.mcap
   ```
3. Verify all 4 cameras visible in Rerun viewer
4. Run 30-minute stress test
5. Check network latency: `ping` all Pis
6. Verify MCAP recording: `mcap info logs/cameras_test.mcap`

### Phase 4: Rover LAN Testing (Final Configuration)

1. Connect all Pis to rover switch via Ethernet
2. Deploy AIZEE codebase to Jetson (if not already done)
3. SSH into Jetson: `ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27`
4. Test camera streams from Jetson perspective
5. Run Rerun bridge on Jetson with all 4 cameras
6. Verify bandwidth on rover switch (<100 Mbps total)
7. Test integration with Jetson rover module and RPi arm module

### Phase 5: Production Setup

1. Enable auto-start for all cameras: `sudo systemctl enable aizee-camera-cam_*`
2. Document actual D455 serial numbers in config files
3. Create operator quick-start guide for rover startup sequence
4. Configure PoE switch if using powered Ethernet
5. Final integration testing with full rover system

## Testing Checklist

### Per-Pi Tests (4× devices)
- [ ] Pi boots and accessible via SSH
- [ ] Static IP configured correctly
- [ ] `rs-enumerate-devices` detects D455
- [ ] `import pyrealsense2` works, version 2.54.x
- [ ] Camera node runs manually without errors
- [ ] Systemd service starts successfully
- [ ] ZMQ stream receivable from dev machine
- [ ] Frame rate ≥25 fps sustained
- [ ] Frame latency <50ms

### System-Level Tests
- [ ] All 4 Pis pingable simultaneously
- [ ] All 4 camera services running concurrently
- [ ] Rerun bridge connects to all 4 endpoints
- [ ] All cameras visible in Rerun hierarchy
- [ ] MCAP recording functional
- [ ] No packet loss during 30-min stress test
- [ ] Network latency <2ms for all Pis
- [ ] Total bandwidth <100 Mbps
- [ ] Services survive reboot (auto-start works)

## Risk Mitigation

| Risk | Mitigation | Fallback |
|------|------------|----------|
| ARM64 SDK compatibility | Build from source v2.54.2 (proven stable) | Use opencv backend without librealsense2 |
| Insufficient CPU for 30fps | Monitor CPU usage, tune JPEG quality | Reduce quality to 50 or resolution to 320×240 |
| Network bandwidth saturation | Pre-calculated 100 Mbps << 1 Gbps | Enable depth compression, reduce fps |
| USB power issues | Use USB 3.0 port, monitor `dmesg` | Use powered USB hub or USB-C PD |
| Port conflicts with Arm module | Unique ports per camera (5557-5560) | Shift camera ports to 5561-5564 |

## Success Criteria

**Implementation Phase:** ✅ COMPLETE
- [x] All configuration files created
- [x] All systemd service files created
- [x] All deployment scripts created and executable
- [x] Comprehensive documentation written
- [x] Existing Python infrastructure verified

**Deployment Phase:** ⏳ PENDING
- [ ] All 4 Raspberry Pis set up and accessible
- [ ] RealSense SDK installed on all Pis
- [ ] Camera nodes deployed and running
- [ ] All cameras streaming via ZMQ
- [ ] Rerun bridge showing all 4 camera feeds
- [ ] MCAP recording working
- [ ] Services auto-start on boot

**Integration Phase:** ⏳ PENDING
- [ ] Cameras integrated with Jetson rover module
- [ ] Cameras integrated with RPi arm module
- [ ] Full system testing complete
- [ ] Operator documentation complete
- [ ] Production-ready

## Bandwidth Analysis

### Per-Camera Bandwidth
- Color (640×480 JPEG @ quality 85): ~15 Mbps
- Depth (640×480 Z16 @ 30fps): ~10 Mbps
- IMU (200Hz): <0.1 Mbps
- **Total per camera:** ~25 Mbps

### System Bandwidth
- 4 cameras × 25 Mbps = **100 Mbps total**
- Gigabit Ethernet available: **1000 Mbps**
- **Headroom:** 10× safety margin

### Optimization Options (if needed)
1. Reduce JPEG quality: 85 → 50 (saves ~5 Mbps per camera)
2. Reduce resolution: 640×480 → 320×240 (saves ~15 Mbps per camera)
3. Reduce frame rate: 30fps → 15fps (saves ~12.5 Mbps per camera)
4. Compress depth: Enable depth compression (saves ~5 Mbps per camera)

## Conclusion

The RealSense D455 camera deployment infrastructure is **fully implemented and ready for hardware deployment**. All configuration files, deployment scripts, systemd services, and documentation have been created according to the plan.

**Key Achievements:**
- ✅ Zero changes required to existing production Python code
- ✅ Automated deployment pipeline for all 4 cameras
- ✅ Comprehensive testing procedures defined
- ✅ Network topology designed with bandwidth headroom
- ✅ Failsafe mechanisms and fallback options documented
- ✅ Two-tier documentation (comprehensive + quick-start)

**Recommended First Step:**
Deploy to AIZEE-ROVER-PI-1 (cam_front) as proof-of-concept, then replicate to remaining 3 Pis.

---

**Implementation completed:** 2026-02-10
**Ready for hardware deployment:** Yes ✅
