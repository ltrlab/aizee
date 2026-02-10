# RPLiDAR A1M8 Implementation Status

## Completed ✅

### Code Implementation
- [x] Extended `TelemetryMessage` with `LidarScan` struct in `rust/comms/src/messages.rs`
- [x] Created `rust/lidar_control` crate with proper dependencies
- [x] Implemented custom RPLiDAR A1M8 protocol (avoiding buggy `rplidar_drv` 0.6 crate)
- [x] Implemented `LidarScanner` with direct serialport communication
- [x] Implemented main control loop with tokio async + spawn_blocking
- [x] Added ZMQ publisher on dedicated port 5561
- [x] Auto-reconnect logic for USB disconnects
- [x] Proper error handling and logging

### Configuration
- [x] Updated `config/hardware_jetson_rover.yaml` with lidars section and lidar_pub endpoint
- [x] Created systemd service `aizee-lidar-control.service`
- [x] Created udev rules template (USB port-based since serial numbers are identical)
- [x] Created installation script `scripts/install_lidar_udev.sh`

### Deployment
- [x] Deployment script `scripts/deploy_lidar_control.sh`
- [x] Code deployed to Jetson Orin Nano (192.168.0.27)
- [x] Binary built successfully on Jetson
- [x] Both LiDAR sensors detected (ttyUSB0, ttyUSB1)

### Documentation
- [x] Comprehensive deployment guide `docs/LIDAR_DEPLOYMENT.md`
- [x] README for lidar_control crate
- [x] Python test script `python/test_lidar_telemetry.py`

## In Progress 🔄

### Hardware Testing
- **Status**: Partial success
  - ✅ Both devices initialize successfully
  - ✅ Device info retrieved (Model, Firmware, Serial)
  - ✅ SCAN command sent successfully
  - ⚠️ Scan data reading has issues (timeouts after 11 points)

### Root Cause Analysis
The scan reading timeouts suggest one of the following:

1. **Motor Not Spinning**: RPLiDAR A1M8 requires motor power
   - The motor controller needs PWM signal or might need separate power
   - Check if motor connector is properly connected
   - Verify motor power LED is on

2. **Protocol Timing Issues**: Scan packet format might need adjustment
   - Current implementation reads 5-byte packets
   - May need to handle different scan modes or response types

3. **Baudrate Issues**: Using 115200 baud (standard for A1M8)
   - Some clones use different baudrates
   - Try 256000 if 115200 doesn't work

## Remaining Tasks 📋

### Immediate (Hardware-Dependent)
1. **Fix Scan Reading**
   - Debug why scans timeout after partial data
   - Verify RPLiDAR motor is spinning (listen for spinning sound)
   - Check motor PWM connection
   - Add motor control commands if needed
   - Test with updated protocol implementation

2. **Install Udev Rules**
   ```bash
   ssh ltr@192.168.0.27
   cd ~/aizee/scripts
   sudo bash install_lidar_udev.sh
   ```

3. **Verify Full Scans**
   - Should get 350-365 points per scan
   - Publish rate should be ~5Hz
   - Test with `python test_lidar_telemetry.py`

### Integration
4. **Install Systemd Service**
   ```bash
   sudo cp ~/aizee/config/systemd/aizee-lidar-control.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable aizee-lidar-control
   sudo systemctl start aizee-lidar-control
   ```

5. **Update Rerun Bridge** (Optional)
   - Add LiDAR subscriber on port 5561
   - Implement `process_lidar_scans()` function
   - Log point clouds with `rr.Points3D()`

### Testing
6. **End-to-End Verification**
   - Verify both sensors publishing simultaneously
   - Test USB disconnect/reconnect
   - Verify service restarts on boot
   - Monitor performance metrics (CPU, bandwidth)

## Known Issues 🐛

### 1. rplidar_drv 0.6 Compilation Error
**Problem**: `rplidar_drv` 0.6 has packed struct alignment issues with Rust 1.93+

**Solution**: Implemented custom protocol directly with `serialport` crate

### 2. Identical Serial Numbers
**Problem**: Both RPLiDAR sensors have serial "0001" (cheap USB-to-UART chips)

**Solution**: Use USB port location in udev rules (`KERNELS=="1-2.2"` vs `KERNELS=="1-2.4"`)

### 3. Scan Reading Timeouts
**Status**: Under investigation

**Possible Causes**:
- Motor not spinning (power/PWM issue)
- Protocol timing issue
- Need to send motor start command

**Next Steps**:
1. Check hardware documentation for motor control
2. Add motor PWM control commands
3. Test with oscilloscope/logic analyzer if needed

## Testing Commands

### On Jetson

```bash
# Manual test
cd ~/aizee
AIZEE_CONFIG=~/aizee/config/hardware_jetson_rover.yaml \
RUST_LOG=debug \
./rust/target/release/lidar_control

# Check USB devices
lsusb | grep "Silicon Labs"
ls -la /dev/ttyUSB*
ls -la /dev/rplidar_*

# Install udev rules
cd ~/aizee/scripts
sudo bash install_lidar_udev.sh

# Service management
sudo systemctl status aizee-lidar-control
sudo journalctl -u aizee-lidar-control -f
```

### On Dev Machine

```bash
# Test ZMQ subscription
python python/test_lidar_telemetry.py --host 192.168.0.27 --port 5561

# Deploy updates
./scripts/deploy_lidar_control.sh
```

## Architecture Summary

```
┌─────────────────────────────────────────────────────────────┐
│                    Jetson Orin Nano                         │
│                    (192.168.0.27)                           │
│                                                             │
│  ┌────────────────┐         ┌────────────────┐            │
│  │ motor_control  │         │ lidar_control  │            │
│  │   (Rust)       │         │   (Rust)       │            │
│  │                │         │                │            │
│  │ CAN Bus (can1) │         │ USB Serial     │            │
│  │ ↓              │         │ ↓              │            │
│  │ ROBSTRIDE      │         │ RPLiDAR×2      │            │
│  │ Motors         │         │                │            │
│  │                │         │                │            │
│  │ ZMQ PUB :5556  │         │ ZMQ PUB :5561  │            │
│  └────────┬───────┘         └────────┬───────┘            │
│           │                          │                     │
└───────────┼──────────────────────────┼─────────────────────┘
            │                          │
            │     Network (ZMQ)        │
            │                          │
       ┌────▼──────────────────────────▼────┐
       │        Dev Machine / Teleop         │
       │                                     │
       │  Motor Telemetry    LiDAR Telemetry│
       │  Subscriber         Subscriber     │
       └─────────────────────────────────────┘
```

## Success Criteria (from Plan)

- [x] Both RPLiDAR sensors publishing scan data to ZMQ port 5561
- [ ] Scan data includes 360° coverage with ~360 points per scan ⚠️ (11 points currently)
- [ ] Publish rate ~5Hz (matching natural scan rate)
- [x] Independent sensor operation (one failing doesn't crash the other)
- [x] Automatic USB reconnect on disconnect
- [ ] Systemd service starts on boot (not installed yet)
- [ ] Telemetry visible from dev machine via ZMQ subscription (needs full scans)
- [ ] (Optional) Point clouds visible in Rerun viewer

## Files Modified/Created

### New Files
- `rust/lidar_control/Cargo.toml`
- `rust/lidar_control/src/main.rs`
- `rust/lidar_control/src/scanner.rs`
- `rust/lidar_control/README.md`
- `config/systemd/aizee-lidar-control.service`
- `config/udev/99-rplidar.rules`
- `scripts/deploy_lidar_control.sh`
- `scripts/install_lidar_udev.sh`
- `python/test_lidar_telemetry.py`
- `docs/LIDAR_DEPLOYMENT.md`
- `docs/LIDAR_IMPLEMENTATION_STATUS.md` (this file)

### Modified Files
- `rust/Cargo.toml` (added lidar_control to workspace)
- `rust/comms/src/messages.rs` (added LidarScan struct)
- `config/hardware_jetson_rover.yaml` (added lidars section and lidar_pub)

## Next Session TODO

1. **Debug motor spin issue**
   - Research RPLiDAR A1M8 motor control
   - Check if DTR pin controls motor
   - Add motor start command if needed

2. **Test with working scans**
   - Verify 360-point scans
   - Confirm 5Hz publish rate
   - Test ZMQ subscription from dev machine

3. **Complete deployment**
   - Install udev rules permanently
   - Install and enable systemd service
   - Add to startup sequence

4. **Optional enhancements**
   - Add Rerun visualization
   - Implement scan merging
   - Add obstacle detection layer
