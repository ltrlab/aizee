# RPLiDAR A1M8 Integration - Final Status & Next Steps

## Current Status

### ✅ What's Working
1. **Core Infrastructure**: Complete rust/lidar_control crate with async control loop
2. **Hardware Connection**: Both LiDAR sensors detected and initializing correctly
3. **Motor Control**: DTR signal management working - motors spinning
4. **ZMQ Publishing**: Dedicated port 5561 configured and publishing
5. **Message Format**: LidarScan struct integrated into TelemetryMessage
6. **Configuration**: Complete YAML config, systemd service, udev rules
7. **Deployment**: All code deployed and building on Jetson

### ⚠️ Current Issue
**Protocol Synchronization Problem**: Custom packet parsing is not correctly synchronizing with the RPLIDAR A1 data stream. Symptoms:
- Sync flags appearing too frequently (almost every packet)
- Scans contain 1-7 points instead of expected 300-400
- Inconsistent behavior between test runs

## Root Cause Analysis

The RPLIDAR A1 protocol implementation has proven more complex than anticipated:

1. **Response Descriptors**: After each command, RPLIDAR sends variable-length response descriptors
2. **Multiple Scan Modes**: A1 supports standard, express, and force scan modes with different data formats
3. **Firmware Variations**: Different firmware versions may have slightly different protocols
4. **State Management**: Device state affects data stream format

## Recommended Solutions (in order of preference)

### Option 1: Use Existing Rust Library ⭐ RECOMMENDED

**Library**: [`rplidar`](https://github.com/cnwzhjs/rplidar.rs) crate (or fork)

**Approach**:
```rust
// Replace custom implementation with proven library
use rplidar::{RposDriver, ScanPoint};

let mut driver = RposDriver::open_port("/dev/rplidar_front")?;
let info = driver.get_device_info()?;
driver.start_scan()?;

loop {
    if let Ok(scan) = driver.grab_scan() {
        // scan contains full 360° rotation
        process_scan(scan);
    }
}
```

**Pros**:
- Proven implementation handling all protocol nuances
- Handles response descriptors correctly
- Supports multiple scan modes
- Active development

**Cons**:
- The `rplidar_drv` 0.6 has compilation issues with Rust 1.93+
- May need to use fork or patch

**Solution to compilation issue**:
- Fork the repository and fix packed struct issues
- Or use an alternative fork that's already been updated
- Or temporarily downgrade Rust on Jetson

### Option 2: Python Bridge (Quick Workaround)

Use proven Python `rplidar` library with PyO3 bindings:

```python
# python/lidar_node.py
from rplidar import RPLidar
import zmq

lidar = RPLidar('/dev/rplidar_front')
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5561")

for scan in lidar.iter_scans():
    msg = create_telemetry(scan)
    pub.send_json(msg)
```

**Pros**:
- Works immediately with proven library
- Easy to implement and test
- Good for rapid prototyping

**Cons**:
- Python runtime overhead
- Separate process management
- Not as elegant as pure Rust solution

### Option 3: Continue Custom Implementation (Time-Intensive)

Debug the current custom protocol parser by:

1. Adding hex dump logging of raw serial data
2. Comparing against official SDK packet captures
3. Implementing proper response descriptor parsing for each command
4. Testing with oscilloscope/logic analyzer

**Estimated effort**: 4-8 hours of debugging with hardware access

## Immediate Next Steps

### Step 1: Decision Point

**Choose your approach**:
- **Quick Win**: Option 2 (Python) - get scans working today
- **Best Solution**: Option 1 (Rust library) - 1-2 hours to fix compilation
- **Learning Exercise**: Option 3 (Custom) - significant debugging time

### Step 2: If Using Option 1 (Rust Library) - RECOMMENDED

```bash
# On dev machine
cd /p/Workspace/aizee/rust/lidar_control

# Update Cargo.toml
# Remove: rplidar_drv = "0.6"
# Add: rplidar = { git = "https://github.com/cnwzhjs/rplidar.rs" }

# Or use a working fork:
# rplidar = { git = "https://github.com/[working-fork]/rplidar.rs" }

# Update scanner.rs to use the library's API
# Deploy and test
```

### Step 3: Rerun Integration (Once Scans Working)

Add to `python/rerun_bridge.py`:

```python
import rerun as rr
import zmq
import numpy as np

# Subscribe to LiDAR port
lidar_sub = ctx.socket(zmq.SUB)
lidar_sub.connect("tcp://192.168.0.27:5561")
lidar_sub.subscribe(b"")

# In main loop
sockets = [motor_sub, lidar_sub, camera_sub]
readable, _, _ = zmq.select(sockets, [], [], timeout=0.1)

if lidar_sub in readable:
    msg = lidar_sub.recv_json()
    process_lidar(msg)

def process_lidar(msg):
    """Convert LiDAR scans to point clouds"""
    for scan in msg.get('lidar_scans', []):
        sensor_id = scan['sensor_id']
        ranges = np.array(scan['ranges'])
        angles = np.linspace(scan['angle_min'], scan['angle_max'], len(ranges))

        # Polar to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.column_stack([x, y, z])
        colors = np.array(scan['intensities'])

        rr.log(
            f"sensors/{sensor_id}/scan",
            rr.Points3D(points, radii=0.02, colors=colors)
        )
```

## Resources

### Working Implementations

- **Python rplidar**: https://github.com/SkoltechRobotics/rplidar
- **Rust rplidar.rs**: https://github.com/cnwzhjs/rplidar.rs
- **ROS rplidar**: https://github.com/Slamtec/rplidar_ros (C++)
- **Adafruit CircuitPython**: https://github.com/adafruit/Adafruit_CircuitPython_RPLIDAR

### Protocol Documentation

- **Official SDK**: https://github.com/Slamtec/rplidar_sdk
- **Protocol Spec**: http://bucket.download.slamtec.com/.../LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
- **Datasheet**: RPLIDAR A1M8 datasheet (Slamtec website)

### Key Learnings

1. **DTR Signal**: Must be cleared for motor to spin (discovered via web research)
2. **USB Port Mapping**: Use `KERNELS` attribute when serial numbers are identical
3. **Response Descriptors**: Each command has variable-length response before scan data
4. **Packet Validation**: Start flag and inverted start flag must be different; check bit must be 1

## Files Ready for Use (Once Scans Working)

All infrastructure is in place:
- ✅ `rust/lidar_control/` - Complete crate structure
- ✅ `config/hardware_jetson_rover.yaml` - LiDAR configuration
- ✅ `config/systemd/aizee-lidar-control.service` - Service file
- ✅ `config/udev/99-rplidar.rules` - USB rules template
- ✅ `python/test_lidar_telemetry.py` - Test script
- ✅ `docs/LIDAR_DEPLOYMENT.md` - Complete deployment guide

Just need to fix the packet parsing!

## Recommendation

**Use Option 1 (Rust library)** - spend 1-2 hours fixing compilation issues or finding a working fork. This provides:
- ✅ Pure Rust solution (matches project architecture)
- ✅ Proven protocol implementation
- ✅ Better performance than Python
- ✅ Maintainable long-term

The custom implementation taught us a lot about the protocol, but using a battle-tested library is the pragmatic choice for a production system.
