# Multi-Device Modular Deployment

This guide covers deploying AIZEE across multiple compute modules using the modular architecture pattern.

## Architecture Overview

```
┌─────────────────┐
│  Dev Machine    │
│  (Windows)      │
│  teleop.py      │
└────────┬────────┘
         │
    POE Switch
    + Ethernet
         │
    ┌────┴────┬─────────┐
    │         │         │
    ▼         ▼         ▼
┌─────────┐ ┌─────────┐ ┌─────────┐
│ Jetson  │ │  RPi4   │ │  RPi4   │
│ (Rover) │ │  (Arm)  │ │ (Torso) │
├─────────┤ ├─────────┤ ├─────────┤
│motor_ctl│ │motor_ctl│ │servo_ctl│
│:5555/56 │ │:5557/58 │ │:5559/60 │
├─────────┤ ├─────────┤ ├─────────┤
│  CAN0   │ │  CAN0   │ │  Serial │
└────┬────┘ └────┬────┘ └────┬────┘
     │           │           │
     ▼           ▼           ▼
  3 motors    3 motors    14 servos
  (base)      (arm)       (torso)
```

## Module Configuration

### Rover Module (Jetson Orin Nano)
- **Hardware**: 3 ROBSTRIDE motors (base)
  - left_wheel (CAN ID 0x02)
  - right_wheel (CAN ID 0x04)
  - swivel (CAN ID 0x03)
- **Network**: 192.168.0.27
- **ZMQ Ports**: :5555 (command), :5556 (telemetry)
- **Config**: `config/hardware_jetson_rover.yaml`

### Arm Module (Raspberry Pi 4)
- **Hardware**: 6 ROBSTRIDE motors (arm)
  - gantry_base (CAN ID 0x05, ROBSTRIDE04)
  - gantry_mid (CAN ID 0x06, ROBSTRIDE03)
  - gantry_end (CAN ID 0x07, ROBSTRIDE02)
  - wrist_pitch (CAN ID 0x08, ROBSTRIDE02)
  - wrist_roll (CAN ID 0x09, ROBSTRIDE00)
  - gripper (CAN ID 0x0A, ROBSTRIDE00)
- **Network**: 192.168.0.28
- **ZMQ Ports**: :5557 (command), :5558 (telemetry)
- **Config**: `config/hardware_rpi4_arm.yaml`
- **CAN Interface**: USB CAN adapter (can0)

> **Production note**: In the standard Jetson-only setup, all 6 arm motors plus the 3 base motors run together on the Jetson via `can1`. This RPi4 arm module is an alternative split configuration.

### Torso Module (Future)
- **Hardware**: 14 Feetech servos via serial
- **Network**: 192.168.0.29
- **ZMQ Ports**: :5559 (command), :5560 (telemetry)
- **Control**: Separate servo control stack (different protocol)

## Deployment Steps

### Phase A: RPi4 Hardware Setup

1. **Flash Raspberry Pi OS**
   ```bash
   # Use Raspberry Pi Imager
   # OS: Raspberry Pi OS Lite (64-bit)
   # Set hostname: aizee-arm
   # Enable SSH
   # Set username: pi
   # Configure WiFi/Ethernet
   ```

2. **Configure Network**
   ```bash
   # On RPi4
   sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.0.28/24
   sudo nmcli con mod "Wired connection 1" ipv4.gateway 192.168.0.1
   sudo nmcli con mod "Wired connection 1" ipv4.method manual
   sudo nmcli con up "Wired connection 1"
   ```

3. **Install Dependencies**
   ```bash
   ssh pi@192.168.0.28

   # System packages
   sudo apt update
   sudo apt install -y git build-essential pkg-config libzmq3-dev can-utils

   # Rust toolchain
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   ```

4. **Setup CAN Interface**
   ```bash
   # Plug in USB CAN adapter
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ip link show can0  # Verify UP state

   # Test CAN (if motors powered)
   candump can0
   ```

### Phase B: Deploy Code to RPi4

**Option 1: Using deployment script (from dev machine)**
```bash
cd P:/Workspace/aizee
./scripts/deploy_rpi4_arm.sh pi@192.168.0.28
```

**Option 2: Manual deployment**
```bash
# From dev machine
cd P:/Workspace/aizee
rsync -av --exclude 'target/' --exclude '.git/' \
    ./ pi@192.168.0.28:~/aizee/

# On RPi4
ssh pi@192.168.0.28
cd ~/aizee/rust/motor_control
cargo build --release
```

### Phase C: Install Systemd Service

```bash
# On RPi4
sudo cp ~/aizee/config/systemd/aizee-motor-control-arm.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aizee-motor-control-arm
sudo systemctl start aizee-motor-control-arm

# Verify
sudo systemctl status aizee-motor-control-arm
sudo journalctl -u aizee-motor-control-arm -f
```

### Phase D: Test Arm Module

**From dev machine or RPi4:**
```bash
python scripts/test_arm_module.py --host 192.168.0.28
```

Expected output:
```
✓ Telemetry received: 3 motors
Enabling motors: ['shoulder_pitch', 'elbow', 'wrist']
  shoulder_pitch: enabled
  elbow: enabled
  wrist: enabled
...
✓ Test complete
```

### Phase E: Configure Unified Teleop

The teleop configuration (`config/teleop.yaml`) already supports multi-module mode:

```yaml
modules:
  rover:
    command: "tcp://192.168.0.27:5555"
    telemetry: "tcp://192.168.0.27:5556"
    motors: ["left_wheel", "right_wheel", "swivel"]
  arm:
    command: "tcp://192.168.0.28:5557"
    telemetry: "tcp://192.168.0.28:5558"
    motors: ["gantry_base", "gantry_mid", "gantry_end", "wrist_pitch", "wrist_roll", "gripper"]
```

**Run unified teleop:**
```bash
cd P:/Workspace/aizee
python python/teleop/teleop.py --config config/teleop.yaml
```

**Controls:**
- **Left stick**: Drive rover (linear/angular)
- **Right stick**: Control arm (future implementation)
- **A button**: Enable all motors
- **B button**: Disable all motors
- **Back**: Emergency stop
- **Start**: Clear e-stop + faults

## Configuration Reference

### Rust Code Changes

The motor_control Rust binary now supports generic device configuration:

```rust
// Before (Jetson-specific)
struct NetworkConfig {
    jetson: JetsonConfig,
}

// After (generic device)
struct NetworkConfig {
    #[serde(alias = "jetson")]  // Backward compatible
    device: DeviceConfig,
}
```

### YAML Config Structure

Module-specific configs use the `device` section:

```yaml
network:
  device:
    ip: 192.168.0.28
    hostname: aizee-arm
    zmq:
      command_sub: "tcp://*:5557"
      telemetry_pub: "tcp://*:5558"
```

### Environment Variable

Use `AIZEE_CONFIG` to select module config:

```bash
# Rover module
AIZEE_CONFIG=config/hardware_jetson_rover.yaml ./rust/target/release/motor_control

# Arm module
AIZEE_CONFIG=config/hardware_rpi4_arm.yaml ./rust/target/release/motor_control
```

## Troubleshooting

### CAN Interface Not Found
```bash
# Check USB CAN adapter detected
lsusb
dmesg | grep -i can

# Manually configure
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### No Telemetry Received
```bash
# Check motor_control running
sudo systemctl status aizee-motor-control-arm

# Test telemetry directly
python3 -c "
import zmq, json
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('tcp://192.168.0.28:5558')
sub.setsockopt(zmq.SUBSCRIBE, b'')
print(json.loads(sub.recv_string()))
"
```

### Permission Denied on CAN
```bash
# Add user to dialout group
sudo usermod -a -G dialout pi
# Logout and login again
```

### Systemd Service Fails to Start
```bash
# Check logs
sudo journalctl -u aizee-motor-control-arm -n 50

# Common issues:
# - CAN interface not configured (check ExecStartPre)
# - Wrong config path (check AIZEE_CONFIG)
# - Binary not built (check cargo build --release)
```

## Network Latency Verification

```bash
# From dev machine to Jetson
ping -c 100 192.168.0.27

# From dev machine to RPi4
ping -c 100 192.168.0.28

# Expected: <2ms on local network
```

## Future: Adding Torso Module

Follow the same pattern:

1. Create `config/hardware_rpi4_torso.yaml`
2. Deploy servo control stack (separate from motor_control)
3. Use ZMQ ports :5559/:5560
4. Add to `config/teleop.yaml` modules section

The torso uses Feetech servos (serial protocol), so it requires a different control binary than the ROBSTRIDE CAN motors.

## Key Design Principles

1. **Fully Independent Modules**: No inter-module communication
2. **Separate CAN Buses**: Each module has isolated CAN interface
3. **Unique ZMQ Ports**: Prevents port conflicts
4. **Config-Driven**: `AIZEE_CONFIG` selects module behavior
5. **Shared Infrastructure**: POE + Ethernet only
6. **Unified Teleop**: Single interface controls all modules

## Safety Notes

- Emergency stop broadcasts to ALL modules
- Watchdog timeouts are per-module (100ms)
- Network failure on one module doesn't affect others
- CAN bus isolation prevents cross-module faults
