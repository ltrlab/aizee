# AIZEE Motor Control

Low-level CAN bus motor control for ROBSTRIDE actuators with deterministic control loops.

## Features

- **ROBSTRIDE Protocol**: Complete implementation of RS03-EN CAN protocol
- **Deterministic Control**: 1kHz arm control loop, 100Hz base control loop
- **Safety Features**: Watchdog timeout, soft limits, emergency stop
- **ZeroMQ Interface**: JSON commands over pub/sub sockets
- **Real-time Telemetry**: 50Hz motor state publishing

## Building

```bash
cd rust/motor_control
cargo build --release
```

## CAN Interface Setup

### For CANable/slcan Adapters

```bash
# Load slcan kernel module
sudo modprobe slcan

# Attach slcan device (replace /dev/ttyACM0 with your adapter)
sudo slcand -o -c -s8 /dev/ttyACM0 can0

# Bring up the interface
sudo ip link set can0 up

# Verify
ip link show can0
```

### For SocketCAN Native Interfaces

On the Jetson Orin Nano (production), motors are on `can1`:

```bash
# Set bitrate to 1 Mbps
sudo ip link set can1 type can bitrate 1000000

# Bring up the interface
sudo ip link set can1 up

# Verify
ip link show can1
```

## Running

```bash
# Set config path (optional, defaults to config/hardware.yaml)
export AIZEE_CONFIG=config/hardware.yaml

# Run motor controller
./target/release/motor_control
```

## Configuration

Edit `config/hardware.yaml` to match your hardware:

```yaml
motors:
  wheels:
    - id: left_wheel
      can_id: 0x01
      type: ROBSTRIDE04
      # ...

can:
  interface: can1  # Jetson production: can1; RPi4 arm module: can0

network:
  device:  # accepts "jetson" as alias for backward compatibility
    zmq:
      command_sub: "tcp://*:5555"
      telemetry_pub: "tcp://*:5556"
```

## Command Interface

Send JSON commands to `tcp://localhost:5555`:

### Enable Motors
```json
{
  "type": "enable",
  "motor_ids": ["left_wheel", "right_wheel"]
}
```

### Drive Base
```json
{
  "type": "drive",
  "linear": 0.5,
  "angular": 0.2
}
```

### Move Arm
```json
{
  "type": "arm_joints",
  "positions": [0.1, 0.5, -0.3],
  "velocities": [0.0, 0.0, 0.0]
}
```

### Emergency Stop
```json
{
  "type": "emergency_stop"
}
```

## Telemetry Format

Receive JSON telemetry from `tcp://localhost:5556`:

```json
{
  "timestamp": 1234567890.123,
  "motors": {
    "left_wheel": {
      "position": 1.5,
      "velocity": 0.5,
      "torque": 2.1,
      "temperature": 45.0,
      "error": null
    }
  }
}
```

## Testing

Use the Python test script:

```bash
# Interactive mode
python python/teleop/simple_test.py

# Automated test sequence
python python/teleop/simple_test.py auto
```

## Troubleshooting

### CAN Interface Not Found

```bash
# Check available interfaces
ip link show

# For slcan adapters, check USB connection
lsusb
ls /dev/ttyACM*
```

### Permission Denied

```bash
# Add user to dialout group (for USB-CAN adapters)
sudo usermod -a -G dialout $USER

# Or run with sudo (not recommended for production)
sudo ./target/release/motor_control
```

### Motor Not Responding

1. Check CAN wiring (CAN-H, CAN-L, 120Ω termination)
2. Verify motor power (40V supply connected)
3. Check motor CAN ID matches config
4. Use `candump can1` to see CAN traffic (Jetson) or `candump can0` (RPi4 arm module)
5. Test with single motor first

### High CPU Usage

The 1kHz control loop is CPU-intensive. Ensure:
- Running on performance governor: `sudo cpufreq-set -g performance`
- No other heavy processes running
- Consider lowering arm_frequency in config if needed

## Architecture

```
┌─────────────────────┐
│  ZeroMQ Commands    │  (TCP:5555)
│  from Teleop        │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────────────────┐
│  Motor Control (main.rs)        │
│  ┌──────────────────────────┐  │
│  │  Command Handler         │  │
│  └──────────┬───────────────┘  │
│             │                   │
│  ┌──────────▼───────────────┐  │
│  │  Safety Monitor          │  │
│  │  - Watchdog              │  │
│  │  - Soft Limits           │  │
│  │  - Error Detection       │  │
│  └──────────┬───────────────┘  │
│             │                   │
│  ┌──────────▼───────────────┐  │
│  │  Control Loop            │  │
│  │  - Arm: 1kHz             │  │
│  │  - Base: 100Hz           │  │
│  └──────────┬───────────────┘  │
│             │                   │
│  ┌──────────▼───────────────┐  │
│  │  ROBSTRIDE Protocol      │  │
│  │  (robstride.rs)          │  │
│  └──────────┬───────────────┘  │
└─────────────┼───────────────────┘
              │
              ▼
        ┌──────────┐
        │  CAN Bus │  (SocketCAN)
        └──────────┘
              │
        ┌─────┴──────┐
        ▼            ▼
    [Motor 1]   [Motor 2] ...
```

## Protocol Details

Based on ROBSTRIDE RS03-EN specification:

- **CAN ID Format**: Extended 29-bit
  - Bits 0-7: Motor ID
  - Bits 8-15: Host ID (0xAA)
  - Bits 16-20: Error flags
  - Bits 24-28: Message type

- **Message Types**:
  - 0: Info
  - 1: Control (position/velocity/torque)
  - 2: Feedback
  - 3: Enable
  - 4: Disable
  - 6: Zero Position
  - 17: Read Parameter
  - 18: Write Parameter

- **Feedback Format** (8 bytes):
  - Bytes 0-1: Position (uint16, -4π to 4π)
  - Bytes 2-3: Velocity (uint16, model-dependent range)
  - Bytes 4-5: Torque (uint16, model-dependent range)
  - Bytes 6-7: Temperature (uint16, °C × 10)

See `robstride.rs` for complete implementation.

## License

MIT License - see top-level LICENSE file.
