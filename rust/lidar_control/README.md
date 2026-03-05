# lidar_control - RPLiDAR A1M8 Interface

Rust service for interfacing with RPLiDAR A1M8 sensors on the AIZEE rover.

## Overview

This crate provides a standalone service that:
- Interfaces with two RPLiDAR A1M8 sensors via USB serial
- Publishes scan data via ZeroMQ on port 5561
- Runs independently from motor control for failure isolation
- Supports automatic reconnection on USB disconnect
- Uses async Tokio with `spawn_blocking` for serial I/O

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  lidar_control                      │
│                                                     │
│  ┌──────────────┐      ┌──────────────┐           │
│  │ LidarScanner │      │ LidarScanner │           │
│  │   (front)    │      │   (back)     │           │
│  └──────┬───────┘      └──────┬───────┘           │
│         │                     │                    │
│         │ spawn_blocking      │ spawn_blocking     │
│         │                     │                    │
│         └─────────┬───────────┘                    │
│                   │                                │
│            mpsc::channel                           │
│                   │                                │
│                   ▼                                │
│          ┌─────────────────┐                       │
│          │  Main Loop      │                       │
│          │  (async tokio)  │                       │
│          └────────┬────────┘                       │
│                   │                                │
│                   ▼                                │
│          ┌─────────────────┐                       │
│          │   ZMQ PUB       │                       │
│          │   tcp://*:5561  │                       │
│          └─────────────────┘                       │
└─────────────────────────────────────────────────────┘
```

## Message Format

Published as `TelemetryMessage` JSON on ZMQ port 5561:

```json
{
  "timestamp": 1707584123.456,
  "motors": {},
  "battery_voltage": null,
  "lidar_scans": [
    {
      "sensor_id": "lidar_front",
      "angle_min": 0.0,
      "angle_max": 6.283185,
      "angle_increment": 0.017453,
      "range_min": 0.15,
      "range_max": 12.0,
      "ranges": [1.23, 2.45, 3.67, ...],
      "intensities": [255, 248, 250, ...]
    },
    {
      "sensor_id": "lidar_back",
      "angle_min": 0.0,
      "angle_max": 6.283185,
      "angle_increment": 0.017453,
      "range_min": 0.15,
      "range_max": 12.0,
      "ranges": [0.89, 1.56, 2.34, ...],
      "intensities": [252, 255, 249, ...]
    }
  ]
}
```

## Configuration

Configured via `config/hardware_jetson_rover.yaml`:

```yaml
lidars:
  - id: lidar_front
    device: /dev/rplidar_front
    scan_mode: standard

  - id: lidar_back
    device: /dev/rplidar_back
    scan_mode: standard

network:
  device:
    zmq:
      lidar_pub: "tcp://*:5561"
```

## Building

```bash
# Build release binary
cargo build --release -p lidar_control

# Binary location
./target/release/lidar_control
```

## Running

```bash
# Set config path
export AIZEE_CONFIG=/path/to/config/hardware_jetson_rover.yaml

# Set log level (optional)
export RUST_LOG=info

# Run
./target/release/lidar_control
```

## Dependencies

- **rplidar_drv** (0.6): RPLiDAR SDK bindings
- **serialport** (4.2): Cross-platform serial port access
- **tokio**: Async runtime
- **zmq**: ZeroMQ bindings
- **comms**: Internal crate (message definitions)

## Error Handling

The service is designed for robustness:

1. **Independent sensors**: One sensor failing doesn't crash the other
2. **Auto-reconnect**: USB disconnect triggers reconnection attempts
3. **Graceful degradation**: Continues operating with one sensor if other fails
4. **Blocking I/O isolation**: Serial I/O runs in `spawn_blocking` threads

## Testing

Subscribe to telemetry with Python:

```python
import zmq

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://192.168.0.27:5561")
sub.subscribe(b"")

msg = sub.recv_json()
print(f"Received {len(msg['lidar_scans'])} scans")
```

## Deployment

See `docs/subsystems/LIDAR.md` for complete deployment guide.

## Performance

- **Scan rate**: ~5.5Hz (natural RPLiDAR A1M8 rate)
- **Publish rate**: 5Hz (200ms interval)
- **Points per scan**: 350-365
- **CPU usage**: <5% per sensor
- **Memory**: ~8-12 MB

## Module Structure

- `main.rs`: Tokio runtime, ZMQ publisher, main control loop
- `scanner.rs`: `LidarScanner` struct, hardware interface, reconnect logic
