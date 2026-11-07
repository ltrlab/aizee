# LiDAR System

AIZEE supports **2× SLAMTEC RPLiDAR A1M8** sensors (front + back) for 360°
scanning. The Rust service `rust/lidar_control` reads both over USB serial and
publishes aggregated scans over ZMQ on `:5561`.

LiDAR is **optional**. The systemd unit `aizee-lidar-control` exists but is
**often left disabled** — enable it only when LiDAR is wired and needed.

| Sensor      | Device symlink         | Config field          |
|-------------|------------------------|-----------------------|
| lidar_front | `/dev/rplidar_front`   | `lidars[].id: lidar_front` |
| lidar_back  | `/dev/rplidar_back`    | `lidars[].id: lidar_back`  |

Symlinks are assigned by USB port location (both sensors share serial number
`0001`) via `config/udev/99-rplidar.rules`.

## Configuration

`config/hardware_jetson_rover.yaml`:

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

## Message Format

Published as a `TelemetryMessage` JSON on `:5561`. Each scan entry carries
`sensor_id`, angle bounds/increment, `range_min`/`range_max` (0.15–12.0 m),
and the `ranges` / `intensities` arrays:

```json
{
  "timestamp": 1707584123.456,
  "lidar_scans": [
    {
      "sensor_id": "lidar_front",
      "angle_min": 0.0,
      "angle_max": 6.283185,
      "angle_increment": 0.017453,
      "range_min": 0.15,
      "range_max": 12.0,
      "ranges": [1.23, 2.45, ...],
      "intensities": [255, 248, ...]
    }
  ]
}
```

## Architecture

```
RPLiDAR A1M8 ×2 ──USB serial──> udev (/dev/rplidar_front, /dev/rplidar_back)
                                   │
                          rust/lidar_control
                            ├─ LidarScanner (front)   spawn_blocking
                            ├─ LidarScanner (back)    spawn_blocking
                            └─ async main loop ──> ZMQ PUB tcp://*:5561
```

Each sensor runs in its own `spawn_blocking` thread for failure isolation; one
sensor failing does not crash the other, and USB disconnects trigger
reconnection. DTR must be cleared for the A1M8 motor to spin (handled by the
driver). Built on the `rplidar_drv` SDK with an async Tokio runtime. Module
layout: `main.rs` (runtime + ZMQ publisher), `scanner.rs` (`LidarScanner`,
hardware interface, reconnect).

## Building

```bash
cargo build --release -p lidar_control
# Binary: rust/target/release/lidar_control
```

## Deploying

```bash
# From the dev machine
./scripts/deploy_lidar_control.sh

# On the Jetson — install udev rules and the service
sudo bash ~/aizee/scripts/install_lidar_udev.sh
bash ~/aizee/scripts/install_lidar_service.sh
```

## Service Management

```bash
sudo systemctl {start|stop|restart|status} aizee-lidar-control
sudo systemctl {enable|disable} aizee-lidar-control   # often disabled
sudo journalctl -u aizee-lidar-control -f
```

The unit checks that `/dev/rplidar_front` and `/dev/rplidar_back` exist before
starting, waits ~2 s for motor control to stabilize, and restarts on failure.

## Manual Run (development)

```bash
# On the Jetson
AIZEE_CONFIG=~/aizee/config/hardware_jetson_rover.yaml \
RUST_LOG=info \
./rust/target/release/lidar_control
```

## Viewing Scans

```bash
# From the dev machine — point cloud in Rerun
python python/rerun_bridge.py --lidar tcp://192.168.0.27:5561

# Combined with cameras / telemetry
python python/rerun_bridge.py \
    --cameras tcp://192.168.0.27:5563 tcp://192.168.0.27:5564 \
    --lidar tcp://192.168.0.27:5561
```

Quick subscribe check in Python:

```python
import zmq
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://192.168.0.27:5561")
sub.subscribe(b"")
msg = sub.recv_json()
print(f"Received {len(msg['lidar_scans'])} scans")
```

## Performance

- Scan rate: ~5.5 Hz (native A1M8 rate); published at 5 Hz (200 ms aggregation)
- ~350–365 points per scan; range 0.15–12.0 m
- CPU < 5% per sensor, ~8–12 MB memory

## References

- SLAMTEC rplidar_sdk: https://github.com/Slamtec/rplidar_sdk
- `rust/lidar_control/README.md` — crate-level details
