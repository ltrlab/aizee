# Camera System — Intel RealSense D455

Four Intel RealSense D455 cameras are deployed on dedicated Raspberry Pi 4 nodes connected via PoE Ethernet.

## Network Layout

| Camera   | Pi   | PoE IP      | Relay port on Jetson |
|----------|------|-------------|----------------------|
| cam_front | PI-1 | 10.42.0.11 | :5557 |
| cam_rear  | PI-2 | 10.42.0.12 | :5558 |
| cam_left  | PI-3 | 10.42.0.13 | :5559 |
| cam_right | PI-4 | 10.42.0.14 | :5560 |

Pis are only reachable from the dev machine **via Jetson hop** (PoE subnet is not directly routable). The Jetson runs `aizee-camera-relay` (`python/camera_relay.py`) which subscribes to each Pi's ZMQ stream on the PoE interface and re-publishes on all Jetson interfaces — so the dev machine connects to `tcp://192.168.0.27:5557-5560`.

## Camera Backend

Cameras are accessed via **OpenCV/V4L2** (not the RealSense SDK), because `pyrealsense2` v2.56.2 produced "bad optional access" errors on ARM64 Pi 4. V4L2 provides stable access to:

- **RGB stream**: `/dev/video4` — 640×480 JPEG @ ~2–5 fps (JPEG quality 20)
- **Infrared (depth proxy)**: `/dev/video2` — 640×480

No native depth or IMU data is available in the current implementation.

## Service Management

Services on each Pi start automatically at boot and auto-restart on failure.

```bash
# Start all 4 camera nodes (from dev machine)
./scripts/start_all_cameras.sh

# Stop all
./scripts/stop_all_cameras.sh

# Check a specific camera (via Jetson hop)
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 \
     'sudo systemctl status aizee-camera-cam_front'"

# View logs
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 \
     'sudo journalctl -u aizee-camera-cam_front -n 40'"
```

## Deploying Code Changes

Changes to `python/nodes/camera_node.py` or camera configs deploy through the Jetson:

```bash
./scripts/deploy_all_cameras.sh        # All 4 Pis
./scripts/deploy_rpi4_camera.sh cam_front  # Single Pi
```

## Camera Relay (Jetson)

`python/camera_relay.py` runs as `aizee-camera-relay` on the Jetson. It bridges Pi streams from the PoE subnet to the WiFi interface.

```bash
# Manage relay service on Jetson
sudo systemctl {start|stop|restart|status} aizee-camera-relay
sudo journalctl -u aizee-camera-relay -f
```

## Arm Cameras (Jetson, USB)

Two additional RealSense D435 cameras are mounted at the arm's end-effector and connected directly to the Jetson via USB. They run as separate services triggered by udev on USB plug/unplug:

- `aizee-arm-cam-left` → port :5563
- `aizee-arm-cam-right` → port :5564
- Configs: `config/hardware_jetson_arm_cam_{left,right}.yaml`
- Deploy: `./scripts/deploy_arm_cameras.sh`

If cameras are moved to different USB ports, update the `KERNEL==` values in `config/udev/99-aizee-realsense.rules` (find values with `udevadm info -a /dev/bus/usb/<BUS>/<DEV> | grep KERNEL`).

## Viewing Streams

```bash
# From dev machine — all rover cameras + arm cameras + LiDAR + UPS
python python/rerun_bridge.py \
    --cameras tcp://192.168.0.27:5557 tcp://192.168.0.27:5558 \
              tcp://192.168.0.27:5559 tcp://192.168.0.27:5560 \
              tcp://192.168.0.27:5563 tcp://192.168.0.27:5564 \
    --lidar tcp://192.168.0.27:5561 \
    --ups tcp://192.168.0.27:5562
```

Test all 4 rover streams:
```bash
./scripts/test_all_camera_streams.sh
```

## Troubleshooting

**Camera not streaming**:
```bash
# Check USB device on Pi
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'lsusb | grep Intel'"

# Check V4L2 devices
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'ls /dev/video*'"
```

**Relay not forwarding**:
```bash
sudo journalctl -u aizee-camera-relay -f  # on Jetson
```

**New Pi bootstrap**: run `./scripts/setup_pi_ethernet.sh <1-4>` to install the SSH key and set a static PoE IP.
