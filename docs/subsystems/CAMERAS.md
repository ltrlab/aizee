# Camera System

The arm carries two cameras, both connected directly to the Jetson via USB and
published over ZMQ:

| Camera      | Sensor           | Node                       | Config                                   | Port  |
|-------------|------------------|----------------------------|------------------------------------------|-------|
| gripper_cam | ELP-USBFHD01M-L21 (UVC) | `python/nodes/gripper_camera_node.py` | `config/hardware_jetson_gripper_cam.yaml` | :5563 (color), :5573 (ctrl REP) |
| scene_cam   | Intel RealSense  | `python/nodes/camera_node.py`         | `config/hardware_jetson_scene_cam.yaml`   | :5564 |

> History: the gripper camera replaced a stereo Intel RealSense **D435 wrist pair**
> (`arm_cam_left` / `arm_cam_right`, retired 2026-05-13), and the four PoE Raspberry Pi 4
> **D455 rover surround cameras** + `camera_relay` were retired when the arm was separated
> from the rover (2026-05-30). The scene camera was added 2026-06-03.

## Service Management

Both cameras run as systemd services on the Jetson, triggered by udev on USB plug/unplug:

- `aizee-gripper-cam` (udev: `config/udev/99-aizee-gripper-cam.rules` → `/dev/aizee_gripper_cam`)
- `aizee-scene-cam`   (udev: `config/udev/99-aizee-scene-cam.rules`   → `/dev/aizee_scene_cam`)

```bash
# On the Jetson
sudo systemctl {start|stop|restart|status} aizee-gripper-cam
sudo systemctl {start|stop|restart|status} aizee-scene-cam
sudo journalctl -u aizee-gripper-cam -f
```

## Deploying Code Changes

```bash
./scripts/deploy_gripper_camera.sh        # gripper cam node + config + udev + service
./scripts/deploy_scene_cam.sh             # scene cam node + config + udev + service
```

`deploy_scene_cam.sh` finishes by running `scripts/test_realsense.py` to confirm the
RealSense is enumerated and streaming.

## Viewing Streams

```bash
# From the dev machine — gripper + scene cameras + telemetry + UPS
python python/rerun_bridge.py \
    --cameras tcp://192.168.0.27:5563 tcp://192.168.0.27:5564 \
    --telemetry tcp://192.168.0.27:5556 \
    --ups tcp://192.168.0.27:5562
```

`collect_demo.py` receives both cameras directly (see `--gripper-cam` / `--scene-cam`).

## Troubleshooting

```bash
# Confirm the USB symlinks exist on the Jetson
ls -l /dev/aizee_gripper_cam /dev/aizee_scene_cam

# Confirm the RealSense scene cam is enumerated and streaming
python scripts/test_realsense.py --config config/hardware_jetson_scene_cam.yaml
```

If a camera moves to a different USB port and the symlink disappears, update the matching
udev rule (`99-aizee-gripper-cam.rules` / `99-aizee-scene-cam.rules`) and replug.
