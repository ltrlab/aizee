# Camera System

AIZEE runs up to two cameras, both connected directly to the Jetson via USB and
published over ZMQ. The gripper camera is the **primary observation** and is
always present; the scene camera is **optional** and auto-detected at runtime.

| Camera      | Sensor                  | Node                                  | Config                                    | ZMQ ports                          |
|-------------|-------------------------|---------------------------------------|-------------------------------------------|------------------------------------|
| gripper_cam | ELP-USBFHD01M-L21 (USB UVC) | `python/nodes/gripper_camera_node.py` | `config/hardware_jetson_gripper_cam.yaml` | PUB `:5563`, control REP `:5573`   |
| scene_cam   | Intel RealSense D435 / D435i / D455 | `python/nodes/camera_node.py`         | `config/hardware_jetson_scene_cam.yaml`   | PUB `:5564`                        |

## Stream Details

**gripper_cam** — color only, captured at **1024x768 @ 30 fps** MJPG, JPEG-encoded
(quality 85) and published with no publisher-side downscale, so capture, wire,
recording, training, and inference all share the same resolution. V4L2 controls
(manual exposure, gain, etc.) are set from the config after the device opens and
can be changed at runtime over the control REP socket on `:5573` (GUI sliders).

**scene_cam** — color + depth at **640x480 @ 15 fps** (15 fps is a stability
workaround for the Jetson Orin's USB controller under sustained RealSense load).
Color is JPEG-encoded; depth is Z16. Depth intrinsics and scale are included in
the message header for point-cloud reconstruction. IMU is disabled by default.

Presence of the scene cam selects the operating mode for `collect_demo.py`:

- **scene cam present → static mode** — arm on a fixed stand; the scene view is
  recorded (episode bumped to HDF5 v5).
- **scene cam absent → rover mode** — gripper cam only; scene cam is hidden and
  not required.

## Service Management

Both cameras run as systemd services on the Jetson, triggered by udev on USB
plug/unplug:

- `aizee-gripper-cam` (udev: `config/udev/99-aizee-gripper-cam.rules` → `/dev/aizee_gripper_cam`)
- `aizee-scene-cam`   (udev: `config/udev/99-aizee-scene-cam.rules`   → `/dev/aizee_scene_cam`)

```bash
# On the Jetson
sudo systemctl {start|stop|restart|status} aizee-gripper-cam
sudo systemctl {start|stop|restart|status} aizee-scene-cam
sudo journalctl -u aizee-gripper-cam -f
```

The scene-cam node deliberately exits non-zero (rather than reinitializing
in-process) on a RealSense frame error; systemd's `Restart=on-failure` then
restarts it cleanly, avoiding a librealsense zombie-thread leak.

## Deploying Code Changes

```bash
./scripts/deploy_gripper_camera.sh        # gripper cam node + config + udev + service
./scripts/deploy_scene_cam.sh             # scene cam node + config + udev + service
```

`deploy_scene_cam.sh` finishes by running `scripts/test_realsense.py` to confirm
the RealSense is enumerated and streaming.

## Viewing Streams

```bash
# From the dev machine — gripper + scene cameras + telemetry + UPS
python python/rerun_bridge.py \
    --cameras tcp://192.168.0.27:5563 tcp://192.168.0.27:5564 \
    --telemetry tcp://192.168.0.27:5556 \
    --ups tcp://192.168.0.27:5562
```

`python/scripts/collect_demo.py` subscribes to both cameras directly via
`--gripper-cam` / `--gripper-cam-ctrl` / `--scene-cam` (defaults point at the
Jetson at `192.168.0.27`). Passing an empty `--scene-cam` disables scene-cam
subscribe/record/preview.

## Troubleshooting

```bash
# Confirm the USB symlinks exist on the Jetson
ls -l /dev/aizee_gripper_cam /dev/aizee_scene_cam

# Confirm the RealSense scene cam is enumerated and streaming
python scripts/test_realsense.py --config config/hardware_jetson_scene_cam.yaml
```

If a camera moves to a different USB port and the symlink disappears, update the
matching udev rule (`99-aizee-gripper-cam.rules` / `99-aizee-scene-cam.rules`)
and replug.

> History: the gripper camera replaced a stereo Intel RealSense **D435 wrist
> pair** (`arm_cam_left` / `arm_cam_right`, retired 2026-05-13), and the four
> PoE Raspberry Pi 4 **D455 rover surround cameras** + `camera_relay` were
> retired when the arm was separated from the rover (2026-05-30). The scene
> camera was added 2026-06-03. Neither retired system is in use.
