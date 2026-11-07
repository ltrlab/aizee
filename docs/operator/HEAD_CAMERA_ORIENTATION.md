# Head camera image orientation and alignment

The head camera is the Intel RealSense D435 on the neck, published by the `aizee-scene-cam` service (the `aizee-` name is
Minerva's). Its pose per recorded frame comes from the neck's angles and the camera's mount; Helm measures that mount on the
table's tags (Teleop > Calibrate > **2 Head camera**, [Helm manual](HELM_MANUAL.md#calibrate-mode)), which writes
`<asset>-head-camera.json` beside the head's other operator files, with the same solver as the
[wrist cameras](WRIST_CAMERA_CALIBRATION.md). The recorded pose is Helm's saved alignment when there is one, else the CAD mount.
No document records an alignment solved on the robot. The rest of this page is the image-orientation correction.

## Orientation

`python/nodes/camera_node.py` accepts `camera.image_rotation_deg: 0` or `180`
in its YAML configuration, or `--image-rotation-deg` without `--config`.
The default is zero. The generic scene-camera configuration remains unchanged.
The D435 was first mounted upside down on the static neck, so its deployed
configuration on the Jetson (`~/aizee/config/hardware_jetson_scene_cam.yaml`, not this
repository's copy) set `camera.image_rotation_deg: 180`. On the head the camera sits
upright, so that setting is `0`.
No wrist-camera configuration changes are required.

The publisher applies the same half-turn to RGB and native Z16 depth before
JPEG encoding and depth serialization. Each native grid keeps its own dimensions;
optional RGB downscaling still happens afterward. Depth is not registered to RGB
by this change, and the two sensors' existing extrinsic relationship is unchanged.
The packet's `image_rotation_deg` describes the applied raster correction. Helm
retains it in camera status and each recorded frame's `task_state.cameras`
provenance. Missing metadata from an older publisher stays unknown (`null`).

Depth intrinsics use the original sensor optical frame (+X right, +Y down,
+Z forward), explicitly labelled `depth.intrinsics_frame: sensor_optical`.
For a half-turn, the publisher emits `fx'=-fx`, `fy'=-fy`,
`cx'=width-1-cx`, and `cy'=height-1-cy`. This is the pixel transform `K'=A*K`:
`(u'-cx')/fx'` and `(v'-cy')/fy'` still produce the original sensor-frame ray.
The existing depth decoder therefore produces the same camera-frame pointcloud.
Consumers that require positive focal lengths must explicitly transform the
optical frame; they must not silently replace these signs or reapply the rotation.
IMU vectors and camera mounting extrinsics are unchanged. The static-neck mount
has not been calibrated, and image correction supplies no new calibration.

Offline tests exercise the actual publisher with a stub SDK, asymmetric color
and depth data, multipart serialization, optional color downscaling, the existing
depth decoder, configuration/CLI validation, and capture metadata admission.
They do not connect to a camera or establish deployed image orientation.

```sh
python -m pytest tests/unit/test_camera_node_orientation.py tests/unit/test_helm_task_capture.py tests/contract/test_shared_wire.py -q
```
