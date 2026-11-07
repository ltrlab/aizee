# Minerva hardware

What Minerva is made of, by joint and by bus, with the ports its software uses. Only facts from the repository's
configuration, code or recorded measurements are here; where a value is not recorded the page says so. The values
below describe the unit the repository was built on ([NEW_UNIT_PROFILE.md](../../getting-started/NEW_UNIT_PROFILE.md)).
The device log of that unit, with its serials and its bring-up history, is [MINERVA_PRODUCTION_01.md](../../devices/MINERVA_PRODUCTION_01.md).

Minerva is a two-arm torso: two 6-joint arms with parallel grippers, a two-joint Feetech neck carrying a RealSense
camera, and a wrist camera on each gripper. It runs on a Jetson and is operated through [Helm](../../architecture/HELM_ON_JETSON.md).
It has no powered base. The joint order is in `src/robot_platform/robots/definitions/minerva.json`.

## Joints and motors

Each arm has seven RobStride motors, CAN ids 4 to 10, on its own bus. The two arms use the same ids and are told apart
only by their bus.

| Joint (each arm) | CAN id | Motor | Notes |
| --- | --- | --- | --- |
| `*_arm_j1` | 4 | RS03 | Shoulder swivel |
| `*_arm_j2` | 5 | RS04 | Shoulder hinge |
| `*_arm_j3` | 6 | RS03 | |
| `*_arm_j4` | 7 | RS02 | |
| `*_arm_j5` | 8 | RS02 | Wrist |
| `*_arm_j6` | 9 | RS00 | Wrist roll |
| `*_gripper` | 10 | RS00 | Sliding jaws; the motor angle, not the jaw position, is the joint |

| Side | Bus | Adapter |
| --- | --- | --- |
| Left | `canL` | USB CAN adapter, named by its USB serial in `config/udev/80-minerva-can.rules` |
| Right | `canR` | the same |

The models above were read off the physical labels (`config/devices/minerva-production-01.json`, `motor_models_verified`).
The older `config/hardware_minerva_{left,right}.yaml` still mark the id-4 shoulder as assumed; the device file
supersedes them. The adapters are Jhoinrch RH-02 (classic CAN) running the `gs_usb` driver; the buses run at 1 Mbit/s
(`scripts/aizee-can-up`). Motor UIDs and firmware versions are recorded in the mechanical reference file, per unit.
Vendor manuals for the four models are in `docs/vendor/`.

The software joints are `left_arm_j1`-`j6`, `left_gripper`, the same on the right, `head_pan`, `head_tilt`, and
`lift`. **`lift` is inactive**: the definition lists it, but no receiver drives it
([firmware/lift_actuator/README.md](../../../firmware/lift_actuator/README.md) is a bench design that has never run on
hardware). Episodes (format 9) carry 16 axes: both arms with grippers, `head_pan` and `head_tilt`
([EPISODE_FORMAT.md](../../data/EPISODE_FORMAT.md)); format 9 has not yet been recorded on the robot.

## Neck

Two Feetech STS3250 servos (model number 2825) on a Waveshare bus servo adapter, on a USB serial port of the Jetson
(pinned by its `/dev/serial/by-id` path in `config/devices/minerva-head.json`), at 1 Mbit/s.

| Joint | Servo id | CAD joint | Role |
| --- | --- | --- | --- |
| `head_pan` | 1 | `neck_z` | Turns the head |
| `head_tilt` | 2 | `neck_y` | Tilts the head |

The head has its own receiver process, `head_controller`. The servos take a position goal and hold it; they keep no
command watchdog. The servos run on 12 V. Details: [NECK_FEETECH.md](NECK_FEETECH.md) and the model in
[HEAD_NECK_MODEL.md](HEAD_NECK_MODEL.md).

## Cameras

| Role | Camera | Host connection | Stream | Control |
| --- | --- | --- | --- | --- |
| `left_wrist` | USB UVC colour camera, ELP, USB id `05a3:9230` | Jetson USB | ZMQ `5563` | `5573` |
| `right_wrist` | USB UVC colour camera, "HD USB Camera", USB id `32e4:9230` | Jetson USB | ZMQ `5565` | `5574` |
| `head` | Intel RealSense D435, colour and depth | Jetson USB 3, on the neck | ZMQ `5564` | none |

The wrist cameras publish 640 x 480 MJPEG at 30 fps, and the head camera's colour 640 x 480 at 15 fps (reduced from
30 for stability on the Jetson's USB controller). Recording stores 320 x 240 RGB. The head camera is the unit
`scene_cam` in the camera configuration, and its service is called `aizee-scene-cam`. The cameras' mounts and
intrinsics are measured per unit ([WRIST_CAMERA_CALIBRATION.md](../../operator/WRIST_CAMERA_CALIBRATION.md),
[HEAD_CAMERA_ORIENTATION.md](../../operator/HEAD_CAMERA_ORIENTATION.md)).

## Computer, power and stop

- **Computer:** NVIDIA Jetson Orin Nano developer kit (Super), JetPack 6 (Ubuntu 22.04, L4T R36, kernel 5.15).
- **UPS:** Waveshare UPS Power Module (C) with a 3S lithium pack of 18650 cells, read through an INA219 on I2C bus 7
  at address 0x41. The 28 Wh capacity and 0.2 ohm pack resistance in `config/devices/minerva-ups.json` are assumed
  until a logged discharge measures them. See [UPS_MONITOR.md](UPS_MONITOR.md).
- **Arm motor power:** a foot switch cuts the arms' motor power. The supply's voltage and current rating are not
  recorded in the repository. No part of the software can cut it (`programmatic_power_cutoff: false`), and the arms
  are not held when it is cut: the device profile records that they may fall on the test stand.
- **Head supply:** the neck's 12 V is **not** cut by the foot switch. Disable (H) in Helm cuts the head's torque.
- **Stops:** the foot switch, Helm's Disable and Shutdown, and the pedals are described in
  [SAFETY.md](../../operator/SAFETY.md). The ESP-NOW wireless stop bridge was retired and no replacement is installed.

The power budget, wiring harness and a bill of materials are not documented in the repository.

## Leader arms

The operator's two GELLO leaders, 7 Dynamixel XL330 servos each on an OpenRB-150 board, are not on the robot. They
plug into the operator station: [OPENRB_LEADER.md](../OPENRB_LEADER.md).

## Network ports

Ports the software on and around the Jetson uses. "Loopback" means bound to `127.0.0.1` on the Jetson only.

| Port | Proto | Bound | What | Where it is set |
| --- | --- | --- | --- | --- |
| 22 | TCP | all | SSH. Helm, the deploy and Check use pinned SSH; Helm uses it to `127.0.0.1` | system sshd |
| 8760 | TCP, HTTPS | all | Helm for the operator's devices (login required); the headset page is `/xr` | `ops/helm_service.py` `PORT` |
| 8762 | TCP, HTTP | loopback | Helm for the robot's own tools and SSH tunnels (login still required) | `LOOPBACK_PORT` |
| 8761 | TCP | loopback | The GELLO bridge's relay joins Helm here | `helm/bridge.py` `PORT` |
| 5590 | ZMQ | loopback | The arms' receiver (`guarded_controller`) | `control/live_receiver.py` |
| 5591 | ZMQ | loopback | The head's receiver (`head_controller`) | `config/devices/minerva-head.json`, `live.endpoint` |
| 5596 | ZMQ | loopback | The head pose feed (head angles and the camera's pose) | `neck/head_pose.py` |
| 5563, 5565, 5564 | ZMQ PUB | all | Camera streams: left wrist, right wrist, head | `config/hardware_minerva_*_wrist.yaml`, `hardware_jetson_scene_cam.yaml` |
| 5573, 5574 | ZMQ REP | all | Left and right wrist cameras' runtime V4L2 controls | the same YAML files |
| UDP, a port chosen per session | UDP | the operator PC | The leader link: the robot's loop requests the latest signed leader sample from the PC every cycle | `teleop/leader_link.py` |
| 8088 | TCP, HTTP | all | `aizee-heartbeat` status dashboard, an AIZEE-era service some units still run | `config/systemd/aizee-heartbeat.service` |
| 8765, 8766 | TCP, HTTP | operator PC | Model viewer (`robot-platform view`) and the joint-calibration viewer (`scripts/calibrate_minerva.ps1`) | `viewer/cli.py`, `control/joint_calibration.py` |

Retired or AIZEE ports that still appear in defaults:

| Port | What |
| --- | --- |
| 5555/5556, 5575/5576 | The old per-arm `motor_control` services (`aizee-minerva-left`, `aizee-minerva-right`: command and telemetry). Nothing runs them now; `src/robot_platform/settings/defaults/minerva.json` and the legacy collector still name them |
| 5562 | AIZEE's UPS publisher. Minerva's `minerva-ups` publishes no socket; Helm reads its state file over SSH |

Camera streams and controls are bound to every interface, not only loopback; the recorder and Helm read them on
loopback. The tether and the access point are the networks the robot is meant to sit on
([MINERVA_NETWORK.md](../../architecture/MINERVA_NETWORK.md)).
