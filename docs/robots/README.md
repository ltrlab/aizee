# The robots

A robot here is a set of files the shared code selects. Nothing in `control/`, `data/` or `helm/` is meant to know a
robot by anything but its definition, though Helm's workflow is Minerva's so far.

| | Minerva | AIZEE |
| --- | --- | --- |
| Definition (joints, cameras, capabilities) | `src/robot_platform/robots/definitions/minerva.json` | `src/robot_platform/robots/definitions/aizee.json` |
| Deployment settings | `src/robot_platform/settings/defaults/minerva.json` | `src/robot_platform/settings/defaults/aizee.json` |
| Recording format | `data/formats/minerva.py`; Helm's task capture, [format 9](../data/EPISODE_FORMAT.md) | `data/formats/aizee.py`; versions 4 and 5 |
| Control | `rust/control_core`, the guarded receiver and head controller | `rust/motor_control`, over ZeroMQ |
| Operator tool | [Helm](../operator/HELM_MANUAL.md) | `python/scripts/collect_demo.py` |
| Training | `scripts/training_server/` (LeRobot ACT), `python/training/train_minerva.py` | `python/training/train.py` |
| Hardware files | `config/devices/minerva-*.json`, `config/hardware_minerva_*.yaml` | `config/hardware_jetson_*.yaml` |
| Guides | [Hardware](minerva/HARDWARE.md) and the [install guides](../getting-started/INSTALL_JETSON.md) | [AIZEE](aizee/README.md) |

Ask the package for what it knows:

```sh
robot-platform list
robot-platform describe --robot minerva
robot-platform config --robot aizee --settings settings/examples/usb-tether.json
```

The two share the ports of the older motor service, so they are alternate deployments on one computer, not two robots
on one Jetson.

## Adding a robot

1. Write its definition in `robots/definitions/`, or pass a JSON path as `--robot`. `load_robot` checks it.
2. Add its model, so the viewer can show it: [Model viewer](../simulation/MODEL_VIEWER.md).
3. Write its settings in `settings/defaults/`, with the endpoints it serves.
4. If it records in a new shape, add the format in `data/formats/`.
5. Give each physical unit its own profile: [A new unit's profile](../getting-started/NEW_UNIT_PROFILE.md).
