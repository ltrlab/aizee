# Aizee / Minerva robotics platform

One Python package (`robot_platform`) and one root Cargo workspace serve two
robots: Aizee, a mobile manipulator, and Minerva, a dual-arm torso. Robot
definitions, deployment settings, and application adapters are selected
independently.

## Start here

Run these commands from the repository root in a Python 3.10+ environment:

```sh
python -m pip install -e ".[operator,simulation,dev]"
robot-platform list
robot-platform run collect --robot minerva --settings settings/examples/usb-tether.json --plan
robot-platform run collect --robot aizee --settings settings/examples/usb-tether.json --plan
```

`--plan` prints the resolved launch without starting the application or opening
hardware. Remove it to launch the selected collector. Use `python -m robot_platform`
if the console command is not on PATH. Applications still need this source
checkout; the shared libraries and profiles are packaged.

### Simulate without hardware

```sh
robot-platform sim run --robot minerva --output simulation_runs/minerva-001
# In another terminal, use that run's generated settings:
robot-platform run collect --robot minerva --settings simulation_runs/minerva-001/settings.json
```

Use `--robot aizee` in both commands for Aizee. Every run needs a new output
directory. Motors start disabled; simulation uses loopback endpoints and
suppresses physical leader/serial discovery. See the
[simulation guide](docs/architecture/SIMULATION_IMPLEMENTATION.md) for faults,
recording, trace reports, and current limitations.

### Train

```sh
python -m pip install -e ".[learning]"
robot-platform run train --robot minerva --plan
robot-platform run train --robot aizee --plan
```

Default data/output paths are `episodes/<robot>` and `checkpoints/<robot>`.
Select older datasets explicitly with `--set applications.train.data_dir=PATH`.
Old local checkpoints were removed during cleanup; train or supply a compatible
checkpoint before inference. No pretrained weights ship with the repository.
On Jetson, preserve the NVIDIA/JetPack-specific Torch installation.

## Repository map

```text
pyproject.toml                Python package, dependency extras, CLI, offline tests
Cargo.toml / Cargo.lock      Shared Rust workspace and locked dependencies
src/robot_platform/
  robots/                   Aizee / Minerva joint and camera definitions
  settings/                 Packaged defaults and overlay validation
  control/                  Shared sessions, freshness, command leases, e-stop
  transport/                MessagePack, ZMQ, discovery
  data/                     Recording formats, provenance, measurement traces
  simulation/               Synthetic plant, loopback server, fault scenarios
  characterization/         Protocol/config audit and bench-profile validation
  apps/                     Collection/training launch adapters
rust/                       comms, motor_control, lidar_control
python/                     Active collector UIs, learning, devices, IK, tools
settings/examples/          Operator/machine overlays and bench-profile template
config/                     Hardware profiles, calibration, systemd/udev
firmware/                   Leaders, e-stop, display, lift; required vendor BSP
urdf/aizee/                 Existing Onshape export and Git LFS meshes
scripts/                    Build, deploy, inventory, simulation validation
tests/                      Offline tests, fixtures, explicit hardware tools
docs/                       Current guides, plans, subsystem references
```

The [detailed map](docs/architecture/REPOSITORY_MAP.md) and
[configuration guide](config/README.md) explain ownership and remaining migration
boundaries. `python/` contains active application code and compatibility shims.

## Robots and deployment

| Robot | Active control | Controller profile | Command / telemetry |
|---|---|---|---|
| Aizee | Six arm joints, gripper, separate wheel drive | `config/hardware_jetson_rover.yaml` | `5555 / 5556` |
| Minerva left | Six arm joints and gripper | `config/hardware_minerva_left.yaml` | `5555 / 5556` |
| Minerva right | Six arm joints and gripper | `config/hardware_minerva_right.yaml` | `5575 / 5576` |

Minerva's head pan/tilt and lift are reserved, inactive slots in its 17-slot
recording/model schema. Wrist cameras use ports 5563/5565; scene/head uses 5564.
Shared settings select endpoints explicitly. Direct older tools may still use
network discovery. Aizee and Minerva motor services share ports and are alternate
deployments on a single host.

Build Rust services from the repository root on Linux/Jetson:

```sh
cargo build --locked --release -p motor_control -p lidar_control
```

Windows ARM64 cross-builds use `scripts/build_jetson.ps1`; see the
[build guide](rust/JETSON_CROSS_BUILD.md). Output stays under `rust/target`.
Manual controller launches require `AIZEE_CONFIG` set to the chosen profile.
Existing service units already set it.

The [Jetson setup guide](docs/quickstart/FRESH_JETSON_SETUP.md) distinguishes the
new development device from robot-service installation. Physical deployments
remain explicit operations.

## Validation and status

```sh
python -m pytest
python scripts/validate_simulation.py --output simulation_runs/validation-new
```

Default pytest discovery covers offline unit/contract tests. See
[tests/README.md](tests/README.md) for additional suites and hardware tooling.
The simulation baseline passed 248 Python tests and 12 live scenarios on
2026-09-17. ARM64 services cross-compile locally. Jetson runtime validation,
the virtual-CAN controller tier, CAD dynamics, and measured actuator calibration
remain open. Configured loop rates are targets, not measured guarantees.

## Documentation

- [Documentation index](docs/README.md)
- [Package installation and launch settings](docs/architecture/MONOREPO_IMPLEMENTATION.md)
- [Shared session integration](docs/architecture/SESSION_BACKEND_MIGRATION.md)
- [Simulation and physical validation plan](docs/architecture/SIMULATION_VALIDATION_PLAN.md)
- [Minerva bring-up](docs/MINERVA_BRINGUP.md)
- [Aizee learning pipeline](docs/LEARNING_PIPELINE.md)
- [Cleanup record](docs/architecture/REPOSITORY_CLEANUP.md)

![Aizee](image.png)

[Aizee CAD in Onshape](https://cad.onshape.com/documents/191b8a861f2900918f30776f/w/8fcf08900b23701d0eff1c6a/e/fcb77dc7adbafb5b753006f3)
· [MIT license](LICENSE) · [LTRLAB](https://github.com/ltrlab)
