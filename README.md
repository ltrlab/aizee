# Robot platform

The software [LTR LABS](https://github.com/ltrlab) uses to run, teleoperate, record from and train its robots: one
Python package (`robot_platform`), one Rust workspace, and the firmware, CAD and calibration data they use. A robot is
a definition the shared code selects, so more than one can live here.

| Robot | What it is | Status |
| --- | --- | --- |
| [**Minerva**](docs/robots/minerva/HARDWARE.md) | Two-arm torso: 14 RobStride motors on CAN, a 2-axis Feetech neck, two wrist cameras and a head depth camera, on a Jetson. Two GELLO leader arms drive the arms and a Quest headset steers the head, through Helm, its web UI. | The current robot. |
| [**AIZEE**](docs/robots/aizee/README.md) | One arm: a swivel, a gantry and a wrist with a gripper, plus wheel drive. | The earlier prototype. It stays as a robot option. |

[docs/STATUS.md](docs/STATUS.md) says what has run on a real robot and what has only passed offline tests.

## Where to go

| You want to | Read |
| --- | --- |
| Try Helm or the model with no robot | [Simulate](docs/getting-started/SIMULATE.md), below |
| Operate Minerva | [Helm manual](docs/operator/HELM_MANUAL.md), and first [Safety](docs/operator/SAFETY.md) |
| Install it on a robot | [Jetson](docs/getting-started/INSTALL_JETSON.md), [operator station](docs/getting-started/INSTALL_OPERATOR_STATION.md), [a new unit](docs/getting-started/NEW_UNIT_PROFILE.md) |
| Record, export and train | [Episode format](docs/data/EPISODE_FORMAT.md), [dataset pipeline](docs/data/DATASET_PIPELINE.md), [training](docs/data/TRAINING.md) |
| Change the code | [Development workflow](docs/DEVELOPMENT.md), [architecture](docs/architecture/OVERVIEW.md) |
| Find anything else | [Documentation index](docs/README.md) |

## Try it without a robot

Python 3.12, [Git LFS](https://git-lfs.com/) for the meshes, and on Linux `.venv-dev/bin/python` where these say
`.venv-dev/Scripts/python.exe`:

```sh
git lfs pull
python scripts/dev.py setup
.venv-dev/Scripts/python.exe -m robot_platform list
```

See and jog a robot's model at <http://127.0.0.1:8765>:

```sh
.venv-dev/Scripts/python.exe -m robot_platform view --robot minerva
```

Open Helm on a simulated Minerva at <http://127.0.0.1:8760/>; its header says **Simulation only**:

```sh
.venv-dev/Scripts/python.exe -m robot_platform helm --profile config/devices/minerva-joint-calibration.json --teleop config/devices/minerva-teleop-direct.json --sim
```

None of these open a motor or start a service. Anything physical is a separate, explicit step in the install guides.

## Repository map

```text
src/robot_platform/    The Python package
  helm/                  Helm: the operator web UI and the teleop loop, with its pages in static/
  control/               Guarded sessions, freshness, stopping, calibration, commissioning
  teleop/                Leader arms, the GELLO bridge, the headset, policies as sources of commands
  neck/                  Minerva's Feetech neck
  data/                  Episode formats, recording, export to LeRobot
  vision/                Camera calibration and the table's fiducials
  simulation/            MuJoCo task scene, plants, the loopback simulator
  evaluation/            Scripted demonstrations and N-trial policy evaluation
  ops/                   Releases, deployment and the services installed on the Jetson
  robots/ settings/      Robot definitions and deployment settings, one of each per robot
  models/ viewer/        Mechanical model profiles and the browser viewer
  apps/ transport/       Launchers for the older collectors and trainers; MessagePack, ZeroMQ
  characterization/      Protocol and configuration audit, bench profiles
python/                Older collectors (collect_minerva, AIZEE's collect_demo), training models, tools
rust/                  control_core (the guarded receiver and head controller), motor_control, collision, comms, lidar_control
firmware/              openrb_leader (the leader arms), lift_actuator, tufty2040 (AIZEE's display)
config/                Device profiles, calibration, hardware files, systemd and udev: see config/README.md
urdf/                  CAD exports and MuJoCo models, with meshes in Git LFS
scripts/               Build, deploy, benchmarks, simulation tools, training server: see scripts/README.md
settings/examples/     Settings overlays
tests/                 Offline tests and fixtures: see tests/README.md
docs/                  See docs/README.md
```

## Checking a change

```sh
python scripts/dev.py check
```

This builds and tests the Rust crates, then runs the required tests with no skips allowed. It needs no hardware, and
passing it does not qualify anything on a robot. [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) has the setup, the faster
lane for small changes and the commit checks.

## License

[MIT](LICENSE). The RobStride manuals in `docs/vendor/` are the vendor's own.
