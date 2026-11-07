# Documentation

Find what you are doing below. [Status](STATUS.md) says what is proven on a robot and what is not.

Two folders are records rather than guides, and you never need them to use the robot: [`history/`](history/) holds the
dated plans and per-feature records behind these pages, and [`devices/`](devices/) holds one unit's device logs and
measured evidence, which its calibration files cite.

## Try it without a robot

- [Simulate](getting-started/SIMULATE.md): Helm on a simulated Minerva
- [Model viewer](simulation/MODEL_VIEWER.md): see and jog a robot's model, inspect collisions, add a robot

## Install

Do these in order for a new robot.

1. [Jetson](getting-started/INSTALL_JETSON.md): from a fresh Jetson to Helm running on it
2. [Operator station](getting-started/INSTALL_OPERATOR_STATION.md): the PC with the leader arms, the bridge, the pedals, the headset
3. [A new unit's profile](getting-started/NEW_UNIT_PROFILE.md): what is specific to one robot and how to make it
4. [Calibration, in order](operator/CALIBRATION.md)

## Operate Minerva

- [Safety](operator/SAFETY.md): what stops what, and what does not. Read it first.
- [Helm manual](operator/HELM_MANUAL.md): every screen, from login to teleop
- [Recording and task workflow](operator/HELM_TASK_WORKFLOW.md)
- [Replay driving the arms](operator/LIVE_REPLAY.md)
- [Policy mode](operator/POLICY_MODE.md)
- [The headset on a slow link](operator/REMOTE_HEADSET.md)
- [Troubleshooting](operator/TROUBLESHOOTING.md)

Calibration procedures, in the order [Calibration](operator/CALIBRATION.md) gives them:
[mechanical reference](operator/MECHANICAL_REFERENCE.md),
[joint calibration](operator/JOINT_CALIBRATION.md),
[powered commissioning](operator/POWERED_COMMISSIONING.md),
[wrist cameras](operator/WRIST_CAMERA_CALIBRATION.md),
[head camera](operator/HEAD_CAMERA_ORIENTATION.md),
[table fiducials](operator/TABLE_FIDUCIALS.md).

## Data and learning

- [Episode format](data/EPISODE_FORMAT.md): what a recording holds
- [Dataset pipeline](data/DATASET_PIPELINE.md): steady capture, Replay review, export to LeRobot
- [Training](data/TRAINING.md): from an exported dataset to a policy on the robot
- [Training server scripts](../scripts/training_server/README.md)
- [Policy evaluation](simulation/MINERVA_EVAL.md): N-trial closed-loop runs, one report for simulation and the robot

## Robots

- [The robots](robots/README.md): what a robot is in this repository, and how to add one
- **Minerva**: [hardware](robots/minerva/HARDWARE.md), [neck](robots/minerva/NECK_FEETECH.md),
  [head and neck model](robots/minerva/HEAD_NECK_MODEL.md), [CAD](robots/minerva/MINERVA_CAD.md),
  [physics measurements](robots/minerva/MINERVA_PHYSICS_MEASUREMENTS.md), [UPS](robots/minerva/UPS_MONITOR.md),
  [gripper force feedback](robots/minerva/GRIPPER_FORCE_FEEDBACK.md)
- **Leader arms**: [OpenRB leader](robots/OPENRB_LEADER.md), [firmware](../firmware/openrb_leader/README.md)
- **AIZEE**: [start here](robots/aizee/README.md), including which `aizee-*` names belong to which robot

## How it works

- [Architecture overview](architecture/OVERVIEW.md): processes, packages, data flow, where everything lives
- [Helm on the Jetson](architecture/HELM_ON_JETSON.md)
- [Guarded controller](architecture/GUARDED_CONTROLLER.md) and [guarded teleop](architecture/GUARDED_TELEOP.md)
- [GELLO teleop](architecture/GELLO_TELEOP.md) and [teleop latency](architecture/TELEOP_LATENCY.md)
- [Minerva's network](architecture/MINERVA_NETWORK.md) and [its CPU split](architecture/JETSON_CPU_SPLIT.md)
- [Collision runtime](architecture/COLLISION_RUNTIME_IMPLEMENTATION.md)
- Plans still being carried out: [headset 3D](architecture/HEADSET_3D_PLAN.md), [VR arm teleop](architecture/VR_ARM_TELEOP_PLAN.md)
- [Configuration ownership](../config/README.md) and the [Rust crates](../rust/README.md)

## Simulation

- [Simulator implementation](simulation/SIMULATION_IMPLEMENTATION.md) and [MuJoCo](simulation/MUJOCO.md)
- [Minerva's MJCF](simulation/MINERVA_MJCF.md) and [task scene](simulation/MINERVA_TASK_SCENE.md)
- [Simulation against the real robot](simulation/SIM2REAL.md)
- The generation line: [`simulation/`](simulation/) files named `SIM_*`

## Develop

- [Development workflow](DEVELOPMENT.md): setup, the gate, the quick lane, commit checks
- [Agent and contributor rules](../AGENTS.md)
- [CI and release prerequisites](development/CI_RELEASE_GATE.md)
- [Tests](../tests/README.md) and [scripts](../scripts/README.md)
- [Task template](development/TASK_TEMPLATE.md)

## Vendor manuals

RobStride motor manuals, which `characterization/audit.py` reads: [`vendor/`](vendor/).
