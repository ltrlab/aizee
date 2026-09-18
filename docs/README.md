# Aizee / Minerva documentation

Guides for the shared platform, robot applications and device deployments.

## Repository architecture

- **[Implemented monorepo guide](architecture/MONOREPO_IMPLEMENTATION.md)** — package installation, interchangeable profiles/settings, application launchers, shared backend APIs, and current migration status
- **[Session backend migration](architecture/SESSION_BACKEND_MIGRATION.md)** — collector/inference integration, command lifecycle, freshness and remaining legacy tools
- **[Simulation and hardware validation plan](architecture/SIMULATION_VALIDATION_PLAN.md)** — staged simulator design, actuator measurements, real-world calibration, and acceptance gates
- **[Simulation implementation](architecture/SIMULATION_IMPLEMENTATION.md)** — runnable loopback simulator, launch commands, measurement safeguards, gap disposition, and remaining hardware evidence

- **[Aizee / Minerva repository map](architecture/REPOSITORY_MAP.md)** — current ownership, runtime topology, dependency tangles, and source evidence
- **[Monorepo organization plan](architecture/MONOREPO_PLAN.md)** — proposed shared-platform layout, dependency rules, and staged migration gates
- **[Complete file inventory](architecture/REPOSITORY_INVENTORY.md)** — generated tracked-file catalogue and proposed destination areas
- **[Configuration ownership](../config/README.md)** — active hardware/calibration files and their consumers
- **[Cleanup record](architecture/REPOSITORY_CLEANUP.md)** — removed files, replacements and retained assets

## Getting started
- **[quickstart/FRESH_JETSON_SETUP.md](quickstart/FRESH_JETSON_SETUP.md)** — development-host setup, then separate Aizee robot provisioning
- **[MINERVA_BRINGUP.md](MINERVA_BRINGUP.md)** — existing Minerva deployment, buses, leaders and calibration
- **[quickstart/JETSON_QUICK_START.md](quickstart/JETSON_QUICK_START.md)** — connect to the robot and run teleop / data collection (daily reference)
- **[quickstart/QUICK_START_AFTER_REBOOT.md](quickstart/QUICK_START_AFTER_REBOOT.md)** — post-reboot verification, service management, troubleshooting

> First check after connecting: the **heartbeat dashboard** at `http://<jetson-ip>:8088`
> (e.g. `http://10.42.0.1:8088` over USB-C, `http://192.168.50.1:8088` over the `aizee` AP) —
> service status, recent logs, host metrics, and live robot telemetry on one page.

## Subsystems
- **[subsystems/MOTORS.md](subsystems/MOTORS.md)** — 9× ROBSTRIDE on `can1`; roster/CAN IDs, control loops, parameter read/write protocol
- **[subsystems/CAMERAS.md](subsystems/CAMERAS.md)** — gripper UVC (primary) + optional RealSense scene cam, both on the Jetson
- **[subsystems/OPENRB_LEADER.md](subsystems/OPENRB_LEADER.md)** — OpenRB-150 + Dynamixel XL330 leader: firmware, setup, wire protocol
- **[subsystems/UPS.md](subsystems/UPS.md)** — INA219 battery monitoring
- **[subsystems/TUFTY2040.md](subsystems/TUFTY2040.md)** — Tufty2040 status display
- **[subsystems/LIDAR.md](subsystems/LIDAR.md)** — 2× RPLiDAR A1M8 (optional)

## Learning
- **[LEARNING_PIPELINE.md](LEARNING_PIPELINE.md)** — end-to-end guide: calibration → data collection → training → evaluation → deployment
- **[learning/RESEARCH_PLAN.md](learning/RESEARCH_PLAN.md)** — dated Aizee learning research proposal; not the current migration roadmap

## Roadmap
- **[PHASES.md](PHASES.md)** — implementation status and roadmap

## Reference
- **[MINERVA_ACTUATORS.md](MINERVA_ACTUATORS.md)** — historical CubeMars proposal; current Minerva profiles use ROBSTRIDE
- ROBSTRIDE motor manuals: `RS00`/`RS02`/`RS03`/`RS04` User Manual PDFs (this directory)
- [../rust/motor_control/README.md](../rust/motor_control/README.md) — motor control crate
- [../rust/JETSON_CROSS_BUILD.md](../rust/JETSON_CROSS_BUILD.md) — local Windows-to-Jetson builds, verified ARM64 artifacts and toolchain setup
- [../rust/lidar_control/README.md](../rust/lidar_control/README.md) — LiDAR control crate
- [../tests/README.md](../tests/README.md) — test suite
- [../README.md](../README.md) — project overview & hardware specs
