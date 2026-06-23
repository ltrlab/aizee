# AIZEE Documentation

Technical documentation for the AIZEE mobile manipulation platform.

## Getting started
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
- **[learning/RESEARCH_PLAN.md](learning/RESEARCH_PLAN.md)** — research direction and experiment plan

## Roadmap
- **[PHASES.md](PHASES.md)** — implementation status and roadmap

## Reference
- ROBSTRIDE motor manuals: `RS00`/`RS02`/`RS03`/`RS04` User Manual PDFs (this directory)
- [../rust/motor_control/README.md](../rust/motor_control/README.md) — motor control crate
- [../rust/lidar_control/README.md](../rust/lidar_control/README.md) — LiDAR control crate
- [../tests/README.md](../tests/README.md) — test suite
- [../README.md](../README.md) — project overview & hardware specs
