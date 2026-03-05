# AIZEE Documentation

This directory contains all technical documentation for the AIZEE robotics platform.

## Quick Navigation

### Getting Started
- **[quickstart/](quickstart/)**: Quick start guides
  - [QUICK_START_AFTER_REBOOT.md](quickstart/QUICK_START_AFTER_REBOOT.md): Post-reboot startup (primary daily reference)
  - [JETSON_QUICK_START.md](quickstart/JETSON_QUICK_START.md): Teleop quick commands on Jetson
  - [QUICK_START_MULTIDEVICE.md](quickstart/QUICK_START_MULTIDEVICE.md): Optional RPi4 arm module deployment

### Deployment
- **[deployment/](deployment/)**: System deployment documentation
  - [MULTI_DEVICE_DEPLOYMENT.md](deployment/MULTI_DEVICE_DEPLOYMENT.md): Multi-module architecture (Jetson rover + RPi4 arm split)

### Subsystems
- **[subsystems/](subsystems/)**: Component-specific documentation
  - [CAMERAS.md](subsystems/CAMERAS.md): Intel RealSense D455 camera system (4 RPi nodes)
  - [LIDAR.md](subsystems/LIDAR.md): RPLiDAR A1M8 dual sensor integration
  - [MOTORS.md](subsystems/MOTORS.md): ROBSTRIDE motor parameter config — CAN read/write, LIMIT_TORQUE/LIMIT_CUR
  - [UPS.md](subsystems/UPS.md): INA219 battery monitoring system
  - [TUFTY2040.md](subsystems/TUFTY2040.md): Tufty2040 status display — firmware deploy & layout


## External Documentation

Component-specific READMEs in subdirectories:
- [../rust/motor_control/README.md](../rust/motor_control/README.md): Motor control system
- [../rust/lidar_control/README.md](../rust/lidar_control/README.md): LiDAR control implementation
- [../scripts/README_CAMERA_SCRIPTS.md](../scripts/README_CAMERA_SCRIPTS.md): Camera deployment scripts
- [../tests/README.md](../tests/README.md): Testing framework

## Root-Level Documentation

- [../CLAUDE.md](../CLAUDE.md): **PRIMARY REFERENCE** for Claude Code - comprehensive system guide
- [../README.md](../README.md): Project overview and hardware specifications
