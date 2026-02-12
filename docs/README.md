# AIZEE Documentation

This directory contains all technical documentation for the AIZEE robotics platform.

## Quick Navigation

### Getting Started
- **[PHASES.md](PHASES.md)**: Implementation roadmap and development phases
- **[quickstart/](quickstart/)**: Quick start guides for different scenarios
  - [QUICK_START_MULTIDEVICE.md](quickstart/QUICK_START_MULTIDEVICE.md): Multi-device deployment (10-minute setup)
  - [QUICK_START_AFTER_REBOOT.md](quickstart/QUICK_START_AFTER_REBOOT.md): Post-reboot startup
  - [JETSON_QUICK_START.md](quickstart/JETSON_QUICK_START.md): Jetson Orin Nano setup

### Deployment
- **[deployment/](deployment/)**: System deployment documentation
  - [MULTI_DEVICE_DEPLOYMENT.md](deployment/MULTI_DEVICE_DEPLOYMENT.md): Multi-module architecture deployment
  - [IMPLEMENTATION_SUMMARY.md](deployment/IMPLEMENTATION_SUMMARY.md): Architecture implementation details
  - [DEPLOYMENT_LOG.md](deployment/DEPLOYMENT_LOG.md): Deployment history and lessons learned
  - [TROUBLESHOOTING_CAN.md](deployment/TROUBLESHOOTING_CAN.md): Dual CAN interface troubleshooting

### Subsystems
- **[subsystems/](subsystems/)**: Component-specific documentation
  - [CAMERAS.md](subsystems/CAMERAS.md): Intel RealSense D455 camera system (4 RPi nodes)
  - [LIDAR.md](subsystems/LIDAR.md): RPLiDAR A1M8 dual sensor integration
  - [UPS.md](subsystems/UPS.md): INA219 battery monitoring system

### Archive
- **[archive/](archive/)**: Historical documentation and superseded implementation notes
  - Preserved for reference but may be outdated

## External Documentation

Component-specific READMEs in subdirectories:
- [../rust/motor_control/README.md](../rust/motor_control/README.md): Motor control system
- [../rust/lidar_control/README.md](../rust/lidar_control/README.md): LiDAR control implementation
- [../scripts/README_CAMERA_SCRIPTS.md](../scripts/README_CAMERA_SCRIPTS.md): Camera deployment scripts
- [../tests/README.md](../tests/README.md): Testing framework

## Root-Level Documentation

- [../CLAUDE.md](../CLAUDE.md): **PRIMARY REFERENCE** for Claude Code - comprehensive system guide
- [../README.md](../README.md): Project overview and hardware specifications
