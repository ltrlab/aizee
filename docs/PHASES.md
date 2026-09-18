# Shared platform roadmap

Updated 2026-09-18. Status describes source and recorded test evidence, not a
claim that the latest changes have been deployed to a robot.

| Stage | Current state | Next gate |
|---|---|---|
| Monorepo foundation | One Python package and one three-crate Cargo workspace | Continue extracting active application internals |
| Shared session | Both collectors, integrated Aizee replay and Minerva policy migrated | Standalone Aizee policy/replay and selected older transports |
| Fast simulation | Both robots, synthetic cameras, source freshness, command leases, fault scenarios and traces | Interactive workflow and 60-minute loaded soak |
| Production controller tests | Rust instrumentation and ARM64 builds available | Virtual-CAN motor emulator and Linux/Jetson scenario runner |
| Measured actuator model | Audit and bench-profile gates implemented | Inventory, resolve scales/limits, capture Minerva, fit and hold out runs |
| Coupled robot dynamics | Model requirements planned | Versioned Minerva CAD, audited mass/frames/collisions, physics integration |
| Independent measurements | Planned after motor-telemetry calibration | External torque/angle instruments and uncertainty assessment |

## Immediate sequence

1. Build and validate the virtual-CAN tier using the actual Rust controller.
2. Bring the dedicated Jetson online as a development/build host and exercise
   both cross-built and native binaries there.
3. Prepare a versioned Minerva simulation assembly in Onshape. Audit the 14
   active arm/gripper coordinates before integrating its dynamics.
4. Collect real Minerva traces on the existing robot compute unit. Establish
   signs, bounds, firmware and torque encoding before powered characterization.
5. Fit actuator response and test against separate held-out runs; then validate
   coupled motion and loads. Motor-reported torque is the initial source.

## Retained capabilities

Existing applications cover Aizee/Minerva collection, leader teleoperation,
ACT/JEPA/flow learning, replay, gravity calibration, camera services, dashboard,
UPS, display, e-stop firmware and optional LiDAR. Their presence in source is
not evidence of a fresh hardware deployment or validated physical model.

The unused binding placeholder, obsolete hardware examples and old checkpoints
were removed in the [cleanup](architecture/REPOSITORY_CLEANUP.md).

See the [implementation guide](architecture/MONOREPO_IMPLEMENTATION.md),
[simulation evidence](architecture/SIMULATION_IMPLEMENTATION.md), and
[validation plan](architecture/SIMULATION_VALIDATION_PLAN.md) for detailed gates.
