# Shared platform roadmap

Updated 2026-09-21. Status describes source and recorded test evidence, not a
claim that the latest changes have been deployed to a robot.

| Stage | Current state | Next gate |
|---|---|---|
| Monorepo foundation | One Python package and one four-crate Cargo workspace | Continue extracting active application internals |
| Shared session | Both collectors, integrated Aizee replay and Minerva policy migrated | Standalone Aizee policy/replay and selected older transports |
| Shared model tools | One profile-driven viewer, URDF audit and collision inspector for catalog or external robots | Runtime guarded motion and remaining robot-specific hardware applications |
| Collision runtime | CAD confirmation and conservative keep-out modes; 1,000-cycle scoped Jetson replay at 25 Hz with zero misses | Resolve same-arm interfaces, full-session load/lease tests and measured stopping |
| Development Jetson | Identified Orin Nano, native builds/tests and preserved benchmark/change logs | Qualify intended robot host separately; no motor services enabled here |
| Fast simulation | Both robots, synthetic cameras, source freshness, command leases, fault scenarios and traces | Interactive workflow and 60-minute loaded soak |
| Production controller tests | Rust instrumentation and ARM64 builds available | Virtual-CAN motor emulator and Linux/Jetson scenario runner |
| Measured actuator model | Audit and bench-profile gates implemented | Inventory, resolve scales/limits, capture Minerva, fit and hold out runs |
| Coupled robot dynamics | Intended export has correct joint graph; offline collision/probe foundations | Calibrate arm limits independently, verify masses and collision gate, integrate physics |
| Independent measurements | Planned after motor-telemetry calibration | External torque/angle instruments and uncertainty assessment |

## Immediate sequence

1. Resolve unapproved same-arm/body interfaces and extend the scoped keep-out
   replay to the full-session workload with receiver-side command leases.
   Follow the [25 Hz acceptance plan](architecture/COLLISION_25HZ_HARDWARE_PLAN.md).
2. Build and validate the virtual-CAN tier using the actual Rust controller,
   then qualify timing with camera, logging and transport load on the target.
3. Retain the imported 1.1 visual / 1.2 collision models and provenance. Assign
   missing physical materials/mass overrides and verify jaw coupling before
   articulated physics. See the [CAD handoff guide](MINERVA_CAD.md).
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
