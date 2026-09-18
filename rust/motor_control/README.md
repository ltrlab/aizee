# Shared motor controller

Linux SocketCAN controller for the Aizee and Minerva ROBSTRIDE deployments.
The root Cargo workspace owns dependencies and build output stays in
`rust/target/`.

## Build and select a robot

```sh
cargo build --locked --release -p motor_control
AIZEE_CONFIG=config/hardware_jetson_rover.yaml ./rust/target/release/motor_control
```

Run from the repository root. `AIZEE_CONFIG` is required: there is no generic
hardware fallback. Existing systemd units set the appropriate profile.

| Profile | Bus | Commands / telemetry |
|---|---|---|
| `config/hardware_jetson_rover.yaml` | `can1` | `5555 / 5556` |
| `config/hardware_minerva_left.yaml` | `canL` | `5555 / 5556` |
| `config/hardware_minerva_right.yaml` | `canR` | `5575 / 5576` |

Minerva runs two controller processes. The profile parser retains the
historical `wheels`, `swivel`, and `arm` grouping; Minerva uses those groups to
represent its per-arm motors. Do not substitute one robot's profile for another.

The configured arm/base/telemetry rates are 400/100/50 Hz with a 0.5-second
watchdog in current profiles. Actual achieved rates require measurement.
Watchdog hold, disable, and emergency stop are different controller states.

## Wire interface

Commands use MessagePack over ZMQ PUSH/PULL. Telemetry uses MessagePack
PUB/SUB. The schemas live in `rust/comms/src/messages.rs`; Python encoders
live in `robot_platform.transport.wire`. Prefer the shared Python session for
robot joint mapping, freshness and per-arm routing.

Telemetry schema 2 includes a producer identity and per-motor feedback
sequence, sample age, validity and provenance. Enable placeholders are not
physical samples. The legacy JSON-only example tools have been removed.

## Trace and verification

Set `ROBOT_PLATFORM_TRACE` to a new JSONL path to record command handling,
CAN submissions/receipts and telemetry publication. Ctrl-C finalizes the trace.
These are host userspace timestamps, not actuator acknowledgements.

```sh
cargo test --locked -p comms
python -m pytest
```

See [simulation validation](../../docs/architecture/SIMULATION_IMPLEMENTATION.md)
for source freshness, trace completeness, the synthetic test suite and the
pending virtual-CAN integration tier. See the
[cross-build guide](../JETSON_CROSS_BUILD.md) for Windows-to-Jetson builds.

## Hardware reference

The codec uses the extended 29-bit ROBSTRIDE protocol. Model/firmware-specific
scales are under audit; current RS00 and RS03 ranges conflict with bundled
manuals. Do not treat matching Python/Rust implementations as physical proof.
The audit and evidence requirements are in the simulation guide above.

For bus setup and robot-specific inventory, use
[Minerva bring-up](../../docs/MINERVA_BRINGUP.md),
[Aizee motors](../../docs/subsystems/MOTORS.md), and the selected service unit.
No hardware test is part of the default offline test command.
