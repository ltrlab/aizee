# Simulation implementation and evidence

Updated 2026-09-17. This implements the first slice of the
[validation plan](SIMULATION_VALIDATION_PLAN.md): a fast simulator for both
robots, shared source-freshness and producer-lease behavior, and measurement
instrumentation. All current model parameters are **synthetic**. No robot was
powered, characterized, or deployed as part of this implementation.

## Run it now

From the repository checkout, with your Python environment activated:

```sh
python -m pip install -e ".[simulation,operator,dev]"
robot-platform sim run --robot minerva --output simulation_runs/minerva-001
```

In another terminal, use the **same run's generated settings**:

```sh
robot-platform run collect --robot minerva --settings simulation_runs/minerva-001/settings.json
```

The launcher prints an absolute settings path. Use that path when the terminals
have different working directories. In this Windows checkout the existing
environment's Python is `.venv-platform/Scripts/python.exe`; you can use
`python -m robot_platform` in place of `robot-platform`.

Use `--robot aizee` in both commands for Aizee. Each output directory must be
new. Ports are allocated and bound before settings are written. Keep the server
running while using the collector; an old run's settings do not identify a new
server. Ctrl-C closes the server and finalizes its trace. Collector GUI controls
operate the synthetic motors; motors start disabled. Simulation suppresses
physical leader discovery, connection, hotplug, and serial e-stop readers.
Minerva supports GUI jogging without leaders. The fast tier has no physical
device path or hardware network discovery, and simulation settings reject
non-loopback hosts/endpoints. Remote simulation is not yet supported.

Bounded run and injected fault examples:

```sh
robot-platform sim run --robot minerva --output simulation_runs/frozen-001 --duration 15 --scenario frozen-feedback
robot-platform sim run --robot aizee --output simulation_runs/nominal-001 --duration 10 --seed 12
```

Scenarios: `nominal`, `feedback-loss`, `frozen-feedback`, `malformed-feedback`,
`camera-loss`, `emergency-stop`. Faults begin at simulated time 1 second. Arm
feedback faults affect the first arm; e-stop affects all arms. Frozen feedback
freezes one motor's sample while the publisher continues to send messages.
`--no-cameras` is available for transport-only work.

Each run contains:

- `settings.json`: resolved endpoints, strict source freshness, isolated episode
  and checkpoint paths, and simulation identity.
- `manifest.json`: robot contract, parameters, seed, source revision/hash when
  run from a checkout, environment, scenario, completion, overruns, and drops.
- `trace.h5`: command receipt/rejection, telemetry, scenario, and full plant state
  events with sequence numbers and host-monotonic timestamps. Plant events also
  include simulation time. This is a measurement trace, not a training episode.
- `episodes/`: collector recordings when requested. Both recording formats carry
  a simulation provenance attribute; Minerva recordings reopen in its actual
  training dataset reader.

The pure stepped plant is seeded and repeatable for the same commands and time
steps. Live-process wall-clock scheduling and command arrival are not bitwise
deterministic. Nominal telemetry is 50 Hz; JPEG test images are published every
second cycle, at 640x480. They test wire/decode/recording behavior, not visual
fidelity. The arm model is independent damped joints with torque saturation and
response lag. Aizee drive is an approximate base kinematic model with wheel
enable/disable/fault state, without wheel feedback or contact dynamics. Controller
timeout/hold behavior is approximate until the production-controller tier runs.

## Code map

| Area | Implementation |
| --- | --- |
| Deterministic plant and states | `src/robot_platform/simulation/plant.py` |
| Loopback MessagePack/ZMQ server, cameras, scenarios, manifests | `src/robot_platform/simulation/server.py` |
| CLI | `src/robot_platform/simulation/cli.py` |
| Source-sample validity and age | `src/robot_platform/control/freshness.py`, `session.py` |
| Producer leases and queued-command lifecycle | `src/robot_platform/control/background.py` |
| Software stop | Shared session and guarded controller; hardware bridge retired 2026-09-24 |
| HDF5 trace writer/reader and episode provenance | `src/robot_platform/data/traces/`, `data/provenance.py` |
| Repository audit and bench-profile validation | `src/robot_platform/characterization/audit.py` |
| Existing bench compatibility tool | `python/scripts/characterize_actuator.py`, `actuator_char/` |
| Production CAN/command/telemetry trace hooks | `rust/motor_control/src/trace.rs`, `main.rs` |
| Wire reference fixture | `tests/fixtures/simulation/telemetry_v2.*` |

## Specific gaps: disposition

| Gap from the plan | Implemented treatment | Remaining evidence |
| --- | --- | --- |
| Configured 400 Hz control, 50 Hz telemetry, 0.5 s watchdog were treated as performance | Raw CAN and application/controller boundary events have explicit clocks; live manifests count overruns | Measure achieved rates and delays on Minerva and the new Jetson |
| Published timestamps hid cached CAN feedback | Rust publishes process identity and per-motor sample sequence, age, validity, and provenance; sessions use the oldest required motor sample | Physical frozen-CAN tests and queue-delay measurements |
| Repeated cached samples looked like independent bench observations | CAN host receive times, kernel timestamps, raw bytes, sequences, sample-age/new flags; analysis excludes repeats | Clock resolution, kernel timestamp provenance, and actual sample-rate study |
| Producer could stall while worker resent forever | A target lease defaults to 0.5 s and requires producer renewal; worker sends and incoming telemetry do not renew it | Measure downstream watchdog response, including queues; lease expiry does not itself disable motors |
| Encoder/reference and torque-scale assumptions | Manual audit with hashes/pages; old pre-gearbox claims corrected; deployed conflicting scales preserved and flagged | Device model/firmware/shaft-reference/captured-frame evidence |
| Placeholder model/position limits and reversed calibration bounds | Audit reports affected entries; powered bench runs require a per-device verified profile with increasing limits | Determine physical sign, zero, travel, limits, payload, and safe test envelope |
| Single-destination e-stop example | ESP-NOW/serial design retired by user on 2026-09-24; software stop remains | Replacement power-cutoff design deferred; supervised test stand only |
| Bench faults could be misinterpreted | Filter target CAN ID; preserve sixth error bit; check fresh feedback and envelope; slew-limit from measured starting pose | Physical stop and bounded trajectory verification |
| Synthetic data could appear to be real evidence | Separate run directories, manifests, episode attributes, and strict synthetic label | Real measurements and held-out fits before promoting any channel |
| Invalid analysis could report success | Never-settled step returns infinity; no independent torque/backlash/Kt claim; irregular sample times withhold frequency metrics | Measurement uncertainty and valid independent instrumentation later |
| Tests exercised helpers but missed real application startup | Both collector main loops now run against live TCP simulation in subprocess tests; serial opening is trapped | Interactive GUI exercise, full recording workflow, and 60-minute loaded soak |

### Freshness and lease contract

Schema v2 adds top-level `telemetry_schema_version=2` and `producer_id`. Each
motor's `feedback` includes `sequence`, `age_s`, `valid`, and `provenance`.
Rust uses `can` only after measured feedback. Synthetic enable placeholders are
invalid. The simulator uses `simulation` for its sampled synthetic plant.
Repeated sequences cannot reset freshness; stale/out-of-order samples, retired
producer identities, invalid fields, and schema downgrade fail closed. A new
producer identity allows sequence restart. One invalid required motor invalidates
that arm, not its healthy peer.

Publisher age plus local elapsed time is a **lower bound** on sample age: unknown
network/queue delay is not magically measured. Absolute timestamps from different
machines are not subtracted for this check. Legacy telemetry remains compatible
by default and explicitly reports unknown source freshness. Set
`control.require_source_freshness=true` after upgrading the publisher; generated
simulation settings already do so. `control.stale_after_s` controls the age limit.

`control.target_lease_s` defaults to 0.5. A producer must call `set_target` again
before that deadline. An expired held target is forgotten and not revived by
telemetry recovery. Direct queued motion also expires. Explicit zero drive stops
and service delivery retain their separate handling. Once sending stops, Rust's
existing watchdog governs its hold behavior; this is not an instantaneous e-stop.

## Hardware audit and measurement preparation

This command reads repository files only; exit code **2** means outstanding
hardware prerequisites, which is currently expected:

```sh
robot-platform sim audit --repo . --output minerva-hardware-audit.json
```

The bundled manuals' **29-bit private protocol, section 4.1.2** gives these
ranges (not the separate 11-bit MIT chapter):

| Model | Velocity rad/s | Torque Nm | Kp | Kd | PDF page (1-based) |
| --- | ---: | ---: | ---: | ---: | ---: |
| RS00 | +/-33 | +/-14 | 500 | 5 | 39 |
| RS02 | +/-44 | +/-17 | 500 | 5 | 43 |
| RS03 | +/-20 | +/-60 | 5000 | 100 | 42 |
| RS04 | +/-15 | +/-120 | 5000 | 100 | 45 |

Sources are `docs/RS00User Manual260112.pdf` through
`docs/RS04User Manual260112.pdf`; the audit pins their SHA-256 hashes.
The deployed RS00 codec uses 30/2/100 and RS03 velocity uses 50. These remain
**unresolved firmware/protocol conflicts**. Matching Python and Rust is not proof
of physical correctness. Audit also flags `left_arm_j1`, `left_arm_j3`,
`right_arm_j2`, and `right_arm_j4` directional bounds. They have not been sorted
or silently turned into approved motion limits.

`settings/examples/bench-profile.template.json` intentionally has false
verification flags and no numeric motion authorization. Fill it from device
evidence before a real `--bench-profile` run. The bench runner currently supports
only a verified `legacy_deployed` protocol; firmware that matches different
scales needs an explicit codec profile first. Validation checks the record's
shape and consistency; it cannot verify the operator's evidence itself.

Offline characterization plumbing:

```sh
python python/scripts/characterize_actuator.py --model RS02 --can-id 7 --routine step --dry-run --dur 1
```

This creates unique CSV/HDF5 paths with unavailable feedback and produces no
physical accuracy result. Real runs log motor-reported torque first, as agreed.
The optional encoder adds host receive identity; it has no synchronized device
clock. Timing metrics include command slew and host/transport uncertainty.
Only one command owner may operate a bench motor. A direct-CAN sweep cannot run
alongside the production controller on the same motor. Physical envelopes and
stop verification must precede any powered sweep.

### Production Rust trace

Set `ROBOT_PLATFORM_TRACE` to a **new** JSONL filename on the controller process
to capture command handling, CAN submission/acceptance/errors/receive, and
telemetry publication. Use distinct files for left and right. The writer has a
bounded nonblocking queue; drops/errors are explicit. This hook adds tracing to
the existing controller, not a standalone passive CAN sniffer.

CAN timestamps are host userspace boundaries. `can_tx_accepted` means submission
to the kernel, not actuator acknowledgement or on-wire timing. A complete trace
ends with `trace_end` and zero dropped events. Ctrl-C performs trace finalization;
abrupt exit/SIGTERM/power loss can leave an incomplete file, which is not valid
measurement evidence. The Python HDF5 reader rejects incomplete/dropped traces.

### Retired hardware stop bridge

The ESP-NOW receiver, serial bridge, service, udev rule and deployment helper
were removed at the user's request on 2026-09-24. The installed production
bridge was stopped and uninstalled with backups. Software emergency-stop
commands and fault latching remain. They do not cut electrical power. See the
[current commissioning procedure](../POWERED_COMMISSIONING.md) and
[device log](../devices/MINERVA_PRODUCTION_01.md).

## Validation and next stages

Run the live process matrix for both robots and all six scenarios:

```sh
python scripts/validate_simulation.py --output simulation_runs/validation-new
```

This launches separate simulator CLI processes, drives them through the shared
threaded TCP client, and checks actual received commands, camera decoding,
source freshness, producer lease expiry/renewal, e-stop/clear behavior, and trace
completion. The new output directory receives per-case logs, full simulator
traces/manifests, and `report.json`. `--scenario nominal` selects one scenario
for both robots. Nonblocking queue backpressure is recorded separately from
unexpected errors; subsequent command delivery is required to pass.

The 2026-09-17 live matrix passed **12/12 cases**, with no trace drops or rejected
commands. Nominal runs lasted 15 seconds per robot; fault runs lasted 6 seconds.
Mean publication rate was approximately 50 Hz. Nominal p99 gaps were 47 ms, with
a 125 ms maximum for Minerva on this Windows host. These are functional checks
with scheduling jitter, not a real-time performance guarantee or a completed
60-minute soak. E-stop removes simulated drive torque; inertia can still coast.

Run the normal offline suite with `python -m pytest`. It includes both robot
contracts, source-age/restart/malformed cases, leases, per-arm faults, lifecycle,
live MessagePack/camera transport, two collector process tests, and a Minerva
recording/dataset round trip. Process tests require collector dependencies and
skip explicitly when those dependencies are absent. The cross-language fixture
is also consumed by `cargo test -p comms` (one pre-existing network test ignored).

The final extended Python regression run passed **248 tests**, including the
existing characterization, recording/replay, and Minerva segmentation tests.
Both actual collector processes received live observations and exited cleanly;
their serial-device trap recorded no hardware access attempts. A separate
three-second CLI run completed 150 telemetry cycles with zero trace drops.
This short run is not the planned load/soak acceptance test.

The updated `motor_control` and `lidar_control` have been cross-built for
AArch64 Linux. ELF inspection confirms the AArch64 loader, no library search
paths, and maximum GLIBC requirement 2.34, within the configured 2.35 baseline.
They have **not run on a Jetson**. See [cross-build guide](../../rust/JETSON_CROSS_BUILD.md).

M0/M1 now have runnable software, audit output, tests, and trace evidence. Their
physical reference fixtures and full interactive/soak acceptance remain open.
M2's Rust instrumentation is implemented; the virtual-CAN firmware emulator and
production-controller scenario runner are **not yet implemented**. Run those
on Linux/the dedicated Jetson before claiming production-controller equivalence.
M3 needs actual Minerva traces and held-out fits; M4 needs audited Minerva
geometry and coupled dynamics. Independent torque instruments remain the later
M5 phase. No current model may be labelled telemetry-calibrated or externally
validated.

## Offline collision and limit-probe foundation

The intended Minerva export now has the correct 17-link graph, 12 arm rotations,
four jaw slides and two mimic relations. Source mass properties and arm limits
remain absent; exporter numerical defaults are not operating limits.

`sim check-collision` checks complete robot states and sampled synchronized
paths using FCL, including jaw mimic and explicit obstacle/exclusion data.
`sim validate-limit-probe` exercises torque/stall candidate detection on
synthetic signals. These commands do not drive motors or enforce a runtime
motion gate. See [commands, current controller blockers and acceptance stages](COLLISION_AND_LIMIT_CALIBRATION.md).
