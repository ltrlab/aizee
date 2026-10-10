# Current development status

Updated 2026-10-10. This file is the current entry point; dated phase tables and
device logs retain historical evidence, not an implicit statement of readiness.

The integration checkpoint preserves the 11 uncommitted overhaul files captured
after `a66afa7`. The original `overhaul` working directory remains independent.
The two test failures seen in the earlier review were subsequently edited in that
checkpoint; the new baseline must be verified against its own frozen source.

## Implemented and verified

The Windows baseline passed 95 required Python tests with zero skips and 104
portable controller/collision Rust tests on 2026-10-10. The baseline commit is
`5e3b965`; local evidence is `build/dev-check/20261010T155025Z-3ee526b4/report.json`.
GitHub CI is configured for Windows and Linux but has not yet run remotely.

The combined Windows gate passed **125 required Python tests, zero skips, and
104 Rust tests** with MuJoCo 3.15.0. Evidence:
`build/dev-check/20261010T160014Z-56534c06/report.json`. Documentation and lockfile
comment cleanup followed that run; package pins and tested source behavior are unchanged.

- Fresh-worktree setup, hash-locked dependencies, required offline gate, source
  and binary identity reports, and Windows/Linux CI configuration are implemented.
- MuJoCo runs behind `PlantReceiver` and the actual Rust receiver. The original
  `ArmPlant` remains available for cheap, deterministic fault scenarios.
- Analytic gravity/URDF import, coupled inertia, signed transmissions, contact,
  quiet/dropped feedback and generated lease-expiry cases pass.
- The synthetic recorded rehearsal completed a requested stop. Evidence:
  `simulation_runs/mujoco-complete/report.json` and its `trace.csv`.
- The Minerva audit rejects 17 placeholder masses and reports two mimic joints.
  Evidence: `build/minerva-physics-audit.json`. The measurement checklist is ready;
  the user confirmed that physical mass properties are not yet available.

The user's current priority is direct position teleop without collision detection.
`codex/teleop-direct-mirror` implements the [direct-mode contract and qualification
tools](development/DIRECT_TELEOP.md) from integration commit `8b4f2c4`:

- Explicit collision-disabled one/two-arm configuration bypasses geometry and
  envelope creation. It retains joint/control bounds, source/feedback freshness,
  watchdogs, exclusive authority and the existing physical enable gates.
- Each following cycle sends one atomic direct update and receives feedback plus
  its next one-use freshness token. Replay cannot extend target or lease lifetime.
- A separate 100 Hz profile removes interpolation, with bounded velocity
  feedforward. Repeated samples keep their age; source loss holds and requires
  Engage. Receiver expiry recovers only into shutdown and disable.
- Timing traces distinguish source/mapping/RPC/acceptance/application, full loop
  work and scheduling misses. Synthetic harness/vcan benchmarks save source,
  config and binary identity with their evidence. Physical motion onset is not
  inferred from receiver acknowledgement.

The first behavior commit `b8b13c7` passed the required gate at
`build/dev-check/20261010T165500Z-2bdfa61f/report.json`. The Linux AArch64 release
service cross-build passed; log: `build/direct-qualification/linux-build.log`.
An expanded gate found one stale error-message assertion (240 other Python
tests passed); that expectation was updated for the new explicit mode.

Final required Windows gate: **251 Python tests, zero skips/xfails, and 111 Rust
tests passed** at `build/dev-check/20261010T171542Z-d2c7616e/report.json`. Source
content was unchanged throughout that run. The focused new/affected subset also
passed 32 tests (`build/direct-qualification/final-focused.xml`). Only these
documentation results were finalized after the gate; runtime/test source did not
change. Final build/evidence identity is in `build/direct-qualification/report.json`.

Synthetic debug-harness measurements (14 joints, one direct RPC per update):

| Rate | Accepted updates | Processing p50 / p95 / p99 / max (ms) | Scheduled misses | Result |
|---|---:|---|---:|---|
| 50 Hz, initial | 641 / 1000 | 1.376 / 1.865 / 2.487 / 16.681 | 1 | Ownership revoked after overrun; incomplete |
| 50 Hz, diagnostic | 1000 / 1000 | 1.398 / 1.753 / 2.104 / 3.471 | 0 | Completed, silence expiry and stop verified |
| 100 Hz | 1000 / 1000 | 1.284 / 1.749 / 2.200 / 5.336 | 0 | Completed, silence expiry and stop verified |
| 200 Hz | 1000 / 1000 | 1.154 / 1.566 / 1.944 / 3.213 | 0 | Completed, silence expiry and stop verified |

Reports and traces are in `simulation_runs/direct-harness-{50,100,200}hz-20261010/`
and `simulation_runs/direct-harness-50hz-diagnostic-20261010/`. The initial 50 Hz
failure remains evidence; its report did not retain the underlying receiver stop
reason beyond `wrong_owner`. Benchmark failure recording now preserves that
status before cleanup. No control threshold changed for the diagnostic rerun.

These short samples establish synthetic receiver-path overhead only. Harness
application delay reads zero because application shares a logical tick with
acceptance; this is not measured physical latency. The full live-loop 8 ms p99
target has not been demonstrated. Keep the 100 Hz default; the 200 Hz run is an
evaluation, not approval for deployment.

Linux vcan execution remains pending on a Linux development host: this Windows machine has
no running Linux service environment. Jetson timing, unique real leader sample
throughput and physical tracking qualification remain open. No deployment or
motor command was performed. Collision optimization and physical-model
identification are separate work; the integration branch has not been merged.

## Known limits

- Minerva's active URDF has 17 link masses of `1e-9 kg`, with placeholder inertia.
  The model profile explicitly calls it a kinematic view. No valid coupled
  dynamics qualification can come from those values.
- Recorded two-arm Jetson collision queries exceeded the available compute
  budget. Passing desktop tests does not establish Jetson deadline compliance.
- Jaw slides are in metres; motor references are in radians. The motor-to-jaw
  transmission and coupling need explicit physical calibration.
- Physical masses/inertias, actuator response/friction and independent validation
  are required for the Minerva MuJoCo model. Synthetic tests establish adapter
  behavior only.

See [work boundaries and gates](DEVELOPMENT.md), [ready task briefs](development/NEXT_TASKS.md),
[MuJoCo integration](MUJOCO.md) and the [measurement checklist](MINERVA_PHYSICS_MEASUREMENTS.md).
Each merged task should update this file with its source commit, evidence location and remaining
uncertainty, keeping long experiment history in the existing device logs.
