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

Earlier required Windows gate: **251 Python tests, zero skips/xfails, and 111 Rust
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

These earlier samples establish synthetic receiver-path overhead only. Harness
application delay reads zero because application shares a logical tick with
acceptance; it is not measured physical latency. Keep the 100 Hz default; the
200 Hz run is an evaluation, not approval for deployment.

### Direct teleop live-test preparation

**Prepared for an attended left-arm trial after authorized activation and stopped
motor checks. Dual-arm timing qualification remains blocked.** The wired Jetson
is available at `10.42.0.1`; isolated user/network namespaces made actual Linux
vcan testing possible without exposing physical CAN. Both leader USB boards were
present. No physical CAN command, motor enable, service activation or active-release
switch was performed; physical canL/canR remained DOWN.

- Receiver vcan: 3000/3000 at 100 Hz, 2.88 ms p99 processing, zero deadline misses,
  and replay/expiry/requested-stop/stale-feedback/emergency-stop checks passed.
- Actual full loop, synthetic left-arm input: 1000/1000, 4.83 ms p99, zero following
  misses; recorder/UI enabled and X return/disable completed.
- Actual full loop with real USB leaders over the tether and independent vcan:
  1000/1000, **6.46 ms p99**, max 8.89 ms, zero following deadline misses, one RPC
  per cycle, all 1000 applications observed. 911 unique leader samples, about
  91 Hz, with 25.60 ms p99 conservative fresh-source age. Leaders were nearly
  stationary; this does not qualify moving-leader or physical tracking.
- Dual-arm final candidate: 1604 updates accepted before one 10.00 ms overrun
  skipped a slot and the receiver expired the anchored command. Its 6.41 ms p99
  alone is insufficient for a pass. First divergence and all failed trials remain
  in the evidence; no watchdog or timing acceptance was relaxed.

Profiling found whole receiver-result copies in the direct feedback cache,
including recorder history (1.34 ms median on Jetson). The cache now owns only
the feedback it needs, retaining same-reply checks, source age and duplicate
validation. The vcan fixture's motor emulator also moved to a separate process
after application garbage collection caused a false feedback timeout. These
are separate, evidenced fixes; the dual-arm outlier remains an open task.

Final frozen-source Windows gate passed **403 Python tests, zero skips/xfails,
and 111 Rust tests**. Evidence is
`build/dev-check/20261010T175128Z-bdca2760/report.json`; the earlier expanded gate
also passed 397 Python tests. Results are recorded in
`build/direct-qualification/live-readiness.json`. Only documentation was updated
after the final gate; tested runtime and benchmark sources stayed unchanged.

The candidate under `/home/ltr/minerva/validation/direct-20261010/release` passed
all software-only Minerva Check stages and receiver configuration validation for
left/right/both. Motor reads and live sweeps were skipped, so Check is correctly
INCOMPLETE. The active release remains `20261010T142650Z-a66afa7`; Helm still runs
`current`, even when preflight used `--remote-root`. Candidate activation is a
separate deployment-owner action before a physical trial.

The launcher now verifies a checkout-owned Python import and supports an explicit
existing SSH key. Saved operator signs/history were preserved in this worktree;
two calibration binding differences were proved to be line-ending-only before
copying. The original 11 dirty overhaul files and original evidence remain intact.
See the [trial sequence and exact limitations](development/DIRECT_TELEOP.md#live-trial-preparation-2026-10-10).
Aggregate and full Jetson evidence: `build/direct-qualification/live-readiness.json`
and `build/direct-qualification/readiness-evidence.tar.gz`. Integration is not merged.

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
