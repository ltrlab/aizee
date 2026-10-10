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
`codex/teleop-direct-mirror` is prepared from integration commit `8b4f2c4`, with
the [implementation and acceptance brief](development/DIRECT_TELEOP.md). Runtime
behavior has not changed yet: the existing advisory mode still computes collision
queries and the physical receiver requires a collision policy. Implement explicit
disabled behavior first, then measure and shorten the command path before increasing
the update rate. Collision timing optimization and physical-model identification
are separate work; Linux vcan, Jetson timing and hardware qualification remain open.

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
