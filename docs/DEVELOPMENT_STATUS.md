# Current development status

Updated 2026-10-10. This file is the current entry point; dated phase tables and
device logs retain historical evidence, not an implicit statement of readiness.

The integration checkpoint preserves the 11 uncommitted overhaul files captured
after `a66afa7`. The original `overhaul` working directory remains independent.
The two test failures seen in the earlier review were subsequently edited in that
checkpoint; the new baseline must be verified against its own frozen source.

## Current work

The Windows baseline passed 95 required Python tests with zero skips and all
portable controller/collision Rust tests on 2026-10-10. The local evidence is
`build/dev-check/20261010T155025Z-3ee526b4/report.json`. GitHub CI is configured
for Windows and Linux but has not yet run remotely.

- Establish a fresh-worktree, hash-locked offline control gate and CI.
- Use MuJoCo for coupled physics behind `PlantReceiver` and the actual Rust
  receiver; retain `ArmPlant` for cheap, deterministic fault scenarios.
- Audit Minerva physical inputs before treating a simulation as robot evidence.

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

See [work boundaries and gates](DEVELOPMENT.md) for the branch order. Each merged
task should update this file with its source commit, evidence location and remaining
uncertainty, keeping long experiment history in the existing device logs.
