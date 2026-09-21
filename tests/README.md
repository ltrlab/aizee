# Tests and hardware tools

Collision-limit review tests in `unit/test_limit_sweep.py` cover independent
branches versus the shared body, coupled jaw motion, first-contact refinement,
inward margins, unresolved-contact locking, and revision-bound viewer ranges.

## Offline regression suite

From the repository root with the shared package installed:

```sh
python -m pip install -e ".[simulation,operator,dev]"
python -m pytest
```

Default discovery covers `tests/unit/` and `tests/contract/`. These exercise
robot/settings contracts, shared sessions, freshness and command leases, real
loopback transport, collector subprocesses, recording formats, and actuator
codec/analysis checks. They do not open physical robot interfaces. Optional
dependencies are required for the
collector process tests; absent dependencies are reported as skips.

Additional regression checks:

```sh
python -m pytest tests/ml/test_record_replay.py python/tests/ml/test_minerva_autosegment.py python/tests/ml/test_minerva_settings.py python/tests/ml/test_minerva_segments.py::test_minerva_segments
python python/tests/ml/test_minerva_collector_smoke.py
python python/tests/ml/test_minerva_shutdown.py
cargo test --locked -p comms
```

Run the Qt checks in their own processes: Qt application ownership/teardown is
process-wide. They use offscreen rendering and explicitly disable leader
hardware. Recording dataset checks also need the learning dependencies.

## Live simulation matrix

```sh
python scripts/validate_simulation.py --output simulation_runs/validation-new
```

Both robots run nominal, feedback-loss, frozen-feedback, malformed-feedback,
camera-loss and emergency-stop scenarios. The output directory must be new.
Reports include traces, camera decoding, lease expiry/renewal, source freshness,
and stop/clear behavior. This is a short functional matrix, not the planned
60-minute soak or physical calibration. See the
[simulation guide](../docs/architecture/SIMULATION_IMPLEMENTATION.md).

## CAD import audit

```sh
python -m robot_platform sim audit-model --urdf urdf/minerva/urdf/minerva_urdf.urdf --package-root urdf/minerva --expectations urdf/minerva/source_manifest.json
```

This offline command checks local mesh references, graph structure, joint
names/types/limits, and numeric inertias. Exit code 2 is expected for the first
Minerva export; it fails the intended joint contract. Static success does not
validate density, coupling, actuator response or real dynamics. See the
[import findings and repair sequence](../urdf/minerva/README.md).

Auditor and CLI regression cases live in `unit/test_model_audit.py` and
`unit/test_model_audit_cli.py`; they require the simulation dependencies.

## Directory ownership

| Directory | Use |
|---|---|
| `unit/`, `contract/`, `fixtures/` | Default offline validation and wire fixtures |
| `ml/` | Explicit learning and replay regression checks |
| `../python/tests/ml/` | Existing Minerva model/GUI/recording checks |

Old two-motor scripts, unverified duplicate CAN codecs, enable-on-import scans
and JSON-only command exercises were removed. The retained actuator codec and
analysis cases now live in `unit/test_actuator_char.py`. The shared simulator
replaces the old mock publisher. Production Rust + virtual CAN
integration is the next implementation stage and is not available yet.

## Physical characterization

Hardware diagnostics are separate, explicitly invoked tools: passive discovery
in `python/nodes/find_motors.py`, targeted diagnosis in
`python/scripts/minerva_motor_probe.py`, and acquisition through
`python/scripts/characterize_actuator.py`. Use the
[measurement plan](../docs/architecture/SIMULATION_VALIDATION_PLAN.md) and
`settings/examples/bench-profile.template.json` to establish the actual device,
firmware, codec, limits, and stop behavior first. The template intentionally
does not authorize a powered run.

The current audit reports unresolved scale and calibration-limit discrepancies:

```sh
robot-platform sim audit --repo . --output minerva-hardware-audit.json
```

Exit code 2 indicates outstanding hardware prerequisites. It is not a
simulation-test failure. Motor-reported torque is the first measurement source;
independent torque validation remains later work.

## Offline collision and contact detection

The shared model/viewer suite is `unit/test_model_viewer.py`. Install `viewer`
and `collision` extras to exercise mesh conversion, local HTTP routes and FCL.
It also tests an unrelated wheel/slide topology, model/profile validation,
coupled limits and saved pose reuse across tools.

With the optional `collision` dependencies installed, run:

```sh
python -m pytest tests/unit/test_collision.py tests/unit/test_collision_cli.py tests/unit/test_limit_probe.py
python -m robot_platform sim validate-limit-probe
```

These cover whole-robot geometry, jaw mimic/frames, synchronized sampled paths,
missing geometry/state rejection, source freshness and sustained torque/stall
classification. They send no hardware commands and do not validate physical
limits or runtime collision prevention. See the
[implementation and remaining gates](../docs/architecture/COLLISION_AND_LIMIT_CALIBRATION.md).

## Native collision, keep-out and viewer scheduling

Build the native library before testing so native cases run rather than skip:

```powershell
./scripts/build_collision.ps1 -Test
.venv-platform/Scripts/python.exe -m pytest -q
node --test tests/viewer/collision-checks.test.mjs
```

On Linux, use `cargo test --locked -p robot_collision` and
`cargo build --locked --release -p robot_collision`, then the active Python
environment. Native ABI 3 is required by the Python adapter.
The September 21 checkpoint passed **413 Python, six Rust collision and seven
JavaScript tests** on the desktop. Native/selected Python tests also passed on
the identified development Jetson; see its [log](../docs/devices/JETSON_DEV_01.md).

Runtime regressions cover enclosing geometry validation, CAD refinement,
scope partitioning, explicit finger omissions, swept clearance, changing
prismatic reach, policy identity, stale state and simulation command dispatch.
Browser scheduling tests cover coalesced edits and invalidation when pose,
model, scope, geometry mode or inspection gap changes.
The offline [keep-out benchmark](../scripts/benchmark_keepout.py) records pose
latency, source-CAD audits and optionally paced synthetic paths. The 25 Hz
inter-arm run does not qualify whole-robot motion or physical stopping.
