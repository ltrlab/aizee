# Tests and hardware tools

For the current required development gate, use `python scripts/dev.py check`.
See [development workflow](../docs/DEVELOPMENT.md) and
[current status](../docs/DEVELOPMENT_STATUS.md). Counts and commissioning notes
below are historical evidence from their stated dates, not the current gate.

GELLO viewer/input increment (September 26 UTC): **110 focused Python tests
passed** across leader input, concurrent board reads, source bridge, display
calibration and virtual teleop. They cover partial-pair rejection, original
timestamp preservation, transient display staleness without controller rearm,
calibration identity binding, loopback request boundaries, full-cycle deadlines
and late/lost registration acknowledgement reporting. The short actual-leader
25 Hz virtual receiver run passed 250 cycles; see
[GELLO evidence](../docs/devices/MINERVA_GELLO_VIEWER_20260926.json).

Live calibration increment: **541 Python unit/contract tests passed**, followed
by **56 calibration/viewer tests** after the final live-feed changes.
Direction/offset candidates, rejected
partial or stale reads, identity mismatches, return-to-reference, resumable steps,
Windows file replacement and live viewer revision checks are covered in
`unit/test_joint_calibration.py`. The stopped reader was also exercised on all
14 Minerva motors. See [calibration workflow](../docs/JOINT_CALIBRATION.md).

September 24 guarded teleop/reference increment: **512 Python unit/contract
tests and 47 portable Rust tests passed**, including collision-envelope expiry,
per-pair swept bounds, source-aged OpenRB/bridge input, raw encoder reference
checks and disabled-only mechanical-zero failure handling. ARM64 control binaries
cross-compiled; Clippy passed with warnings denied. The production-host vcan
collision run passed 1,000 cycles at 25 Hz. USB/network timing failures remain
recorded and do not qualify live hardware teleoperation. See
[current evidence and limits](../docs/GUARDED_TELEOP.md).

Collision-limit review tests in `unit/test_limit_sweep.py` cover independent
branches versus the shared body, coupled jaw motion, first-contact refinement,
inward margins, unresolved-contact locking, and revision-bound viewer ranges.

## Offline regression suite

### Native guarded controller

The Linux commissioning tier also runs the actual SocketCAN/ZMQ receiver:
`scripts/guarded_vcan_bench.py` exercises startup gates, expiry, stale feedback,
replay, invalid batches, scheduler stalls, SIGTERM and restart epochs on vcan.
See [commissioning instructions](../docs/POWERED_COMMISSIONING.md). It never opens
a physical bus for motor emulation. These are synthetic protocol/timing tests,
not motor-dynamics or collision-supervisor qualification.

Portable CAN tests require fresh pinned watchdog proof before physical enable
and reject changed identity/readback, stale replies, receive failures and reused
proof. Finite trial tests cover bounded breakaway in both directions, no motion,
effort/pose failures and post-stop overspeed. A movement detection alone cannot
pass a trial that exceeds its stop bounds. Staged damping tests also cover reply
loss, unexpected Reset, excess speed/travel, failure to settle and command gaps.
Latest local checks: 40 Rust tests and
452 Python unit/contract tests; physical outcomes are in the device log.

Build and test the portable Rust core with `cargo test --locked -p robot_control`
and `cargo build --locked -p robot_control --bin control_harness`, or run
`./scripts/test_control_core.ps1` on the configured Windows development host.
Then run `python -m pytest tests/contract/test_guarded_controller.py -q`.
These contract tests execute Rust through MessagePack and cover Aizee/Minerva
session routing and faults. They skip explicitly if the binary is absent.
See the [protocol and limitations](../docs/architecture/GUARDED_CONTROLLER.md).

### Python suite

`unit/test_transmission_calibration.py` checks offline encoder/aperture fitting:
both transmission signs, direction-dependent error, small/one-direction datasets,
uncertainty, ambiguous or faulted diagnostics, source/identity consistency,
interpolation bounds and preservation of existing output. Synthetic test data
does not establish hardware calibration.

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
