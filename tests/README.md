# Tests and hardware tools

## Offline regression suite

From the repository root with the shared package installed:

```sh
python -m pip install -e ".[simulation,operator,dev]"
python -m pytest
```

Default discovery covers `tests/unit/` and `tests/contract/`. These exercise
robot/settings contracts, shared sessions, freshness and command leases, real
loopback transport, collector subprocesses, and recording formats. They do not
open physical robot interfaces. Optional dependencies are required for the
collector process tests; absent dependencies are reported as skips.

Additional regression checks:

```sh
python -m pytest tests/direct_can/test_actuator_char.py tests/ml/test_record_replay.py python/tests/ml/test_minerva_autosegment.py python/tests/ml/test_minerva_settings.py python/tests/ml/test_minerva_segments.py::test_minerva_segments
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

## Directory ownership

| Directory | Use |
|---|---|
| `unit/`, `contract/`, `fixtures/` | Default offline validation and wire fixtures |
| `ml/` | Explicit learning and replay regression checks |
| `python/tests/ml/` | Existing Minerva model/GUI/recording checks |
| `direct_can/` | Explicit hardware experiments, plus offline `test_actuator_char.py` |
| `utils/` | Manual CAN/command diagnostics; not automatic tests |

Old two-motor integration scripts and JSON-only telemetry exercises were
removed. They referenced absent configs or obsolete robot names. The shared
simulator replaces the old mock publisher. Production Rust + virtual CAN
integration is the next implementation stage and is not available yet.

## Physical characterization

Hardware diagnostics are separate, explicitly invoked tools. Their old example
motor IDs and gains are not an approved test envelope. Use the
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
