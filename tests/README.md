# Tests

## The gate

```sh
python scripts/dev.py setup     # creates .venv-dev from the hash-locked development requirements
python scripts/dev.py check     # Rust build and tests, then the required Python tests, zero skips or xfails
python scripts/dev.py quick     # the gate's tests that touch what changed since the last passing check
```

`check` is the required development gate; `quick` is the shorter lane for changes that leave the motion contract, Rust
and the locks alone. The workflow is in [DEVELOPMENT.md](../docs/DEVELOPMENT.md), CI in
[CI_RELEASE_GATE.md](../docs/development/CI_RELEASE_GATE.md). Each run writes a report under `build/dev-check/`.

The gate runs an explicit list of files (`CORE` in `scripts/dev.py`), not just pytest's default discovery. Plain
`python -m pytest` runs `tests/unit` and `tests/contract` (`pyproject.toml`). A passing offline gate is not hardware
qualification: deterministic tests come first, then Linux vcan, Jetson timing and supervised hardware validation.

**What the gate leaves out.** Tests that need torch or OpenCV (the environment does not install them), and
`tests/unit/test_helm_webrtc_peer.py`, which needs `aiortc` and is run by name from an environment that has it
(`requirements/helm-webrtc.in`). The Qt and learning checks under `tests/ml/` and `python/tests/ml/` are also explicit
commands.

## Layout

| Directory | Use |
|---|---|
| `unit/` | Single-module tests: control, Helm, teleop, neck, ops, data, simulation, vision. |
| `contract/` | Cross-module behaviour: the session adapters, the Python-to-Rust guarded receiver (it skips explicitly if the binary is absent), Helm teleop against the virtual receiver, recording, policy mode, evaluation. |
| `ui/*.mjs` | Node tests of Helm's and the headset's browser scripts. Run through `unit/test_browser_scripts.py` and `node --test`. |
| `viewer/` | `collision-checks.test.mjs`, the viewer's browser scheduling (`node --test tests/viewer/collision-checks.test.mjs`). |
| `evidence/` | Tests that replay recorded evidence which is not in the repository (`test_sim2real_20261015.py` skips when the report is absent). Run by name with `MINERVA_EVIDENCE` and `MINERVA_OPERATOR_RUNS` set. |
| `fixtures/` | Wire fixtures, task-scene and workcell files, physics inputs. |
| `ml/`, `../python/tests/ml/` | Learning and replay regressions, and the older Minerva collector/segment checks. |

`tests/unit/test_browser_scripts.py` also parses every script under `helm/static/` and `viewer/static/` as an ES module,
as the page loads it.

## Native and virtual-CAN tests

```sh
cargo test --locked -p robot_control -p robot_collision
./scripts/test_control_core.ps1                          # Windows host
python -m pytest tests/contract/test_guarded_controller.py -q
```

The contract tests run the Rust receiver through MessagePack and cover both robots' session routing and faults.
Virtual CAN integration exists: on Linux, `scripts/guarded_vcan_bench.py` runs the actual SocketCAN/ZMQ receiver
against synthetic motors on vcan (startup gates, expiry, stale feedback, replay, invalid batches, scheduler stalls,
SIGTERM and restart epochs), and `scripts/benchmark_guarded_teleop_vcan.py` times the guarded loop. It never opens a
physical bus. These are protocol and timing tests, not motor-dynamics qualification.

Build the collision library before the collision tests so native cases run rather than skip
(`./scripts/build_collision.ps1 -Test` on Windows; on Linux `cargo build --locked --release -p robot_collision`).
The Python adapter needs native ABI 3.

## Other offline checks

```sh
python scripts/validate_simulation.py --output simulation_runs/validation-new      # new output directory
python -m robot_platform sim audit-model --urdf urdf/minerva/urdf/minerva_urdf.urdf --package-root urdf/minerva \
    --expectations urdf/minerva/source_manifest.json                               # exit 2 is expected for that export
robot-platform sim audit --repo . --output minerva-hardware-audit.json             # exit 2: outstanding hardware prerequisites
```

`validate_simulation.py` runs both robots through nominal, feedback-loss, frozen-feedback, malformed-feedback,
camera-loss and emergency-stop scenarios; see the [simulation guide](../docs/simulation/SIMULATION_IMPLEMENTATION.md).
Exit code 2 from the audits means outstanding prerequisites, not a test failure.

## Hardware tools are not tests

Hardware diagnostics are explicitly invoked tools: passive discovery in `python/nodes/find_motors.py`, targeted
diagnosis in `python/scripts/minerva_motor_probe.py`, acquisition through `python/scripts/characterize_actuator.py`.
`settings/examples/bench-profile.template.json` does not authorize a powered run. Nothing in the default suites opens a
physical interface.
