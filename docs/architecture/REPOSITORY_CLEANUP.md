# Repository cleanup — 2026-09-18

The pre-cleanup source snapshot is commit `fee7d6e`. Removed tracked files are
recoverable from Git history. Local checkpoints were already ignored and were
deleted at the user's request; they are not backed up by that commit.

## Removed and replaced

| Removed content | Reason / replacement |
|---|---|
| Old `checkpoints/` tree | 31 files, 6,679,157,933 bytes; obsolete ACT/JEPA weights and training logs |
| `viz/embeddings.png`, `viz/probe.png` | Generated results of old training runs; analysis tools recreate plots |
| Empty `test_models/`, stale Python `build/`, pytest cache | Unused local output; source and active environments retained |
| `config/hardware.yaml` | Obsolete three-joint topology; Rust now requires an explicit hardware profile |
| `config/teleop_rover_only.yaml` | Unreferenced old drive-only topology/address; current teleop profile remains |
| `python/scripts/mock_arm_publisher.py` | Replaced by `robot-platform sim run` with isolated settings, faults and provenance |
| `collect_minerva_app/cmd_sender.py`, `telem.py` | No production callers after the shared-session migration; old smoke-only paths removed |
| Old telemetry receiver functions in both collector receiver modules | Shared session owns motor telemetry; camera/UPS/e-stop receivers remain |
| `python/teleop/simple_test.py`, `detailed_motor_test.py` | Obsolete JSON telemetry and three-joint examples |
| `tests/integration/` two-motor scripts | Missing hardware config, obsolete rosters and JSON telemetry |
| `tests/utils/capture_can_frames.sh`, `debug_motor.sh` | Missing config references; capture script also had empty PID variables |
| `scripts/check_system_status.sh` | Duplicate historical host check; use dashboard or `check_rover_status.sh` |
| `scripts/fix_can1_on_jetson.sh` | One-off service migration with obsolete workstation paths; deployment scripts install current units |
| `scripts/reset_usb_can_adapter.sh` | Duplicate remote reset implementation; deployed `aizee-reset-usb-can` remains |
| `rust/bindings/` and PyO3-only lockfile dependencies | Placeholder extension with no consumers; shared integration is ZMQ |
| `requirements.txt`, `requirements_training.txt` | Superseded by package extras; preview dependencies moved into `.[preview]` |
| Unconsumed Minerva YAML fields | Duplicate morphology, unused model/logging/ramp fields and legacy single-arm endpoints; definitions/settings/checkpoints own those values |

## Retained deliberately

- Recorded episodes and recent simulation reports/traces: unique data and
  validation evidence, not obsolete source.
- Active `python/` application, learning, calibration, replay and diagnostic
  CLIs, including compatibility imports used by scripts and tests.
- Hardware calibration snapshots, firmware, the required OpenRB BSP, and
  Aizee CAD/meshes. No audited Minerva CAD model replaces these yet.
- `requirements_jetson.txt`: device-specific NumPy/OpenCV installation
  constraints. Workstation extras must not override the JetPack runtime.
- The dated actuator proposal and learning research notes, labeled as historical
  planning rather than the current hardware inventory or implementation status.
- The local cross-build toolchain and Python environment used for verification.

## Documentation/configuration updates

The root README, repository map, test guide, motor-controller guide and roadmap
now describe the shared platform. Installation uses package extras. The Jetson
guide distinguishes development-host setup from Aizee robot provisioning.
Configuration ownership is documented in `config/README.md`; current hardware
and calibration values remain unchanged. The generated inventory includes the
shared package and omits removed paths.

Generated checkpoint/plot directories stay ignored. Future inference examples
refer to newly trained or explicitly supplied weights; no checkpoint is assumed
to exist after cleanup.

`scripts/run_policy.sh` now requires a checkpoint argument and reports usage
before any connection if it is absent. Host/key values can be supplied through
environment variables, and remote arguments retain quoting.

## Verification

- 248 offline Python regressions passed after the final configuration cleanup.
- The offscreen Minerva collector passed recording/dataset, camera, GUI,
  segmentation and settings checks; receiver/GUI shutdown passed separately.
  The shutdown test now waits for initialization instead of assuming a one-second
  startup. Both GUI tests isolate preferences and disable speech backend discovery.
- Native Rust `comms`: six passed, one existing ignored network test.
- Both Rust services rebuilt for AArch64 Linux with the pruned lockfile.
- Python wheel rebuilt; config JSON/YAML parses; policy shell syntax and
  missing-checkpoint behavior checked without contacting a robot.

The existing 12-case simulation report remains the 2026-09-17 baseline; it was
not rerun as a separate matrix for this cleanup. No hardware was deployed or
powered. Build warnings about existing unused Rust members remain.
