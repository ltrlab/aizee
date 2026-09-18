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
  CLIs, including compatibility imports still used by applications.
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

## Follow-up audit: readers, writers and generated state

The first cleanup (`3b551be`) incorrectly counted write references as evidence
that `config/ready_pose.json` was used. The follow-up traced reads separately
from writes and checked application callers instead of relying on test imports.

| Removed or changed | Evidence / replacement |
|---|---|
| `config/ready_pose.json` and Aizee/Minerva Save Ready controls | No reader; only P-key handlers wrote these files. Removed handlers, UI controls, shortcuts and CLI option so they cannot recreate unused state. Policy poses in datasets/checkpoints remain active. |
| `config/calibration_poses.json` | Old manually selected capture, not a default input. New captures go to ignored `calibration_runs/aizee/` and are selected explicitly with `--poses`. |
| Generated fields in `config/joint_limits.yaml` | Collision counts, sampled postures, timings and an absolute workstation path had no runtime consumer. Preserved every configured effective/URDF bound and manual-edit marker. New sweeps default to an ignored report rather than overwriting configured limits. |
| `tests/direct_can/{motor_control_test,sine_wave_test}.py` | Obsolete two-motor experiments with duplicate, unverified codecs. The retained characterization codec/analysis suite moved to `tests/unit/test_actuator_char.py` for default discovery. |
| Remaining `tests/utils/` scripts and `python/nodes/query_motors.py` | Old command formats, hardcoded prototype IDs and duplicate active scans, including an enable-on-import scan. Retained passive discovery, targeted Minerva diagnosis and the verified-profile characterization CLI. |
| `collect_demo_app/minerva_recording.py` | Test-only compatibility shim; production Minerva uses its own adapter to the shared writer. Tests now import the shared writer directly. |
| Old commented distributed topology in `config/teleop.yaml` | Obsolete secondary host example, superseded by shared robot settings. |

The [configuration reader audit](../../config/README.md#calibration-status)
documents why each remaining calibration/alignment file is retained. Active
hardware values were preserved. Guides, shortcuts and the inventory now match
the remaining source tree.

Follow-up verification:

- 248 offline regressions passed, including both collector subprocesses and
  the characterization tests now discovered under `tests/unit/`.
- Minerva offscreen recording/GUI checks passed. The Aizee window constructed
  offscreen; retained calibration controls queued the expected actions, and
  the removed P shortcut/button were absent. No commands reached hardware.
- All retained calibration/alignment JSON matched the prior commit. Loading
  both old and cleaned joint-limit YAML through the actual IK implementation
  produced identical lower/upper bounds. The existing FK/IK check also passed
  all 50 warm-start cases (cold-start diagnostic: 14/20, informational).
- All 22 config/settings JSON/YAML files parsed; the collision-sweep CLI
  advertised the new report path. Updated Python files parsed successfully.
- All 466 local Markdown links resolved. The inventory regenerated without
  parse errors, and all 11 deleted tracked files were absent on disk.

No Rust source changed in this follow-up; the preceding Rust tests and Jetson
build remain applicable. The full CAD collision sweep and physical calibration
were not run as part of cleanup.
