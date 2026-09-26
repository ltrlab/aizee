# Shared monorepo: implemented foundation

The repository now has **one installable Python distribution**, `ltrlab-robot-platform` (import namespace `robot_platform`), and **one Rust workspace**, rooted at `Cargo.toml`.

This is the first implementation stage of the [organization plan](MONOREPO_PLAN.md). Shared code and profiles are packaged. Existing collector/training applications run through explicit compatibility adapters while their large UI and workflow modules are migrated incrementally.

## What lives where now

| Area | Implemented responsibility |
|---|---|
| `pyproject.toml` | Package metadata, dependency extras, `robot-platform` command, offline test selection |
| `src/robot_platform/contracts.py` | Validated joint, camera, capability and robot definitions |
| `src/robot_platform/robots/definitions/` | Packaged Aizee and Minerva JSON definitions |
| `src/robot_platform/settings/` | Packaged defaults, ordered settings overlays, endpoint validation |
| `src/robot_platform/transport/` | Shared MessagePack wire format and network discovery |
| `src/robot_platform/control/` | RobotSession, telemetry validity/freshness, endpoint routing, explicit and threaded ZMQ clients |
| `src/robot_platform/data/formats/` | Aizee v4/v5 and Minerva v6/v7 HDF5 writers |
| `src/robot_platform/apps/` | Validated collection/training launch adapters |
| `settings/examples/` | Machine/operator overlays independent of application source |
| `Cargo.toml` | Five Rust crates in one root workspace: motor, comms, lidar, collision and guarded control core |
| `rust/control_core/` | Robot-independent guarded controller, CAN adapter, Linux commissioning service and native fault harness |
| `.cargo/config.toml` | Build output remains `rust/target` to preserve deployed service paths |
| `python/` | Legacy application implementations and compatibility import shims |
| `tests/unit/`, `tests/contract/` | Default offline tests, including both robot profiles |

The [repository map](REPOSITORY_MAP.md) and inventory describe the current checkout.
The [cleanup record](REPOSITORY_CLEANUP.md) lists retired source and local artifacts.

## Install

From the repository root, in your chosen Python environment:

```bash
python -m pip install -e ".[operator,dev]"
robot-platform list
```

Use `.[learning]` for model training and `.[language]` when real sentence-transformer embeddings are needed. `.[control,data]` installs the shared runtime/data dependencies without GUI or training dependencies. Device-specific RealSense/Jetson GPU packages retain their existing platform-specific installation instructions; they are not forced into the base package.

Use `.[preview]` for optional mesh simplification and `.[webxr]` for Quest
transport. Use `.[viewer,collision]` for the shared robot model viewer and FCL
inspection; [mechanical profiles](../MODEL_VIEWER.md) keep model selection
independent of motor/recording contracts. The legacy `/preview` route embeds
this shared viewer.

The old workstation/training requirements files were removed;
`pyproject.toml` owns these dependency selections.

The base package supports profile inspection and launch-plan generation without Torch, Qt, NumPy or camera SDKs. Data writers require the `data` extra. The explicit ZMQ client requires the `control` extra. The session's pure routing/observation code uses only the standard library.

On the Jetson, bootstrap/setup now install the shared editable package alongside `requirements_jetson.txt`, preserving the existing service interpreter. Existing installations also need this package installed before using updated `python/common/wire.py` compatibility shims.

## Select the robot and settings

The same commands work with either built-in robot:

```bash
robot-platform describe --robot aizee
robot-platform describe --robot minerva

robot-platform config --robot minerva --settings settings/examples/usb-tether.json
robot-platform run collect --robot minerva --settings settings/examples/usb-tether.json --plan
robot-platform run collect --robot aizee --settings settings/examples/usb-tether.json --plan
```

`--plan` only prints the resolved command, working directory and settings. It does not import the legacy application, probe the network, open a serial port or move a robot. Removing `--plan` runs the chosen application with its existing controls and startup behavior.

Training uses the same selector:

```bash
robot-platform run train --robot minerva --set applications.train.data_dir=episodes/minerva --plan
robot-platform run train --robot aizee --set applications.train.data_dir=episodes --plan
```

The new Aizee preset writes/reads `episodes/aizee` by default. Existing demonstrations in `episodes/` are preserved; select that path explicitly when training on them.

### Precedence and paths

1. Packaged settings for the selected robot.
2. Each `--settings FILE`, in the order supplied; nested objects merge and lists replace.
3. `--set dotted.key=value`, in order; values may be JSON scalars or lists.
4. `--host`, if supplied, overrides the host setting.

Endpoint strings containing `{host}` use the resolved host. Explicit camera/controller endpoint hosts are preserved, allowing separate machines. Aizee's optional scene camera can be disabled with `--set endpoints.cameras.scene=null`; required cameras cannot be disabled.

Settings files are JSON. Example:

```json
{
  "host": "10.42.0.1",
  "applications": {
    "collect": {
      "gui": false,
      "output_dir": "episodes/minerva",
      "options": ["--kp-scale", "0.3"]
    }
  }
}
```

Configured data/output paths are resolved against the caller's current directory. The compatibility process runs with the repository as its working directory so existing calibration/URDF lookups keep working. Other legacy path arguments passed in `options` retain their application's repository-relative behavior; use absolute paths for those when launching elsewhere.

Extra application arguments go after `--`, for example `-- --dry-run` for a collector or `-- --epochs 20` for training. Settings-owned options (including abbreviated spellings of endpoint/output/GUI flags) are rejected in this passthrough; change their settings fields instead. Existing Minerva GUI tuning preferences still apply to legacy tuning options; this stage does not replace that preference store.

The launcher preserves the resolved endpoints exactly. Direct legacy launches retain their original automatic host discovery. This difference is intentional: a concrete settings profile must not silently redirect to another machine.

## Interchangeable components and compatibility

`load_robot(name_or_json_path)` accepts built-in definitions or a custom schema-v1 JSON definition. `load_settings(spec, paths, overrides)` supplies deployment/application choices independently. `RobotSession(spec, settings)` consumes the same contract for Aizee, Minerva, or another definition with valid endpoint routes.

```python
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.control import RobotSession

spec = load_robot("minerva")  # or "aizee", or a custom definition JSON path
settings = load_settings(spec, overrides={"host": "10.42.0.1"})
session = RobotSession(spec, settings)  # no network connection
observation = session.observation()
assert not observation.ready  # no telemetry ingested yet
```

Joint routes must cover every active joint exactly once. Positional wire order is explicit. Duplicate endpoints, unknown cameras, incompatible robot settings and reordered group joints are rejected.

Minerva has 17 schema slots and 14 active arm/gripper slots. Head and lift are explicitly inactive. The new session exposes missing/inactive/stale observations as NaN with validity masks, and rejects nonzero commands to inactive joints. Historical recording formats retain their existing behavior through separate format adapters.

The packaged definitions describe the **native actuator telemetry**: arm and gripper positions are radians, while the reserved lift position is metres. They do not claim normalized gripper openings or measured jaw widths. Existing policy/GUI normalization conventions remain in the legacy models/adapters until an explicit calibrated conversion is introduced.

Application adapters also validate their joint/camera/capability contract. A compatible custom definition can reuse one of the existing adapters, but a seven-joint collector cannot be attached to a 17-joint definition by changing only its name. `--adapter` selects an explicit compatible adapter; mismatches fail before application startup.

## Shared backend behavior

- Wire helpers now have one implementation; old `common.wire` imports re-export it.
- Both collectors use shared discovery functions; Minerva no longer imports Aizee's private resolver.
- Legacy arm constants read joint vocabulary (and Minerva camera roles) from the packaged robot definitions; gains and calibration behavior remain in the existing modules.
- Both historical HDF5 writers now live in the package. Minerva no longer imports its writer from the Aizee collector package.
- `RobotSession` merges real Rust `motors` telemetry with monotonic receive times and validates routing without opening sockets.
- `ZmqRobotClient` has explicit connect/poll/send operations, including receive-only connections. It does not automatically enable motors, clip targets or implement application hold/shutdown policies.
- `ThreadedRobotClient` owns all its sockets on one worker thread, coalesces pending motion, and optionally resends a submitted target. Freshness is checked on every actual arm transmission; service commands have bounded queues and visible delivery failures.

Both collectors and the Minerva policy node now use the common session. The
Aizee collector retains its calibrated tracking computation and integrated
replay, while the Minerva collector retains partial-arm operation and legacy
17-slot arrays. Minerva inference uses separate left/right controller endpoints
and checks source freshness again after inference. See the
[session migration guide](SESSION_BACKEND_MIGRATION.md) for responsibilities,
behavior changes and remaining standalone tools.

## Rust workspace

On a Linux development machine or Jetson with the Rust toolchain and native dependencies:

```bash
cargo metadata --no-deps --format-version 1
cargo build --release -p motor_control
cargo build --release -p lidar_control
```

Crates remain under `rust/` for this stage, but `rust/Cargo.toml` is removed. Cargo finds the root workspace from the crate directories. The shared `comms` path dependency is declared once. Deployment bundles include the root manifest and `.cargo/config.toml`, and remove the obsolete nested manifest on the target. Services continue to use `rust/target/release/...`.

The root `Cargo.lock` has now been generated by Cargo and used for successful
release builds of both services. Windows can cross-compile for the dedicated
Jetson using workspace-local Rust/Zig tools:

```powershell
./scripts/build_jetson.ps1 -Bootstrap
./scripts/build_jetson.ps1 -Offline
```

Both `motor_control` and `lidar_control` have been built as ARM64 Linux ELF
binaries for the JetPack 6 / Ubuntu 22.04 baseline. See the
[Rust build guide](../../rust/README.md) for tool versions, dependencies and
artifact verification. Cross-compilation does not validate runtime behavior on
CAN buses or serial hardware.

## Validation and remaining migration

Run the installed package's default offline suite from the repository:

```bash
python -m pytest
```

Default discovery is restricted to `tests/unit` and `tests/contract`; existing motor exercises are not run implicitly. Existing ML/GUI integration suites remain explicit commands until they are classified and moved.

The implementation checks cover profile precedence, compatibility rejection, immutable launch plans, seven-/17-slot routing, missing/stale telemetry, fake ZMQ sockets, camera envelopes, discovery and v4–v7 recording contracts. Application `--help` is used to verify imports without starting the control loop. Wheel checks verify robot/settings resources are included.

The simulation baseline passed **248 Python tests** and **12 live process
scenarios** on 2026-09-17. Both Rust services cross-compile for Linux AArch64;
`comms` has six passing tests and one pre-existing ignored socket test. Current
simulation evidence and remaining physical gates are maintained in the
[simulation guide](SIMULATION_IMPLEMENTATION.md). Cleanup verification is
recorded in the [cleanup report](REPOSITORY_CLEANUP.md).

Still to migrate:

1. Migrate standalone Aizee inference, standalone replay and older diagnostic transports to RobotSession. Both collectors, integrated Aizee replay and Minerva inference now use it.
2. Extract collector acquisition/UI services and move legacy application implementations into the package. Current launch adapters require an editable checkout or `--repo`/`ROBOT_PLATFORM_REPO`; standalone wheels provide shared libraries and profiles, not all legacy assets/apps.
3. Move gains, calibrated limits, camera dimensions and tuning preferences into a unified validated configuration model. Current settings cover endpoints, app selection, paths/options and the session's freshness threshold; the migrated collectors now apply that threshold.
4. Consolidate machine/service configuration, runtime calibration storage, learning resources and additional test tiers.
5. Extend the dedicated Jetson's native collision builds/tests and scoped 25 Hz replay to production motor-service and full application validation. The development host is provisioned; physical motion and intended deployment-host checks remain. See [device log](../devices/JETSON_DEV_01.md).

No hardware deployment is performed by package installation, plan generation or the offline suite.
