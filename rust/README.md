# Shared Rust backend

The repository root owns the only Cargo workspace. Aizee and Minerva use the
same backend binaries with different hardware configuration files.

| Crate | Responsibility |
| --- | --- |
| `comms` | Shared command/telemetry types, MessagePack, and ZeroMQ transport |
| `motor_control` | ROBSTRIDE CAN control; configuration selects an arm or rover |
| `lidar_control` | RPLiDAR acquisition and telemetry |
| `robot_collision` (`rust/collision`) | Persistent collision bounds and a shared-library interface for CAD-confirmed checks |
| `robot_control` (`rust/control_core`) | Portable guarded core, native harness, RobStride adapter with mandatory watchdog startup, Linux SocketCAN/ZMQ service and finite watchdog/jog/breakaway trials |

## Build from the repository root

```bash
cargo build --release -p motor_control
cargo build --release -p lidar_control
cargo test -p comms
cargo test --locked -p robot_control
cargo build --locked -p robot_control --bin control_harness
```

`motor_control` requires Linux SocketCAN. Native Linux builds need Rust and a
C/C++ toolchain; `zmq-sys` builds its bundled ZeroMQ sources. LiDAR opens the
configured serial-device paths and disables the unused `serialport` enumeration
feature, so it does not require linking `libudev`. `scripts/setup_jetson.sh` installs the
device build dependencies.

## Cross-compile locally for Jetson

Windows x64 can build both Linux AArch64 services using the existing Visual
Studio C++ Build Tools and a repository-local Rust/Zig toolchain:

```powershell
./scripts/build_jetson.ps1 -Bootstrap  # First setup; downloads pinned tools
./scripts/build_jetson.ps1 -Offline    # Rebuild using cached dependencies
```

See [Jetson cross-compilation](JETSON_CROSS_BUILD.md) for output paths, runtime
requirements, artifact verification, and validation limits.

Cargo also discovers the root workspace when invoked from `rust/` or a crate
directory. Use `-p` to select a package when building from the root or `rust/`.
Do not create another `rust/Cargo.toml` workspace manifest.

The root `.cargo/config.toml` keeps build output in `rust/target/`. For example:

```bash
AIZEE_CONFIG=config/hardware_minerva_left.yaml ./rust/target/release/motor_control
```

The right-arm service uses the same executable with
`config/hardware_minerva_right.yaml`; the Aizee rover uses
`config/hardware_jetson_rover.yaml`. Existing systemd executable paths remain
valid. Run Cargo from inside this checkout so it loads the root Cargo config.

## Deployment and dependency locking

Deployments must copy the root `Cargo.toml`, `.cargo/config.toml`, and all five
workspace members, even when building one package. Cargo loads every member's
manifest while resolving the workspace. Deployment scripts also remove the
obsolete remote `rust/Cargo.toml` when upgrading an existing checkout.

The shared lockfile is `/Cargo.lock`, including the resolved RPLiDAR driver's
Git revision. Use `--locked` for reproducible dependency selection. Deployment
scripts include it when present. Old per-crate lockfiles are ignored and are
not used by this workspace. To update dependencies intentionally, use
`cargo update`, then review the lockfile changes and rebuild both services.
