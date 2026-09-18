# Local Jetson cross-compilation

The Windows development machine can build `motor_control` and `lidar_control`
for a Jetson Orin Nano without connecting to it. Both robots use these shared
binaries; their hardware YAML files select the physical devices and endpoints.

## Target and tools

The target is **Linux AArch64, glibc 2.35**, matching the Ubuntu 22.04 baseline
of [JetPack 6](https://developer.nvidia.com/embedded/jetpack-sdk-62). This is not
a CUDA build: these services use CPU, SocketCAN, serial devices, and ZeroMQ.

| Component | Version / location |
| --- | --- |
| Rust | 1.98.1, Windows MSVC host, `aarch64-unknown-linux-gnu` target |
| Zig | 0.14.1, downloaded from ziglang.org with a pinned SHA-256 |
| cargo-zigbuild | 0.23.3, official Windows release with a pinned SHA-256 |
| Host compiler | Existing Visual Studio 2022 C++ Build Tools |
| Dependency versions | Root `Cargo.lock`; RPLiDAR Git revision is locked |
| Tools and caches | `rust/target/jetson-toolchain/` (ignored by Git) |

The setup does not change the system PATH, enable WSL, or install Docker. The
PowerShell helper restores its process environment when it finishes. The
initial bootstrap downloads tools and Cargo dependencies; subsequent builds
can use the local cache.

## Build

From the repository root in 64-bit PowerShell:

```powershell
# First setup and release build. Requires network access.
./scripts/build_jetson.ps1 -Bootstrap

# Subsequent release builds, with no network access.
./scripts/build_jetson.ps1 -Offline

# Build one service when only that code changed.
./scripts/build_jetson.ps1 -Offline -Package motor_control
```

The equivalent Cargo operation is:

```text
cargo zigbuild --release --locked --target aarch64-unknown-linux-gnu.2.35 -p motor_control -p lidar_control
```

The helper supplies the repository-local tool/cache paths. The glibc suffix is
understood by [cargo-zigbuild](https://github.com/rust-cross/cargo-zigbuild);
Cargo's output directory omits that suffix:

```text
rust/target/aarch64-unknown-linux-gnu/release/motor_control
rust/target/aarch64-unknown-linux-gnu/release/lidar_control
```

These ELF files cannot execute on Windows. Native builds on the Jetson still
write to `rust/target/release/`; existing systemd paths refer to that location.
Installing cross-built artifacts into those paths belongs to the later Jetson
provisioning step. This helper performs no deployment or hardware operation.

## Verify binaries without running them

The build helper checks that both outputs are little-endian, 64-bit AArch64
ELF files and prints their SHA-256 hashes. For a complete library/version report:

```powershell
# pyelftools is an optional artifact-inspection dependency.
python -m pip install pyelftools
python scripts/build_jetson_verify.py --output rust/target/aarch64-unknown-linux-gnu/release/elf-inspection.json
```

The verifier rejects an unexpected machine/interpreter or a glibc requirement
newer than 2.35. It reads ELF metadata only and never runs a service.

### Verified build: 2026-09-17

Both services compiled successfully with the versions above. Inspection found:

| Property | Both binaries |
| --- | --- |
| Machine | ELF64 little-endian AArch64 |
| Interpreter | `/lib/ld-linux-aarch64.so.1` |
| Dynamic dependencies | `libc.so.6`, `libm.so.6` |
| Highest required GLIBC symbol version | 2.34 |
| Required GLIBCXX symbol versions | None |
| Embedded runtime library search paths | None |

ZeroMQ and the C++ runtime are included in the executables. The LiDAR crate
disables `serialport`'s optional `libudev` enumeration because it opens explicitly
configured paths; the underlying RPLiDAR driver uses the same feature selection.
Linux udev rules may still provide those device aliases at runtime.

The compiler emitted existing unused-import and dead-code warnings. Static
inspection establishes architecture and library compatibility; it does not
verify hardware behavior, CAN timing, serial permissions, or systemd startup.
Those checks remain for provisioning the dedicated Jetson.

The native Windows `cargo test --locked -p comms --offline` suite also passed:
six tests passed, including the Python-produced schema-v2 telemetry fixture,
with the existing manual socket test ignored. The shared
crate links Windows `advapi32` explicitly for vendored ZeroMQ; that import
library is selected by the compilation target and is absent from Linux builds.
