# CI and mandatory release prerequisites

CI and the developer gate need the same prerequisites: an ARM64 build of the robot binaries (so the release test does not
skip), exact file bytes, and a software renderer. This page records what they are and why. Control bounds, receiver
behaviour and robot operation are not touched by any of it.

**Triggers.** `.github/workflows/offline-control.yml` runs on pull requests, on pushes to `main`, and on manual dispatch,
on `ubuntu-latest` and `windows-latest`.

**Why the ARM64 build is a prerequisite.** A clean checkout has no ARM64 `guarded_controller`, so
`test_a_built_release_is_small_complete_and_verifiable` would skip while pytest still returns success. The developer
gate rejects any skip, so the cross-build is a required step, not an option.

## Build and check

With CPython 3.12, Node.js 22, Git LFS and Rust 1.98.1 selected:

```sh
python scripts/dev.py setup
python scripts/build_release.py --bootstrap
python scripts/dev.py check
```

The cross-build helper installs the exact hash-locked Windows/Linux x86_64 wheels
from `requirements/cross-build.lock` into `build/cross-tools/venv`, adds the
`aarch64-unknown-linux-gnu` Rust target, and builds `robot_control` with its Linux
service feature plus `robot_collision`. It uses Zig 0.14.1 and cargo-zigbuild
0.23.3 with glibc 2.35. It builds all three required receiver binaries and the
collision shared library. No ARM64 binary is executed, and no robot is contacted.

`--jobs` bounds compiler concurrency (default 2; use 1 alongside another build).
Native gate builds default to `CARGO_BUILD_JOBS=2`; callers may set that variable.
Outputs and Zig caches always belong to this worktree. Shared compiler/download
caches are allowed; another checkout's native outputs or editable install are not.
After bootstrap and dependency caching, `--offline` prohibits Cargo downloads.
Bootstrap and offline are mutually exclusive; offline never installs missing tools.

The developer gate preflights release inputs before native tests, retaining the
existing release validator's presence, AArch64 and source-freshness requirements.
It records all four ARM64 hashes as well as the native harness/collision hashes.
The release archive test remains required, and any skip/xfail still fails.

CI runs the same setup, cross-build and check commands on both Windows and Linux.
Its uploaded evidence includes `build/cross-build/` and `build/dev-check/`, including
failure logs. These reports establish build/package/control evidence only. Linux
vcan, loaded Jetson timing and attended physical qualification remain separate.

Acceptance: clean Windows and Linux jobs both complete without required skips or xfails; source identity remains
unchanged; native and ARM64 artifact hashes and a verified release archive are present in the gate evidence. A workflow
file alone is not a passing run; record the actual CI result.

## Portability rules the first clean runs established

- The qualified joint limits bind the exact bytes of `config/devices/minerva-reference-20261009.json` (including its CRLF
  line endings), so Git preserves that file's bytes on every checkout policy. Checksum validation stays exact, and a
  regression checks real Git exports under both policies.
- MuJoCo joint validation compares numeric enum values, not platform-specific pybind enums, and still accepts only
  hinge and slide joints.
- Scheduling tests exercise the documented Linux no-op contract and keep the real Windows priority and timer assertions.
  No required skip is accepted.

## Wheel and capture dependencies

The required packaging regression builds a real wheel, installs it into a temporary
directory, then verifies Helm's HTML/CSS/JavaScript from outside the checkout.
It also checks the wheel's dependency metadata. Install the `helm` extra for the
operator UI and live capture dependencies; use the smaller `capture` extra for the
robot-side capture dependencies alongside the existing robot runtime. Both include
the base package's MessagePack dependency. Add the `physics` extra on a simulation
host; task scenes require MuJoCo 3.15's surface-velocity support. The existing
development lock already pins MuJoCo 3.15.0 and did not need regeneration.

Robot release manifests declare `h5py` and `PIL` in addition to the existing runtime
modules. Smoke checks import every declared module before loading the model or
starting the native synthetic harness. A missing/broken dependency names the module
and fails smoke, allowing the existing deployment rollback behavior to apply.
Host task simulation is not a robot release dependency. Provision robot dependencies
before activating a release; copying the release archive does not install packages.

## Actual camera rendering on CI

The task receiver/capture contract renders the configured head and right-wrist
cameras through MuJoCo. Ubuntu uses the system `libosmesa6` software renderer and
`MUJOCO_GL=osmesa`, selected before Python imports MuJoCo. This requires no display
server or GPU. Hosted Windows keeps its offscreen GLFW backend and explicitly
loads the pinned Mesa 26.2.4 MSVC x64 software driver. Both jobs must execute
the render contract; missing graphics support is a failed prerequisite, not an
allowed skip. Rendered simulation frames do not qualify physical camera calibration.

A hosted Windows runner has no usable OpenGL (the scene contract fails with `gladLoadGL error`), which is a missing
rendering prerequisite, not a camera-contract waiver.

`python scripts/ci_windows_gl.py` downloads the [Mesa distribution](https://github.com/pal1000/mesa-dist-win/releases/tag/26.2.4)
into ignored `build/ci-mesa/`, verifies the fixed archive SHA256 before extraction,
and verifies both selected DLL hashes before loading. It installs an opt-in startup
hook only in this checkout's `.venv-dev`; it never installs a system driver or
changes the global Python runtime. The hook is inert without `ROBOT_CI_MESA_DIR`.
With that variable set, a missing or corrupt DLL aborts interpreter startup.
Ordinary local development continues using its existing graphics backend.

Before enabling the gate environment, the setup launches a fresh Python child
which launches another renderer child. The latter must render nonblank MuJoCo RGB
and report actual OpenGL vendor `Mesa`, renderer `llvmpipe`, and the pinned version.
The uploaded `build/ci-mesa/report.json` records those strings, child identity,
image hash and dependency hashes. The ordinary receiver/capture test then exercises
its actual process renderer under the inherited environment. No rendering test is
replaced or skipped, and no control timing or freshness bound changes.
