# CI and mandatory release prerequisites

Base: `e20443178df85ef99ae12a8989749f9a005beaf0`.
Scope: developer/CI build orchestration, mandatory release evidence and its tests.
Runtime, control bounds, receiver behavior and robot operation are unchanged.

## Failure reproduced

A clean checkout has no ARM64 `guarded_controller`. The existing
`test_a_built_release_is_small_complete_and_verifiable` therefore skips, although
pytest returns success. The required developer gate rejects that skip. Previously
the Windows/Linux workflow ran setup and check without supplying these release
inputs. This is a missing CI prerequisite, not an optional release test.

Reproducer (before cross-building):

```sh
python scripts/dev.py setup
# Use .venv-dev/Scripts/python.exe on Windows, .venv-dev/bin/python on Linux.
.venv-dev/bin/python -m pytest -q tests/unit/test_ops_release.py::test_a_built_release_is_small_complete_and_verifiable --junitxml=build/missing-arm64.xml
```

The original focused reproducer recorded one skip in
`build/ci-prerequisite-reproducer/missing-arm64.xml`. Passing that XML through the
required gate's results check failed and named the missing release test.

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

Acceptance: clean Windows and Linux jobs both complete without required skips or
xfails; source identity remains unchanged; native and ARM64 artifact hashes and a
verified release archive are present in the required gate evidence. The integrator
must record actual CI results; workflow configuration alone is not a passing run.

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
