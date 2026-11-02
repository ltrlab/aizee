# Development workflow

Use CPython 3.12, Git LFS, Node.js 22 and Rust 1.98.1 for the offline control gate. This is a
development environment; Jetson deployment keeps its existing build/release flow.

```sh
git lfs pull
python scripts/dev.py setup
python scripts/build_release.py --bootstrap
python scripts/dev.py check
```

Setup creates `.venv-dev` in this checkout, installs hash-locked dependencies and
an editable package from this checkout only. It does not change `.venv-platform`.
Check builds/tests both portable Rust crates, builds the receiver harness and
collision library, checks their presence/ABI, then runs the required control and
rehearsal tests. A skip or xfail fails this gate. `--offline` uses Cargo's cache.
The tests run in two phases: first across the CPU cores (pytest-xdist; `ROBOT_GATE_WORKERS`
sets the workers, by default half the CPUs up to 12), each test file on one worker unless
`tests/conftest.py` lists it as independent tests, then the tests marked `serial` alone.
Those wait on wall-clock progress (real rendering, live loops), which the other workers' load
only slows. Mark a test `serial` for that reason only, never to hide a race between tests.
The workers run one BLAS thread each (numpy's and scipy's OpenBLAS otherwise reserve memory for
every CPU thread, 1.4 GB a process on a 22-thread laptop, which ran 11 workers out of memory),
and a worker that crashes fails the run at once, naming its test.
Other suites remain available through this environment's `python -m pytest`;
the focused gate does not claim to cover learning, full UI behavior, Linux services or hardware.
Node checks all shipped browser modules. ARM64 release inputs are mandatory even
on a Windows/x86_64 host: the gate validates and hashes them, then builds and
verifies the release archive without executing those binaries. The portable
cross-build command above installs separately hash-locked tools and targets
JetPack 6's glibc 2.35 baseline; after provisioning, use `--offline` instead of
`--bootstrap` when all Cargo dependencies are cached. The existing provisioned
Windows `build_jetson.ps1 -Offline -Package robot_control,robot_collision` is also
supported. See [CI and release prerequisites](development/CI_RELEASE_GATE.md).

On a Windows host with the repository's existing downloaded toolchain, share its
compiler/cache by setting `ROBOT_RUST_TOOLCHAIN_ROOT` to the absolute directory
containing `cargo-home` and `rustup-home`. Otherwise install/select Rust 1.98.1
using rustup. The gate checks the compiler version and always writes binaries to
**this worktree's** `rust/target`, regardless of inherited target-directory settings.

Every check creates `build/dev-check/<unique-run>/report.json`, a command log and
pytest XML. The report contains source commit, dirty state, a source content hash,
dependency-lock hash and binary hashes. It rejects source edits made during the
run. Keep the entire run directory when reporting a failure; it is not a release
qualification record or a substitute for the existing release manifest.
`python scripts/gate_time_study.py [run...]` splits a run's time into its stages, test files and
slowest tests, and compares runs. A run that spans the computer's sleep counts the sleep.

### Quick lane

`python scripts/dev.py quick tests/unit/test_x.py ...` runs only the named test files, plus every changed test and,
for a changed page file, the page checks: seconds, for iterating on one feature. Without names it runs the gate's
tests that name a changed module (or a changed config or script file), compared with this checkout's last passing
check (`--base <commit>` picks another). A change to a module most tests import, such as `helm/teleop.py`, selects
most of the gate that way, so name the tests for those. The lane keeps the gate's rules (no skips, no source edits
during the run), writes `build/dev-quick/<run>/report.json`, and refuses any change to the motion contract
(approach, collision envelope, guarded lifecycle, receiver checks, the Rust crates) or to the locks.

A change that leaves the motion contract, Rust and the locks untouched may be deployed once it passes, with the
deployment owner's approval of that deploy (user decision, 2026-10-29: "We need to be able to make small changes like
this quicker"). The full `check` still has to pass before the branch is merged, and before any motion-contract, Rust
or lock change is deployed.

## Dependency updates

The development lock is separate from optional training/operator dependencies.
Edit `requirements/dev.in`, then deliberately regenerate and review the lock:

```sh
uv pip compile requirements/dev.in --python-version 3.12 --universal --generate-hashes -o requirements/dev.lock
python scripts/dev.py setup
python scripts/dev.py check
```

Never regenerate dependency locks as an unrelated fix for a failing test. The
application supports Python 3.10+; this reproducible gate currently targets 3.12.

## Parallel branches

Start each feature from the current `codex/overhaul-integration` commit. Use an
absolute, unique worktree path. For example:

```sh
git worktree add ../aizee-collision -b codex/collision-query-budget codex/overhaul-integration
```

Run setup inside the new worktree. Reuse dependency download caches and the Rust
compiler, but never another worktree's editable install or native binaries. Give
each process its own ports and recording/output directories. Do not start a
second hardware service; hardware deployment has one designated owner.

The integrator merges one branch at a time, runs the gate on the merged source,
and records the resulting report. Refresh remaining branches from that integration
commit before finishing them. Never use the default `main` base implicitly while
the overhaul integration line is ahead of it.

## Work boundaries and merge order

| Branch | Owned area | Required result | Dependency |
|---|---|---|---|
| `codex/dev-baseline` | Setup, locks, CI, shared instructions | Reproducible required gate; no missing-native skips | Checkpoint of ongoing fixes |
| `codex/mujoco-backend` | Physics adapter, model audit, synthetic fixtures | Real receiver against MuJoCo; invalid physical inputs rejected | Baseline |
| `codex/teleop-direct-mirror` | Python/Rust direct teleop contract and timing evidence | Explicit collision-disabled mode; measured latest-target following | Current integration |
| `codex/collision-query-budget` | Native/query hot path and performance fixtures | Same verdicts, recorded worst-case query costs | Baseline |
| `codex/rehearsal-evidence` | Reproducing traces, seeds, fault scenarios | First divergence report for each known failure | Baseline |
| `codex/helm-diagnostics` | Diagnostics presentation and recorded inputs | Display freshness/expiry/timing without changing motion | Baseline |
| `codex/minerva-physical-model` | CAD export, inertias, mappings, model evidence | Measured single-arm model and independent holdout validation | MuJoCo + physical inputs |
| `codex/two-arm-motion-contract` | Approach, envelopes, lifecycle and Rust checks together | Agreed bounds and dual-arm rehearsal acceptance | Collision timing + rehearsal evidence |
| `codex/calibration-repeatability` | Measurement repeats and fit validation | Repeatable estimates with uncertainty and holdouts | Model/evidence |
| `codex/release-reproducibility` | Existing release manifests/build provenance | Auditable source/model/binary identity | Baseline |

The current priority is collision-disabled position teleop on
`codex/teleop-direct-mirror`; see [its implementation brief](development/DIRECT_TELEOP.md).
One owner changes the Python/Rust control contract together. Rehearsal evidence
and diagnostics may proceed independently with stable fixtures and coordinated
data fields. Collision optimization is outside the direct teleop critical path.
Do not start multiple rewrites of the shared control files. Ruckig remains a
separate future spike and is not a dependency of direct teleop or MuJoCo.

## Acceptance ladder

1. Reproduce with a fixed input and explicit expectation.
2. Unit/contracts and deterministic rehearsals, including required native builds.
3. Linux virtual CAN against the actual service and fault injection.
4. Real-time Jetson timing with the intended workload and model.
5. Authorized supervised physical trials, one arm before both.

After two failed attempts to fix the same reproducer, collect a discriminating
trace or measurement. Do not start a third speculative rewrite or move a threshold
without evidence and an explicit contract decision.
