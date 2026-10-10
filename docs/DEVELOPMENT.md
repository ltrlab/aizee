# Development workflow

Use CPython 3.12, Git LFS and Rust 1.98.1 for the offline control gate. This is a
development environment; Jetson deployment keeps its existing build/release flow.

```sh
git lfs pull
python scripts/dev.py setup
python scripts/dev.py check
```

Setup creates `.venv-dev` in this checkout, installs hash-locked dependencies and
an editable package from this checkout only. It does not change `.venv-platform`.
Check builds/tests both portable Rust crates, builds the receiver harness and
collision library, checks their presence/ABI, then runs the required control and
rehearsal tests. A skip or xfail fails this gate. `--offline` uses Cargo's cache.
Other suites remain available through this environment's `python -m pytest`;
the focused gate does not claim to cover learning, UI, Linux services or hardware.

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
| `codex/collision-query-budget` | Native/query hot path and performance fixtures | Same verdicts, recorded worst-case query costs | Baseline |
| `codex/rehearsal-evidence` | Reproducing traces, seeds, fault scenarios | First divergence report for each known failure | Baseline |
| `codex/helm-diagnostics` | Diagnostics presentation and recorded inputs | Display freshness/expiry/timing without changing motion | Baseline |
| `codex/minerva-physical-model` | CAD export, inertias, mappings, model evidence | Measured single-arm model and independent holdout validation | MuJoCo + physical inputs |
| `codex/two-arm-motion-contract` | Approach, envelopes, lifecycle and Rust checks together | Agreed bounds and dual-arm rehearsal acceptance | Collision timing + rehearsal evidence |
| `codex/calibration-repeatability` | Measurement repeats and fit validation | Repeatable estimates with uncertainty and holdouts | Model/evidence |
| `codex/release-reproducibility` | Existing release manifests/build provenance | Auditable source/model/binary identity | Baseline |

The first independent work lanes are collision performance, rehearsal evidence
and diagnostics. The motion-contract owner integrates shared seam changes. Do
not start multiple rewrites of that seam. Ruckig remains a separate future spike;
it is not a dependency of the MuJoCo or baseline work.

## Acceptance ladder

1. Reproduce with a fixed input and explicit expectation.
2. Unit/contracts and deterministic rehearsals, including required native builds.
3. Linux virtual CAN against the actual service and fault injection.
4. Real-time Jetson timing with the intended workload and model.
5. Authorized supervised physical trials, one arm before both.

After two failed attempts to fix the same reproducer, collect a discriminating
trace or measurement. Do not start a third speculative rewrite or move a threshold
without evidence and an explicit contract decision.
