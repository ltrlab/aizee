# Platform integration, 2026-10-16

## Preserved source and merge order

The original `overhaul` checkout stays at `a66afa7` with all 12 modified files
(including the planning link) unchanged by hash and all inventoried untracked
paths present. Private build/recording files were not published. The isolated
managed integration worktree owns `codex/integration-20261016`.

| Step | Source | Result |
| --- | --- | --- |
| Preserved checkpoint | `f846b65` | No destructive reset or cleanup |
| Completed fault fixes | `e204431` | Deferred-request rehearsals included |
| Receiver ramp continuation | `6b47de8` | 741 Python + 132 Rust passed |
| Calibrated simulation merge | `cd6d6f7` merging `d7a1610` | 796 Python + 132 Rust passed |
| Portable CI prerequisites | `610c66b` | 801 Python + 132 Rust passed |

The ArmPlant merge retains single clock snapshots/rounded ticks and deferred
request acceptance with actual emitted-command tracking. `claude/sim2real` is
already an ancestor of calibrated simulation and was not merged separately.
Frozen reports in the integration worktree:

- `build/dev-check/20261016T183805Z-d5c261e9/report.json`
- `build/dev-check/20261016T185008Z-3f84fc24/report.json`
- `build/dev-check/20261016T185826Z-c352cd9e/report.json`

The last report binds clean commit `610c66bb67586f39cd48dc642d755a1459cc6740`.
[Public CI](https://github.com/ltrlab/aizee/actions/runs/38952454408) qualifies clean
Windows/Linux runners. The 349 audited model paths and two derived MJCF OBJ paths
were published after explicit approval. Credentials, recordings and unrelated
untracked files remain private.

## Candidate ownership

| Branch | Base | Scope | Focused evidence |
| --- | --- | --- | --- |
| `codex/ci-release-gate` | `e204431` | Cross-build, release checks, wheel assets | 746 Python / 131 Rust; 24 packaging checks |
| `codex/helm-task-workflow` | `d7a1610` | Simulation isolation, GUI, live capture | 86 affected tests and syntax checks |
| `codex/minerva-task-scene` | `d7a1610` | Contact scene, atomic writer, fixture | 27 tests with real object contacts |
| Integration lead | `f846b65` | Merge contracts, required test union, robot observations | Frozen gate after each accepted stage |

The candidate imports these contributions. Only the integration lead owns the
physical robot. Completed Claude branches are included; the installed Claude CLI
was unauthenticated, so new work used Codex agents. Independent review reproduced
and fixed an abort/publication race, source-time distortion, missed interpolated
contacts, stale success display and expensive control-tick snapshot copies.

## Scope of the task evidence

The cube is a free dynamic body. FK-driven robot geometry and terminal finger-pad
contacts pick it up, transfer it and release it into a bucket. No grasp attachment
or runtime cube-pose override is used. The scripted fixture records pickup,
handoff and settled-bucket events; it is excluded from training and does not prove
leader/controller or real-time performance. See [scene limitations](MINERVA_TASK_SCENE.md).

Initial live/sim capture declares exactly right-wrist/head RGB and 14 named motor
axes. Episodes retain paired measured positions/receiver commands, authority and
action-valid masks, clocks, provenance and missing-torque masks. Queue/validation
failures retain partial files. Successful completion requires explicit finish.
Legacy training readers do not automatically ingest versioned raw task episodes.

## Remaining qualification

- Remote CI and required integration checks; full candidate source gate.
- Graphical workflow and actual leader/receiver/scene/recording throughput.
- New release smoke and isolated Jetson timing before activation.
- Attended both-arm checks, workcell/camera calibration and a real trial.
- Common validator/training adapter, evaluation/promotion and human correction.

Observed deployment is `20261016T181317Z-6b47de8`. Right wrist and head cameras
stream after operator connection. The existing report's right J5/J6 lag failure
and pending left sweep remain readiness items, not repaired hardware.
