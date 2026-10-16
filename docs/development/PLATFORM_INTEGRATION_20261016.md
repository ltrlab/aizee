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
| Task capture, GUI and packaging | `89a37d0` | 874 Python + 132 Rust passed |
| Camera synchronization and portability | `2c7867c` | 879 Python + 132 Rust; Windows and Ubuntu CI passed |

The ArmPlant merge retains single clock snapshots/rounded ticks and deferred
request acceptance with actual emitted-command tracking. `claude/sim2real` is
already an ancestor of calibrated simulation and was not merged separately.
Frozen reports in the integration worktree:

- `build/dev-check/20261016T183805Z-d5c261e9/report.json`
- `build/dev-check/20261016T185008Z-3f84fc24/report.json`
- `build/dev-check/20261016T185826Z-c352cd9e/report.json`

Additional frozen reports are `build/dev-check/20261016T191302Z-bad12190/report.json`
and `build/dev-check/20261016T192254Z-af1dd6ed/report.json`. The last binds clean
commit `2c7867c024dae2c9d19cd472a960d0122bb1efa4`.
[Public CI](https://github.com/ltrlab/aizee/actions/runs/38955598530) passed on
Windows and Ubuntu. The first Linux run on `610c66b` exposed byte-sensitive CRLF
calibration and enum/scheduling portability issues; they were fixed without
changing the qualified calibration hash or allowing platform test skips.
Both control jobs are required on `codex/overhaul-integration`, including admins,
with strict up-to-date status and force-push/deletion disabled. Main is unchanged. The 349 audited model paths and two derived MJCF OBJ paths
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

- The process-isolation candidate needs its own frozen gate and PR CI; the accepted baseline is green and protected.
- Graphical workflow and actual leader/receiver/scene/recording throughput.
- New release smoke and isolated Jetson timing before activation.
- Attended both-arm checks, workcell/camera calibration and a real trial.
- Common validator/training adapter, evaluation/promotion and human correction.

Observed deployment is `20261016T181317Z-6b47de8`. Right wrist and head cameras
stream after operator connection. The existing report's right J5/J6 lag failure
and pending left sweep remain readiness items, not repaired hardware.

## Robot-side capture observations

Candidate `2c7867c` was staged under a unique validation directory and its full
manifest and module/native smoke checks passed. The active `current` symlink and
all existing releases were verified unchanged. No generic activating deployment
or physical motor command was used.

A bounded eight-frame camera history replaced latest-only pairing. A 22-second
Jetson probe admitted 401/401 right-wrist/head pairs at the existing 20 ms skew
and 200 ms age bounds (max skew 19.962 ms, max age 78.669 ms). This verifies
camera pairing, not receiver/recording timing or sensor exposure synchronization.

A 50-second isolated virtual-CAN baseline on `2c7867c` accepted/applied 5000/5000
commands with zero following deadline misses/skips; full-loop p99 7.840 ms and
maximum 9.829 ms. All 14 virtual joints were serviced, but the saved manual
selection moved only left J1, so this is not full bimanual moving qualification.
The matching camera-loaded run failed before following with stale recorder rows;
no complete episode was published. Both runs ended DISABLED, kept physical CAN
inaccessible, and preserved the active release and existing releases. Their
private evidence remains in `build/integration-20261016/jetson-camera-*-2c-v6`
in the original workspace. The process candidate requires a new child-aware
qualification with all intended joints explicitly selected.

## Process isolation candidate evidence

Live JPEG decoding/writing uses an owned subprocess; simulation separates
control, contact physics, and one bounded renderer. Status and command queues,
startup and liveness deadlines, source freshness, authority-loss cancellation,
and parent-death/finalization fences are explicit. The renderer retains exact
MuJoCo integration state and paired receiver values. The required gate includes
actual renderer/receiver contracts, not only mocked frames.

A 30.257-second visible-scene run with continuous time and the actual Rust
receiver/ArmPlant produced 237 completed head/right-wrist frames at 320x240:
7.888 Hz achieved, 210 ms maximum source gap, all 14 scripted axes moving.
There were 4201 controller cycles, no receiver fault, and a clean close.
All 236 nonterminal action masks were valid; the terminal mask was false.
This is synthetic motion/capture evidence, excluded from dataset promotion.
The browser CAD preview was closed during this run; earlier runs with its
continuous renderer active failed and retained partial files. Full GUI and real
leader USB qualification are separate acceptance items. Evidence is in the
scene worktree at `build/task_loop_browser_closed`, episode SHA256
`fe54fe7efaf4be4058fb11186e4fee24728eb6f62e87d009db3dbc291bc30e82`.

The native receiver reports an integral-millisecond status clock and fractional
recorder stamps. Capture now treats status n as interval [n,n+1), retains the
raw row, rejects rows at/after n+1, and computes all ages/cutoffs conservatively
from the upper bound. The existing 30 ms deadline is unchanged; RPC and 1 ms
clock quantization uncertainty are explicit. The observed 2106/2106.23714 ms
pair is a regression fixture. Camera selection and physical motion are unchanged.
