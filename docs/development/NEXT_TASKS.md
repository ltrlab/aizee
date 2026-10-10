# Ready work lanes after the baseline

Resolve `codex/overhaul-integration` to a full SHA before starting each task, record
it in the task brief, then create a separate worktree and run setup. These briefs
are scoped follow-up work, not claims that optimization or hardware validation is done.

## Current priority: direct position teleop without collision processing

Branch: `codex/teleop-direct-mirror`, based on integration commit
`8b4f2c4a9543699fd86d2054dbf658be920b72bb`.

The [implementation brief](DIRECT_TELEOP.md) assigns one owner to the Python/Rust
contract and separates collision-disabled behavior from latency optimization.
Start at the existing rate, then target 100 Hz with the 400 Hz receiver; evaluate
200 Hz using measured unique leader throughput and deadline headroom. This task
does not depend on collision-query optimization or measured Minerva inertias.

## Collision query budget (separate from direct teleop)

Branch: `codex/collision-query-budget`.

Own `rust/collision`, `src/robot_platform/simulation/native_collision.py` and new
performance fixtures. Coordinate any necessary `collision_guard.py` API changes
with the motion-contract owner. Preserve the existing envelope, clearance and
speed semantics, including finger exclusions and active collision ABI.

Reproduce the close-arm query path documented near the end of
`docs/devices/MINERVA_PRODUCTION_01.md`. Preserve model, pose and scope with the
trace. Measure per-stage timing and allocations, then change one hot path. Check
verdict parity on the saved poses and boundary cases before comparing timings.

Acceptance: `python scripts/dev.py check`; offline benchmark with the same model
and input seed; before/after p50, p99 and maximum including the near-arm case.
Report desktop and Jetson results separately. The task is not accepted for live
dual-arm use until the intended Jetson workload meets the existing compute budget;
do not raise that budget to obtain a pass. Physical motion is not needed to profile
the query and is outside this task.

## Rehearsal evidence

Branch: `codex/rehearsal-evidence`.

Own new trace fixtures and replay/report code under `tests/fixtures` and
`src/robot_platform/simulation`, plus focused test additions. Coordinate changes
to `ArmPlant`/`PlantReceiver` with the MuJoCo adapter owner. Leave control bounds,
fault handling and reference generation unchanged.

Start with encoder jitter near an envelope edge, gripper stiction during shutdown,
leader-age expiry and slow near-arm collision compilation. Preserve the original
input and identify the first differing receiver/lifecycle state. Reports must bind
source/model identity, simulated versus host time, random seed, feedback and command
age, verdict expiry and stage timing. Do not bless the newest output as the expected
baseline without an explicit behavior decision.

Acceptance: each known scenario has a repeatable failing or passing expectation;
an intentionally perturbed replay reports its first divergence; the development
gate stays green. Synthetic friction/noise assumptions remain labeled.

## Helm diagnostics

Branch: `codex/helm-diagnostics`.

Own diagnostics presentation and its tests. Start with recorded input fixtures;
coordinate any new data fields with the teleop owner. Do not change
`TeleopCore`, guarded lifecycle transitions, rate limits, collision bounds or
receiver authority as part of a UI task.

Show source feedback age, leader age, command/verdict time remaining, collision
computation duration and the exact reason a motion was held or stopped. Preserve
the originating timestamp and distinguish missing data from a valid zero.

Acceptance: recorded healthy/stale/expired/slow-query scenarios display the right
reason and timestamps without affecting motion commands; presentation tests and
the development gate pass. No hardware connection is needed.

## Minerva physical model

Branch: `codex/minerva-physical-model`.

Own a versioned physical asset bundle, binding/calibration data and independent
model validation fixtures. Follow `docs/MINERVA_PHYSICS_MEASUREMENTS.md`.

The next input is measured mass/COM or corrected CAD mass properties, not another
simulator rewrite. Kinematic frame comparisons can proceed before those inputs.
Acceptance requires no placeholders, a clean structural audit, measured jaw/motor
mapping, independent static-torque checks and held-out motion traces with declared
tolerances. Keep physical validation incomplete while measurements are unavailable.
