# Direct position teleop, without collision processing

Prepared 2026-10-10 at the user's request. This is the next implementation task;
the branch preparation does not change runtime behavior.

- Branch: `codex/teleop-direct-mirror`.
- Base: `codex/overhaul-integration`, commit
  `8b4f2c4a9543699fd86d2054dbf658be920b72bb`.
- Worktree: the completed baseline/MuJoCo checkout, now assigned to this task:
  `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`.
- Owner: one agent owns the Python/Rust teleop contract together. Do not run
  another editing agent in this checkout.
- Goal: promptly mirror the newest calibrated leader joint positions, with no
  collision geometry queries, collision envelopes or collision recovery search.
  Retain joint, velocity, acceleration, effort and tracking-error bounds, source
  freshness, exclusive ownership, watchdogs, feedback checks and enable/stop behavior.

## What the existing implementation establishes

The profile selects `advisory`, but `AdvisoryCollisionCompiler` still calls
`backend.proximity`. `TeleopCore.build` opens the collision backend, and the
guarded session still compiles and exchanges envelopes. The two-arm path forces
a motion bound. Both Python live configuration and Rust service validation must
change together to permit an explicitly selected collision-disabled mode.

The current profile has a 20 ms interpolation interval and a 50 Hz loop derived
from that interval. The leader sampler is configured for 200 Hz; that is not a
measurement of unique encoder sample throughput. The receiver runs at 400 Hz.
The older latency table in `../architecture/TELEOP_LATENCY.md` predates these
settings; its 100 ms estimate is not a current benchmark.

Removing collision work alone leaves loop scheduling, receiver round trips,
interpolation and physical tracking delay. Measure those separately.

## Scope and ownership

Own the coordinated changes in:

- `src/robot_platform/helm/teleop.py` and the teleop profile/configuration tests.
- `src/robot_platform/teleop/guarded_lifecycle.py`, `lifecycle.py` and their contracts.
- `src/robot_platform/control/guarded.py`, `guarded_config.py`, `live_receiver.py`.
- `rust/control_core` configuration, request/response handling and relevant tests.
- A dedicated direct teleop profile, latency trace/replay fixtures and reports.

Touch leader sampling/transport only after traces identify a bottleneck there.
Coordinate diagnostics fields with the diagnostics owner. Existing collision
algorithms, enforced/advisory semantics and their tests remain regression cases.
Minerva physical-model identification and collision optimization are separate
tasks; neither blocks this software control-path work. No IK, motion planner,
new transport stack, controller-law rewrite or predictive leader extrapolation
belongs in this task.

## Small changes in order

### 1. Explicit collision-disabled behavior at the existing rate

Add an explicit `collision_mode: disabled` selection and a corresponding explicit
physical-service configuration choice. Missing or malformed collision configuration
must not implicitly select it. Carry the selection through profile loading,
configuration derivation, lifecycle and Rust service validation for one or two arms.

Do not construct the collision backend or compiler in this mode. Read any needed
joint/calibration metadata independently. Arm/move operations carry no collision
envelope; do not fabricate an always-clear verdict. Keep authority renewal working:
currently a plain move without an envelope does not renew the lease automatically.
Generic offer tokens also serve ownership freshness, so they are not all removable
merely because collision checking is disabled.

First acceptance: both arm configurations work in the offline receiver harness;
patch collision backend creation/query entry points to raise and prove a complete
enable/engage/follow/disengage/stop session never calls them. Reject ambiguous mode
configuration, preserve the enforced/advisory contracts, and exercise stale source,
stale feedback, lease expiry, limit violations and requested stop. Keep the current
150 ms lease and 30 ms feedback timeout; do not lengthen them to accommodate work.

### 2. Measure and shorten the command path

Record correlated source sequence, unique sample rate, source age, map completion,
RPC duration, receiver acceptance, target application and deadline misses. Trace
desired mapped position, limited command position and measured position separately,
including why a limiter activated. Use monotonic intervals on each host; do not
subtract unsynchronized PC and Jetson timestamps. Preserve conservative source-age
bounds or explicitly verified clock synchronization.

Keep only the latest unconsumed target. Drop superseded targets rather than queue
them. Test out-of-order and duplicate samples without treating an old sample as new.
Reduce steady-state control to one update/feedback exchange per period if receiver
round trips dominate. Combine target acceptance and lease renewal with receiver-issued
freshness/sequence validation; replay or a delayed queued update must never extend
authority. Check expiry between samples and through transport stalls.

### 3. Direct following at 100 Hz, then evaluate 200 Hz

Separate `loop_hz` from `interpolation_ms`. Start at 100 Hz with the existing 400 Hz
receiver. Compare zero interpolation against the smallest supported receiver ramp;
remove the existing 20 ms ramp when the tests and trace support doing so. The core
already accepts zero interpolation, but its nonzero velocity feedforward then
persists until another command or expiry. Specify and test stale-sample velocity
behavior before using that setting.

In engaged following, use the newest calibrated joint target subject only to the
declared physical command bounds. Keep controlled alignment on Engage and existing
hold/stop transitions. Reuse the established gains and calibrated feedforward until
measurements justify a separate change. Report bound-induced tracking lag separately
from message latency. Preserve the existing leader mapping initially; it scales
calibrated ranges and is not guaranteed to be 1:1 angular mirroring. A unit angular
gain, sign and offset map would be a separately tested calibration decision.

Proposed engineering target: cycle processing p99 below 8 ms at 100 Hz under the
intended two-arm workload, with deadline misses and maximum also reported. This is
an acceptance target, not a measured result. Evaluate 200 Hz only if unique leader
samples and transport support it and cycle processing p99 is below 4 ms. If not,
retain 100 Hz and use the trace to identify the next bottleneck. Software latency
and physical joint tracking each need their own before/after measurement.

### 4. Qualify the path without expanding the rewrite

Run `python scripts/dev.py check` after each bounded change, then the saved direct
teleop replay/benchmark with source/config identity and fixed inputs. Add the new
required contracts to the gate. The starting integration gate passed 125 Python
and 104 Rust tests; a new mode needs additional coverage.

Next run the actual service on Linux virtual CAN with faults, then Jetson timing
under the intended load. Physical trials follow with a designated operator, one
arm first and then both. Save p50/p95/p99/max, sample age, missed deadlines, command
and feedback traces, stop causes and limiter activity. Keep physical deployment
readiness separate from an offline pass. MuJoCo's synthetic fixtures can support
regressions; unmeasured Minerva inertias cannot establish hardware tracking accuracy.

## Handoff and integration

Land the behavior change and latency change as separate reviewable commits, each
with its evidence directory. The integrator reruns the gate before merging to
`codex/overhaul-integration`. Rehearsal and diagnostics can proceed in other
worktrees with stable input fixtures; changes to shared control files wait for this
owner. Collision-query optimization is no longer on the direct teleop critical path.

Results: branch and scope prepared; runtime implementation, new tests, Linux vcan,
Jetson timing and hardware measurements remain outstanding.
