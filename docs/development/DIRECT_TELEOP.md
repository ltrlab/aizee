# Direct position teleop, without collision processing

Implemented 2026-10-10 at the user's request. The explicit direct mode, combined
update exchange, 100 Hz zero-ramp profile and offline qualification tooling are
available. Physical timing and tracking remain to be measured.

- Branch: `codex/teleop-direct-mirror`.
- Base: `codex/overhaul-integration`, commit
  `8b4f2c4a9543699fd86d2054dbf658be920b72bb`.
- Worktree: the completed baseline/MuJoCo checkout, now assigned to this task:
  `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`.
- Owner: one owner coordinates the Python/Rust teleop contract together. Delegate
  bounded review/test files explicitly; never allow overlapping edits.
- Goal: promptly mirror the newest calibrated leader joint positions, with no
  collision geometry queries, collision envelopes or collision recovery search.
  Retain joint, velocity, acceleration, effort and tracking-error bounds, source
  freshness, exclusive ownership, watchdogs, feedback checks and enable/stop behavior.

## Baseline and scope

The original profile selects `advisory`, and `AdvisoryCompiler` still calls
`backend.proximity`. `TeleopCore.build` opens the collision backend, and the
guarded session still compiles and exchanges envelopes. The two-arm path forces
a motion bound. Both Python live configuration and Rust service validation must
change together to permit an explicitly selected collision-disabled mode. This
branch makes that coordinated change; the original profile remains available.

The original profile has a 20 ms interpolation interval and a 50 Hz loop derived
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

## Implementation and acceptance contract

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

## Selecting and measuring the implementation

The dedicated profile is `config/devices/minerva-teleop-direct.json`. It selects
100 Hz and `interpolation_ms: 0`, with a 400 Hz receiver. Command lifetime is
30 ms, velocity feedforward lifetime 15 ms, feedback timeout 30 ms and lease
150 ms. These are independent receiver-enforced windows. The next one-use
freshness offer accompanies an accepted update, giving one request/reply per
steady following cycle. Replayed, reordered, invalid or expired batches cannot
renew a target. Feedback/target freshness, calibrated bounds, acceleration and
near-limit braking remain active; no collision backend is constructed.

Direct mode preserves I (Idle), E (Enable), T (Engage/hold), X (return and disable)
and H (immediate disable). Repeated source samples retain their original age and
hold with zero velocity feedforward. Sustained source loss requires Engage again.
Receiver expiry revokes ownership; the fault recovery path can only return to
reference and disable, not resume following. Physical service mode is explicitly
`teleop_direct`, with the existing watchdog, reference and operator enable gates.
Both Python and Rust must be updated together; no old-receiver fallback exists.

After the qualification ladder, select the profile from this checkout:

```powershell
.\scripts\helm.ps1 -TeleopProfile config/devices/minerva-teleop-direct.json -SshKeyFile C:/Users/ltr/Workspace/ssh-keys/aizee_rover_id
```

The launcher selects this checkout's `.venv-dev`, falling back only to its
`.venv-platform`, and rejects an editable import from a different checkout.
`-SshKeyFile` uses an explicitly supplied existing key without copying it; it
restores the previous process environment on exit. This
profile deliberately cannot run servo tuning; the original profile owns that
existing workflow. Saved range scaling/sign/offset mapping is preserved, so
"direct" does not promise a 1:1 leader/follower angular ratio. Existing gains and
feedforward are reused but the zero-ramp law has no physical tracking qualification.

Live evidence includes `direct-latency.jsonl` beside the receiver recording:
source sequence/age, poll/map/RPC time, desired/commanded/measured positions,
limiter reasons, receiver acceptance and successful hardware-batch submission.
Receiver timestamps are compared only within that receiver's clock. Application
is not motor motion onset. `cycle_ms` measures through session completion;
`full_loop_ms` additionally includes command handling, previous trace serialization
and writing, flushes and UI snapshot construction. The loop records scheduled
lateness, completion deadline misses and skipped slots, and never queues catch-up
targets. A loop record is finalized after UI construction and persisted on the
next tick or clean close; an abrupt process kill can lose that final record.

The synthetic benchmark isolates 14-joint receiver/transport overhead. It does
not measure USB leader throughput, the full lifecycle/UI workload or physical
tracking. Harness time advances only when the client invokes it. Only the Linux
vcan backend exercises an independently scheduled real service. Each output
directory contains source content/config/binary hashes, trace and report.

```powershell
.\.venv-dev\Scripts\python.exe scripts/benchmark_direct_teleop.py --backend harness --hz 100 --cycles 1000 --output simulation_runs/direct-harness-100hz-new
```

On a Linux development host, prepare dedicated virtual interfaces and run the
same benchmark against the actual service (these names must be kernel `vcan`):

```sh
cargo build --release --locked -p robot_control --features robot_control/linux-service
sudo modprobe vcan
sudo ip link add dev vcanDirectL type vcan
sudo ip link add dev vcanDirectR type vcan
sudo ip link set vcanDirectL up
sudo ip link set vcanDirectR up
.venv-dev/bin/python scripts/benchmark_direct_teleop.py --backend vcan --binary rust/target/release/guarded_controller --hz 100 --cycles 3000 --output simulation_runs/direct-vcan-100hz-new
```

The benchmark also exercises producer-silence expiry, command/consumed-offer
replay, expired source validity, requested stop, stale feedback and emergency
stop against independently ticking Linux vcan. Processing, scheduling and fault
acceptance are separate report fields. Evaluate `--hz 200`
separately; that option does not promote the default profile to 200 Hz.

Current verification and remaining platform work are recorded in
[development status](../DEVELOPMENT_STATUS.md). Qualification and the later
authorized activation/stopped checks are recorded separately below.

## Live trial preparation, 2026-10-10

The release is **active and ready to begin an attended left-arm trial**; stopped
motor checks passed. Dual-arm timing qualification remains blocked. This is
not approval for unattended operation or evidence of physical tracking accuracy.

The Jetson at `10.42.0.1` passed host-key and machine-id verification. Qualification
ran under `/home/ltr/minerva/validation/direct-20261010`; its pristine `release/`
is separate from `qualification/`, which adds benchmark fixtures. Both have
verified manifests; the latter also has an externally stored source manifest.
After user authorization, `/home/ltr/minerva/current` was switched to clean release
`20261010T175730Z-abfbbbd`, whose 122 runtime files are byte-identical to the tested
candidate. All previous releases were preserved, including rollback target
`20261010T142650Z-a66afa7`. **`check --remote-root` selects preflight only; Helm
executes the active release.** Activation evidence is in
`build/direct-qualification/activation.json`.

The complete-loop benchmark, `scripts/benchmark_direct_loop.py`, runs actual
TeleopCore/run_loop, recorder and UI construction. Its virtual motor emulator
runs in a separate process so application garbage collection cannot pause CAN
feedback. It preserves the first failed receiver result before cleanup and uses
a bounded, separately reported drain phase to observe the last measured target
before H/X. It never substitutes success for missing application observations.

Measured results on the Jetson, at 100 Hz:

| Workload | Measured following updates | Full-loop p99 | Following deadline misses | Result |
|---|---:|---:|---:|---|
| Synthetic 14-joint receiver/transport only | 3000/3000 | Not measured; RPC path 2.88 ms | 0 | Linux vcan timing and fault checks passed |
| Synthetic left arm, saved operator mapping | 1000/1000 | 4.83 ms | 0 | Completed X return/disable |
| Real USB + tether, left arm, harness | 1000/1000 | 4.75 ms | 0 | Software harness only |
| Real USB + tether, left arm, independent vcan | 1000/1000 | 6.46 ms | 0 | All target applications observed; X return/disable |
| Synthetic both arms, final candidate | 1604/3000 accepted | 6.41 ms before failure | 1 | Command expired after an overrun; incomplete |

The real-leader vcan test includes recording/UI work, with 911 unique consumed
leader samples (about 91 Hz), 25.60 ms p99 conservative fresh-source age, and
3 ms p99 receiver acceptance-to-application. Leaders were nearly stationary;
this does not measure physical motion-onset latency or moving-leader tracking.
Repeated samples held position with zero velocity feedforward and kept their
original age. The direct command/feedback/source windows were not widened.

The dual-arm failure is retained in `build/direct-qualification/full-loop-vcan-both-r4/`.
An earlier diagnostic isolated unnecessary whole-result deep copies (including
recorder history): 1.34 ms median on the Jetson. DirectLifecycleSession now copies
only the feedback it retains, preserving same-reply validation, sample ownership,
duplicate checks and conservative timestamps. The resulting p99 improved, but
one 10.00 ms cycle still skipped a slot and the anchored command expired. Keep
dual-arm live trials blocked until the full workload passes a longer run; do not
widen watchdogs to hide the failure. The saved trace is the next reproducer.

Operator evidence was copied from the original checkout, not re-seeded. Two raw
binding hashes differed solely due to CRLF/LF conversion; byte equality after
normalization and JSON equality were verified, with old/new hashes retained in
`build/direct-qualification/operator-evidence.json`. Route definitions match,
and all 14 saved route signs/history were copied for the direct profile. The
original calibration files and original overhaul edits are unchanged.

Software-only preflight passed all applicable checks, including direct receiver
configuration validation for 7/7/14 joints, dependency smoke, correct local/remote
files, pinned adapters and absence of another motor owner. The authorized stopped
scan after activation also passed: all 14 motors matched identity/firmware,
reported fault-free Reset state and correct power-on range, and lay inside swept
limits. Both CAN links returned DOWN. The left arm meets the starting pose; the
right gripper is +0.13 rad from reference versus a 0.10 rad tolerance. No enable
or physical motion was requested. Evidence:
`calibration_runs/minerva-check/20261010T175855Z.json`. Live sweeps remain pending,
so generic Minerva Check is still **INCOMPLETE**, not a physical qualification GO.

For the next attended trial:

1. Keep the left arm in the checked reference pose. Repeat stopped Minerva Check
   if the hardware, power state or pose changed since the recorded scan.
2. Launch the command above from the feature worktree. Select **Jetson → Live
   left arm**, with the robot in its reference pose and the foot switch in reach.
3. Start the session, use I → E → T, and make a small movement of one leader joint
   at a time. Check sign, lag, overshoot and limit behavior in the saved trace.
4. Test T hold/re-engage and X return/disable; use H or the foot switch if immediate
   disable is needed. Stop on a fault; do not restart repeatedly without inspecting
   the first receiver reason and desired/commanded/measured records.

Servo tuning/live sweep belongs to `minerva-teleop.json`; direct mode intentionally
does not run Tune. Physical servo qualification and direct-law tracking evidence
remain separate. Do not interpret a generic Check result as direct-law approval.

Evidence aggregate: `build/direct-qualification/live-readiness.json`. The full
Jetson archive is `build/direct-qualification/readiness-evidence.tar.gz`, including
failed trials, stage profiling, namespace proof and the real-leader runner scripts.
