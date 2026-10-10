# Direct position teleop, without collision processing

Implemented 2026-10-10 at the user's request. The explicit direct mode, combined
update exchange, 100 Hz zero-ramp profile and offline qualification tooling are
available. Physical timing and tracking remain to be measured.

**Current status:** the first physical left-arm trial failed with jerky motion and
a duplicate-sample holding-lead fault. The local correction is not deployed or
physically qualified; do not repeat a live trial on the strength of the prior
near-stationary timing result. See the failed-trial investigation below.

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

## Failed left-arm trial, 2026-10-10

Run `live-20261010T180632Z` on activated release `20261010T175730Z-abfbbbd`
entered HOLD at 18:06:43 UTC, ENGAGING at 18:06:50, TELEOP at 18:07:03,
and faulted at 18:07:36. The user reported all joints jumping. The existing
recovery returned to reference and disabled at 18:07:43. No diagnostic work
after that report enabled motors, restarted teleop or deployed a replacement.

Preserved evidence: `build/direct-qualification/live-fault-20261010/`, including
the original run, download hashes, `analysis/analysis.json`, plots, and before/after
checkpoint reproducers. Raw recordings remain on the Jetson. Source/gain/config
identity matches the qualified candidate. Readiness for further live trials is
withdrawn; previous stationary and virtual timing reports remain historical data.

Confirmed observations:

- TELEOP lasted 32.56 seconds with zero scheduled misses/skips; full-loop p99 was
  7.076 ms and maximum 9.601 ms. The native receiver reported zero misses,
  overruns or rejected CAN frames. Earlier IDLE startup overruns were separate.
- 343 TELEOP reads repeated a leader sample, about 10.5%; unique input rate was
  about 89.4 Hz versus the 100 Hz consumer. Final-second traces show synchronized
  pauses and restarted ramps at roughly 100 ms intervals, with corresponding
  measured velocity/effort oscillations. Stationary HOLD positions varied only
  about 0.00038–0.00077 rad peak-to-peak in the saved drive feedback.
- A validated repeated sample became `target=None`. The lifecycle then emitted
  zero velocity and removed friction feedforward before resetting its ramp
  history. The next advancing source sample restarted acceleration from zero.
- The faulting repeat had no timing miss. The saved receiver position reconstructs
  J3 at -1.399970531 rad versus the prior target -1.370457905 rad: 0.029512627 rad
  error, above the static holding bound 0.027731747 rad. The receiver tracking
  bound remained 0.046219579 rad. Offline replay reproduces the same exception.

The local correction reuses an immutable, validated, still-fresh goal only in
explicit direct mode and applies the ordinary bounded approach using current
feedback. It does not extrapolate the leader position. Source timestamps and
sequences remain unchanged; repeated and missing samples consume the same loss
counter (production allowance remains eight at 100 Hz), and only advancing input
resets it. Stale/failed polls are not reused. Existing dispatch margins and native
source-anchored command deadlines remain in force. Exhaustion still enters
measured HOLD and requires Engage. Legacy guarded duplicate rejection is unchanged.

Diagnostics now retain source/feedback when proposal construction fails and log
commanded velocity/effort, measured velocity/effort, feedback age/sequence and
lifecycle transitions. The original recording lacks commanded feedforward, so
the pulse overlay is inferred from command increments; it is not measured motor
torque. The J1 effort channel was constant and is not useful torque evidence.

Checkpoint comparisons show that the candidate corrects J3 back inside the same
lead bound and avoids the duplicate-specific exception. A synthetic moving J2
checkpoint using actual gains changes velocities from 0.58 → 0 → 0.08 rad/s to
0.58 → 0.66 → 0.74 rad/s. This is a software-path comparison, not a physical plant.

Remaining work at the duplicate-only checkpoint (superseded by the retry results below):

1. Qualify moving, quantized leader trajectories with about 90 Hz unique samples,
   a 100 Hz consumer, duplicate bursts, reversals, reached goals and lost input.
   Reaching a sampled goal can still drop command-derived velocity feedforward
   to zero; the current fix alone does not prove smoothness. Evaluate that
   feedforward behavior separately rather than changing motor gains blindly.
2. Run the corrected full workload on isolated Jetson vcan with recording enabled;
   retain all deadlines and watchdogs. No physical services are part of that gate.
3. Review the resulting command/effort trace, then prepare a deliberate single-joint,
   low-speed attended trial. A software pass is not physical tracking approval.

Motor gains, gravity/friction calibration, joint/lead/speed bounds, source age,
command lifetime and watchdog limits were not changed. The active robot release
is still the failed candidate; the correction must not be described as deployed.

The final required gate passed **416 Python tests, zero skips/xfails, and 111 Rust
tests**: `build/dev-check/20261010T182211Z-516a4b69/report.json`. Runtime and tests
were frozen for the gate; only documentation/results were finalized afterward.
Independent review also found a one-ULP endpoint construction issue: direct
lead endpoints now round inward, so a subsequent missing sample cannot fail the
unchanged strict hold check because of subtraction roundoff. Both signs are tested.

The separate deterministic command comparison in
`build/direct-qualification/live-fault-20261010/moving-source-qualification/`
ran 24 cases/24,556 cycles: 90 Hz quantized input, 100 Hz consumer, ramps, reversals
and holds, using exact left-arm lifecycle settings and ideal next-cycle position
feedback. It had no lifecycle faults or position/speed violations, and removed
all duplicate zero-velocity drops while the sampled goal was still unreached.
It is not a dynamics simulation. Slow-motion pulses largely persisted: J1/J3/J4/J6
drop counts were unchanged, and J2 fell only from 319 to 295. In the fast J2 case,
duplicate drops fell from 18 to 4, but a remaining goal-reached velocity jump was
0.8832 rad/s (legacy maximum 0.7200). **At that duplicate-only checkpoint, the command smoothness gate had not passed.**
Velocity-reference generation needs a separate evidenced correction; the passing
functional regression gate is not justification to deploy this partial candidate.

## Position-only retry contract

The next candidate explicitly sets `receiver.direct.position_only=true`. Normal
commands carry zero velocity and zero friction/ramp feedforward, while retaining
saved PD gains and gravity compensation. The receiver rejects a nonzero velocity
or ramp in a client update before consuming its authority. Its own braking after
expiry/fault still uses measured motion. The internal reference velocity remains
separate so position slew and acceleration do not reset just because wire velocity
is zero. This removes the reproduced derivative/friction command pulses; actual
physical smoothness and high-speed damping lag remain unmeasured.

The separate `minerva-teleop-direct-trial.json` profile is an attended diagnostic
baseline: only left J1 follows, at no more than 0.05 rad/s, within ±0.03 rad of its
first Enable hold position. Other joints hold that same latched pose. Hold,
source loss and re-Engage cannot move the origin; Disable clears it. Every source
joint still passes freshness/range/sequence validation. Incompatible lead and
trial bounds fault rather than moving the window or exceeding the trial slew.
Shutdown/recovery may return all joints to reference outside the trial window.
These are command bounds, not a guarantee against physical overshoot.
The 0.05 rad/s cap applies to following and planned return motion. Explicit
Hold/source-loss stops may capture the measured selected-joint pose inside the
fixed window without a slew ramp; inactive-joint targets remain latched.

Use `scripts/qualify_direct_motion.py --output <new directory>` for seeded,
quantized 90 Hz source / 100 Hz consumer command qualification. It compares
position and internal reference velocity against retained-feedforward behavior,
checks zero wire velocity/ramp and gravity-only effort, and includes an old-mode
negative control plus source-loss/mutation/expiry checks. Its ideal follower is
explicitly not a measured physical model. The full-loop benchmark's
`--source-mode quantized90 --profile config/devices/minerva-teleop-direct-trial.json`
then exercises the actual receiver and saved mapping, with held goals, reversals,
immutable repeats and explicit coverage checks. A stationary timing pass alone
does not satisfy that moving-source check.

### Cyclic-collection timing failure in the retry candidate

Candidate `20261010T190423Z-4e6c6ce` passed the frozen offline gate (490 Python,
115 Rust) and receiver-only vcan qualification (3,000 updates, all fault checks).
Its moving vcan run passed every command coverage/bound check but failed after
647 accepted following updates: a 4.956 ms mapping outlier made the full cycle
10.167 ms, skipped one scheduled slot, then expired the unchanged 30 ms command
deadline. Both the failed run and the diagnostic rerun are preserved under
`build/direct-retry/` and Jetson validation `direct-retry-20261010T190000Z`.

The unchanged-runtime diagnostic correlated generation-1 GC pauses of 5.483,
5.689 and 5.758 ms with the slow cycles; thread CPU matched wall time and each
collection reclaimed zero objects. The last pause produced a 10.508 ms loop,
another skipped slot and command expiry. Freezing startup objects alone leaves
later recorder/mapper objects subject to automatic collection. The bounded
correction suspends automatic cyclic GC only during direct control, keeps normal
reference counting, closes control resources first, and restores the caller's GC
state during cleanup. Guarded modes retain their previous behavior. Fresh timing
and longer-run allocation/RSS measurements are required; no deadline is relaxed.


### Final retry qualification: 2026-10-10

Runtime `20261010T191330Z-1624fda` (clean commit `1624fda`) passed the complete
offline gate: 494 Python tests, zero skips/xfails, 115 Rust tests. Report:
`build/dev-check/20261010T191234Z-f8d643e8/report.json`. The Linux AArch64 native
build passed. Runtime content SHA-256 is
`3a9f8ef486aa2fce587053761262861e048b4b11199ec3f82e5500cc1bf51dca`.
This documentation-only result note follows the frozen gate and release package.

The final Jetson qualification used a private network namespace containing only
loopback and virtual CAN. The native receiver, full TeleopCore/run_loop, recorder,
UI construction and saved operator bindings were exercised together:

| Test | Accepted/applied | Following work p99 / max | Following misses/skips |
|---|---:|---:|---:|
| Moving 90 Hz quantized input, 100 Hz consumer | 5,000 / 5,000 | 4.815 / 6.993 ms | 0 / 0 |
| Real USB leaders and wired tether, stationary input | 1,000 / 1,000 | 6.328 / 8.670 ms | 0 / 0 |
| Receiver-only protocol and fault qualification | 3,000 / 3,000 | 2.690 / 6.585 ms (RPC cycle) | 0 / 0 |

All receiver fault checks passed. Moving coverage included 500 immutable repeats,
held/reached goals, both reversal directions, zero wire velocity/ramp effort,
gravity-only effort, held inactive joints, and the fixed J1 window/speed cap.
The 50-second moving test saw no cyclic collection during following and RSS was
50,647,040 bytes at all ten samples. Allocation counts stabilized after warmup;
this is bounded evidence, not proof for unlimited operation. GC returned to its
original enabled state and unfroze on close. Both full-loop runs had one startup
enable-phase scheduling miss (14.512 ms synthetic, 16.143 ms USB); zero-miss
claims cover following only. All services ended disabled and closed.

The actual USB input was stationary during measured following. Its pass qualifies
USB/tether timing only; synthetic moving input separately covers commands, and
neither test validates physical smoothness or high-speed damping lag.

Stopped preflight checked all 14 motor identities, fault-free Reset state,
positions inside saved travel and near reference. The overall Minerva Check is
INCOMPLETE because attended live joint sweeps remain pending. No motor enable was
requested. Final verification found both CAN links down, no controller owner,
and the prior release `20261010T175730Z-abfbbbd` still active.

The next attended trial is left J1 only, 0.05 rad/s and ±0.03 rad; all other left
joints hold. Use the trial profile and `build/direct-retry/operator-runs` because
the original route record is bound to the full direct profile. Original routes
are preserved; only the copied profile binding changed, with hashes recorded.
J1 remains Kp 150 / Kd 9.8. Idle (I) already enables physical motors; starting
the Helm server only starts the stopped reader. Shutdown/recovery can return all
joints to reference outside the trial window. Collision detection is disabled.

Activation of this specific runtime and reopening Helm on the trial profile are
the remaining operator actions. No physical tracking approval is inferred from
these software checks. `build/direct-retry/readiness.json` records all evidence
paths/hashes. The final remote archive SHA-256 is
`c492c38fd12dbe3eb25023dcd9477c1b92e226ef661a40b149625f1baa13dbe7`.
The original failed physical trace and both pre-GC retry failures remain intact.
