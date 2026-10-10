# Manual teleop tuning

Branch: `codex/teleop-direct-mirror`; base for this change: `a0406fd`.

The operator controls tuning from Helm's Teleop tab using
`config/devices/minerva-teleop-direct-tuning.json`. Each motor has Kp, Kd,
following speed, Follow, and an optional travel window. This profile uses direct
position-only commands with collision detection disabled. One live arm is
supported at a time; Virtual can exercise both arms without physical motors.

## Operator workflow

1. While the session is stopped, choose the arm and prepare settings. Apply
   prepares the next session. Start performs the existing identity, reference,
   feedback and watchdog checks. Idle then Enable holds the measured pose.
2. Engage (T) aligns with the leaders and follows. T again holds. In HOLD, edit
   settings and press Apply. Wait for the receiver acknowledgement, then T
   resumes following. Editing or applying never automatically engages motion.
3. Revert edits discards the browser draft. Save applied persists acknowledged
   settings. Restore saved applies the saved values through the same gates.
4. H immediately disables; X returns to reference and disables. Both keyboard
   actions remain available while numeric inputs have focus or Apply is waiting.

Unchecked joints hold their current commanded position and remain powered.
Initial defaults reproduce the small left-J1 experiment: 0.05 rad/s and a
±0.03 rad window, with other joints held. The operator can select other joints
and change those values without rebuilding. A blank travel field uses the joint's
full usable range. Finite windows remain centered on the first Enable pose;
re-engaging does not walk that origin. A window smaller than the current target
is rejected. These are command limits, not a guarantee against physical overshoot.

Following speed controls following and alignment. Shutdown retains its qualified
profile speed independently; it can move all joints outside experimental windows
to return to reference. Native feedback speed, position, tracking, effort-command,
stopping and freshness bounds remain in force. Gain ceilings in this explicit
manual mode are the motor's encoding limits; they are not physical tuning
recommendations or measured stability limits.

## Apply and persistence contract

Each complete ordered edit includes a session ID and revision. Stale sessions,
partial batches, invalid fields and values outside displayed bounds are rejected.
Running edits require HOLD and settled feedback. A gain change uses one normal
direct update, keeping the previous target and zero velocity feedforward. The
native receiver atomically commits the full batch and its fallback hold gains;
the UI waits for `last_applied_target.sequence` before promoting the revision.

A settling-only rejection preserves the prior gains and held command with the
original offer and lifetime. It never retries the gain edit. Failed ownership,
freshness, identity, deadlines or hardware writes still fault normally.

Manual presets live in the selected runs directory under
`<asset>-teleop/<profile-id>-manual.json`. They bind the profile, calibration,
motor identities/firmware and measured servo record. They explicitly identify
operator-selected values and do not modify or masquerade as servo calibration.

## Timing and validation

The previous physical trial was reported small, smooth and without chatter by
the operator. Preserved records nevertheless show `command_expired` after about
71 seconds of TELEOP: 7.811 ms mandatory work plus 2.554 ms observation/other
overhead crossed the 10 ms slot; the next command arrived too late. This was not
a stopping fault. Evidence: `build/manual-tuning/live-20261010T214120Z` and
`build/manual-tuning/analysis`.

The direct loop now budgets full observation/guide updates at approximately
10 Hz, leaving every control and recording cycle intact. State changes, keys,
faults and tuning acknowledgements publish immediately with cached joint rows.
Observation timestamps distinguish old measurements from current control status.
No command deadlines, watchdogs, skipped-slot or fault assertions are relaxed.

The first 125-second manual candidate qualification (`e13753b`) failed after
118.8 seconds: an on-time cycle spent about 49 ms before input mapping, with a
48.6 ms aggregate maximum in synchronous trace recording. The receiver reported
zero missed deadlines or overruns. The failed candidate remains unactivated;
its raw traces and first-divergence report are preserved under the task's
`qualification/candidate-e13753b` evidence directory.

Recording now submits immutable data to a bounded FIFO worker. File open,
write, flush and close operations run outside the control loop. No queued data
is silently dropped: queue exhaustion or writer failure faults the session,
and cleanup disables and closes the receiver before waiting up to five seconds
for evidence to drain. A recording failure cannot interrupt a disable RPC.
JSON encoding and enqueue costs remain in full-loop timing measurements.
This change requires a fresh candidate qualification before activation.

Run `python scripts/dev.py check --offline` for the required zero-skip gate.
It includes manual lifecycle, native gain-commit, API/concurrency, persistence,
timing-budget and real native-receiver integration tests. Then run the new
candidate on Jetson virtual CAN for longer than the original failure, including
HOLD → Apply → Engage and recording. Hardware activation is a separate reviewed
step; this change does not authorize automatic live tuning or motor enable.

Task evidence and the candidate qualification manifest are under
`build/manual-tuning`. The currently active robot release must be checked against
that manifest before launching this profile on physical motors.
