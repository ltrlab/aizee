# Live teleop faults of 2026-10-15 and their fixes

- Branch and worktree: `claude/teleop-fault-fixes`, in `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
  (where Helm's `helm-tuning` launch runs).
- Base commit: `a211031` (`claude/vff-toggle`, the switch's control-law status). The branch merges `34d42e9`
  (`claude/actuator-response`), which holds the active release's recorder commits `3bdae8f` and `5852fe2`, so a
  release from it keeps recording each drive's whole command.
- Owner: Claude (motion contract changes coordinated in this branch: receiver checks, lifecycle input checks).

## Problem and evidence

Four live sessions within 7 minutes on release `20261015T034019Z-5852fe2` (evidence on the Jetson,
`~/minerva/evidence/live-20261015T2231..2238*`). Three fault kinds there had never occurred in any of the
earlier sessions. Velocity feed-forward played no part: no session reached Engage with it on, and the fourth
ran position-only.

| Session | Feed-forward | What happened | Cause |
|---|---|---|---|
| `223157Z` | on | Ended at start, motors never enabled | The robot waited 10 s for the PC's leader link |
| `223503Z` | on | Receiver fault before Idle, motors off | Motors-off audit: "right_arm_j4: moving at +0.225 rad/s, 3 readings in a row over 0.2 rad/s"; its position never moved |
| `223714Z` | on | Fault 13 ms after Enable, holding | Lifecycle: "feedback sequence or timestamp moved backwards"; no sequence went back |
| `223850Z` | off | 100 s of teleop, then HOLD (leader link stopped 22:40:50), then right J5 left Run mode on overcurrent while holding at 0.1 Nm | The tether dropped; the J5 drive event is hardware, not addressed here |

After the fourth session the PC's SSH connection was reset at 22:42 (paramiko "Socket exception 10054"), but the
robot's loop never saw an end of stream. It idled in FAULT holding the receiver, so Helm's reader reported
"Guarded receiver already running", until the user power-cycled the Jetson. The Jetson's sshd has no
`ClientAlive` setting, and TCP keepalive gives up only after 7200 s plus 9 probes 75 s apart.

## Fixes (one commit each)

1. `d66349f` receiver: a still motor's speed noise is not motion. The wrist drives' speed readings at rest have
   a std of about 0.09 rad/s, reach 0.25-0.32 rad/s in every session, and are unchanged since the night before.
   Their positions stay within two encoder counts. The reference gate and the motors-off audit now share
   `reference::Motion`. A motor counts as moving when `MOVING_READINGS` (3) readings in a row exceed
   `STILL_RAD_S` (0.2 rad/s) and its position has moved `MOVED_RAD` (2 mrad, about five counts) since the
   first of them. The driven overspeed checks are unchanged.
2. `602a172` lifecycle: feedback is ordered by the receiver's own stamp. At Enable the feedback age jumps from
   about 4 to about 10 ms in every session. In `223714Z`, a joint's newer sample, stamped in the same receiver
   millisecond as the last, mapped 0.1 ms earlier through the next offer's start (`sampled_at = offer_started -
   (now_ms - sampled_at_ms)`). Rows now carry `source_ms`. When both rows have one, the lifecycle orders samples
   by it, and an unchanged sequence must keep it. Freshness and skew still use the mapped time.
3. `6087138` edge: a robot session ends when Helm goes silent. Helm beats every `BEAT_S` (0.5 s), and its start
   frame carries `SILENCE_S` (5 s). `run_loop` and `run_tune_loop` end a session that hears nothing for that
   long, as if Helm had closed it: an engaged arm, or a sweep, is lowered first. `events.jsonl` logs
   `helm_gone`. An older Helm sends no `silence_s` and is never timed out; an older robot ignores beats.

## Reproducers and acceptance

Each fix has a test that failed before it: a run of still-motor speed readings (`tests/reference.rs`,
`tests/watchdog_trial.rs`), the Enable stamps of `223714Z` (`tests/unit/test_teleop_lifecycle.py`), and a
session whose Helm goes silent (`tests/unit/test_helm_edge.py`, `test_helm_teleop.py`, `test_helm_tune.py`).

Acceptance: `python scripts/dev.py check` passes with zero failures, skips or xfails. Then the Jetson
private-vcan manual-tuning qualification runs both arms, position-only and feed-forward, with zero misses.

## Results

- Gate on `b7de042`: 735 tests, zero failures, skips or xfails (`build/dev-check/20261015T232336Z-a7e3be48/
  report.json`). The first run failed only because the ARM64 receiver was older than the changed Rust
  sources; `scripts/build_jetson.ps1` rebuilt it (`guarded_controller` SHA-256 `a7213954...`).
- Release `20261015T232920Z-b7de042`, qualified in `~/minerva/validation/faults-b7de042-01` on private
  virtual CAN, 125 s each:

  | Run | Helm's live view on the Jetson | Following misses | Full loop p99 / max |
  |---|---|---|---|
  | Feed-forward, both arms | reading | 0 of 13,012 | 5.56 / 7.98 ms |
  | Feed-forward, left arm | reading | 0 of 13,012 | 5.38 / 6.63 ms |
  | Position-only, both arms | reading | 1 of 13,011 (12.6 ms) | 5.76 / 12.61 ms |
  | Position-only, both arms, again | reading | 1 of 13,011 (10.2 ms) | 5.64 / 10.19 ms |
  | Position-only, both arms | disconnected | 0 of 13,012 | 5.62 / 8.37 ms |

  The two misses were single stalls in different stages: Python poll and map with a normal receiver call, then
  one 7.6 ms receiver call. They were first put down to Helm's live view reading the arms beside the
  qualification; a later run with it disconnected also missed once (next section), so that was wrong. Deployed as `20261015T234351Z-b7de042` (receiver
  `a7213954...`, the qualified binary); smoke test passed.

## Remaining uncertainty

- Right J5's drive overcurrent while holding (first on the right arm; the left arm's J4/J5 latch only at
  power-off). Watch it; if it recurs while holding, inspect the right wrist wiring as was done for the left.
- The tether adapter dropped: check that nothing shares its USB path and that its power saving is still off.
- Session `223157Z`'s leader link never connected; Helm's message for it was not kept.
- Two sessions deploy to the robot (this branch and `claude/actuator-response`). A release from either must
  carry the other's deployed commits, or it silently removes them.

## A gripper that never answers again (2026-10-15, later)

Session `live-20261015T234712Z` (position-only; the switch had not been applied since Helm restarted) on release
`20261015T234351Z-b7de042`:

- 21 s into teleop, the right gripper's drive stopped answering while it moved: 2.92 rad of its 5.3 rad range,
  -0.84 rad/s, about 0.1 Nm. It was not at an end and not loaded. Its last samples arrived in a burst, then none
  for the 403 s until the motors were power-cycled. Afterwards Helm's reader still found it "Not answering", so
  its power or CAN connection is lost (hardware).
- The receiver tripped (`feedback_stale`) and braked the arm to a hold, as designed. But a stop completed only
  when every joint read at or below `settled_velocity` (0.15 rad/s), and the gripper's frozen last reading never
  did. The receiver stayed "stopping" and held the arm up, and the loop, which lowers the arm only after the
  stop, waited with an unchanged message. The receiver would also have refused to re-arm while any drive was
  quiet.

Fix (the operator's choice: lower without a lost gripper):

- `fd19868` receiver: a quiet drive's speed is unknown, so a stop completes on the readings that still arrive. A
  gripper quiet while the arm is braked or held is set aside until the arm is disabled. It keeps its last
  command; its silence and frozen readings no longer block re-arming or trip the lowering; the status lists it
  (`set_aside`). A quiet arm joint is never set aside: the arm holds, and re-arming stays refused until it
  answers.
- `95a9903` clients: rows carry `aside`, accepted only for a gripper and only while the arm is taken back after
  a fault. The shutdown, and the sweep's lowering, hold that gripper and do not wait for it. Helm names it and
  asks to check its power and wiring. While a quiet arm joint keeps the arm held, Helm says to support the arm
  and press H if it stays that way.

Reproducers: `tests/controller.rs` (quiet gripper and quiet arm joint, both stuck "stopping" before),
`tests/unit/test_teleop_lifecycle.py` (the stale gripper row was rejected before), and
`tests/contract/test_rehearsal.py`. In the rehearsals, with the real receiver against a simulated arm, the right
gripper goes quiet for good while moving in teleop, and with J2 raised in a sweep. The arm is lowered to the
reference and disabled and never falls. Without the lifecycle change the teleop rehearsal ends "Lowering
stopped ... stale feedback: right_gripper", as live.

Gate on `95a9903`: 740 tests, zero failures, skips or xfails (`build/dev-check/20261016T001838Z-85d61df1/
report.json`). Receiver `guarded_controller` SHA-256 `81b685a3...`. 

Qualified in `~/minerva/validation/aside-95a9903-01` on private virtual CAN, Helm's live view disconnected,
125 s each:

| Run | Following misses | Full loop p99 / max |
|---|---|---|
| Feed-forward, both arms | 1 of 13,011 (12.6 ms) | 5.60 / 12.63 ms |
| Position-only, both arms | 0 of 13,012 | 5.57 / 5.87 ms |
| Feed-forward, left arm | 0 of 13,011 | 4.75 / 5.63 ms |

The live view was off, so it is not why `b7de042`'s runs missed (corrected above). Since the recorder commits
(`3bdae8f`, `5852fe2`), qualification runs show a single 10-12.6 ms stall in 4 of 13 runs, against none in
`5a28ad7`'s 3. Each stall inflates whichever stage the loop is in (poll and map, or one receiver call), and p99 is
unchanged. The release it replaced has the same recorder path. Deployed as `20261016T003435Z-95a9903` (receiver
`81b685a3...`) after the operator chose this fix; the smoke test passed. Right J5's and the right gripper's
drives remain hardware items: after the power cycle the gripper still did not answer.
