# Feature task: record each drive's whole command at every receiver tick

- Branch and worktree: `claude/actuator-response`, `C:/Users/ltr/Workspace/aizee-actuator-response`
- Base commit (full SHA): `5a28ad78604b777ea8266935d04011a2fc05e360` (`claude/vff-toggle`, the active release
  `20261015T024454Z-5a28ad7`). It was first written on `6fb0510` and rebased so that a deploy keeps the
  velocity feed-forward switch.
- Owner: Claude Code, at the operator's request (2026-10-14), for the actuator-response calibration track:
  fit each joint's real response and reproduce it in the sim.
- Problem: the receiver's 400 Hz recorder kept the commanded position only. The commanded velocity, feed-forward
  effort, kp and kd that each drive executed were not recorded per tick, so a recorded session could not be
  replayed through a joint model. The direct teleop trace has velocity and effort at 100 Hz, but not between
  client cycles (the receiver interpolates) and not for sweeps or returns.
- Hypothesis: appending the drive command's velocity, effort, kp and kd after the existing columns records what each
  drive was told, without moving any column that readers index today.
- Owned files: `rust/control_core/src/recorder.rs` and its test, the `recent` doc in `service.rs`,
  `servo_sweep.py` (row layout, `recorder_columns`, `full_rows`), the `columns.json` writers in `helm/teleop.py` and
  `helm/tune.py`, `PlantReceiver._row`, and the two tests that pinned the old width.
- Shared interfaces held fixed: the receiver's control law, checks, timing and protocol messages (only the length
  of `recent` rows changes); every existing column's index.
- Dependencies / integration order: on top of the active release; the velocity feed-forward line must include
  this commit before its next deploy, or the deploy drops it.
- Acceptance: the gate, then the Jetson private-vcan manual-tuning qualification (both arms, both laws, 125 s),
  with the same bounds as the release it extends.

## Change

Each recorder row is t_ms, then per joint the measured position, velocity and effort and the commanded position (as
before), then per joint the commanded velocity, effort, kp and kd. Rows are 1 + 8 x joints wide.
`servo_sweep.recorder_columns` names them for Helm's teleop and Tune `columns.json`. `full_rows` gives a row from
an older receiver, which lacks the second block, NaN there: unknown, never zero. The virtual harness's synthesized
row and the simulated arm's rows carry the same fields.

A reply with the receiver's full 256-row ring (only after the client has stalled ~640 ms) grows from ~130 KB to
~260 KB. A typical 100 Hz reply carries ~4 rows (~2 KB more). The service's 64 KB `maxmsgsize` limits incoming
requests only.

## Results

- Gate: 717 Python tests, zero failures/skips/xfails, and 125 Rust tests, on `5852fe2`
  (`build/dev-check/20261015T033211Z-cd47aaf0/report.json`), with fresh ARM64 binaries.
- Jetson qualification, first candidate `3bdae8f` (`~/minerva/validation/recorder-3bdae8f-01`):
  - **Position-only, both arms:** passed.
  - **Feed-forward, both arms: failed scheduling.** All 13,011 cycles were accepted with no receiver fault, but one
    9.96 ms cycle missed its slot: the receiver reply took 5.27 ms (p50 3.15) and recording bookkeeping 2.58 ms.
  - **Cost of the wider rows.** Against the active release's run, the receiver call's p50 rose 0.09 ms (larger
    replies to decode) and the rest of the loop's 0.09 ms. Part of the latter was `result_rows` converting every
    result's rows through numpy (16 us on the PC). `5852fe2` passes full-width rows through untouched (0.7 us).
- Jetson qualification of `5852fe2` (`~/minerva/validation/recorder-5852fe2-01`). Every run completed with all cycles
  accepted and no misses:

  | Run | Cycles | Loop p50 / p99 / max (ms) | Powered misses / skips |
  |---|---|---|---|
  | Feed-forward, left | 13,012 | 2.30 / 5.16 / 6.51 | 0 / 0 |
  | Feed-forward, both | 13,010 | 4.71 / 5.69 / 7.94 | 0 / 0 |
  | Position-only, both | 13,012 | 4.70 / 5.69 / 6.27 | 0 / 0 |

  - Each run recorded 113 columns per tick (1 + 8 x 14) with no NaN.
  - Left J1's recorded command velocity reached 0.05 rad/s under feed-forward and stayed 0 under position-only.
  - Its kp/kd ramp 135 -> 150 and 8.8 -> 9.8 as gains are applied.
- Deployed release `20261015T034019Z-5852fe2`, with teleop and Tune stopped:
  - the receiver binary (`475f6b09...`) is identical to the qualified one;
  - runtime `src/` and `config/` are identical apart from the deploy smoke test's bytecode caches;
  - `20261015T024454Z-5a28ad7` stays on the Jetson as the rollback target.

## Remaining uncertainty

- Recordings made before this release lack the command block (NaN). Sweeps fit from them need their segment gains
  reconstructed from the sweep plan.
- Feedback age (each reading's `sampled_at_ms`) and drive temperature are still not per tick; the delay fit may
  need the age.
