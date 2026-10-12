# Feature task: say what tripped the receiver

- Branch and worktree: `claude/receiver-trip-detail`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `b7ed8cb01519b1c9409b9df3eed701cefdbeeee3` on `claude/motor-temperature` (stacked on `claude/leader-firmware`,
  `claude/leader-timing`, `claude/tuning-speed-estimate`, `claude/live-view-rate`, `claude/source-loss-lead`;
  none in integration yet)
- Owner: Claude Code, at the operator's request (2026-10-12). AGENTS.md gives receiver checks one owner; this
  changes no check, bound or order, only what the receiver reports about one.
- Problem / saved reproducer: live session `~/minerva/evidence/live-20261012T033419Z` faulted after 130 s of
  following with "Receiver stopping: limit_violation · … wrong_owner". The receiver's own 400 Hz samples showed
  every joint's position, speed (against the approach curve, `scratchpad replay_limits.py`) and torque inside
  its limits when it braked (receiver clock 153.802 s); temperature, the one remaining limit, was not recorded.
  Six minutes later every drive read 23–28 °C (the stopped reader's new temperature readout). Which joint and
  check tripped could not be told.
- Hypothesis: none about the trip itself; the receiver must say what tripped it.
- Owned files: `rust/control_core/src/{protocol.rs,lib.rs}`, `rust/control_core/tests/{controller.rs,no_alloc_tick.rs}`,
  `src/robot_platform/control/live_receiver.py` (`trip_text`), `src/robot_platform/helm/tune.py`,
  `src/robot_platform/teleop/latency.py`, their tests.
- Shared interfaces held fixed: every receiver check, bound and their order; the 400 Hz sample rows (four
  values per joint, which servo analysis reads by stride); the tick's zero allocations.

## Change

- The limit, stale, feedback, tracking and collision checks return a `TripRecord` (reason, joint index, a static
  check name, reading, limit, receiver time) from the same code that decides them. It is copyable, so the tick
  still never allocates (`no_alloc_tick` now also checks a fault's trip is recorded). The status reports it as
  `last_trip` (`protocol::Trip`, joint named), kept only when a trip or fault changes the receiver's state or
  reason: a violation persisting while the arm brakes does not overwrite what stopped it, and a refused
  request returns its reason only. It outlives the re-arm that clears the reason, so a session's final status
  keeps it.
- Checks named: `position_below_lower`, `position_above_upper`, `speed` (against three times the speed allowed
  there), `sustained_speed` (three readings over it), `effort`, `temperature`, `tracking_error`, `collision_box`,
  `collision_box_command`, `feedback_age_ms`, `all_feedback_age_ms`, `drive_fault`, `invalid_feedback`,
  `feedback_regressed`.
- Helm's teleop and servo-sweep fault messages lead with it ("Left J3 moving at 1.90 rad/s for three readings,
  over the 1.60 rad/s allowed there") when its reason is the one reported.
- The direct teleop trace records each drive's temperature every cycle (`measured_temperature`).

## Results

- Gate: 693 Python tests, zero failures/skips/xfails, Rust included (76 controller tests), on `43ae080`
  (`build/dev-check/20261012T041942Z-7052dc36/report.json`), with fresh ARM64 binaries.
- Jetson manual-tuning qualification (private namespace, virtual CAN, 125 s), release
  `20261012T042409Z-43ae080`, `~/minerva/validation/trip-detail-43ae080-01/`:
  - left: complete, 13,012 following cycles all accepted, no powered misses, full loop p99 3.62, max 3.87 ms.
  - both, first run: **failed** at cycle 8,301 (83 s): one 10.38 ms cycle (receiver reply 3.9 ms against
    2.1 typical, started 1.0 ms late) skipped a slot, the next command arrived after the receiver's 20 ms
    command window, and the receiver stopped (`command_expired`). GC was off. Unchanged code; the same
    Python passed on `228ce6a` an hour earlier (`trip-detail-228ce6a-01`, max 6.13 ms).
  - both, second run: complete, 13,012 cycles all accepted, no powered misses, p99 4.57, max 6.33 ms.
- The final status's `last_trip` in every run is `invalid_feedback` at 4–7 ms: the receiver faults before its
  first feedback and the startup recovers it, as it always has; now it is visible.

## Remaining uncertainty

- Which limit tripped the 2026-10-12 session is still unknown; the next trip names itself.
- A single cycle over ~10 ms that skips a slot expires the 20 ms command window and stops the arm, in this
  release and the deployed one. Its cause (here 6 ms outside the receiver call) is not identified.
