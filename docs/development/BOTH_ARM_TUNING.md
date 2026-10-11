# Feature task: manual tuning and teleop on both live arms

- Branch and worktree: `claude/both-arm-tuning`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `f846b65` (`codex/overhaul-integration`, after the passing both-arm direct timing)
- Owner: Claude Code, at the operator's request ("build the both arm tuning and teleop")
- Problem: the manual-tuning profile refused a live receiver on both arms ("one live arm at a time"); the
  operator wants to tune both arms.
- Hypothesis: the tuning code is already per joint (Virtual ran both arms), so lifting the refusal is enough
  in code; the risk is Jetson timing with 14 joints.
- Owned files: `helm/teleop.py` (refusals, receiver choices, copies), `control/plain_data.py`,
  `control/guarded.py`, `teleop/direct_lifecycle.py`, `teleop/lifecycle.py` (copies only),
  `scripts/qualify_manual_tuning_loop.py`, tests named below, `MANUAL_TELEOP_TUNING.md`.
- Shared interfaces held fixed: the receiver protocol and Rust service; every timing, watchdog, freshness,
  speed, travel and stopping bound; the Hold → Apply → Engage handshake; tuning defaults (left J1 follows
  ±0.03 rad, everything else holds).

## Change

1. Live left, right or both under manual tuning; Helm offers Live · both. One receiver drives 14 joints; one
   Apply writes both arms' gains without moving a target; each arm follows its own leader.
2. `qualify_manual_tuning_loop.py` (Codex's Jetson-only script) is versioned with `--arm left|right|both`. With
   both, each live J1 is tuned and follows (the right one within the left default's ±0.03 rad window), and
   each must move and reverse.
3. First both-arm qualification (`d0cff65`) failed: the first Apply cycle took 12.16 ms (gain-write round
   trip 6.03 ms, ~6 ms Python) and the command expired. Profiling showed `copy.deepcopy` bookkeeping was most
   of the loop's Python time (73 % of an ordinary both-arm HOLD tick; the Apply rollback snapshots on top).
   `plain_copy` makes the same independent copy of plain data and hands anything else to deepcopy. On the PC
   a both-arm HOLD tick went 1.87 → 1.29 ms and an Apply tick 2.92 → 1.73 ms. The qualification's own
   gain-write evidence copies use it too (they sit inside the timed exchange; production has none).

## Acceptance and results

- Gate: 652 Python tests, zero failures/skips/xfails, Rust included,
  `build/dev-check/20261011T225016Z-e202c54f/report.json` (`c891563`). New tests:
  `test_manual_tuning_drives_both_live_arms_in_one_receiver` (real Rust receiver core),
  `tests/unit/test_qualify_manual_tuning.py`, `tests/unit/test_plain_data.py`.
- Jetson, private virtual CAN, release `20261011T225811Z-c891563`, saved tuning-profile operator runs,
  `~/minerva/validation/both-tuning-c891563-01/`:
  - **Both arms** (`tuning-both-125s`): complete, 125 s, all five Hold/Apply/Engage checks, both J1s moved and
    reversed, zero misses or skipped slots, ended DISABLED. Full loop p50 3.94, p99 4.83, max 5.33 ms; Apply
    cycles max 7.1 ms (was 12.16 ms).
  - **Left arm** regression (`tuning-left-125s`): complete; full loop p50 2.13, p99 4.69, max 5.48 ms; Apply
    max 5.7 ms (the `20c0f0e` run: p50 4.48, max 7.10, Apply 8.2 ms).
  - The failed `d0cff65` attempt is kept in `~/minerva/validation/both-tuning-d0cff65-01/`.

## Follow-up: source-loss hold uses following's lead (`claude/source-loss-lead`, `0b2900d`)

The first live left-arm sessions on `0c1ef49` (2026-10-11): two ended cleanly with X after about 53 s and 65 s
of following; the third faulted 5 s after Engage with "Measured pose exceeds the holding lead bound"
(Jetson `~/minerva/evidence/live-20261011T232521Z/`). A fast wrist move (the leader asked J5/J6 for about
3 rad/s; commands were capped near 1 rad/s) left J5 0.095 rad behind its command, which following allows
because it bounds the lead from where the joint will be (measured position carried on at its speed over the
sample's age). One cycle then had no usable leader sample, and that path checked the raw reading: 0.0880 >
0.0874 rad. Recovery worked: braked to a hold, lowered to the reference, disabled.

Both paths now use `_lead_origin`. A stopped joint gets no credit and faults as before; one moving away faults
sooner; joints held rather than following keep the raw check. `test_one_late_leader_sample_holds_a_moving_
joint_following_had_allowed_to_lag` fails on the old code with the live error and passes now; its stopped
counterpart faults on both. Gate: 654 Python tests, zero failures/skips, Rust included,
`build/dev-check/20261011T234152Z-328961e9/report.json`. Jetson tuning qualification, release
`20261011T234934Z-0b2900d`, `~/minerva/validation/both-tuning-0b2900d-01/`: both arms complete (full loop
p50 4.70, p99 5.65, max 7.02 ms; Apply max 8.0 ms) and left arm complete (2.67 / 3.36 / 5.82 ms; Apply 6.5 ms),
zero misses or skipped slots. Helm's stopped reader was likely running during these runs.

## Remaining uncertainty / physical inputs needed

- Emulated motors and synthetic leaders; not physical tracking or gain stability.
- No arm-to-arm protection in this profile: keep travel windows small on joints that can bring the hands
  together until a two-arm bound exists.
- Right arm: live sweeps and direction checks pending; the right gripper was 0.13 rad from reference (start
  tolerance 0.10) at the last stopped check.
- Live use: `0c1ef49` was deployed on 2026-10-11 (without the source-loss fix); `0b2900d` or later needs
  deploying, and Helm launched from this checkout with tuning-profile operator runs.
