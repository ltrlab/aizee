# Feature task: top following speed in the tuning panel

- Branch and worktree: `claude/tuning-speed-estimate`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `a36603bc7d9a834d7ac7e85e942a4710e9e4da73` on `claude/live-view-rate` (on `claude/source-loss-lead`; neither in
  integration yet)
- Owner: Claude Code, at the operator's request
- Problem / saved reproducer: in direct teleop, left J1 lagged the leader at Kp 250 / Kd 9.8 although its
  following speed was 1.5 rad/s. Jetson `~/minerva/evidence/live-20261012T004922Z`: the leader reached
  2.36 rad/s, commands and the joint 0.55 rad/s (p99), |leader − command| p99 0.52 rad, |command − measured|
  at most 0.030 rad. The operator asked for the speed math in Settings → Motor tuning, against the limit.
- Hypothesis: direct teleop sends position only, so the drive's damping opposes all motion. A command leads
  the joint's predicted position by at most `max_lead` (0.6 × the tracking bound, `guarded_config`), predicted
  by the feedback's age plus the receiver's interpolation (`TeleopLifecycle._lead_origin`). At a steady speed v,
  Kp × (lead + v × ahead) = Kd × v + friction, so v = (Kp × lead − friction) ÷ (Kd − Kp × ahead); with
  Kd ≤ Kp × ahead the lead never caps it.
- Owned files: `src/robot_platform/helm/manual_tuning.py` (`following`), `src/robot_platform/helm/static/helm.js`
  and `helm.css` (the tuning card), `tests/unit/test_manual_tuning.py`, `tests/unit/test_manual_tuning_api.py`.
- Shared interfaces held fixed: every receiver and lifecycle bound, the saved tuning document (`values` never
  carries `following`), the edge's `receiver_rows`.
- Required behavior: the estimate is display only and follows the draft Kp, Kd and speed as they are edited.
- Acceptance command and expected result: `python scripts/dev.py check`; the panel on Minerva's real profile.

## Evidence for the model

Left J1, lead 0.0234 rad, from the six recorded sessions on 2026-10-12 (gains from each `live-canL.json`):

| Kp / Kd | Measured \|v\| p99 | \|leader − command\| p99 | Lead-limited speed (model) |
|---|---|---|---|
| 250 / 9.8 | 0.55 rad/s | 0.52 rad | 0.55 with ~2 Nm friction (0.80 without) |
| 500 / 6 | 1.23 | 0.26 | far above the 1.5 limit |
| 300 / 4 (two sessions) | 1.12, 1.03 | 0.23, 0.07 | far above |
| 350 / 5 (two sessions) | 0.93, 0.99 | 0.04, 0.05 | far above |

At the lead limit the error along the motion was 0.029 rad = lead + v × 11.3 ms, the feedback's median age
(11.6 ms). The panel uses one loop period (10 ms) for that age. Above the model's limit the joint followed the
leader up to the speed cap and the 32 rad/s² acceleration bound.

## Results

- Live Helm on Minerva's profile: J1 at 350/5 shows "Top speed is your 1.5 rad/s limit; the 0.023 rad lead
  alone would allow ≈ 5.45 rad/s." Typing 250/9.8 shows ≈ 0.80 rad/s with the unmeasured-friction note;
  edits reverted. Grippers show none (their lead spans their range).
- Gate: 665 Python tests, zero failures/skips/xfails, Rust tests included, on `2f17b01`
  (`build/dev-check/20261012T023315Z-dcc76c8c/report.json`).

## Remaining uncertainty

- Left J1's drive reports no torque: at most 0.001 Nm in both sessions checked, while left J2 reported up to
  15 Nm. Its servo sweep therefore stored 5e-20 Nm of friction, and the panel treats under 0.01 Nm as not
  measured. The fit above puts it near 2 Nm. Why the drive reports none is not yet known.
- The model is steady state in open travel; near either end the approach curve shrinks the lead and the speed.

## Correction (2026-10-13, branch `claude/speed-estimate-fix`)

The delay term had the wrong sign. The 0.029 rad above is the error the loop sees against its own (aged)
feedback. The lead origin already carries the joint forward by the feedback's age, so the command leads the
joint's position *at computation* by the lead. The drive then sees the command about one loop later (its way
to the drive and half the 10 ms it holds), when the joint has closed v × lag of the lead. At a steady speed:
Kp × (lead − v × lag) = Kd × v + friction, so **v = (Kp × lead − friction) ÷ (Kd + Kp × lag)**, lag one loop
(`following.command_lag_s`, formerly `lead_ahead_s`). The old form put the lag on the other side, which placed
most gains far above their limit.

Evidence, both-arm session `~/minerva/evidence/live-20261013T031504Z` (manual tuning, 400 Hz samples and the
per-cycle log): J3 at 320 / 8 trailed the leader by up to 0.7–1.0 rad while held near 0.7 rad/s in both
directions. At the lead limit its drive-side error (command − position at 400 Hz) was 0.021–0.022 rad, against
0.035 rad loop-side.

| Joint, Kp / Kd | Corrected estimate | Measured | Old estimate |
|---|---|---|---|
| Left J3, 320 / 8, lead 0.0277, friction 0.46 | 0.75 rad/s | 0.69–0.71 (p50 while lead-limited) | 1.75 (shown as the 1.5 limit) |
| Right J3, 320 / 8, lead 0.0259, friction 0.46 | 0.70 | 0.69–0.71 | 1.6 |
| Left J1, 250 / 9.8, lead 0.0234, friction 0 | 0.48 | 0.55 (p99) | 0.80 |
| Left J1, 300 / 4 | 1.00 | 1.03–1.12 (p99) | 7.0 |
| Left J1, 350 / 5 | 0.96 | 0.93–0.99 (p99) | 5.45 |
| Left J1, 500 / 6 | 1.06 | 1.23 (p99) | 11.7 |

The J1 rows fit without the ~2 Nm of friction the old form needed. p99 runs above the steady estimate in
transients. Raising Kp is now the lever for speed: as Kp grows the estimate approaches lead ÷ lag. Lowering
Kd also raises it but damps less.

Every arm joint in the same session, the panel's estimate on Minerva's profile against the joint's speed in
the cycles its command sat at the lead limit (`limits: tracking_lead` in `direct-latency.jsonl`, p50):

| Joint | Left: panel / measured (rad/s) | Right: panel / measured (rad/s) |
|---|---|---|
| J1 | 0.69 / 0.69 | 0.68 / 0.61 |
| J2 | 1.00 / 0.94 | 1.09 / 0.98 |
| J3 | 0.75 / 0.75 | 0.70 / 0.71 |
| J4 | 1.08 / 0.92 | 1.11 / 1.08 |
| J5 | 2.15 / 2.17 | 1.09 / 1.09 |
| J6 | 1.61 / 1.50 | 1.24 / 1.15 |

Within 15%, mostly within 8%; where it errs it is high (friction or gravity left over).

- Results: gate 700 Python tests, zero failures/skips/xfails, Rust included, on `f63aed1`
  (`build/dev-check/20261014T005220Z-d1ece389/report.json`); the panel on Minerva's live tuning profile shows
  the table's estimates.
