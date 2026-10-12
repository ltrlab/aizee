# Feature task: live view scan rate

- Branch and worktree: `claude/live-view-rate`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `52733cf0e4308b20a72bbac06baf5d6fd34d6f55` (`claude/source-loss-lead`, the deployed line; not yet in integration)
- Owner: Claude Code, at the operator's request
- Problem / saved reproducer: outside a session, Helm's 3D view shows the torque-off stopped reader, and its
  banner flipped between LIVE and STALE about once a second. Sampled at 4 Hz on Minerva (release `52733cf`):
  a frame every ~0.42 s, each already 0.43–0.63 s old on arrival (one 421 ms scan plus the round trip), so
  it crossed the 0.75 s display limit just before the next frame landed. The viewer also skips a frame that
  arrives over 0.75 s old, so the model moved about once a second.
- Hypothesis: the scan, not the link, sets the age. Each of the 14 drives took three requests (stop,
  position, power-on range) spaced 10 ms apart across both arms, 42 × 10 ms ≈ 420 ms. The two arms are on
  separate USB CAN adapters (the reader checks each adapter's serial), and nothing in the live stream uses
  the power-on range.
- Owned files: `src/robot_platform/control/joint_capture.py`, `tests/unit/test_joint_capture_preflight.py`.
- Shared interfaces held fixed: the 0.75 s display freshness, the 0.5 s scan and 0.75 s round-trip limits,
  the stopped-only allowlist, the identity preflight, the 5 ms spacing in `SocketCanMaintenance`, and
  Check's scan, which still reads every drive's power-on range.
- Dependencies / integration order: after `claude/source-loss-lead`; the change itself touches neither.
- Required behavior and numerical/time bounds: at least 10 ms between requests on each adapter, as before;
  no stale display while every drive answers.
- Acceptance command and expected result: `python scripts/dev.py check`; then the live stopped reader on
  Minerva shows LIVE throughout, motors disabled.
- Source/model identity and seed: Helm from this branch; the reader stages `joint_capture.py` on the Jetson
  by content hash (`joint-calibration-ebdbff8de36945b1`).

## Change

`paced` keeps the 10 ms spacing per bus instead of across both, and `live_scan` asks one motor per bus at a
time, so one adapter answers while the other waits out its spacing. Rows stay in roster order. The live
stream passes `power_on=False`; Check's single scan still reads the power-on range.

## Results

- Live on Minerva, arms disabled: scan p50 139.5 ms, max 140.3 ms (was 421 ms); 6.9 frames/s (was 2.4);
  Helm's packet age 0.15–0.28 s; the viewer's banner LIVE in all 143 samples over 8 s, age 150–390 ms,
  zero STALE; no rejected scans.
- Gate: 663 Python tests, zero failures/skips/xfails, Rust tests included, on `4945672`
  (`build/dev-check/20261012T012817Z-c1b1950b/report.json`). The scan tests now run in the gate.

## Audit size

The reader's audit on the Jetson (`~/aizee/device-maintenance/joint-calibration-*/captures/*-frames.jsonl`)
logged every CAN frame of every scan: ~32 KB/s before, ~67 KB/s (~240 MB per hour while Helm is open on the
tether) at the faster rate; 31 GB were free on 2026-10-12. At the operator's request it now keeps the CAN trace
for the preflight, the first frame and every rejected scan, as Check keeps its trace only on failure; later
healthy frames log their positions alone. Live: ~18 KB/s (~65 MB per hour), 2.6 KB per frame, the first line
holding all 168 preflight and first-scan events. Gate on `31ee57b`: 664 Python tests, zero
failures/skips/xfails, Rust included (`build/dev-check/20261012T022420Z-0542ce01/report.json`).

## Remaining uncertainty

- Check's scan uses the same alternating order but was not run live on this branch.
- After a session ends, the reader still waits its 5 s handoff and repeats the identity preflight before the
  view is live again.
