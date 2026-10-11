# Feature task: bounded teleop cycle history

- Branch and worktree: `claude/teleop-trace-history`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `8e5e4512c8f700831c6bb89517dd4827696c008b` (`codex/overhaul-integration`)
- Owner: Claude Code, at the operator's request after the Jetson qualification of `8e5e451` failed
- Problem / saved reproducer: the 125 s manual-tuning qualification of `8e5e451` on the Jetson (private
  namespace, virtual CAN, `qualify_manual_tuning_loop.py`) expired a command after 90.4 s of following. One
  cycle took 11.76 ms: a 7.64 ms receiver reply (median 3.18 ms) in the same cycle as a 2.64 ms `_record`
  call. `TeleopCore._record` kept 500 full trace entries and deleted the oldest 100 at once, so `_record`
  stalled about 3.5 ms every ~100 cycles (87 spikes; all 58 cycles over 7.5 ms had one). The view step, with
  the new telemetry, was in none of them. Evidence: Jetson
  `~/minerva/validation/settings-telemetry-8e5e451-01/tuning-vcan-125s/`.
- Hypothesis: the batch delete frees 100 large dicts in one cycle. Readers only use the last 100 cycle
  durations, and the full trace already goes to the latency file.
- Owned files: `src/robot_platform/helm/teleop.py` (cycle history only), `tests/unit/test_direct_teleop.py`.
- Shared interfaces held fixed: the latency file's trace entries, the view's `cycle_ms` and `full_loop_ms`
  (last 100 cycles), `_recent_cycle_s` (last 25), every timing, watchdog, freshness and stopping bound.
- Dependencies / integration order: after `codex/helm-settings-telemetry` (already in integration).
- Required behavior and numerical/time bounds: no change to the qualification's targets; zero following
  misses or skipped slots over 125 s.
- Acceptance command and expected result: `python scripts/dev.py check`; then the Jetson manual-tuning
  qualification, `complete: true` with both timing targets met.
- Source/model identity and seed: release `20261011T164524Z-20c0f0e` (clean `20c0f0e`), same ARM64 binaries
  as the qualified `30c4835` release (hashes match), the same saved operator routes and qualification script.

## Change

`cycles` keeps only `started_at`/`completed_at` per cycle and, with `_loop_timings`, is a
`deque(maxlen=CYCLE_HISTORY)` (100): each cycle drops one small entry, and each full trace entry is freed once
the latency file has it. A separate thread would not help: freeing Python objects holds the interpreter lock,
so the loop would wait for it instead (the switch interval is 5 ms).

## Results

- Gate: 647 Python tests, zero failures/skips/xfails, Rust tests included,
  `build/dev-check/20261011T163923Z-9b6ad783/report.json`.
- Jetson qualification, `~/minerva/validation/trace-history-20c0f0e-01/tuning-vcan-125s/`: complete, 125 s,
  12,501 following cycles, zero misses or skipped slots. Full loop p50 4.48, p99 5.23, p99.9 5.67, max 7.10 ms
  (was 4.51 / 5.57 / 7.92 / 11.76 ms); `_record` max 0.27 ms (was 3.66 ms), no spikes over 2 ms (was 87);
  cycles over 7.5 ms: 0 (was 58). Codex's passing `30c4835` run had max 8.04 ms.
- Staging note: the first attempt failed at startup because the qualification copy was repacked on Windows and
  its binaries lost the execute bit (`Permission denied`); a wrapper outside the verified tree
  (`diag_qualify.py`) showed the hidden error. Restoring mode 755 (contents identical to the release copy)
  fixed it. Both failed attempts are kept beside the passing run.
- Both arms (`benchmark_direct_loop.py --backend vcan --arm both --source-mode quantized90 --cycles 12500`,
  direct profile, saved direct-profile routes, same release and sandbox): complete, 12,500/12,500 following
  updates, zero misses or skipped slots, full loop p50 4.09, p99 6.68, max 6.97 ms; RPC p99 4.43 ms; ended
  DISABLED. Startup Idle/Enable cycles (14.1 / 11.3 ms) are outside following, as in earlier runs.
  Evidence: `~/minerva/validation/trace-history-20c0f0e-01/both-vcan-12500/`.

## Remaining uncertainty

- One 125 s run on emulated motors; it is not a physical or two-arm qualification.
- `TuneCore` (servo sweeps) and `shadow_runtime` keep their history the same way; not changed here.
- The rare slow receiver reply (7.64 ms once in 9,037 cycles) is not explained; with the spikes gone the loop
  has about 5 ms of margin against it.
