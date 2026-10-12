# Feature task: leader firmware 2

- Branch and worktree: `claude/leader-firmware`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `17201ffb064e7f3a931891c9048683517409f4cb` on `claude/leader-timing` (stacked; none of
  the stack is in integration yet)
- Owner: Claude Code, at the operator's request; the operator flashed both boards (2026-10-12)
- Problem / saved reproducer: [`LEADER_TIMING.md`](LEADER_TIMING.md). The boards read their seven XL330
  servos only when polled, 4–5 ms per read, 3.5 ms of it the servos' factory Return Delay Time (250 = 500 µs).
- Hypothesis: continuous reads with replies from the latest complete read cut a poll to the USB round trip,
  and Return Delay Time 0 cuts a read to the bus transfer.
- Owned files: `firmware/openrb_leader/src/main.cpp`, `docs/subsystems/OPENRB_LEADER.md`,
  `scripts/openrb_leader_setup.py`, `tests/unit/test_openrb_leader_setup.py`, `scripts/dev.py` (gate list).
- Shared interfaces held fixed: the 0xA6 frame layout and its host decoding (`teleop/openrb.py`), IDENT, the
  legacy 0xA5, SCAN, REID and CENTER, force feedback's 200 mA cap and two-stage watchdog, every host freshness
  bound.
- Acceptance: `python scripts/dev.py check`; on the boards, `openrb_leader_setup.py info`, zero failed reads idle
  and under Helm's sampler, and `scripts/probe_leader_timing.py`.

## Change

Firmware 2 reads continuously and answers a poll from the newest complete read. A poll is never answered twice
with the same sample (the host faults on a repeated sequence): it waits for the next read, and only after three
failed reads in a row (3 ms each at most) is it answered as incomplete, which the host skips. The frame's times
come from one microsecond clock, the read start rounded down and the reply up, and the reply time never steps
back. Force feedback is collected without waiting on USB and written to every active slot with one Sync Write
between reads. New `INFO` and crc-guarded `RETURN_DELAY` commands; `scripts/openrb_leader_setup.py` drives them.

With Return Delay Time 0 the seven replies arrive back to back, and the SAMD core runs USB at the highest
interrupt priority and the servo UART at the lowest: under polling a USB interrupt overran the receiver (left 26,
right 23 failed reads in ~40,000; idle 0 and 1 in 17,500), and one failure landed on a sampler's first sample,
which a session start treats as a failure. `335e11e` lets the UART interrupt preempt USB, shortens a lost reply's
wait from 10 to 3 ms and rides out a single failed read.

## Hardware changes (2026-10-12)

- Both OpenRB-150 boards (left COM7, serial `86AE…0E12`; right COM4, `D5C5…2A0C`) flashed by the operator with
  firmware 2 at `335e11e`. Rollback: build and upload `06afe86`'s `main.cpp`.
- All 14 leader servos (XL330 firmware 53): Return Delay Time 250 → 0, read back, via
  `openrb_leader_setup.py return-delay 0 --confirm`. Stored in the servos; reflashing does not undo it, and a
  replaced servo arrives at 250.

## Results

| | Before (`20261012T025512Z`, after `next_poll`) | Firmware 2, delay 0 (`20261012T032711Z`) |
|---|---|---|
| Servo read on the board | 4–5 ms | 1.7 ms (max 1.9) |
| Poll round trip, each board | 5.5 ms | 1.9 ms (p99 2.2, max 3.1) |
| Both boards together | 172 Hz | 523 Hz (max 3.3 ms) |
| New samples per board | 173/s, one per poll | 560/s (the board's own read rate) |
| Sample age at a 100 Hz consumer, p50 / p99 / max | 13.4 / 16.5 / 21.2 ms | 7.3 / 8.3 / 8.4 ms |
| Failed reads | — | 0 in 88,336 (left) and 81,882 (right) |

Helm's sampler still polls at 200 Hz; each poll now gets a sample at most one read old. The age is the host's
conservative bound (board age at reply plus the whole round trip).

- Gate: 685 Python tests, zero failures/skips/xfails, Rust included, on `335e11e`
  (`build/dev-check/20261012T032800Z-2ac398cf/report.json`); the firmware itself has no host tests.

## Remaining uncertainty

- Force feedback's new path (non-blocking parser, Sync Write, watchdog zeroing by Sync Write) built but was not
  exercised on hardware; nothing in Helm sends 0xCC yet.
- Not done: Fast Sync Read and a 2–3 Mbps bus (each servo's baud is another EEPROM write).
- The robot-side leader age is expected near half its 20 ms; the next live session's trace will show it.
