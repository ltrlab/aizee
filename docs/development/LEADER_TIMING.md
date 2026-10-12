# Feature task: leader input timing

- Branch and worktree: `claude/leader-timing`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `86c6af95cdb8aaa3325508457d587148dd52ab74` on `claude/tuning-speed-estimate` (stacked; none of the stack is in
  integration yet)
- Owner: Claude Code, at the operator's request
- Problem / saved reproducer: live session `~/minerva/evidence/live-20261012T024105Z` (2026-10-12) held after
  111 s of following: 11 leader requests in a row got no reply ("No leader reply within 4 ms"), while the leader
  sample counter kept advancing (398671 → 398682 over 120 ms), so the boards were fine and the PC → Jetson path
  went quiet. The same trace shows leader samples 20.0 ms old at use (p50; p99 25.6) against a 0.89 ms link
  round trip, and samples advancing only 0.91 per 10 ms cycle (~91 Hz) though Helm's sampler runs at 200 Hz.
- Hypotheses: (1) the sampler, not the boards, sets the rate; (2) nothing recorded on the PC whether a quiet
  link lost requests or delayed replies.
- Owned files: `scripts/probe_leader_timing.py`, `src/robot_platform/teleop/leader_bridge.py` (`next_poll`),
  `src/robot_platform/teleop/leader_link.py` (`LinkTiming`), `src/robot_platform/helm/edge.py` (`_save_link`),
  `src/robot_platform/helm/teleop.py` (`link_evidence`), their tests, `scripts/dev.py` (gate list).
- Shared interfaces held fixed: the OpenRB wire protocol and firmware, every freshness, skew and age bound and
  the age accounting (`decode_frame`), the 4 ms reply deadline and the 8-miss hold on the robot.
- Acceptance: `python scripts/dev.py check`; the probe on the real boards.

## Probe (read-only: IDENT and FRESH_POLL only)

`python scripts/probe_leader_timing.py --seconds 10`, with the leader-link process's scheduling (1 ms timer,
high priority, no throttling). Saved under `build/leader-timing/`.

| | Before (`20261012T025358Z`) | After `next_poll` (`20261012T025512Z`) |
|---|---|---|
| One board, back to back | 5.5 ms per poll, 180 Hz; servo read 4–5 ms of it, USB and host ~0.5 ms | same |
| Both boards together | 5.8 ms, 173 Hz | 5.8 ms, 172 Hz |
| Helm's sampler at "200 Hz" | **90 Hz**; age at a 100 Hz consumer p50 16.3, p99 22.0 ms | **173 Hz**; p50 13.4, p99 16.5 ms |

A pair read (5.8 ms) overran the 5 ms period, and after every overrun the sampler waited a whole period before
reading again: 5.8 + 5 ms per sample. `next_poll` polls again at once after an overrun. The reported age is a
conservative bound: it adds the on-board read time to the whole PC round trip, which already contains it, so
the real age is about 5 ms lower; that accounting is unchanged.

## Link timing

`LinkTiming` in the PC's leader-link server counts request gaps over 20, 50 and 100 ms and keeps the 20 largest
with the number of requests that then arrived within 5 ms of each other: a burst means the requests were held
up on the way in (or the PC process did not run), none means they were lost. It also keeps the reply
turnaround. Helm writes it to `<runs>/<asset>-teleop/leader-link/leader-link-<UTC>.json` as each Jetson session
ends. Times are the PC's own; the UTC stamp only places an event within a session.

## Results

- Gate: 680 Python tests (now including the leader link and edge relay tests), zero
  failures/skips/xfails, Rust included, on `8e0822a` (`build/dev-check/20261012T025825Z-0c97f8c9/report.json`).

## Next: leader firmware (not started; needs flashing and a servo setting)

Measured: the servo read is 4–5 ms of every 5.5 ms poll, and the board reads only when asked. At 1 Mbps a
7-servo Sync Read is 21 bytes out and 7 × 15 bytes back (1.3 ms), plus each XL330's Return Delay Time,
factory 500 µs, 3.5 ms in all. Nothing in the firmware or setup script changes it.

1. Free-running reads with instant replies: the board reads continuously and answers FRESH_POLL from the
   latest complete read, so a poll costs the USB round trip (~0.5 ms) and `decode_frame`'s accounting (board
   age at reply plus the round trip) stays exact for cached samples. Microsecond timestamps in a new frame
   header, since millisecond truncation can under-state a cached sample's age by up to 1 ms.
2. Return Delay Time 0 on the leader servos (one EEPROM write each, torque off): 4.8 → ~1.3 ms per read.
3. Bus at 2 or 3 Mbps (the SAMD21 UART's 16× sampling tops out at 3 Mbps; XL330 supports both), and Fast Sync
   Read (Dynamixel2Arduino 0.8.1 has it) if the servos' firmware supports it: ~0.2–0.5 ms per read.
4. Gripper force feedback kept and made cheap: today 0xCC writes each servo with a status reply and its
   handler spins up to 150 ms waiting for payload bytes, stalling reads. Instead: a non-blocking parser, one
   Sync Write of the gripper's goal current (no reply, ~0.1–0.2 ms) after each read, the 200 mA cap and the
   two-stage watchdog unchanged, and the mode switch only when force feedback is turned on or off (Operating
   Mode is EEPROM). The robot's gripper effort can ride on the Jetson's existing 100 Hz sample request.
5. The OpenRB-150's SAMD21 is a single Cortex-M0+; there is no second core. Its DMA controller can move the
   servo UART's bytes without per-byte interrupts if the CPU budget ever needs it; at ~1 ms of bus time per
   cycle it does not. A dual-core RP2040 board would need its own TTL transceiver and power wiring.

Expected leader age at use: ~3–5 ms instead of ~11 ms real (13.4 reported) after `next_poll`.
