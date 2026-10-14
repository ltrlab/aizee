# Feature task: the left leader board stops answering USB (firmware 3)

- Branch and worktree: `claude/leader-usb-watchdog`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`, on
  `claude/velocity-feedforward` (stacked; none of the stack is in integration yet)
- Owner: Claude Code, at the operator's request (2026-10-14): "look at the firmware before I do disassembly"
- Flashing stays the operator's.

## Symptoms (2026-10-14)

- Sessions failed to start: the leader link could not open the left board (COM7), then every retry said
  "Access is denied"; replugging USB did not help, power cycles did only until the next close.
- Logged test after a power cycle, left board on the PC's root port: 20 s of continuous 100 Hz polls perfect
  (96–98 samples/s, 0 skipped, every reply ≤ 3.3 ms); the port closed normally; reopening it 2 s later hung
  inside Windows. A hung open leaves the port held, hence "Access is denied" afterwards.
- The same test on the right board (COM4, behind a USB 2.0 hub): three close/reopen cycles and an idle spell,
  every open < 10 ms, every read clean.
- Left board moved to that hub, after a power cycle: Windows reported "Device Descriptor Request Failed".
- Windows reports no USB power saving on either board; both run firmware 2 (`335e11e`).

## Reading of the firmware and the core

No unbounded wait in our code, and every reply buffer is in bounds (poll 50 bytes, INFO 48, return delay 24,
scan clamped). The board's Arduino core (`openrb150_bsp`, ArduinoCore-samd 1.8.x) handles each USB setup request
inside the USB interrupt: it busy-waits on endpoint 0 (`setAddress`, `recvControl`) and clears endpoint 0's
pending flags when it leaves. Firmware 2 put the servo UART (SERCOM2) above USB, so through every continuous
Sync Read (a reply byte every 10 µs) that handler is interrupted and stretched; a request's next stage can
then be cleared unseen and never complete. That stops USB setup requests only (opening a port, enumeration),
not polling, and only by chance. It matches all of the above, and the left board's first failures came the
day firmware 2 went on (2026-10-12). Then a bench supply was credited and the USB-first build (`b6283c2`) was
reverted before it was ever flashed. The right board runs the same firmware, so the trigger is a matter of
timing; nothing here proves it.

## Firmware 3

- USB above the servo UART again (`b6283c2`'s priorities). Cost: 0.02–0.07% of reads overrun under polling;
  a failed read costs 3 ms and the firmware rides it out.
- A watchdog: a loop that stops for about a second (a wedged USB request stops it too) resets the board,
  which reconnects over USB instead of waiting for a power cycle. Fed every pass and in CENTER's wait.
- `0x57` RESET_CAUSE (older boards ignore it): `scripts/openrb_leader_setup.py info` prints why each board last
  reset. A brown-out means the supply (disassembly); a watchdog, that it locked up and recovered.
- Builds: 28,272 bytes flash, 4,996 RAM. Frame layouts and Helm unchanged.

## How to tell

Flash both boards, then `timeout 60 python -u scripts/probe_leader_reopen.py left` (and `right`), or start and
stop Helm sessions across Helm restarts, and `python scripts/openrb_leader_setup.py info`. Fixed: every reopen
answers and the last reset stays power-on. Locking up but recovering: `watchdog`. Power: `brown-out`.
