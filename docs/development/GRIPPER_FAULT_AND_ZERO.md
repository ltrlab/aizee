# Feature task: the both-arm command expiry, a gripper's return squeeze, and Gripper zero

- Branches and worktree: `claude/loop-catch-up`, `claude/gripper-return-squeeze`, `claude/gripper-zero`, stacked
  on `claude/leader-usb-priority`, in `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Owner: Claude Code, at the operator's request (2026-10-13)
- Reproducer: live both-arm session `~/minerva/evidence/live-20261013T023835Z` faulted 10 s into teleop with
  "Receiver stopping: command_expired · Guarded controller rejected request: wrong_owner". The right gripper
  was holding an object; the return after the fault skipped its gear teeth.

## Root cause of the fault

- With both arms live, the loop alternated 3.7 and 6.2 ms cycles (the receiver call 1.2 and 3.6 ms). One cycle
  took 11.6 ms: the receiver call 4.1 ms and 4.4 ms more than usual in the Python loop (GC off). The trace
  cannot say what stalled; the receiver slowed in the same cycle, so the whole Jetson process likely did.
- An overrun skipped to the next 10 ms slot, so the next command went out about 20 ms after the last. A
  command's lifetime runs from the offer it uses (issued at the previous acceptance) and is bounded by its
  leader sample's age: the command accepted at 16500 ms expired at 16512 ms. The receiver stopped the arms
  (`command_expired`) and the loop's next request no longer owned them (`wrong_owner`). The Jetson
  qualification of `43ae080` failed the same way once (a 10.4 ms cycle).
- Fix (`e8b43a4`): after an overrun the next cycle starts at once, and the schedule follows on from there. The
  overrun still counts as skipped slots and a scheduled deadline miss. No freshness bound changed.

## The gear skip

The fault's return (Shutdown) drove the right gripper's command from 1.53 rad to its closed reference at
0.5 rad/s with no lead limit (a gripper's lead spans its range). Blocked by the object, the squeeze rose to
6.2 Nm and the motor turned 1.35 rad past the jaws, skipping teeth; it ended at 0.21 rad, squeezing 5.2 Nm.
Fix (`4467f69`): during Shutdown a gripper's command leads it by at most `RETURN_SQUEEZE_NM` (1 Nm) at its Kp,
and a gripper stopped with its command at that lead has arrived, so the arm disables with the gripper where
it stopped. A free gripper still closes to its reference; grip in teleop is unchanged.

## Gripper zero

Settings → Gripper zero (`ac730bb`, `2c2943f`, `5707d10`): per gripper, its live motor reading (closed should
read 0) and a Zero button with an in-page confirmation (close the jaws fully by hand, hold them still). The
server refuses while teleop, a servo sweep or Check runs or on Wi-Fi, stops the live reader and runs
`probe zero_gripper <gripper>` on the robot: the 2026-10-09 re-zero's stopped maintenance for that one drive
(every drive's identity and Reset verified, the drive still, zero and flash save with read-back, no enable,
no retry). The gripper zero is fully closed jaws (operator-confirmed, device log), so the swept range holds
again once the zero is back there. A power cycle confirms the drive kept it.

## Results

- Gate: 700 Python tests, zero failures/skips/xfails, Rust included, on `5707d10`
  (`build/dev-check/20261013T030023Z-be0d7663/report.json`).
- Jetson manual-tuning qualification, release `20261013T030603Z-5707d10`
  (`~/minerva/validation/fixes-5707d10-01/`): left and both arms complete, 13,012 following cycles each, all
  accepted, no powered misses, full loop p99 4.74 / 5.68 ms, max 5.61 / 6.22 ms.
- Deployed: release `20261013T031118Z-5707d10`, receiver binary identical to the qualified one; the robot's
  `zero_gripper` refuses anything but a gripper. The zero itself has not been run.

## Remaining uncertainty

- What stalls the Jetson loop for ~5 ms now and then is not identified; one stall no longer costs a slot.
- The return squeeze was tested offline (lifecycle tests); not yet on hardware with an object in a gripper.
