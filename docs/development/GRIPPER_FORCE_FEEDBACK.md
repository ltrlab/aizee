# Feature task: gripper force feedback in the leader triggers

- **Branch and worktree:** `claude/gripper-force-feedback` in `C:\Users\ltr\Workspace\aizee-helm-pedals`.
- **Base commit:** `e2b6066b55813132efc36c0b642fa1e91c179320` (`claude/helm-replay`, the deployed terminal release `12e98a3` plus its record).
- **Owner:** Claude (the 2026-10-20 session), at the user's request: "check the gripper torque then build force feedback".
  The maximum force setting and force bars followed: "a force max that it caps at that is configurable in the settings"
  and "a force bar that changes color from green to red like the wii sports golf swing bar".
- **Problem:**
  - The operator cannot feel the robot's grasp.
  - The leader boards' firmware already has `FF_CURRENT` (per-servo goal current, ±200 mA, with a 200 ms zeroing and 1 s disabling watchdog).
  - The older collectors (`python/scripts/collect_demo.py`, `collect_minerva.py`) used it for the gripper trigger.
  - Helm's guarded teleop path had no feedback: the guarded leader driver was read-only, and the leader link carried samples only.
- **Evidence for the signal** (2026-10-20, read-only on the Jetson from the 400 Hz recorder of live sessions):
  - **Squeeze:** a held cube squeezes the right gripper at a steady −1.9 to −2.2 Nm (demo 1, `live-20261021T004507Z`, about 152–156 s and 204–208 s). In `live-20261019T010602Z` it squeezes at −1.1 to −1.5 Nm. The gripper is still, and its command sits about 0.08 rad past its position.
  - **Free air:** |effort| p95 is 0.17–0.33 Nm. Fast moves spike briefly to 0.8 Nm.
  - **Sign convention:** both grippers' effort tracks the controller's kp·err − kd·vel (correlation +0.5 to +0.75). Closing moves read negative and opening moves positive, and both rest at about 0 rad, which is closed. So squeeze torque is −effort on both. The left gripper's squeeze level was not observed: none of its sessions held an object.
  - **The old collectors' law does not fit:** a 0.02 Nm deadband and 400 mA/Nm would feed back free-air noise and saturate on every grasp.
- **Design:**
  - **Robot loop** (`teleop/grip_feedback.py`, `helm.teleop.TeleopCore._send_grip_feedback`): each cycle it reads the receiver's own feedback, which carries each drive's effort, from the previous exchange. It then works out a resist strength in mA, never negative:
    - squeeze torque = max(0, −effort) while |velocity| ≤ 0.3 rad/s;
    - minus a 0.5 Nm deadband, × 100 mA/Nm × the operator's strength (0–1);
    - capped at the operator's maximum force (default 150 mA, at most 200), then smoothed (α 0.4).
    - It is sent only in TELEOP and only when switched on; otherwise the request carries no strengths.
  - **Leader link** (`teleop/leader_bridge.py`): a sample request may carry `grip: {leader: mA}`.
    - A malformed value turns feedback off and never faults teleop.
    - A request without the field turns feedback off.
  - **Operator PC** (`LatestLeaderInputs`, `teleop/openrb.py`): the sampler thread, the only writer to the boards, builds each board's `FF_CURRENT` frame and sends it just before that board's read.
    - **What it energises:** only the trigger servo the leader profile names, in its `open_sign` direction. The other six slots are always `INT16_MIN` (torque off).
    - **When it sends:** an active trigger is refreshed every 20 ms, or on a change of 2 mA or more.
    - **When it turns off:** strengths older than 100 ms switch all slots off once. Closing a board that received feedback sends an all-off frame.
  - **Helm, Teleop panel** (under the pedals): a live switch, off at every session start, and a force bar per gripper.
    - Each bar shows the gripper's squeeze against the maximum force at full strength: (squeeze − deadband) × gain ÷ maximum.
    - Its colour runs from green through yellow to red at the maximum (`static/grip.js`, hue 120 → 0).
    - The bars move even with feedback off, so the operator can watch the grip before switching it on.
    - A readout gives each trigger's current while it pushes.
  - **Helm, Settings → Grippers:** the maximum trigger force (0–200 mA, steps of 10) and the strength (0–100 %).
    - Strength scales only what the trigger pushes, not the bar.
    - Both are saved in `<runs>/<asset>-teleop/grip-feedback.json`, start every later session and apply at once to a running one.
    - The commands are `POST /api/helm/teleop/grip_feedback` (`enabled`, `strength`, `cap_ma`) and the loop command `grip_feedback`.
    - The nav item formerly called "Gripper zero" is now "Grippers"; it holds both.
  - **Leader profile** (`config/devices/minerva-leaders-01.json`): `trigger: {channel: 7, open_sign: 1}` on both leaders. The sign is a candidate: the gripper routes say the leader angle increasing opens the gripper, and a positive Dynamixel current turns toward increasing position. A leader without `trigger` gets no feedback.
- **Safety:**
  - Feedback can only push a trigger open, and only one servo per board is energised.
  - The current is capped at the operator's maximum (default 150 mA) by the loop, and at 200 mA by the settings check, the PC and the firmware.
  - It is off by default, off outside TELEOP, and off when the session or link ends. The firmware watchdog backs that up.
  - The leader rail runs on its 5 V supply.
- **Shared interfaces held fixed:**
  - the robot's commands, lifecycle and receiver;
  - the leader sample format and freshness checks;
  - the leader link's authentication;
  - the firmware.
- **Acceptance:**
  - `python scripts/dev.py check --offline`, zero skips.
  - Bench, with the user holding the leader:
    1. Switched on in TELEOP with nothing grasped, the trigger stays free.
    2. Closing the robot gripper on an object makes the trigger push open, with the readout rising.
    3. Releasing the object frees it again.
    4. Switching it off, holding (T) or ending the session frees the trigger.
    5. If the trigger pushes closed instead, `open_sign` flips for that leader.
- **Results:** `tests/unit/test_grip_feedback.py` has 24 tests covering:
  - the frame layout and CRC against the firmware;
  - the trigger-only, open-only currents;
  - the law (deadband, hold gate, closing sign, cap, strength, smoothing);
  - request validation;
  - sampler scheduling (refresh, change, staleness, off once);
  - write-before-read order and the close frame;
  - the loop reading effort from receiver feedback, only in TELEOP;
  - the live switch through the session loop;
  - strengths crossing the real signed UDP link;
  - the force bars' levels (shown with feedback off, unscaled by strength, full past a zero maximum);
  - the maximum force and strength: bounds, live in the loop, saved and reloaded by the next session, and saved with no session running.
  - `tests/ui/test_helm_grip.mjs` (run by `tests/unit/test_browser_scripts.py`) checks the bar colours and the panel's words.
  - **Local Helm check** (`--no-live`, port 8792):
    - The panel renders the bars from an injected TELEOP state: 20 % green, 55 % yellow, 93 % and 100 % red.
    - The Settings slider saved 120 mA to the file, and a page reload showed 120 mA.
- **Remaining uncertainty:**
  - Not yet felt on hardware: the bench check above.
  - The left gripper's squeeze level, and whether the defaults feel right, are to be tuned on the bench.
