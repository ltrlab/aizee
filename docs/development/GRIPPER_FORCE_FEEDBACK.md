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
  - **Robot loop** (`teleop/grip_feedback.py`, `helm.teleop.TeleopCore._send_grip_feedback`): each cycle it reads the receiver's own feedback, which carries each drive's effort, from the previous exchange. It then works out a resist strength in mA, never negative (since 2026-10-24, see Results):
    - The squeeze is max(0, −effort), low-passed over 30 ms.
    - **A grip** starts once the gripper has stayed still (|v| ≤ 0.3 rad/s) with the squeeze past the grip threshold
      (0.5 Nm) for the hold time (100 ms). It lasts until the squeeze stays under 75 % of the threshold, or the
      gripper moves faster than 1 rad/s (it opened, or the object slipped), for 30 ms. Speed blips and squeeze dips
      inside a hold do not end it.
    - **The push while gripping:** the contact push (40 % of the maximum) at the threshold, rising with the squeeze
      to the maximum at the full-force squeeze (1.2 Nm). Under the threshold it fades out to nothing where the grip
      ends.
    - It is × the operator's strength (0–1), at most the operator's maximum force (default 150 mA, at most 200), and
      smoothed: it rises with half the smoothing time (40 ms) and falls with all of it. The hold time and smoothing
      run on the loop's own clock.
    - It is sent only in TELEOP and only when switched on; otherwise the request carries no strengths.
    - Until 2026-10-24 the law was a deadband and a gain, gated on speed: a 0.5 Nm deadband and 100 mA/Nm, then
      briefly 0.3 Nm and 400 mA/Nm (`4a90810`) after the user asked for much more sensitive feedback.
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
    - They arrive with the loop's regular view (about 10 Hz, at worst the 0.25 s status heartbeat). They are not part of
      the status key, so a squeeze never makes the loop publish extra views.
    - A readout gives each trigger's current while it pushes.
  - **Helm, Settings → Grippers:** every setting of the feel, one slider each, applied at once to a running session:
    - the maximum trigger force (0–200 mA) and the strength (0–100 %), which scales the whole push but not the bar;
    - the grip threshold (0.2–2 Nm) and the hold time (0–300 ms): lower or shorter feels contact sooner but pulses
      on a free close's stuck start;
    - the contact push (0–100 %), the full-force squeeze (0.5–4 Nm) and the smoothing (0–200 ms).
    - A live line shows each gripper's squeeze, whether it counts as holding, and its push. **Defaults** clears the
      saved settings.
    - Only the settings the operator changed are saved, in `<runs>/<asset>-teleop/grip-feedback.json`, so a
      release's new defaults reach the rest. They start every later session.
    - The commands are `POST /api/helm/teleop/grip_feedback` (`enabled`, `reset`, and any of `strength`, `cap_ma`,
      `grip_nm`, `full_nm`, `contact_push`, `hold_ms`, `smoothing_ms`) and the loop command `grip_feedback`.
    - The nav item formerly called "Gripper zero" is now "Grippers"; it holds both.
  - **Leader profile** (`config/devices/gello-dual-viewer.json`, the `leader_profile` of every teleop profile):
    `trigger: {channel: 7, open_sign: 1}` on both leaders.
    - The sign is a candidate: the gripper routes say the leader angle increasing opens the gripper, and a positive
      Dynamixel current turns toward increasing position.
    - A leader without `trigger` gets no feedback. The loop reads the triggers from the leader profile it runs with:
      it sends strengths only for those leaders, and Helm shows feedback as available only when there are some.
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
- **Results:** `tests/unit/test_grip_feedback.py` has 33 tests covering:
  - the frame layout and CRC against the firmware;
  - the trigger-only, open-only currents;
  - the law: the hold time, a free close's stuck start and the rest level never pushing, the contact push and full
    force, blips and dips inside a hold, release and slip, the closing sign, cap, strength and smoothing, the loop's
    own clock;
  - request validation;
  - sampler scheduling (refresh, change, staleness, off once);
  - write-before-read order and the close frame;
  - the loop reading effort from receiver feedback, only in TELEOP;
  - the live switch through the session loop;
  - strengths crossing the real signed UDP link;
  - the force bars' levels (shown with feedback off, unscaled by strength, full past a zero maximum);
  - the settings: bounds, live in the loop, only the changed ones saved, reloaded by the next session, saved with no
    session running, Defaults, and a slider for every setting within its bounds.
  - `tests/ui/test_helm_grip.mjs` (run by `tests/unit/test_browser_scripts.py`) checks the bar colours and the panel's words.
  - **Local Helm check** (`--no-live`, port 8792):
    - The panel renders the bars from an injected TELEOP state: 20 % green, 55 % yellow, 93 % and 100 % red.
    - The Settings slider saved 120 mA to the file, and a page reload showed 120 mA.
  - **Gate:** `scripts/dev.py check --offline` passed on `0f96a61`: 1189 tests, none skipped
    (`build/dev-check/20261021T022040Z-baeca8f5/report.json`).
  - **Deployed 2026-10-21 at about 02:44Z** with the user's approval ("yes, deploy it once the gate passes"), by the
    process in `build/terminal-deploy-20261017/`. The first attempt at 02:35Z found the Jetson offline: its UPS had run flat.
    - Jetson release `20261021T023352Z-0f96a61` (previous `20261020T014400Z-b503ee7`): staged, verified and activated, with
      both CAN links down, no receiver running and the disarmed smoke test passing.
    - Terminal console `helm-recording-0f96a6131d` (previous `helm-recording-12e98a307d`), then Helm refreshed.
    - Receiver restored to Live · both and the task set up again. The saved settings read strength 1.0 and 150 mA.
  - **First bench try, 2026-10-21 (session `live-20261021T033534Z`): the user felt nothing.**
    - The signal was there: in about 157 s of TELEOP the right gripper squeezed up to 4.3 Nm (p99 3.4 Nm). Replaying
      the law on the 100 Hz recorder gives the full 150 mA on the right trigger for about 62 s in all; the left reached
      96 mA briefly.
    - Cause: the `trigger` entries were in `minerva-leaders-01.json`, the bench scripts' identity file, which teleop
      never reads. On the terminal, the leader process had no triggers and dropped every strength; nothing was written
      to the boards.
    - Meanwhile the loop on the Jetson computed the strengths, and Helm showed the switch as usable.
    - The tests pinned the file that was edited, not the file teleop loads.
    - **Fix:**
      - The triggers moved to `gello-dual-viewer.json`, and `minerva-leaders-01.json` is back as it was.
      - A test resolves every `minerva-teleop*.json`'s `leader_profile` and requires both triggers. A contract test checks
        the real loop's `grip_triggers` after building with the shipped files.
      - The loop now reports feedback unavailable without triggers.
    - **Fix deployed 2026-10-21 at about 11:47Z** at the user's request ("stop the session and deploy it"). The gate
      passed on `3e7f18b` with 1221 tests and none skipped. The Jetson runs `20261021T035909Z-3e7f18b` and the terminal
      `helm-recording-3e7f18b365`. Both sides' `gello-dual-viewer.json` now name both triggers.
    - The firmware needed nothing: firmware 3 (both boards since 2026-10-18) has `FF_CURRENT` (added 2026-06, reworked
      2026-10-11; version 3 since 2026-10-14).
- **2026-10-24: the triggers pulsed, and felt too weak.**
  - The user, after the 0.3 Nm / 400 mA/Nm law (`4a90810`): "still is not as sensitive... It pulses a little bit,
    the tuning is not good."
  - **Evidence**, read-only on the Jetson from the loop's own 100 Hz log (`direct-latency.jsonl`) of five sessions.
    The replay script is not in the repository.
    - In that session (`live-20261024T025757Z`) nothing was held. The longest "squeezes" (0.9–1.4 Nm) were the right
      gripper closing in free air, with the trigger worked open and closed at about 1.3 Hz. The gripper was 1.0–1.9
      rad open and moving at 0.7–0.9 rad/s.
    - **Pulses:** a free close starts with the gripper stuck by friction for 30–100 ms, squeezing up to about 0.6–0.8 Nm
      before it breaks free. The speed gate let each start through: 72 short pushes in 102 s of TELEOP.
    - **Rest:** the gripper's own friction leaves 0.30–0.35 Nm of squeeze at rest for seconds, right on the 0.3 Nm
      deadband, so the trigger flickered there.
    - **Real holds were already clean:** a held cube squeezes 1.8–2.0 Nm with the gripper still (|v| p90 0.06–0.07
      rad/s, ripple 0.03–0.06 Nm), and the old law gave the full 150 mA through them. A hard squeeze reaches 3.7 Nm.
    - The reported speed matches the position's own derivative (p50 0.68 against 0.69 rad/s), so velocity noise
      was not the cause.
  - **Fix:** the grip law above, with live settings so tuning needs no deploy.
  - **Replay of the old and new laws** on the same five sessions. Pulses are pushes shorter than 300 ms:

    | Session · gripper | Old law: pulses/min | New law: pulses/min | Held cube (old → new) |
    |---|---|---|---|
    | `025757Z` right (free open/close) | 33.7 | 0 | none held |
    | `022456Z` left / right (slow closes) | 30.0 / 15.5 | 0.9 / 1.3 | light holds: 147 → 90 mA |
    | `014307Z` left / right | 13.0 / 1.1 | 0 / 0 | none held |
    | `20261021T004507Z` right (demo 1) | 4.6 | 0.4 | 150 → 150 mA; half force in 10 → 105 ms |
    | `20261021T033534Z` left / right (bench) | 9.2 / 10.3 | 0.4 / 1.5 | 150 → 147 mA; half force in 9 → 63 ms |

    - The new law never pushes at the rest level. The cost is the hold time: a real grip is felt about 60–100 ms later.
    - Hold time and threshold were chosen from these runs. A 50 ms hold left 10.6 pulses/min in `025757Z`; 80 ms left
      5.4/min in `022456Z` right; 120 ms felt holds 15 ms later than 100 ms for little gain. A 0.6 Nm threshold cut
      little more.
    - The remaining pulses are stick-slip on slow closes: the gripper sticks for 100–250 ms at 0.5–0.8 Nm, and the
      data cannot tell that apart from a light grasp.
  - **Gate:** `scripts/dev.py check --offline` passed on `30dffc4`: 1311 tests, none skipped
    (`build/dev-check/20261024T033943Z-e5a73a2f/report.json`). Before it, `dev.py quick` ran the touched tests in 13 s.
  - **Deployed 2026-10-24 at about 15:38Z** at the user's request ("stop the session and deploy it"):
    - The open session (`live-20261024T025757Z`, its arms disabled since 03:00:50Z) was stopped by
      `systemctl --user stop minerva-helm`. That SIGTERM reaches Helm alone (the loop and receiver run in their own SSH
      session), and Helm's shutdown runs the same `teleop.stop()` as its Stop button: 2.9 s, recording closed.
    - Jetson release `20261024T153657Z-30dffc4` (previous `20261024T024559Z-4a90810`) went through soliloquy: staged,
      verified, activated, disarmed smoke test passed, Helm restarted, the GELLO bridge reconnected.
    - Minerva has no saved grip settings, so the new defaults apply.
  - **Not yet felt:** the new law needs the user's bench trial, then tuning in Settings if it is off.
- **Remaining uncertainty:**
  - The grip law of 2026-10-24 is not yet felt on hardware: the bench check above, then the settings.
  - The trigger's direction (`open_sign`) is still to be confirmed on the bench.
  - The left gripper has never held anything in a recorded session, so its squeeze level is unmeasured.
  - Stick-slip on a slow close can still read as a short grip (about 1 a minute in the replays).
