# Gripper force feedback in the leader triggers

While a robot gripper holds something, the operator's leader trigger meets a wall where the object stopped the jaws,
so the operator can feel the grasp. The robot decides when and where the wall is; the operator PC renders it as a
spring from its own trigger readings. It is off by default, works only in Teleop, and can only push a trigger open.

The leader boards' firmware command is `FF_CURRENT` (per-servo goal current, 200 mA at most, with a 200 ms zeroing and
1 s disabling watchdog); see [OPENRB_LEADER.md](../OPENRB_LEADER.md#force-feedback-ff_current).

## The signal

Evidence from the 400 Hz recorder of live sessions, read-only on the Jetson:

- **Squeeze:** a held cube squeezes the right gripper at a steady 1.1 to 2.2 Nm in the sessions seen, with the gripper
  still and its command about 0.08 rad past its position. A hard squeeze reached 3.7 to 4.3 Nm.
- **Free air:** |effort| p95 is 0.17 to 0.33 Nm; fast moves spike briefly to 0.8 Nm; the gripper's own friction leaves
  0.30 to 0.35 Nm of squeeze at rest for seconds.
- **Sign:** both grippers' effort tracks the controller's `kp * error - kd * velocity`. Closing reads negative, opening
  positive, and both rest at about 0 rad, which is closed. So squeeze torque is minus the effort on both. The left
  gripper has never held an object in a recorded session, so its squeeze level is unmeasured.
- The older collectors' law (a 0.02 Nm deadband and 400 mA/Nm) would feed back free-air noise and saturate on every
  grasp.

## Design

- **Robot loop** (`teleop/grip_feedback.py`, `helm.teleop.TeleopCore._send_grip_feedback`): each cycle it reads the
  receiver's own feedback, which carries each drive's effort and position, from the previous exchange.
  - The squeeze is max(0, -effort), low-passed over 30 ms.
  - **A grip** starts once the gripper has stayed still (|v| at most 0.3 rad/s) with the squeeze past the grip
    threshold (0.5 Nm) for the hold time (70 ms). It ends once the squeeze has stayed under 0.15 Nm for 250 ms (the
    operator let go), or the gripper moves faster than 1 rad/s for 30 ms (it opened, or the object slipped). Squeeze
    dips and speed blips inside a hold do not end it.
  - **The wall** is the trigger reading the gripper's leader route maps to the gripper's measured position
    (`teleop.absolute_mapping.reading_for`, the route's inverse, extended straight past its ends). It moves with the
    jaws each cycle, so a soft object gives way.
  - A leader whose route and trigger disagree on which way opens gets no wall: on the wrong side it would push the
    trigger open at once.
  - Each request carries `{'wall': reading (rad), 'k': mA per rad, 'cap': mA}` for a gripping leader, `0` for a free
    one, or nothing when feedback is off. `k` puts the maximum force at `wall_deg` past the wall (default 0.5 degrees);
    both are scaled by the operator's strength.
- **Leader link** (`teleop/leader_bridge.py`): a sample request may carry `grip: {leader: wall or mA}`. A plain mA
  value is a steady push, kept for an older loop. A malformed value turns feedback off and never faults teleop. A
  request without the field turns feedback off.
- **Operator PC** (`LatestLeaderInputs`, `teleop/openrb.py`): the sampler thread, the only writer to the boards, builds
  each board's `FF_CURRENT` frame and sends it just before that board's read.
  - **The spring:** `k` times how far the trigger's latest reading (a poll old, at 200 Hz) is squeezed past the wall,
    at most the cap, toward opening only. A new wall's push rises over 60 ms, so a trigger already squeezed past it is
    not kicked. An unusable reading gives no push, never an exception: one would stop the sampler and with it teleop.
  - **What it energises:** only the trigger servo the leader profile names, in its `open_sign` direction. The other six
    slots are always `INT16_MIN` (torque off).
  - **When it sends:** an active trigger is refreshed every 20 ms, or on a change of 2 mA or more. Feedback older than
    100 ms switches all slots off once; closing a board that received feedback sends an all-off frame.
- **Leader profile** (`config/devices/gello-dual-viewer.json`, the `leader_profile` of every teleop profile):
  `trigger: {channel: 7, open_sign: 1}` on both leaders. The sign is confirmed by feel: the trigger is pushed open. A
  leader without `trigger` gets no feedback, and Helm shows feedback as available only when there are some. (An earlier
  version put the triggers in the bench identity file `minerva-leaders-01.json`, which teleop never reads, so nothing
  reached the boards while Helm showed the switch as usable. A test now resolves every teleop profile's leader profile
  and requires both triggers.)

## In Helm

- **Teleop panel** (under the pedals): a live switch, off at every session start, and a force bar per gripper. Each
  bar shows the push the loop estimates the trigger feels, against the maximum at full strength, coloured from green
  through yellow to red (`static/grip.js`). The bars move even with feedback off, so the operator can watch the grip
  before switching it on. They arrive with the loop's regular view (about 10 Hz) and add no extra views.
- **Settings, Grippers:** every setting of the feel, one slider each, applied at once to a running session:
  - the maximum trigger force (0 to 200 mA) and the strength (0 to 100 %), which scales the whole push but not the bar;
  - "Full force at" (0.2 to 5 degrees past the wall, default 0.5): fewer degrees make a harder wall. If the trigger
    buzzes at the wall, add some;
  - the grip threshold (0.2 to 2 Nm) and the hold time (0 to 300 ms): lower or shorter finds contact sooner but can
    put a brief wall in a free close's stuck start.
  - A live line shows each gripper's squeeze, whether it counts as holding, and its push. **Defaults** clears the saved
    settings. Only the settings the operator changed are saved, in `<runs>/<asset>-teleop/grip-feedback.json`, so a
    release's new defaults reach the rest.
  - The commands are `POST /api/helm/teleop/grip_feedback` (`enabled`, `reset`, and any of `strength`, `cap_ma`,
    `grip_nm`, `hold_ms`, `wall_deg`) and the loop command `grip_feedback`.
- Defaults in code (`DEFAULTS`): maximum 200 mA, grip threshold 0.5 Nm, hold 70 ms, wall 0.5 degrees, release 0.15 Nm
  for 250 ms, still 0.3 rad/s, slip 1 rad/s.

## Safety

- Feedback can only push a trigger open, and only one servo per board is energised.
- The current is capped at the operator's maximum by the loop, and at 200 mA by the settings check, the PC and the
  firmware.
- It is off by default, off outside Teleop, and off when the session or link ends. The firmware watchdog backs that up.
- The leader rail runs on its 5 V supply. To stop it from the robot's side, `systemctl --user stop minerva-helm`
  ends Helm and its session the same way the Stop button does.

## Why a wall, and not a push

An earlier law pushed the trigger open in proportion to the robot's squeeze. It felt weak and pulsed, for two reasons
found in the loop's own 100 Hz log:

- **Pulses from stick-slip:** a free close starts with the gripper stuck by friction for 30 to 100 ms, squeezing up to
  0.6 to 0.8 Nm before it breaks free, and a speed gate alone let each start through (72 short pushes in 102 s of
  Teleop). The grip law above, with its hold time and threshold, never pushes at the rest level. The cost is that a
  real grip is felt 60 to 100 ms later. A 50 ms hold left 10.6 pulses a minute in the worst session; 120 ms felt holds
  15 ms later than 100 ms for little gain.
- **A loop through the operator's finger:** the trigger is both the force display and the gripper's position command.
  A push that travels the robot's round trip opens the trigger, so the gripper loosens, the squeeze falls, the grip
  ends, and the finger closes again, repeating about every 0.45 s. The loop's gain is about the push slope times the
  gripper's 25 Nm/rad over the finger's stiffness, and any push the operator can feel puts it above 1, so no setting
  removes the pulse. A wall pushes nothing where the object stopped the jaws, so it cannot open the grasp, and the PC
  renders its spring a poll from the trigger instead of a round trip away.

Scale, from the saved leader ranges: one degree of trigger is about 0.10 rad of jaw, about 2.5 Nm of squeeze against an
object at the grippers' Kp of 25, and a held cube (about 2 Nm) sits about 0.8 degrees past the wall. So the defaults are
stiff: the maximum at 0.5 degrees, a light hold of 0.4 to 0.5 Nm gives a few tens of mA, and one encoder count of
trigger is about 35 mA.

## What is and is not known

- The trigger pushes open on the robot (`open_sign` +1 confirmed), and the wall law at a 0.8 degree maximum felt good
  to the operator, who asked for more sensitivity; the current defaults are the more sensitive ones. They have not been
  felt on the robot.
- Whether 0.5 degrees at 200 mA stays stable at the trigger (no buzz at the wall) is unknown: the PC's loop is a
  200 Hz poll.
- The left gripper's squeeze level is unmeasured.
- Stick-slip on a slow close can still read as a short grip, about once a minute in the replays.

## Tests

`tests/unit/test_grip_feedback.py` covers the frame layout and CRC against the firmware, the trigger-only and open-only
currents, the law (hold time, a free close's stuck start, the rest level, blips and dips, release and slip, the closing
sign, cap and strength), the wall from the real route inverse (and none for a route and trigger that disagree), the
PC's spring, request validation, sampler scheduling, the live switch, strengths crossing the real signed UDP link, the
force bars' levels, and the settings (bounds, live in the loop, only changed ones saved, Defaults).
`tests/ui/test_helm_grip.mjs` checks the bar colours and the panel's words. All run in the gate.
