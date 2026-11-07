# Safety: what stops what, and what does not

Read this before you power Minerva. It says what each stop does, what happens when something is lost, and what nothing
protects. Every statement comes from the code or a measured record; where a behaviour was only tested offline it says so.

**There is no independent hardware emergency stop that covers every actuator.** The stops are software (Helm's keys,
the receiver's trips and timeouts, the motors' own command-loss watchdog) plus one foot-operated power switch that cuts the
arm motors' supply and does not cut the head's 12 V supply. No document or config in this repository describes any other
stop circuit. Minerva's profiles are all `runtime_approved: false` (Helm refuses to load one that says otherwise), and
every live session is an **attended** test.

## Who and what must be present

- A person at Helm who can reach the foot switch, with the arms' path clear. Helm's Check tab says "Run it before each
  session, with the foot switch within reach", and the live sweeps are described as attended.
- Nobody inside the arms' reach. With the production profile nothing checks arm against arm, arm against table or arm
  against body (see [What is not protected](#what-is-not-protected)).
- A stand or cage that tolerates the arms dropping when torque goes. The project's own record says the test stand was built
  for that ([guarded controller](../architecture/GUARDED_CONTROLLER.md)); no document describes any other supported setup.
- Do not leave a session running unattended. Closing the browser does not stop it.

## Stop controls

| Control | What it does | Torque afterwards |
|---|---|---|
| **H**, Disable, or "Disable now" in the session bar | Removes torque at once, no braking trajectory. The arms go limp and drop. The next session must start from the beginning. | Off |
| **X**, Shutdown, or "Return & disable" | Returns slowly to the reference pose, then disables. The path must be clear: the return is a straight move in joint space and, with collision checking off, is not checked against anything. | Off at the end |
| **T**, Hold (Engage again to follow) | Stops following; the arms hold the measured pose with torque on. | On |
| Stop session | Lowers an enabled arm to the reference first (as X), then disables it and closes the receiver. Helm asks first. | Off at the end |
| Pedals A, B, C | Keyboard keys A, B and C. B pauses (Hold), C steps Idle, Enable, Engage, then Shutdown. They are not a stop circuit: see below. | As the action |
| Foot switch | Cuts the arm motors' power supply. | Off; the arms drop. Not the head's 12 V. |

Details that matter:

- **H asks first while the arms are under torque.** In a session past Idle, or a sweep, H (or a Disable button) opens "Disable
  the arms now?" and the second H, or its **Disable · arms drop** button, disables. Esc cancels; T or X still work. Disabled or
  Idle arms are disabled at once. Press H twice. If the browser misses a press, use the button, then the foot switch.
- **H and X always get through.** Any logged-in device may send H or X, even a device that is "View only", and they are not
  queued behind other actions (`stop_request` in `src/robot_platform/helm/access.py`).
- **Keys need a Helm page with focus.** H, X and the session bar work on every tab; the other keys need the Teleop or
  Settings tab. A page that is not focused, or a browser that has lost its connection, sends nothing.
- **If H cannot be confirmed** Helm says so ("Disable could not be confirmed ... Use the foot switch").
- **Pedals are not an emergency stop.** They are the keyboard keys A, B and C in the operator's browser. A pedal press is
  only accepted if Helm offered the action in the last 750 ms; a pedal depends on the focused page and its connection
  (`OFFER_S` in `src/robot_platform/helm/pedals.py`). C ends in Shutdown, which is a slow return, not a power cut.
- **The head.** H cuts the head's torque at once in any state. X returns it to its reference and then to where it rested
  (never dropped from straight ahead), then disables it. A servo holds its last goal and the head has **no command
  watchdog**: if its receiver stops, it keeps its pose with torque on ([neck](../robots/minerva/NECK_FEETECH.md)).
- **The foot switch** is the only way to remove motor power without software. After one press with the motors on, the
  right gripper reported overcurrent and right J1 left Run mode, most likely from the power cut itself. Helm then faults
  the session; stop it and run Minerva Check before the next start. Pressing it does not stop the head. The device log
  calls the switch a temporary emergency stop, and Check treats "no motor answers" as motor power off or the switch
  pressed. Its wiring is not documented in this repository.

## What happens when something is lost

| Lost | What happens | Where it is done |
|---|---|---|
| A leader's samples (unplugged leader, bridge, PC or link) | After about 80 ms (eight cycles at 100 Hz) following stops and the arms **hold** the measured pose, torque on: "Leader input lost; holding measured pose until Engage". Press T to follow again. | `TeleopLifecycle` in `src/robot_platform/teleop/lifecycle.py`; `source_misses` in `helm/teleop.py` |
| The operator's browser (tab closed, device asleep, Wi-Fi gone) | **Nothing stops.** The session runs in a loop process on the Jetson that does not depend on the page. Another logged-in device can press H or X. | `run_loop` and `edge.BEAT_S` beat from Helm, not the browser |
| Helm itself (service stopped, restarted, crashed) | The loop ends the session as if Helm closed it: an enabled arm is lowered to the reference, then disabled. A silent Helm is detected after 5 s. A deploy does not restart Helm while a session runs. | `helm/edge.py` (`SILENCE_S`), `TeleopCore.land` |
| The loop (it stops sending commands) | The receiver's command and lease timers expire (`command_ms` 30 ms, `lease_ms` 150 ms in the production profile) and it brakes the arm to a **hold**. | `Controller::tick_inner` in `rust/control_core/src/lib.rs` |
| The receiver process | Each motor's own watchdog (register 0x7028, written as 600 and read back before enabling) disables the motor when commands stop. The arm goes limp. Tested after 150 ms of silence, one motor at a time, no load. | `rust/control_core/src/watchdog.rs`; [powered commissioning](POWERED_COMMISSIONING.md) |
| CAN adapter or one drive | One drive silent for 30 ms: the receiver holds the arm on the others. Every drive silent, invalid feedback, a drive fault, or an adapter error: **torque is cut** and a raised arm drops. A drive that leaves Run mode mid-move is treated as silent and never re-enabled; Helm tells you to support the arm and press H. | `stale_error`, `feedback_error`, `fault` in `lib.rs`; `can_adapter.rs` |
| A late loop cycle in the receiver service (over `maximum_lateness_us`, 10 ms in the production profile) or a malformed frame | An emergency stop inside the receiver: **torque is cut**. | `rust/control_core/src/service.rs` |
| Power to the Jetson (UPS low battery) | See the UPS section below. | `src/robot_platform/ops/ups.py` |
| Motor power (supply or foot switch) | The arms drop. No brake is described anywhere in the docs; right J2 fell from horizontal when a trip cut its torque (2026-09-29). | [Device log](../devices/MINERVA_PRODUCTION_01.md) |
| A camera on USB | The arms pause (below). | `helm/camera_watch.py`, `helm/teleop.py` |

### Trips

A trip is the receiver stopping itself. In the running release a trip while the arm is held brakes the arm to a hold that
keeps its gravity feed-forward (it does not cut torque), revokes the session's lease, and Helm then takes the arm back
where it is held, **lowers it to the reference and disables it**. The reason stays on screen
([Troubleshooting](TROUBLESHOOTING.md#reading-a-trip)). The checks that trip it, from `limit_error`, the tracking check
and the collision box in `rust/control_core/src/lib.rs`: a joint past a limit, over its allowed speed, effort or
temperature, further from its command than its tracking bound, or outside its collision box. Grippers are exempt from the
tracking check. The collision box is not active in the production profile.

Torque is cut at once, not held, for: invalid feedback, every drive silent, a drive's own fault, an adapter failure, an
emergency stop, or the receiver's own loop running late.

**Trip hold (merged, not deployed).** The merged code (`_trip_hold_due` in `src/robot_platform/helm/teleop.py`) holds the arms
after a `sustained_speed` trip instead of lowering them, so a joint that lurches is not followed by a long return. T
follows again. Three such pauses within a minute, counting camera pauses, are lowered as any other fault. The release on the
robot (`20261105T040341Z-fc68d9d`) predates it and still lowers after every trip.

**Camera drop pause (in the running release).** If a camera drops off USB, the arms stop following and hold. A late-cycle trip
within 10 s of the drop also holds rather than lowers. Engage (T) follows again. This is the same limit of three pauses in a
minute (`CAMERA_PAUSE_S`, `PAUSE_ATTEMPTS`, `PAUSE_WINDOW_S`).

### UPS low battery

Minerva's UPS is a battery under the Jetson, read by the `minerva-ups` service. The docs do not say it backs up the motors'
supply. On battery at 10 % charge (`shutdown_percent`), or at 3.35 V per cell under load, the service announces a shutdown;
Helm stops any recording and then the session through its own stop path, which lowers an enabled arm first. After 60 s
(`grace_s`) any loop still running is sent SIGTERM (which also lowers the arm), and the Jetson powers off at most 30 s later
(`terminate_s`). The charger coming back cancels it. While it is announced, Helm refuses to start anything that moves the
arms. The header shows a battery pill and a banner on battery. This has been tested offline and by dry run
([UPS](../robots/minerva/UPS_MONITOR.md)); no record shows a low-battery shutdown with the arms engaged.

If the Jetson loses power outright, the receiver and the loop stop with it; the motors' own watchdog then disables them.

### Retired: the ESP-NOW stop bridge

An ESP-NOW stop bridge once existed. It was retired in September 2026 and its firmware, service and rules are gone from the
repository. The docs and the config no longer refer to it as a stop. Any older page that mentions a wireless stop is out of
date.

## What is not protected

- **Collision checking is off.** The production profile `config/devices/minerva-teleop-direct-tuning.json` sets
  `collision_mode: "disabled"` at 100 Hz. Helm hides the "Enforced" receiver choice under it. Nothing stops an arm from hitting the other arm,
  the table, the bucket, the body or the head. The `Enforced` receiver is a virtual rehearsal only.
- **There is no arm-to-body check** in any profile, and none between an arm and the head: the decision recorded for the head
  is that it needs no collision check ([neck](../robots/minerva/NECK_FEETECH.md)).
- **The settings are unqualified.** Gains, speeds, braking and tracking bounds come from Tune sweeps and manual tuning, and
  loaded stopping, host crash and CAN loss responses are attended-test matters
  ([guarded teleop](../architecture/GUARDED_TELEOP.md)). A software disable is a best-effort CAN frame, not isolation; a
  host crash cannot guarantee delivery of one.
- **Gripper force.** A grip is bounded only by the gripper motor's effort limit in the profile (14 Nm) and its gain: a stalled
  grasp holds about the hold gain times the command's lead, up to roughly 10 Nm with the current gains (2 Nm/rad over a 5.1 rad
  lead). Grippers are exempt from the tracking trip and do not release on a stop. A long grasp heats the motor toward the
  70 C limit. A hold after a recording failure may remove a grip squeeze; object retention is not qualified.
- **Head limits.** The head follows the headset inside its saved limits and a saved shoulder-clearance floor; live sessions
  take the head only once that clearance is saved. It follows at most 0.7 rad/s. Beyond that nothing protects the head, and
  its servos' own overheat alarm is the last guard.
- **The Quest.** With the Headset input, a held controller grip drags that arm's gripper, so the arm moves wherever the
  hand goes within the limits. B or Y holds both arms. **No controller button disables (H).** Engage raises a hanging arm to
  a ready pose with the forearm swinging forward, so keep the space ahead of the hands clear. A headset session steered the
  arms live on 2026-11-03 and a joint-speed trip occurred (left J4 lurched into its lower end); no document records a
  completed VR arm session.
- **Lift and base.** The `lift` joint is in the robot definition but inactive: no receiver, no motion. There is no powered base.
- **Policy and Replay.** Both drive the arms through the same lifecycle, limits and stops as the leaders do, and nothing
  more. Replay at most 1x; policy mode runs only a trained checkpoint or `hold_still`. [Status](../STATUS.md) says how far each has run
  on the robot.

## Before every session

1. Motor power on, the foot switch released and in reach, the arms hanging at the reference pose, the path clear.
2. Helm's Check tab: run it and read the verdict ([Helm manual](HELM_MANUAL.md#check)).
3. Know which device is in control and keep a second device logged in for H and X if you can.
4. Start with Idle, then Enable, then Engage. At any doubt press T. Press H only knowing the arms will drop.
