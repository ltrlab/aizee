# Helm manual

Helm is Minerva's operator console: a web page served by the Jetson. It checks the robot, calibrates it, drives the arms
and head with the leaders or a headset, records episodes and plays them back. Read [Safety](SAFETY.md) first.

Helm runs on the Jetson as the user service `minerva-helm` (`helm --on-robot`). Any logged-in device can open it; one device
at a time is in control. The leader arms are on the operator station, which joins Helm through the **GELLO bridge**. How the
service, the bridge and the password are installed is in [Install the Jetson](../getting-started/INSTALL_JETSON.md) and
[Install the operator station](../getting-started/INSTALL_OPERATOR_STATION.md); the design is in
[Helm on the Jetson](../architecture/HELM_ON_JETSON.md). To try Helm without a robot, see
[Simulate](../getting-started/SIMULATE.md) (`helm --sim`).

## A session, start to finish

**Start**

1. Power the motors (foot switch released and in reach) and the Jetson. The arms hang straight down in their reference pose,
   grippers closed, path clear. The head is limp until a session drives it.
2. The GELLO bridge on the operator station starts with the station. Plug in both leaders. Pedals, if used, are keyboard keys.
3. Open `https://<robot-address>:8760` on any device, log in, and press **Take control** (a second press confirms if another device holds it). The header should show **In control**,
   **GELLO** and the battery.
4. Press **Connect**. Helm reads the arms with the motors disabled and the pill shows `Live · frame N · X ms`.
   (If Helm's service was installed with `--live` it connects by itself.)
5. **Check** tab: **Run check**. Do not continue unless the stopped stages pass; run the live sweeps as the check asks.
6. **Teleop** tab: choose **Live** and **Both** (or **Left**/**Right**), **Mode** and **Input**, then **Start session**. Confirm the
   dialog. Wait for the session to be ready.
7. **I** (Idle: motors on, limp), **E** (Enable: hold), **T** (Engage: align, then follow). Hands off the arms from Start
   until Idle.

**Stop**

1. **T** to hold. **X** to return to the reference and disable; wait until the Phase reads Off/Disabled.
2. Stop recording first if one is running, and wait for Complete.
3. **Stop session**, then **Disconnect** if you want the arms' reader off.
4. Switch the motors' power off. Helm keeps running; there is nothing to shut down on the Jetson for a normal day. (The docs do
   not give a Jetson power-off procedure; a UPS low-battery shutdown is automatic and described in [Safety](SAFETY.md).)

## Opening Helm

- **Address:** `https://<robot-address>:8760` on the tether (`10.42.0.1`), the robot's wired address, its Wi-Fi address or its
  access point. Settings → Network lists them. On the robot itself, plain `http://127.0.0.1:8762` serves the same page for its
  own tools and SSH tunnels.
- **Certificate:** self-signed, made on the robot. Each device accepts it once. Compare the fingerprint the browser shows
  with `helm_service status`.
- **Login:** one password, set by the operator at install time. Wrong passwords back off up to a minute. A session lasts
  12 hours from its last use and a Helm restart (a deploy) logs every device out.
- **Control:** only the device in control starts, moves or changes anything. Others see **View only · Take control**; pressing
  it twice (it asks "Take control from ...?") takes control. Any logged-in device can always press **H** or **X**.
- **Touch consoles:** Settings → Display & touch has *Touch mode* (larger controls), *Look* (RetroUI or Original) and
  *Day and night*, saved in that browser.

## The header

| Item | Meaning |
|---|---|
| Tabs | Check, Limits, Leaders, Teleop, Replay, Tune, Settings |
| **In control** / **View only · Take control** | Whether this device may change anything |
| **GELLO** (green) / **GELLO · Wi-Fi** (amber) / **No GELLO** | The bridge from the operator station. Over Wi-Fi leader teleop runs but holds when samples arrive late; use a cable. Replay, policies and Check need no bridge. |
| Network pill | `Wired`, `Tether`, `Wi-Fi`, `Tether + AP`, `AP`, `No address` (the Ethernet cable is on a network whose DHCP did not answer: replug it) |
| Battery pill | The UPS: charge, or the seconds to power-off during a shutdown. A banner appears on battery. |
| Feed pill | `Live · frame N · X ms`, `Live feed stale`, `Connecting…`, `Reconnecting…`, `Waiting for motors`, `Connection failed`, `Not connected`, or `Motors in use: <owner>` |
| **Connect** / **Disconnect** | The stopped-motor reader that feeds the 3D view |
| Moon/sun, **Touch mode** | Day and night, larger controls |

The left of the page is the live view: **Cameras** (head, right wrist, left wrist) beside the **Model** (3D, with joint
telemetry; **Display options**). While a session or sweep runs, a bar with **Hold**, **Return & disable** and **Disable now** is
on every tab. A red frame and a **REC** badge show a running recording.

**The live view.** Connected, Helm reads the arms with their motors disabled. When teleop, a sweep, a return or Check needs
the motors, the reader steps aside and starts again afterwards; meanwhile the model shows that session's arm. The reader
checks the host, CAN adapters, conflicting services and every motor's identity and Reset state, then reads positions only.
It never enables a motor and renews itself every 15 minutes. If motors stop answering (a loose plug, a power cycle) the pill
shows **Waiting for motors** and a banner names every silent motor; Helm keeps the last pose and your sweep, and retries every
few seconds until they answer or you press Disconnect. A different motor identity, unconfirmed CAN cleanup or a problem on the
robot stops instead of retrying. A joint that jumps by more than 1.5 rad across a reconnect loses its unsaved sweep. The arms are
limp: support them when moving them by hand. The live model is a display, not a collision or safety check.

## Check

**Check** answers "is Minerva ready?" before a session. It checks the robot only, not how it is controlled. **Run check**
works through three stopped stages and shows each result as it lands; the fourth, the live joint sweep, is run from the tab
afterwards. A check whose prerequisite failed is skipped, not guessed.

1. **Operator PC**: the stage's on-screen title. It checks the calibration profile, confirmed motor-to-model directions, joint
   limits for every motor, the servo calibration of every arm joint (a warning while joints are not yet swept, a failure if a
   motor was replaced since; grippers are not swept), the receiver settings derived from all of them, the builds and the route.
2. **Link and Jetson**: pinned SSH; the deployed release intact and identical to the files Helm runs from; its smoke test;
   each arm's live receiver configuration validated by the deployed receiver; the Jetson's disk, memory, temperatures and clock.
3. **Motors, stopped**: CAN helpers and serial pinning installed, both adapters on their pinned serials and free, no other motor
   owner, then the stopped scan of all 14 motors (identity, firmware, fault-free Reset, a position and its power-on range each;
   CAN ends down) and every position inside its swept range. A drive at the factory power-on range (0...2 pi) is a warning:
   a joint resting just below zero reads a whole turn high, and the check says how to clear it. When nothing on either bus
   answers, motor power is off or the foot switch is pressed.
4. **Live joint sweep**, one item per arm, attended, foot switch in hand. After the stopped checks pass, each item has
   **Sweep left arm**/**Sweep right arm**; Helm asks first (arm hanging in its reference pose, path clear, the other arm stays
   disabled), then shows progress on the Tune tab. It is the servo sweep in verify mode at the settings teleop uses: per joint
   a slow out-and-back for friction, then a pass across the swept range at its speed. A joint fails on any guard event, on lag
   over 55 % of its tracking bound, or on lag or friction well above what it was swept with. A gripper is not swept: its command
   moves at 2 rad/s from its swept minimum to maximum and back and passes if it covers 95 % of that travel each way within 8 s.
   It ends closed at its stiffest gain.

The verdict is **GO** only when nothing failed or was skipped and both live sweeps passed, **NO GO** when something failed,
**INCOMPLETE** until the live sweeps run, and **OUTDATED** for a report saved before checks were added. Nothing in the stopped
stages enables a motor. The check reads the motors itself, so the live view steps aside; stop teleop and any sweep first.
Every run is saved under `calibration_runs/minerva-check/` (in the runs folder Helm uses) and the last is shown at start.
The stopped stages also run from a command line: `python -m robot_platform check`.

## Limits

Sweep each joint's travel by hand with the motors disabled. The Limits tab holds three flows: **Joint zero**, the joint
sweep with **Flip** and **Looks right**, and **Head**.

**Sweep joint limits**

1. **Connect**. The model follows the disabled arms.
2. Move one joint at a time to each end of its travel and **hold it there for a moment**. An end counts only when two
   consecutive readings agree, so a glitch or a flick past a stop cannot widen a limit. Every joint records continuously.
3. When a joint shows both ends held, press **Confirm**. Confirmation needs a real span (at least 0.1 rad) with the mechanical
   reference inside it.
4. **Save limits** writes every confirmed joint, merging with the existing file. **Reset** clears one joint's sweep. A
   confirmed joint whose range grows is flagged for re-confirmation.

If the model moves opposite to the arm, press **Flip** on that joint; **Looks right** confirms a direction. These edits need
live readings and are saved to the direction session.

**Joint zero.** A joint that goes back on differently (a wrist remounted) no longer reads 0 at the zero pose. The **Joint
zero** card sets arm joints' motor zeros again, nothing enabled:

1. Hold the arm in its zero pose, straight down as in the 3D view's reference pose, and keep it still. **Choose joints to
   zero...**, pick the joints that moved, and press **Zero the chosen joints now**. The robot checks every drive, then zeroes
   and saves the chosen ones.
2. The zeros wait for a power cycle; nothing that moves the arms starts until then. Switch the motors' power off and on, leave
   the arm where it is, tick *I switched the motors' power off and on, and the arm stayed where it was* and press **Verify**.
   Every zeroed joint must read within 0.01 rad of zero; one that reads its old value lost its zero: zero it again.
3. Verify records the arms' next reference revision as `<asset>-arms-reference.json` in the operator files and moves what was
   bound to the old one onto it: the joint limits (less the zeroed joints'), the direction session and the manual tuning
   presets. The old copies go to the run's evidence (`joint-zero-<UTC>/`). Helm then restarts to load it; log in again.
4. Sweep the zeroed joints again and save, then run Check. Move each zeroed joint by hand first: if the 3D view turns it
   the other way, the remount reversed it: **Flip**.

Wrist camera stills and the twin correction recorded before a zero are refused afterwards. Joint zero ran on the robot on
2026-11-01 (right J4 and J5).

**Head.** The head's servos stay limp: turn it by hand. **Read the head**, hold it straight ahead and level and **Set the
reference...**, turn each joint and **Flip** a joint that turns the other way, then **Looks right**. Sweep each joint to both ends,
confirm both and **Save head limits**. Last, the shoulder corners: turn the head to a side and tilt it down until it is just
clear of the shoulder, **Record corner**; record each side's end and halfway, then **Save shoulder clearance**. Live sessions
take the head only once its clearance is saved. A new reference asks for a new sweep and new corners.

**Files.** Limits are raw motor radians from the recorded mechanical zero, with sample counts and times, bound to the
mechanical reference's id and hash and to each motor's bus, CAN id, UID and firmware. Helm ignores a file (and says why) if any
changes, so a new mechanical zero needs a new sweep. On the robot they live in `~/minerva/helm/operator/`, which a deploy never
overwrites unless it carries a changed copy (then your saved copy moves to `operator/replaced/` and the banner says so). The
file records `runtime_approved: false`: sweeping does not qualify motion. Each run keeps evidence in
`~/minerva/helm/runs/helm/<UTC time>/` (every live frame, cleanup reports, copies of saved or replaced limits).

## Leaders

The Leaders tab records each leader channel's full travel (six joints and the trigger per leader) and the leader zero. Teleop
scales each channel's range onto its robot joint's usable range, from the zero.

1. **Connect leaders** (read-only: boards are polled, servos stay torque-free). The leaders are on the operator station; the
   header must show **GELLO**.
2. Move each joint and trigger **all the way to both ends**. An end counts once three consecutive readings (40 ms) reach it.
   Each card shows which robot joint the channel drives.
3. **Confirm** each channel once its recorded range covers its travel, then **Save** (merges with the file).
4. Hold both leaders in the robot's reference pose (arms straight down, triggers released) and press **Set leader zero**. It
   records every channel at once and saves. Set it again at any time, here or in Teleop (**Z**).

On a card's bar: dark is everything swept, green the recorded range, a dashed outline the saved range, amber the zero, white the
current reading. Readings are unwrapped while you sweep because an encoder can cross its 0/2 pi seam; a saved range is a
center and a span and a range within about 20 degrees of a full turn is refused. A jump that persists discards that channel's
unsaved sweep. Ranges and zero are saved to `config/devices/gello-dual-leader-ranges.json` (an operator copy on the robot),
bound to the leader profile and each board's USB serial. The Leaders connection and a teleop session exclude each other.

## Teleop

Teleop starts a session: a loop process on the Jetson (CPUs 4-5) drives the receiver, which drives the motors. The leaders'
samples come through the bridge. The production profile is `config/devices/minerva-teleop-direct-tuning.json`: 100 Hz, direct
position commands, **collision checking off**.

**Choices (before Start session)**

| Row | Options |
|---|---|
| **Receiver** | **Virtual** (no motors move). **Live**: **Left**, **Right**, **Both**, or **Head** (the head alone, steered by the headset; the arms are left as they are). Live needs the wired route and the loop on the Jetson. |
| **Mode** | **Direct** (drive and record, pedal A), **Calibrate** (camera stills, nothing recorded), **Policy** (a trained policy drives) |
| **Input** | **GELLO** (the leaders) or **Headset** (the Quest's controllers drag the grippers; not recorded) |

There is no "Loop runs on" choice on Minerva: every loop runs in its own process on the Jetson. "Enforced" is hidden: it exists
only with a collision-enabled profile in a virtual rehearsal.

**Start session** checks the arms before it accepts commands: identity, firmware, Reset state, each motor still and within
tolerance of its reference (a joint out of pose is named with how far, as is a joint at or beside a receiver limit), brings the
CAN link up, starts the receiver and arms each motor's watchdog. It is refused while the UPS announces a shutdown, while Check
or a sweep runs, and without the bridge for GELLO input. Keep your hands off the arms from Start until Idle: a motor reading
over 0.2 rad/s three times in a row fails the watchdog audit and faults the session.

| Key | Action |
|---|---|
| I | Idle: motors on, zero torque (limp): support the arms |
| E | Enable: hold the current pose |
| T | Engage: align slowly to the leaders, then follow. T again holds |
| X | Shutdown: return slowly to the reference pose, then disable |
| H | Disable: torque off at once; asks first while the arms are under torque (see [Safety](SAFETY.md)) |
| Z | Set zero: the leaders' pose now maps onto the robot's reference. An engaged arm holds first; Engage again follows |
| O | In Policy mode: take over from the policy, or hand back |

The enable is single-use: after **H** or a completed **X** the arms are limp, and a new session (which checks the pose again)
is needed. Keys work anywhere on the page. The phase bar above shows **Off, Idle, Hold, Align, Teleop, Return**; the status line
shows Phase, Receiver, Cycle, Collision (off) and Control. In touch mode a border shows the state: blue Hold, flashing green
Engaging, solid green Teleop, flashing yellow Shutdown, red fault.

**Pedals.** Three keys A, B, C (a USB pedal, or the keys). Each is offered for 750 ms; the Teleop panel shows what each does now.
They are not an emergency stop.

| Pedal | Does |
|---|---|
| **C** | The next stage: Idle, then Enable, then Engage; while following or paused, Shutdown. It cannot skip a transition |
| **B** | Pause (Hold, motors stay on) while aligning or following; Resume aligns to the leaders again. It does not resume a fault or a lost-leader hold. In Policy mode: take over/hand back |
| **A** | Start an episode; again to finish. Needs the task set up, a healthy capture and fresh cameras. Shutdown aborts an unfinished recording |

Settings → Pedals has **Test pedals** (counts A, B, C; sends nothing to Minerva; only while no session runs). Keys are ignored
while typing in a field. Pedals ran on the robot (they are deployed and used with the recording flow).

**How the arms follow.** Each joint's route says which leader channel drives it and its sign; **Flip** reverses a route and
**Looks right** marks it verified (both need the follower held, not engaged). An arm joint scales from the leader zero, which
maps onto the robot's reference pose, with one scale for both directions; where the leader has travel to spare the robot stops
at its limit and the leader goes on past it. A gripper maps end to end. The robot follows no faster than each joint's speed
cap, so a joint with a small leader range (the wrists, the grippers) lags when you move quickly: hold the leaders still while
engaging. A leader reading faster than 20 rad/s is treated as a glitch and the arm holds. Each arm joint's operating range is
its hand-swept range plus 0.02 rad of overtravel before the receiver faults. In the production profile commands are position
targets with the saved gravity feed-forward for J2-J4 (from a Tune sweep); velocity and friction feed-forward are off unless
*Velocity feed-forward* is ticked in Settings (it takes effect at the next session start). Grippers have no stopping reserve,
do not brake on a stop and keep their last target; the effort limit bounds grip force.

**Gripper force feedback.** The **Gripper force feedback** card has an **On** switch and force bars. While a gripper holds
something the leader's trigger meets a wall where the object stopped the jaws; the push grows as you squeeze past it and never
pulls closed. Settings → Grippers sets *Maximum trigger force* (200 mA is the boards' limit), *Strength*, *Full force at*,
*Grip threshold* and *Hold time*, saved for every session. It ran with the real leaders; the wall was felt on 2026-10-24.
Details: [gripper force feedback](../robots/minerva/GRIPPER_FORCE_FEEDBACK.md).

**When something is lost** and **trips**: see [Safety](SAFETY.md) and [Troubleshooting](TROUBLESHOOTING.md). In short: leaders
lost, the arms hold; the receiver trips, the arms hold then Helm lowers and disables them; a camera drops off USB, the arms
pause; a drive goes quiet, the arm holds on the others. **Trip hold** (the arms hold instead of lowering after a joint-speed
trip) is merged but not on the robot yet.

**Live · Both** drives all 14 motors from one receiver. With collision off, nothing keeps the hands apart: keep the leaders'
paths clear. First powered teleop was 2026-09-27, both arms 2026-10-09, and a 12-minute two-arm session with the head on
2026-10-31.

**Settings that apply per session.** Settings → Motor tuning ("Manual motor tuning") has, per motor, *Kp*, *Kd*, following
speed, *Follow* and an optional travel window: **Apply to session**, **Revert edits**, **Save applied**, **Restore saved**,
**Copy left → right** and **Copy right → left**. Edit while stopped or in Hold, press Apply, wait for the receiver's
acknowledgement, then T resumes. Editing never engages motion. Unchecked joints hold their position with the motors on. A
blank travel uses the full usable range; a window stays centered on the first Enable pose. Shutdown keeps its own speed. The
other Settings categories (session timing and speed ceilings) can only tighten what the profile derives and apply at the next
session start (**Apply for next session**, **Revert edits**, **Restore profile defaults**).

## Head and headset

The neck is two Feetech servos on their own receiver; **Head** appears in the Teleop panel while a session drives it. A
teleop session runs the head with the arms and uses the same keys. The head follows the headset while its latest pose is at most
0.2 s old, holds its last target until that pose is a second old, then returns to its zero; without a headset it holds at zero.
It follows at most 0.7 rad/s inside its limits and shoulder clearance.

To steer it: on the Quest's browser open `https://<robot-address>:8760/xr` (accept the certificate, log in), look straight
ahead and level and press Start head tracking. The head follows once the session is engaged (T). A trigger pull, a thumbstick
click or **Zero headset** (Head card, Settings → Headset) makes where the headset looks the head's straight ahead. The headset
shows the head camera, optionally as a 3D picture from its depth, the model, and the wrist cameras on the model's wrists;
A or X opens a ring of controls. Settings → Headset lists the links and *Preview in the 3D view*. One headset steers at a time.
It needs a login, not control. **X** returns the head to its rest. See [neck](../robots/minerva/NECK_FEETECH.md) and
[headset 3D](../architecture/HEADSET_3D_PLAN.md) for detail, and [the headset on a slow link](REMOTE_HEADSET.md) for `/xr?remote`.

**Headset as input.** With **Input: Headset** the Quest's controllers drive the arms. Engage (T) raises a hanging arm to a
ready pose (forearm forward), so keep the space ahead of the hands clear. Hold a controller's grip to drag that arm's gripper as
the hand moves; let go and the arm holds; the trigger closes the gripper; B or Y holds both arms. Nothing is recorded. The plan
and its status are in [VR arm teleop](../architecture/VR_ARM_TELEOP_PLAN.md); it has run live (2026-11-03), with a joint-speed
trip, and is not established as a working mode.

## Calibrate mode

**Mode: Calibrate** replaces the task card with the camera calibration card, a guide in three steps: **1 Wrist cameras**,
**2 Head camera**, **3 Digital twin**. Each step's tab says how far it got and the card opens on the first not done; a box says
the one thing to do next. Nothing is recorded. To take a still, bring the camera over the table's tags and pause (pedal B, or
**T** while following): once the session has held for a second Helm takes a still of that camera. Resuming before then takes
none, and neither does a hold you did not ask for. Once the still is being taken the arms hold until it is saved (about two
seconds): Resume waits and the card says "Taking a still...". **P** (**Capture still**) captures by hand. H, X and Shutdown are
never held back. Wrist cameras want 15 stills each with a 4 x 3 map of where in the image tags were seen; the head wants 10
stills spanning 20 degrees of turn and tilt. **Solve** gives a verdict (about 1 px is good, over 1.5 px checked, over 3 px not
usable; arm boards more than 10 mm apart are checked, 25 mm not usable); **Save alignment** on a solve that is not usable asks
again within 4 s. Procedures: [wrist cameras](WRIST_CAMERA_CALIBRATION.md), [head camera](HEAD_CAMERA_ORIENTATION.md) and
[alignment](CALIBRATION.md). **Digital twin** > **Apply to the model** draws the measured joints through a correction (drawing
only); **Remove** undoes it. The **Head map** card (**Capture**, 15/30/60 s) records the head camera's pictures and depth for a
splat map of the room. Stills have been taken live (first on 2026-11-02); no document records a solve on the robot.

## Recording

**Mode: Direct** shows the task card (**Set up task**, **Record episode**, **Stop recording**, and **Reset simulated scene** in
simulation). Full procedure: [task workflow](HELM_TASK_WORKFLOW.md). In short: set up the task, start the session, Idle,
Enable, Engage; **A** or **Record episode** starts, again finishes. Wait for **Complete** before Shutdown. Recording needs
all three cameras (left wrist, right wrist, head) fresh and the head's pose feed live, and refuses without a live head.
Episodes are HDF5 files under the receiver session folder (`~/minerva/evidence/live-<time>/episodes/`). Recording is
[episode format 9](../data/EPISODE_FORMAT.md): 16 axes. **Format 9 has never run on the robot**; the running release recorded
format 8 (14 axes, no torque), with one real demonstration at the last count.

## Replay tab

Lists the recorded episodes, newest first (**Refresh**; filter *All*, *To review*, *Kept*, *Discarded*). Choose one to play it
back with all three cameras, the model posed at the playhead, a scrubber and a joint timeline (click or drag to seek). Keys:
Space plays or pauses, the arrow keys step a frame (Shift: one second), speeds from the selector. Replay waits while a
recording runs. **Keep**, **Discard** and **Clear** set the training verdict, with an optional note.

**Replay on Minerva** drives the arms from the selected episode: choose 0.25x, 0.5x or 1x and press **Replay on Minerva**, which
starts a session. Engage (T) aligns to the first frame, then plays the recorded commands; T holds; X lowers. Nothing is
recorded and faster than 1x is refused. See [live replay](LIVE_REPLAY.md).

## Policy mode

**Mode: Policy** runs a trained policy that drives the arms in a session with the leaders connected. Choose it in the policy card
and press Start. See [policy mode](POLICY_MODE.md); it drives 14 axes and refuses a policy whose joints differ.

## Tune

The Tune tab measures each joint's servo settings on the real arm, one arm at a time, attended, foot switch in reach. It uses
the live receiver with the same gates as live teleop; the other arm is never enabled. Choose the **Arm** (**Left arm**,
**Right arm**, **Head**) and the joints (grippers are not swept; Check opens and closes them), check that the arm hangs in its
reference pose with its path clear, and press **Start sweep**. The arm is enabled at zero torque, then holds, and every joint
comes slowly to its reference. Then, joint by joint:

1. **Characterise.** Slow ramps out and back at the current gains, widening toward the usable range, with a hold at each end.
   This maps gravity and friction by angle and fits the joint's gravity slice.
2. **Stiffness x damping.** Position steps and a short ramp for each rung of a stiffness ladder and each damping ratio. A
   stiffness is held at the worst-gravity point only where the torque map predicts its sag stays within half the tracking bound.
3. **Speeds and braking** at the chosen gains: trapezoidal moves at increasing speeds, then increasing decelerations.
4. **Verify** the whole range at the chosen settings.

Every planned move is checked against the collision model along its path relative to the reference pose (a pair within 20 mm
that was not that close already stops it); this check is the sweep's own, not the production teleop profile's. A joint stuck
for 3 s ends the task. **The guards** abandon a task if any joint lags past 75 % of its bound, pushes over half its effort
limit, passes 55 C, or the pose's collision check is not clear; the joint then holds and returns to its reference and the sweep
continues. If the receiver trips anyway it brakes the arm to a hold; the sweep takes it back there, lowers every joint to its
reference and disables it. After three trips while lowering the arm is left held: support it, then **H**.

**Finish (X)** returns the joint under test to its reference, then disables. **Disable (H)** removes torque at once (Helm asks
first). **Return to reference** brings an arm that rests off its reference (moved while limp) back, slowly and together along
one straight joint-space line: the 3D view shows the arm as its drives read it; press it, compare the model with the real arm
and confirm; Helm refuses stale readings or ones outside the swept range, and the arm is disabled at the reference.

**What it chooses**, per joint, with reasons on its card: the stiffest stiffness that holds quietly (torque ripple at rest at
most 0.1 Nm), steps with at most 10 % overshoot, within half the tracking bound under load; the damping with the quickest
settling; the highest speed and deceleration that every direction followed within half the tracking bound; the gravity slice;
and a tracking bound of twice the worst lag. Review the recommended values next to the current ones and the guard events, check
the joints you accept and **Save selected**: they go to the servo calibration (`config/devices/<asset>-servo.json`, an
operator copy on the robot), bound to each motor's UID and firmware. A replaced motor must be swept again. The head's Tune is a
slow jog to each end, never a sweep, because the head can reach its shoulder; nothing is saved. Evidence stays on the Jetson in
`~/minerva/evidence/sweep-<time>` (`samples.f32`, `columns.json`, `results.json`, `slow-cycles.jsonl`).

## Settings

A rail with search: **Changed from profile**, **Motor tuning**, **Grippers**, **Pedals**, further session categories, read-only
categories (limits, braking guards, the full profile, bindings), then **Network**, **Headset**, **Cameras**, **Display & touch**.

- **Grippers:** *Gripper zero* (per gripper: its live reading and **Zero the left gripper...** / **Zero the right gripper...**:
  close the jaws fully by hand, inner faces together, and zero; arms stay disabled; a power cycle confirms the drive kept it)
  and the force feedback settings above.
- **Cameras:** the three camera previews' exposure and gain. **Read camera settings**, adjust, apply; values last until the
  camera service restarts. A preview drops frames on a slow link rather than closing.
- **Network:** how Minerva is connected (wired, tether, Wi-Fi) and **Turn the access point on** (the access point `minerva`
  puts devices on Minerva's own Wi-Fi). Not while a session holds the motors.
- **Display & touch:** *Look*, *Touchscreen layout*, *Day and night*, and the 3D telemetry options (position, torque,
  temperature, direction arrows) with **Reset display preferences**.

## Files on the robot

| Where | What |
|---|---|
| `~/minerva/helm/logs/helm.log` | What Helm prints |
| `~/minerva/helm/operator/` | Limits, servo calibration, leader ranges, head files, the arms' reference |
| `~/minerva/helm/runs/` | Helm's sessions and evidence |
| `~/minerva/evidence/live-<time>/` | A live session: `samples.f32`, `events.jsonl`, `live-<bus>-last.json`, `episodes/` |
| `~/minerva/helm/calibration/<run>/` | Camera calibration stills and solves |
| `~/minerva/releases/`, `~/minerva/current` | Deployed releases |

The [earlier, longer manual](../history/features/HELM_MANUAL_PC_DEPLOYMENT.md) keeps more on the control law, gravity
feed-forward and timing. It describes Helm running on an operator PC.
