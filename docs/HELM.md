# Helm

Helm is the operator web UI for the robot. It runs on the operator PC, serves
`http://127.0.0.1:8760/` on loopback only, and combines the live 3D model with
robot workflows: **Check** (Minerva Check, the go/no-go before a session),
**Limits** (hand-swept joint limits on the real, disabled arms), **Leaders**
(hand-swept leader-arm ranges) and **Teleop** (the leader arms driving the
guarded receiver in its virtual form; no motors move).

```powershell
.\scripts\helm.ps1
```

Helm opens without touching the robot. **Via** (top right) chooses how Helm
reaches Minerva's Jetson: **Tether** (the USB Ethernet tether, 10.42.0.1, the
default) or **Wi-Fi** (the address in the device profile). Connect, Minerva
Check and teleop all use it. Wi-Fi is for stopped reads and virtual teleop only:
Minerva Check fails anything but the tether, and live teleop is refused on Wi-Fi.
The choice is locked while the reader, a check or a teleop session runs.

**Connect** starts the same stopped-only reader as the calibration tool: it checks the pinned host, CAN adapter serials,
conflicting services, every motor UID/firmware and Reset state, then reads
positions only. Motors are never enabled; no zero, save or parameter write is
possible over this path. **Disconnect** (or closing Helm with Ctrl+C) stops the
reader and confirms both CAN links are down. The reader renews itself every
15 minutes.

If motors stop answering (a loose plug, a power cycle), the reader shuts down
cleanly and Helm shows **Waiting for motors**, naming every silent motor. It keeps
the last pose and your sweep, and retries every few seconds until they answer or
you press Disconnect; power-cycling the motors while it waits is fine. Each retry
repeats the full identity check. A different motor identity, unconfirmed CAN
cleanup or a problem on the robot computer stops instead of retrying. If a
joint's reading jumps by more than 1.5 rad across a reconnect, its unsaved sweep
is discarded rather than mixed with the earlier readings.

Before connecting: motor power on, both USB-CAN adapters connected, legacy motor
services stopped, and no other calibration tool or leader/robot controller using
the CAN links. The arms are limp; support them while moving them by hand.

## Minerva Check

The **Check** tab answers "is Minerva ready?" before a session. **Run check**
works through the three stopped stages and shows each result as it lands; the
fourth, the live joint sweep, is run from the tab afterwards. A check whose
prerequisite failed is skipped, not guessed:

1. **Operator PC**: the calibration profile, confirmed motor-to-model directions,
   joint limits for every motor, leader ranges and the leader zero, confirmed
   routes, the servo calibration (a warning while joints are not yet swept, a
   failure if a motor was replaced since), the teleop receiver settings derived
   from all of them, both leader arms reading inside their sweeps, fresh ARM64
   builds and a route to the USB Ethernet tether.
2. **Link and Jetson**: pinned SSH over the tether; the deployed release intact
   and identical to this PC's files (otherwise it says to deploy); its smoke test;
   the Jetson's disk, memory, temperatures and clock; and 5 s of leader-link round
   trips at 25 Hz with the real leaders, served from its own responsive process
   as a session will run it.
3. **Motors, stopped**: CAN helpers and serial pinning installed, both adapters on
   their pinned serials and free, no motor service running, then the stopped
   reader scans all 14 motors (identity, firmware, fault-free Reset, one position
   and its power-on range each; CAN ends down) and checks every position is
   inside its swept range. A drive still at the factory power-on range (0…2π)
   is a warning: a joint resting just below its zero at power-up reads a whole
   turn high. Such a reading is named as a whole turn, with how to clear it. A
   drive with a latched fault is named with the fault. When nothing on either bus
   answers, motor power is off or the foot switch is pressed.
4. **Live joint sweep**, one item per arm, attended, with the foot switch in
   hand. Once the stopped checks pass, each item has a **Sweep left arm** /
   **Sweep right arm** button; Helm asks first (the arm hangs in its reference
   pose with its path clear, the other arm stays disabled), then switches to
   the Tune tab for its progress, Finish (X) and Disable (H). X and H also work
   from the Check tab while it runs. It is the servo sweep of the Tune tab in a
   short verify mode, at the settings teleop would use, under the same live
   receiver: per joint, a slow out-and-back for friction, then a pass across
   its swept range at its speed. A joint fails on any guard event, on lag over
   half its tracking bound, or on lag or friction well up on what it was swept
   with (drag, wear or a loose joint). A joint not yet servo-swept is judged on
   the profile settings alone, which is a warning.

The verdict is **GO** only when nothing failed or was skipped and both arms'
live sweeps passed; warnings are listed. After the stopped stages it is
**INCOMPLETE** until the live sweeps are run, and a live sweep's outcome goes
into the report it followed, saved over the same file. Nothing in the stopped
stages enables a motor. The check reads the motors and leaders itself, so it
runs alone: disconnect Limits and stop teleop and any sweep first (the Leaders
tab may stay connected; the check then only confirms the leaders are plugged in).
Every run is saved under `calibration_runs/minerva-check/`, and the last one is
shown when Helm starts. A report saved before checks were added shows as
**OUTDATED**, with what it said then; run the check again (the file is left as
it was). The stopped stages also run from a terminal:

```bash
.venv-platform/Scripts/python.exe -m robot_platform check
```

## Sweep joint limits

1. Connect. The model follows the disabled arms.
2. Move one joint at a time to each end of its travel and **hold it there for a
   moment**. An end counts only when two consecutive readings agree, so a single
   glitch or a flick past a stop cannot widen a limit. Every joint is recorded
   continuously; order does not matter.
3. When a joint shows both ends held, press **Confirm**. Confirmation requires a
   real span (at least 0.1 rad) with the mechanical reference inside it.
4. **Save limits** writes every confirmed joint. You can sweep some joints now and
   the rest later; saving merges with the existing file. **Reset** clears one
   joint's sweep. A confirmed joint whose range grows is flagged for re-confirmation.

If the model moves opposite to the arm, press **Flip** on that joint; **Looks
right** confirms an unconfirmed direction. These edits need live readings and are
saved to the direction session (`calibration_runs/<asset>-directions/`), shared
with `scripts/calibrate_minerva.ps1`.

## Persistent calibration

Limits are saved to `config/devices/minerva-production-01-joint-limits.json`
(next to the calibration profile, named after the device). Values are raw motor
radians from the recorded mechanical zero, with sample counts and times. The
file is bound to the mechanical reference id and file hash and to each motor's
bus, CAN id, UID and firmware; Helm ignores it (and says why) if any of these
change. A new mechanical zero therefore requires a new sweep. The file records
`runtime_approved: false`: the Teleop workspace derives the receiver's limits
from these values with margins (below); sweeping alone does not qualify motion.

Each Helm run keeps local evidence in `calibration_runs/helm/<UTC time>/`: every
live frame (`frames.jsonl`), each reader cleanup report and a copy of every saved
or replaced limits file.

Helm takes the direction session's lock. If a previous tool exited without
removing it, Helm checks that the recorded process is gone before taking over,
and refuses while it is still running.

The live model is a display, not a collision or safety check.

## Sweep leader ranges

The Leaders tab records each leader channel's full travel (six joints and the
gripper trigger per leader) and the leader zero. Teleop scales each channel's
range straight onto its robot joint's usable range, from the zero.

1. **Connect leaders** (read-only: the boards are only polled and their servos
   stay torque-free). Close the leader viewer first; Helm needs the USB ports.
2. Move each joint, and each trigger, **all the way to both ends** of its travel.
   No pause is needed: an end counts once three consecutive readings (40 ms)
   reach it, so one or two glitch readings cannot widen a range. Each card shows
   which robot joint the channel drives.
3. **Confirm** each channel once its recorded range covers its travel.
4. **Save**. Saving merges with the existing file, so you can sweep some channels
   now and the rest later.
5. Hold both leaders in the robot's reference pose (arms straight down, triggers
   released) and press **Set leader zero**. It records every channel at once and
   saves immediately. You can set it again at any time, here or in Teleop (Z).

On each card's bar: dark is everything swept, green is the recorded range (what
Confirm saves), a dashed outline is the saved range, amber is the zero and white
is the current reading.

Readings are unwrapped continuously while you sweep, because an encoder can cross
its 0/2π seam mid-range. A saved range is stored as a center and a span; at
runtime a reading r becomes center + wrap(r − center). That is unambiguous for
any range under a full turn, even after a power cycle resets the servo's turn
count. A range within about 20° of a full turn is refused. One glitch reading is
ignored. A jump that persists, or any jump across a pause or reconnect, discards
that channel's unsaved sweep instead of mixing readings from both sides of it.

Ranges and the zero are saved to `config/devices/gello-dual-leader-ranges.json`
and bound to the leader profile and each board's USB serial. Replaced and saved
copies go to the Helm run's evidence folder. The Leaders connection and a teleop
session exclude each other.

## Teleop (virtual receiver)

The Teleop tab runs the original GUI lifecycle against the guarded Rust receiver
in its virtual form (the native harness): real GELLO leaders, a virtual follower,
**no motors**. The 3D model shows the follower (`VIRTUAL FOLLOWER` badge). It
needs saved limits for every motor, saved leader ranges for every routed channel,
the leader zero and the teleop profile (`config/devices/minerva-teleop.json`,
passed by `scripts/helm.ps1`). Close the leader viewer, and disconnect the Leaders tab,
first: a session owns the leader USB ports. A teleop session and Connect exclude
each other.

**Where the loop runs.** By default the loop runs on Minerva's Jetson, the head
computer that will run the arms' control loop, from the deployed release. Helm
starts it over pinned SSH on the USB Ethernet tether and relays keys, route edits
and the zero. A process on this PC serves the leaders over the authenticated
leader link. The Jetson refuses to start unless its release carries this PC's
calibration files: deploy first (Minerva Check shows whether they match).
**Loop runs on** switches between Minerva's Jetson and this PC while no session
runs; `helm --loop pc` starts on this PC.

Measured with the real leaders, 30 s in TELEOP, against the 40 ms budget:

| Loop on | Cycle p50 / p99 / max | Overruns |
|---|---|---|
| Minerva's Jetson | 6.5 / 14.4 / 26.5 ms | 0 |
| This PC | 4.7 / 5.8 / 8.6 ms | 0 |

The Jetson's longest cycles are waits for a late leader reply (up to 20 ms).
Leader samples older than the 50 ms target limit are skipped for that cycle
(10 of about 650 on the Jetson, 18 on this PC). Those come from the leaders'
sample age, not the link.

**Start session** loads the keep-out model and the receiver: 2–3 s on the Jetson,
whose release has no meshes to load, and about 10 s on this PC. Then:

| Key | Action |
|---|---|
| I | Idle: zero torque, the command follows the measured pose |
| E | Enable: hold the current pose |
| T | Engage: align slowly to the leaders, then follow them. T again holds |
| X | Shutdown: return slowly to the reference pose, then disable |
| H | Disable |
| Z | Set zero: the leaders' pose now maps onto the robot's reference pose (saved) |

Keys work anywhere on the page, including over the 3D view. Start with I, then E
and T. If leader samples stop (unplugged, late), the follower keeps its last
target for two cycles and then holds; press T to engage again.

Each joint card shows its usable band with the leader target, the command and
the (virtual) robot position, plus its route: leader, channel and sign. The
route signs in the profile are candidates. Verify each joint by moving its
leader: **Flip** reverses a route and **Looks right** marks it verified. Both
need the follower held, not engaged, and are saved to
`calibration_runs/<asset>-teleop/routes.json`.

Every channel scales straight and covers the robot's whole usable range. An arm
joint scales from the leader zero, which maps onto the robot's reference pose,
with one scale for both directions: the larger of robot travel over leader
travel on the two sides of the zero. So both robot ends are reached within the
leader's range. On the side where the leader has travel to spare, the robot
stops at its limit and the leader goes on past it (also past its recorded range,
if it can). Each card shows its scale and how far before the leader's end the
robot stops. **Set zero (Z)** works at any time: an engaged
arm holds first rather than jumping, then Engage (T) aligns slowly and follows
from the new zero. A gripper maps end to end (a released trigger and closed
jaws are already ends). For an arm joint, +1 means the leader and robot joint
angles increase together. For a gripper, +1 means the trigger angle increasing opens it.
The CAD slide axes are mirrored between the two grippers, so the sign is defined
by opening instead. Gripper route edits made before the proportional mapping
are dropped; verify the grippers again.

The robot follows no faster than each joint's speed cap. Where a leader's range
is much smaller than the robot's, as with the wrist swivels (about 1.2–2 rad onto
4.6 rad) and the grippers (about 0.9 rad onto 5.1 rad), the robot has to move
several times faster than your hand. It lags when you move quickly. Hold the leader
still while engaging: alignment finishes once the robot settles on the leader's
target. A leader reading that moves faster than 20 rad/s is treated as an encoder
glitch, and the arm holds; a quick trigger squeeze stays well under that.

**Where the limits come from.** Each arm joint's operational interval is its
hand-swept range (the Limits tab), plus 0.02 rad of overtravel at each end
before the receiver faults. The CAD's nominal limits no longer narrow it.

**Joints slow down near the ends.** A joint may approach an end only as fast as
it can still stop at it:
- the receiver checks feedback every tick, so it reacts within the feedback
  timeout and its reaction allowance, 40 ms (the collision verdict's 80 ms
  lifetime bounds the collision box, not a joint limit);
- it brakes at its stop acceleration;
- in open travel it runs at its speed cap, and near an end it never slows below
  0.5 rad/s.

The receiver enforces this braking curve on feedback while it drives the arm,
as it does the cap. Targets keep only the stopping margin at 0.5 rad/s inside
the interval, about 0.03 rad, so neither a faster cap nor a looser tracking
bound costs range: on the right arm 99–100% of each joint's swept travel is
usable, and the leader scaling covers all of it. The collision box keeps its
own wider reserve, clipped at the limits.

**The control law.** Commands are no longer position-only:
- the receiver ramps each 25 Hz command over 40 ms (`interpolation_ms`), from
  where the previous one pointed to its target, with the command's velocity fed
  forward, so a joint's own damping no longer brakes it (J2 at 1.5 rad/s lost
  22 Nm to it, 0.1 rad of lag);
- a joint whose servo sweep is saved also gets 70% of its measured friction fed
  forward in the direction of motion;
- J2–J4 get gravity fed forward at the commanded pose (below), so the position
  loop no longer holds the arm up and J2/J3 lift through their whole range;
- a stop keeps the last command's feed-forward, so the arm does not sag when
  the receiver brakes.

Grippers keep position-only commands: fed-forward velocity would add to the
squeeze while the jaws close on an object.

**Gravity feed-forward.** J2, J3 and J4 turn about parallel horizontal axes, so
the torque each holds against gravity is a sum of sines of the absolute angles
of the links beyond it. `control/gravity.py` finds each arm's chain and each
motor's sense from the model's joint frames and the verified directions. Each
joint's gravity slice (`A sin q`, measured by its servo sweep at the reference
pose, where the arm hangs straight down) is split into link moments. The slices
live in the servo calibration's `gravity` section. The right arm's came from the
2026-09-28 sweep (J2 16.78, J3 6.40, J4 2.51 Nm), the left arm's from the
legacy collector's sweep of it (2026-08-22), until its own sweep refits them.

**Commands keep headroom under what the receiver enforces.** Servos overshoot
the speed they follow:
- the receiver's speed cap is 1.25 × the commanded cap;
- near an end, commands move at most 80% of the curve's speed there.

A command still leads its measured position by at most 60% of the joint's
tracking bound. Caps: J1–J3 1.5 rad/s, J4 2.5, the wrists 4. A servo
calibration saved before this control law (2026-09-28) keeps its gains, braking
and tracking bound but not its speed. The speeds and gains are unqualified.

**Grippers are exempt.** They touch and hold objects, so the receiver treats a
joint marked `"gripper": true` differently. It gets no margin and no stopping
reserve: its targets span its whole range, from the model's closed limit
(0.055 rad, 0.5 mm per jaw) to fully open. It does not brake on a stop; it keeps
its last target, so a grasp is not released. It does not fault when it trails
its command, as it does when stalled on an object, and its command may lead its
position by its whole range. Effort, gain, speed, position and temperature
limits still apply; on real motors the effort limit is what bounds grip force.
The grippers run at 10 rad/s.

**Collision checks are advisory here.** The conservative swept keep-out bound
certifies only about ±0.011 rad per arm joint at the hanging pose, about
0.03 rad/s at 25 Hz. So in the virtual receiver, every cycle checks the target
pose against the same keep-out geometry and shows the result (clear, distance, or
the contacting shapes), but does not block. The receiver still enforces its
limits, stopping reserve and single-use offers. The only physical use of
advisory checks is live teleop of one arm (below); two-arm physical teleop needs
a tighter, still-sound bound with enforced checks.

**Timing.** The 25 Hz loop runs in its own process, so browser traffic cannot
stall a cycle. On Windows it runs at high priority, and it and the receiver
process opt out of power throttling (EcoQoS). Without that, on this laptop's
hybrid CPU some cycles took 27–37 ms instead of about 5, and one 60 s run in
three faulted. A cycle about 40 ms late lets the receiver's 80 ms collision
verdict expire: the receiver stops itself and the session faults with the
receiver's reason. Stop the session and start it again. The panel shows cycle
time (p50 / p99); normal is about 5 / 6 ms.

## Live teleop (one arm)

Live teleop drives **one arm's real motors** through the guarded receiver on
Minerva's Jetson. The other arm is not in the receiver and is never enabled.
Collision checks are advisory, as agreed until a two-arm bound exists: every
cycle checks the target pose against the keep-out geometry, with the other arm
taken at its reference pose. Nothing stops the arm on a contact. The motion
settings are unqualified, so these are attended tests with the foot switch in
reach.

**Before you start:**
- Minerva Check's stopped stages pass (its live sweep items may still be
  pending while the servo settings are being measured). Its "Live receiver
  configuration" item has the
  deployed receiver validate both arms' configurations. "Live start" runs a
  session's own pre-scan on both arms: within 0.1 rad of the reference pose
  (a warning otherwise), and clear of the receiver's limits (a failure otherwise).
- Helm is on **Via Tether**, and the loop runs on Minerva's Jetson (live is not
  available on this PC or over Wi-Fi). The panel says why if Live is greyed out.
- The arm to drive hangs straight down in its reference pose with the gripper
  closed. Every joint must be within 0.1 rad of its reference.
- Keep your hands off the arm from Start until Idle: until the first enable the
  receiver keeps auditing the motors' watchdogs, and a motor reading over
  0.2 rad/s three times in a row fails that audit and faults the session. A
  single reading over it is drive noise: still motors read up to about
  0.2 rad/s.

Choose **Receiver: Live · left arm** (or right). The panel turns red and Start
asks you to confirm.

**Starting a session**, with the motors still disabled throughout:
1. The stopped reader scans the arm: identity, firmware, fault-free Reset, and
   each motor still and within tolerance of its reference. A joint out of pose
   is named, with how far it is off. So is a joint at or next to a receiver
   limit, which would fault the receiver at once (it faults on any reading
   outside its limits, even disarmed).
2. The arm's CAN link comes up.
3. The receiver starts in its one-arm teleop mode. It repeats the identity and
   reference checks and arms each motor's command-loss watchdog before it
   accepts commands.

**Keys** work as in virtual teleop:
- **Idle (I)** enables the motors at zero torque, so the arm is limp: support
  it.
- **Enable (E)** holds the arm where it is.
- **Engage (T)** aligns slowly to the leaders, then follows.
- **Shutdown (X)** returns slowly to the reference pose, then disables.

The receiver's enable is single-use: after Disable (H) or a completed Shutdown
the arm is limp, and a new session, which checks the reference pose again, is
needed to enable it. Stopping a session with the arm powered lowers it to its
reference first, as Shutdown (X) does, then disables it; Helm asks first.
Disable (H) removes torque at once: the arm goes limp where it is.

**What happens when something is lost:**

| Lost | Result |
|---|---|
| Leader samples (unplugged leader, PC or tether gone) | After two cycles the arm holds where it is until you Engage again. The foot switch cuts power at any time. |
| Helm's session (closed, crashed) | The loop on the Jetson lowers the arm to its reference, then disables it and stops the receiver. |
| The loop | The receiver stops the arm when its lease and collision verdict expire (150 ms and 80 ms). |
| The receiver process | Each motor's own watchdog disables it when commands stop. |

**Trips hold the arm.** A joint past a limit, or too fast toward one, over its
effort or temperature, past its tracking bound, or out of its collision box
trips the receiver. It brakes the arm to a hold that keeps its gravity
feed-forward, rather than cutting torque, and revokes the session's lease. The
session takes the arm back where it is held, lowers it to its reference and
disables it; the reason stays on screen. A stop the receiver makes itself (its
lease or collision verdict lapsing) ends the same way. Only lost or invalid
feedback, a drive's own fault, an emergency stop or the receiver's own loop
running late cut torque at once, and a raised arm then drops. Until
2026-09-29 every trip cut torque: right J2 fell from horizontal.

A receiver fault or trip names what its own feedback shows (a joint outside its
limits, too fast, over its effort or temperature, a drive fault, a failed
watchdog audit). The receiver's last status is saved as `live-<bus>-last.json`
in the session's `~/minerva/evidence/live-<time>` folder on the Jetson.

**Grippers** use their full physical range, closing past the model's closed
limit. A closed gripper rests on its hand-swept end and its 16-bit feedback can
read a step (~0.0004 rad) past it, so the receiver's gripper limits extend
0.05 rad (about 0.5 mm of jaw travel) past each swept end. Gripper targets stay
within the swept travel. A stalled grasp holds about hold Kp × lead: up to 10 Nm with the current
gains (2 Nm/rad over a 5.1 rad lead), against the gripper motor's 14 Nm peak.
Long grasps heat the motor toward the receiver's 70 °C limit. Lower the
gripper's `hold_kp` if that matters.

## Servo sweep (Tune)

The Tune tab measures each joint's servo settings on the real arm, so they are
not tuned by hand in teleop sessions. It runs one arm at a time, attended, with
the foot switch in reach. It uses the live receiver with the same gates as live
teleop: the other arm is never enabled, and the loop runs on Minerva's Jetson
over the tether. The receiver's stiffness, damping and speed ceilings are raised
to the sweep's own ceilings (the `sweep` block of the teleop profile).

Choose the arm and the joints (all by default), check that the arm hangs in its
reference pose with its path clear, and press **Start LIVE sweep**. The arm is
enabled at zero torque, then holds, and every joint comes slowly to its
reference. Then, joint by joint, with the rest of the arm held there:

1. **Characterise.** Slow ramps out and back at the current gains, widening
   step by step toward the joint's usable range, with a hold at each end. This
   maps gravity and friction by angle, weighting torque by displacement so
   inertia and stuck samples drop out, and fits the joint's gravity slice. The
   sweep feeds velocity, friction and gravity forward as teleop does, so a heavy
   joint reaches much further than on position-only commands.
2. **Stiffness × damping.** Position steps and a short ramp at the reference
   for each rung of a stiffness ladder (from the ceiling down) and each damping
   ratio. Damping scales as the square root of stiffness, times a multiplier.
   A stiffness is held at the worst-gravity point only where the torque map
   predicts its sag stays within half the tracking bound.
3. **Speeds and braking** at the chosen gains: trapezoidal moves at increasing
   speeds, then braking at increasing decelerations.
4. **Verify** the whole verified range at the chosen settings.

Every planned move is checked against the collision model along its whole
path, relative to the reference pose: a pair that comes within 20 mm and was
not that close already stops it.

**How the plan moves the joint:**
- Each cycle, a command's lead over the measured position is capped at 60% of
  the joint's tracking bound.
- The plan waits for a joint pinned at that cap rather than running ahead of
  it. A joint stuck for 3 s ends the task.
- Stiffness and damping change only once the joint is at rest, ramped over
  0.3 s.
- Speed tests stop at 80% of the receiver's cap, and step tests are at most
  0.03 rad, leaving headroom under the overspeed limit.
- Moves slow down near the ends, as teleop does.

**The guards.** A guard abandons a task if any joint of the arm:
- lags past 75% of its bound;
- pushes over half its effort limit;
- passes 55 °C;
- or the pose's collision check is not clear.

The joint then holds where it is at the current gains and returns to its
reference, and the sweep continues with what the evidence allows. The
receiver's own limits stay in force behind all of this. If the receiver trips
anyway, it brakes the arm to a hold; the sweep takes it back there, lowers every
joint to its reference at its characterisation speed and disables it. The
joint under test shows as stopped. After three trips while lowering, the arm is
left held where it stopped: support it, then press H.

**Bound and speed ceilings.** A joint's `bound_ceiling` and `speed_ceiling` in
the teleop profile's `sweep` block let the session raise its tracking bound and
speed cap, so the sweep can measure there. Neither costs travel. The bound sets
the joint's push, through the lead cap. Bound ceilings are set for J4, J5, J6
and the gripper only; speed ceilings are 1.5 rad/s for J1–J3, 2.5 for J4 and
4 for the wrists.

**Finish (X)** returns the joint under test to its reference, then disables
the arm. **Disable (H)** removes torque at once, and the arm goes limp where it
is. Stopping the session lowers the arm to the reference first, then disables
it; Helm asks first.

**What it chooses**, per joint, with the reasons shown on its card:
- **Stiffness**: the stiffest rung that passes, since it follows and holds
  best. Its steps must not overshoot by more than 10% and must come to rest
  within half the tolerance, and it must hold the joint's load within half the
  tracking bound: friction plus gravity, or with gravity fed forward, 20% of
  it (the model's error).
- **Damping**: the ratio with the quickest settling at that stiffness.
- **Speed and braking**: the highest tested speed that every direction
  followed, and the highest deceleration, with at most half the tracking bound
  of lag. A short joint runs out of room to test its faster speeds; when every
  speed it had room for was followed, it gets its cap. A final pass across the
  whole range at that speed must stay within half the bound too; if it doesn't,
  the speed steps down.
- **Gravity slice**: saved with the joint, for the arm's gravity feed-forward.
- **Tracking bound**: twice the worst lag at that speed (including that final
  pass), between 0.03 rad and the session's bound.

The results show the recommended values next to the current ones, the
measurements (gravity, friction, verified range, worst lag) and any guard
events. Check the joints you accept, whose verification passed, and **Save
selected**. They go to the servo calibration,
`config/devices/<asset>-servo.json`, bound to each motor's UID and firmware.
The receiver takes those joints' gains, speed, braking and tracking bound from
it, and live teleop uses it from its next session. A replaced motor must be
swept again. The release carries `config/devices`, so deploy
(`bash scripts/deploy_minerva_arms.sh`) before the next Minerva Check; until
then its "Release matches this PC" item fails.

Minerva Check's live joint sweep (above) runs this engine in verify mode
against the saved calibration. Its evidence goes to `check-sweep-<time>`.

Evidence stays on the Jetson in `~/minerva/evidence/sweep-<time>`:
- `samples.f32`, every receiver tick as float32 rows, described by
  `columns.json`;
- `results.json`, the per-joint results and events.
