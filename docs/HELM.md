# Helm

Helm is the operator web UI for the robot. It runs on the operator PC, serves
`http://127.0.0.1:8760/` on loopback only, and combines the live 3D model with
robot workflows: **Limits** (hand-swept joint limits on the real, disabled arms),
**Leaders** (hand-swept leader-arm ranges) and **Teleop** (the leader arms driving
the guarded receiver in its virtual form; no motors move).

```powershell
.\scripts\helm.ps1
```

Helm opens without touching the robot. **Connect** starts the same stopped-only
reader as the calibration tool: it checks the pinned host, CAN adapter serials,
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

**Start session** loads the keep-out model and the receiver (about 10 s). Then:

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

**Where the limits come from.** Each joint's operational interval is its swept
range intersected with the CAD range, 0.05 rad inside both ends. Targets must
also keep the receiver's stopping reserve inside that interval: tracking
allowance + speed × (verdict + feedback timeout + reaction) + speed²/(2 × stop
acceleration). The usable band is therefore narrower, and faster joints lose
more of it: the wrist swivels run at 2.5 rad/s over ±2.28 rad instead of
0.8 rad/s over ±2.77 rad. Braking harder and a tighter tracking bound shrink the
reserve instead: the elbows run at 1.2 rad/s, braking at 25 rad/s² with a
0.05 rad tracking bound, over the same range they had at 0.8 rad/s. The part
proportional to speed (about 0.2 s of travel before the receiver stops) cannot
shrink, so about 1.3 rad/s is the most an elbow can run without losing range.
Each command step is also capped by the joint's tracking bound, so every tracking
bound is at least its speed times the 40 ms period. The rehearsal speeds and
gains are unqualified.

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
limits, stopping reserve and single-use offers. Advisory mode is refused for
anything but the virtual receiver. Physical teleop needs a tighter, still-sound
bound with enforced checks.

**Timing.** The 25 Hz loop runs in its own process, so browser traffic cannot
stall a cycle. On Windows it runs at high priority, and it and the receiver
process opt out of power throttling (EcoQoS). Without that, on this laptop's
hybrid CPU some cycles took 27–37 ms instead of about 5, and one 60 s run in
three faulted. A cycle about 40 ms late lets the receiver's 80 ms collision
verdict expire: the receiver stops itself and the session faults with the
receiver's reason. Stop the session and start it again. The panel shows cycle
time (p50 / p99); normal is about 5 / 6 ms.
