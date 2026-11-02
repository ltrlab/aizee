# VR arm control: plan

The operator drives Minerva's arms through the GELLO leaders; the Meta Quest Pro steers the head and shows what the
robot sees. The user asked on 2026-11-01 for the arms to be driven from the Quest's controllers through Helm, as
AIZEE's `collect_demo.py` once did: a relative grab and drag of the gripper, not constant tracking. The user approved
the defaults below the same day, and asked that the headset work on the home network and on Minerva's own access
point, switching easily between them. Branch `claude/vr-arms`, from `58acc08` (`claude/teleop-modes`, what Minerva
runs).

## What exists

- **The headset end** (`helm/static/xr.html`, `xr.js`, `helm/xr.py`): a WebXR page on the Quest sends the headset's
  orientation at 60 Hz over Helm's socket (`/api/helm/xr`); Helm turns it into the head's yaw and pitch from a zero
  and passes it to the teleop loop's process (`TeleopProcess.head_pose`, never blocking: a quiet loop drops poses).
  One headset steers at a time. The trigger zeroes the head; hand tracking only floats the wrist cameras, and
  pinches are ignored because the hands hold the GELLO leaders.
- **Sources other than the leaders** (`helm/teleop.py`, `TeleopCore`): a recorded episode (`teleop.episode_source`) or
  a policy (`teleop.policy_source`) can drive the arms instead. A source returns rows
  `{name, position, sampled_at, sequence}` each cycle; Engage's alignment, the speed limits, the collision bound and
  the receiver's own limits apply to them as to the leaders'.
- **Kinematics**: `vision/chain.py` places a link for given joint positions from a URDF. Nothing solved the inverse
  for Minerva before this branch.
- **AIZEE's version** (`python/teleop/quest_leader.py`, `python/web/`, `python/ik/`, `config/quest_teleop.yaml`; June
  2026, `3dbb95f`), outside the guarded path, one arm:
  - Holding the right grip anchored the controller's pose and the gripper's pose at the last *commanded* joints, so
    a new grip continued where the last one ended; releasing held the arm.
  - The target was the anchored gripper pose plus the hand's motion since the grip, turned into the robot's frame by
    a fixed axis swap and a "forward" yaw (Calibrate forward took it from the head's yaw). The hand's rotation was
    applied in the world frame.
  - Filters: quaternion sign flips, one-frame jumps over 30 cm or 0.5 rad rejected (taken after 3 in a row), a
    low-pass filter (0.3), a workspace box, 0.6 m/s on the target (its time step capped at 33 ms), 4 rad/s a joint
    after the IK. A pose older than 0.1 s released the grip.
  - Damped least-squares IK, 8 iterations from the last answer, 60 times a second.
  - The trigger drove the gripper, clutched or not. B latched a software hold; both grips for 1 s cleared it.
  - `collect_demo` followed the Quest at 3 rad/s and 0.2 rad lead (IK near a singularity jumps; no force feedback).
  - The headset drew a ghost of the commanded arm, the workspace box, the anchor and target, and a red line when
    clamped; buttons re-anchored, set forward and resized the box. `fake_quest.py` sent scripted poses.

## Behaviour (the user's defaults, 2026-11-01)

- **Controllers**, not hand tracking (later). Each controller drives its own arm.
- **Grab and drag, per arm.** Holding the grip attaches the hand: the gripper's pose follows the hand's motion since
  the grip. Releasing holds the arm. Engage (T) makes the arms live; the grip attaches a hand. The grip is the
  device's clutch, apart from Helm's Engage, as `docs/GELLO_TELEOP.md` asks.
- **Anchor**: while following, a new grip anchors on the last commanded target; at Engage, and after any hold, fault
  or camera pause, on the measured joints.
- **1:1** hand to gripper, adjustable in the headset. **All six axes** follow; orientation is clamped near the wrist
  limits.
- **The trigger** drives the gripper whenever the arm is engaged.
- **Forward** is the yaw of Helm's head zero: one zero for the head and the arms. The head zero moves from the
  trigger to a thumbstick click.
- **B / Y** is Hold (T). No controller button disables (H): an H during a replay dropped the arms on 2026-10-24.
  Enable and Engage stay on the keyboard and pedals.
- **Input** is its own switch (GELLO / Headset) beside Mode, so Calibrate, or Policy's corrections, can use the
  headset later.
- **Network**: the headset works on the home network and on Minerva's access point (below).
- No change to the motion contract: the headset is another source of targets.

## Frames

| Frame | What | Where it comes from |
|---|---|---|
| Robot | Minerva's base: front −y, left +x, up +z | `urdf/minerva/model.json` (`front_direction`) |
| Arm joints | CAD angles, unwrapped: `scale · motor + offset`, no turn taken off (the wrist swivels were swept 6.65 rad) | `control.guarded_config.cad_map` |
| Tool point | midway between the two finger carriages, in the gripper link (provisional until the fingertip pads are measured from CAD) | `urdf/minerva/minerva.urdf` |
| VR world | WebXR `local`: y up, −z ahead where the session started | the headset |

**VR to robot.** With the zero's yaw taken out, the VR world's right (+x) is the robot's right (−x), up (+y) is
up (+z), and ahead (−z) is the robot's front (−y). A hand's motion `d` becomes `A · Ry(−yaw0) · d` with
`A = [[−1, 0, 0], [0, 0, 1], [0, 1, 0]]`; a hand's rotation is conjugated the same way and applied in the world
frame, `R_target = R_delta · R_anchor`.

## Network and access point

Minerva's network service (`ops.network`) finds the Ethernet port's mode (a network's address, or the tether's) and
switches the access point `minerva` from Helm's Settings. Minerva always answers at its local address (`ops.network`
`LOCAL_ADDRESS`, the tether's address, `ops.ssh.TETHER`), on the tether and the access point alike. The headset
reaches Helm:

- **On the network**: the Quest on the same Wi-Fi as Minerva, at Minerva's network address (wired, or its Wi-Fi
  client's when unplugged).
- **On the access point**: the Quest on `minerva`, at the local address.

What makes switching easy:

- **One certificate for every address.** Helm's certificate (`ops.helm_service.addresses`) names the local address,
  loopback and the device profile's Wi-Fi address, but not the wired address, so with Ethernet on a network the
  Quest cannot open Helm. It should name every address the network service reports (wired, Wi-Fi, local, the
  computer's `.local` name), and `helm_service` reissue it when they change (the Quest accepts each address once).
- **The right link shown.** Settings → Headset shows the `/xr` link for the mode Minerva is in and the other one;
  the headset page shows the other link too, so a dropped link says where to go.
- **Bookmarks.** One per mode in the Quest's browser. A page cannot switch the Quest's Wi-Fi: that stays in its
  settings. Each address is its own origin: one login and one certificate acceptance each, once.
- **Safe switching.** Helm already refuses the access point switch while a session holds the motors. A lost headset
  holds the arms (stale poses release both grips).
- With Ethernet on a network the access point runs beside it; without Ethernet the access point takes Minerva's only
  Wi-Fi, and devices on the home network lose Minerva.

## Phase 1: Minerva arm IK (started 2026-11-01)

- `teleop/arm_ik.py`: each arm's chain from the robot's base to the tool point; forward kinematics and the
  geometric Jacobian (every arm joint turns about its own z); damped least squares with weighted position and
  orientation, a step cap, warm starts and limits.
- The limits are the receiver's usable range, mapped to CAD angles, so the IK never asks for what the receiver
  refuses.
- **Done when:** round trips from random poses across the swept ranges come back within 1 mm and 0.01 rad; no
  answer is outside its limits; an unreachable target returns the nearest answer and says so; the solve time is
  measured on the laptop and the Jetson.

## Phase 2: VR arms source

- `teleop/vr_arms.py`, a plain class tested offline: per arm, the grip, the filters, the workspace box, the speed
  caps, the trigger to the gripper, release on stale poses or a recenter (WebXR's `reset`), forward from the zero.
- Scripted controller tracks (from `fake_quest.py`'s modes) drive it through the IK.

## Phase 3: Headset page and relay

- `xr.js` adds each controller's grip pose, grip, trigger, thumbstick click and B/Y to the pose message; the head
  zero moves to the thumbstick click.
- `xr.py` checks the controller fields and Helm passes them on with the head pose's rules (never blocking, dropped
  when stale). One headset owns the head and the arms.
- The certificate and links of the network section.

## Phase 4: Loop integration

- A `headset` source beside episode and policy. The IK runs in its own process, as the head's does since the
  2026-10-31 stalls, never in the arms' loop.
- Helm: the Input switch; each arm's grip, tracking and clamp on the Teleop card; a VR profile with tighter speed
  caps (AIZEE's 3 rad/s, 0.2 rad lead as the starting point, sized against Minerva's).

## Phase 5: Feedback in the headset

- A ghost of the target, the workspace boxes and the clamp line, with the headset's model (`xr_model.js`).
- Later: a controller pulse when the jaws stop on an object, where the GELLO trigger has its wall.

## Phase 6: Rehearse, then live

- Offline on `simulation/arm_plant.py`, then `helm --sim` with the real Quest.
- One arm live in the cage with the user driving, then both arms under the enforced bound.

## Open

- The tool point's fingertip offset, from the finger meshes.
- The IK's time on the Jetson, which decides whether one worker serves both arms.
- Whether the Quest's browser resolves `.local` names; until seen, the links use addresses.
