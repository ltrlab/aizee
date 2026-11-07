# VR arm control: plan

The operator drives Minerva's arms through the GELLO leaders; the Meta Quest Pro steers the head and shows what the
robot sees. This plan lets the Quest's controllers drive the arms through Helm, as AIZEE's `collect_demo.py` once did:
a relative grab and drag of the gripper, not constant tracking. The headset must work on a home network and on
Minerva's own access point, switching easily between them.

## Status

| Phase | State |
|---|---|
| 1: Minerva arm IK (`vision/chain.py`, `teleop/arm_ik.py`) | Built. |
| 2: VR arms (`teleop/vr_arms.py`, `teleop/vr_worker.py`) | Built. |
| 3: Headset page and relay (`helm/xr.py`, `static/xr*.js`) | Built. The controller messages, the thumbstick zero and the network links were checked offline and in a local preview, not on a real Quest. |
| 4: Loop integration (the headset source, `teleop/headset_source.py`) | Built behind a VR profile; **never run with a Quest driving the arms.** |
| 5: Feedback in the headset | The headset model and control ring exist (`xr_model.js`, `xr_ring.js`); the target ghost, workspace boxes and controller pulse are unconfirmed or not built. |
| 6: Rehearse, then live | Not done. |

Tests: `tests/unit/test_vr_arms.py`, `test_headset_source.py`, `test_helm_xr.py`, `tests/contract/test_headset_mode.py`.

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
  for Minerva before phase 1.
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

## Behaviour (the chosen defaults)

- **Controllers**, not hand tracking (later). Each controller drives its own arm.
- **Grab and drag, per arm.** Holding the grip attaches the hand: the gripper's pose follows the hand's motion since
  the grip. Releasing holds the arm. Engage (T) makes the arms live; the grip attaches a hand. The grip is the
  device's clutch, apart from Helm's Engage, as `docs/architecture/GELLO_TELEOP.md` asks.
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

- **One certificate for every address.** Helm's certificate (`ops.helm_service.addresses`) named the local address,
  loopback and the device profile's Wi-Fi address, but not the wired address, so with Ethernet on a network the
  Quest could not open Helm. Since phase 3 it names every address the network service reports (wired, Wi-Fi,
  local, and the computer's `.local` name), and `helm_service certificate` issues it again when one is missing (the
  Quest accepts each address once).
- **The right link shown.** Settings → Headset shows the `/xr` link for the mode Minerva is in and the other one;
  the headset page shows the other link too, so a dropped link says where to go.
- **Bookmarks.** One per mode in the Quest's browser. A page cannot switch the Quest's Wi-Fi: that stays in its
  settings. Each address is its own origin: one login and one certificate acceptance each, once.
- **Safe switching.** Helm already refuses the access point switch while a session holds the motors. A lost headset
  holds the arms (stale poses release both grips).
- With Ethernet on a network the access point runs beside it; without Ethernet the access point takes Minerva's only
  Wi-Fi, and devices on the home network lose Minerva.

## Phase 1: Minerva arm IK (built 2026-11-01; Jetson timing in phase 4)

- `teleop/arm_ik.py`: each arm's chain from the robot's base to the tool point; forward kinematics and the
  geometric Jacobian (every arm joint turns about its own z); damped least squares with weighted position and
  orientation (1 : 0.5), a step cap, warm starts and limits. The damping adapts (Levenberg-Marquardt): a fixed 0.05
  crawled near singular poses, 1 in 30 round trips unconverged after 100 iterations.
- The limits are the receiver's usable range, mapped to CAD angles (`cad_limits` from a derived receiver's report),
  so the IK never asks for what the receiver refuses.
- **Checked offline** (`tests/unit/test_arm_ik.py`, the swept travel with every direction sign tried both ways, as
  the signs live in Helm's run state):
  - the tool's pose agrees with `vision/chain.py` to 1e-12, the Jacobian with finite differences;
  - poses come back within 0.2 mm and 0.002 rad from seeds 0.1 rad away, in at most 6 iterations; from 0.3 rad
    away 97 % or more (the rest settle on another configuration with a joint at its limit, and say so);
  - a smooth path at 0.6 rad/s a joint takes at most 4 iterations a 60 Hz frame; no answer leaves its limits; a
    target 3 m away returns the nearest answer, unconverged; a pinned joint leaves the others to move.
- **Time, laptop**: a solve from the last answer takes 0.5 ms (median) and 0.9 ms (95th percentile) for one arm;
  from 0.3 rad away, 0.8 and 1.9 ms. The Jetson's is in phase 4.
- **Found: the reference pose is singular.** The arms hang straight: they cannot move further down, and J1 and J5
  are both vertical, so a 1 cm sideways drag asks ±0.2 rad of J1 against J5. Phase 2 caps each joint's motion a
  frame (3 rad/s, AIZEE's Quest following speed) and the headset says to raise the arm before grabbing it there.

## Phase 2: VR arms (built 2026-11-01)

- `teleop/vr_arms.py` (`VRArms`): plain hand samples in (position, orientation, grip, trigger, B/Y, the time Helm
  received them), each arm's CAD angles and gripper closure (0 open, 1 closed) out; phase 4's source turns them into
  motor targets. `seed(arm, positions)` at Engage and after any hold, fault or pause; `recenter()`, `set_forward(yaw)`,
  `set_scale(s)`.
- Per arm: the grip (attaches above 0.6, lets go below 0.4; a grip held at seeding must be let go first), the hand's
  sign fix, jump check (0.3 m or 0.5 rad; after 3 in a row the hand attaches again there, so a glitch moves nothing)
  and 20 ms low-pass filter; the tool's target at most 0.6 m/s and inside the arm's workspace box (given by phase
  4's profile; none in code: the table's height is not measured yet); each joint at most 3 rad/s. A hand silent for
  0.1 s lets go; back with the grip held, it attaches again where it is.
- Each update reports gripped, tracking, clamped, the IK's convergence and limits, the smallest singular value and
  whether the arm is near a singular pose (below 0.02), and a B/Y press (once).
- **Checked offline** (`tests/unit/test_vr_arms.py`, 22 tests; hands at 60 Hz, updates at 100 Hz): the axis mapping
  and forward yaw (5 cm drags within 1 mm), turns about the same world axis at the tool point, let go and grip again
  continuing, lost hands, a recenter 1 m away, one-pose and persistent glitches, the tool and joint speed caps, the
  workspace box, the trigger, B/Y, scale, and fake_quest.py's 10 cm circle followed within 8 mm.
- **Time, laptop**: both arms 1.1 ms an update (median), 4.3 ms (95th percentile), 8.9 ms at most. It stays out of
  the arms' loop (phase 4).
- **Found: near-singular poses are common.** J2, J3 and J4 are parallel pitch joints, so whenever the wrist is near
  the vertical through the shoulder, a sideways drag at a fixed orientation needs J1 and J5 to turn against each
  other: from one such pose a 5 cm drag asked 0.86 rad of each, and ran into the singular pose. Over the swept
  ranges, half of all poses have a smallest singular value under 0.06 and a fifth under 0.02. A lower orientation
  weight does not help (the solver still finds the exact answer). The joint cap keeps the swing slow; **whether the
  gripper's orientation should give way near those poses instead is for the rehearsal to decide** (phase 6). In
  front of the robot at working height the arms are well conditioned (0.16).

## Phase 3: Headset page and relay (built 2026-11-01; not yet seen in the Quest)

- `xr.js` sends each controller's grip pose (its grip space, in the level 'local' space), grip, trigger, B/Y and
  `ui` with the head pose, at most 60 times a second (`xr_status.handMessage`; both hands fit in under 600 of
  Helm's 1024 bytes). Tracked hands are left out: they hold the GELLO leaders. A controller holding or pressing a
  headset screen is marked `ui`: its arm lets go, its gripper is left alone (`VRArms`), and its grip must be let go
  before it attaches again.
- The head zero moves from the trigger to a thumbstick press on either controller. The trigger is the gripper's,
  and still works the picture's bar.
- `xr.py` checks the controllers (`hand_sample`: positions within 10 m, unit quaternions, grip and trigger from 0 to
  1; anything else refuses the whole pose). Helm keeps each hand's latest sample, with Helm's receive time and a
  restart epoch (`Headset.hand_samples`), for phase 4's worker. Nothing reaches the arms' loop: after the
  2026-10-31 GIL stalls, the worker reads hands in its own process. One headset owns the head and the arms. Helm's
  state shows each hand's tracking, grip, trigger and `ui`.
- **The certificate.** `helm_service status` lists the certificate's names and the addresses it misses
  (`uncovered`). `helm_service certificate [--force]` issues it again when one is missing; then run `helm_service
  restart`, and each device accepts it once more.
- **The links.** Helm's state carries `network.links` (`NetworkView.links`): the network's and the access point's
  `/xr` links, which one is live, and whether the certificate names each. Settings → Headset lists them, red when
  the certificate misses the address and muted while the access point is off, with the command that reissues the
  certificate. The headset page shows the other link ("Also at") and, when Helm stops answering, says where to go.
- **Checked offline**:
  - `test_helm_xr.py` (19 tests) and `test_helm_xr_view.mjs`;
  - `test_helm_network.py` and `.mjs`: the links, with the certificate read once;
  - `test_helm_service.py`: reissue;
  - `test_vr_arms.py` (23 tests): the `ui` hand;
  - `dev.py quick`: 137 tests.

  Settings → Headset and the headset page were also checked in a local preview. **Not yet tried on a real Quest**:
  the controller messages, the thumbstick zero and the links.

## Phase 4: Loop integration (built 2026-11-02 but the VR profile; not yet run with a Quest)

- **The IK in its own process** (`teleop/vr_worker.py`), as the head's runs since the 2026-10-31 stalls. Helm's
  loop starts `python -m robot_platform.teleop.vr_worker` and talks to it in the policy worker's length-prefixed
  frames (the PC's loop process is daemonic: no multiprocessing). It builds both arms' chains from the model and the
  receiver's usable limits, then updates on its own 20 ms grid from the newest hands and seed. Hands are stamped on
  arrival in the worker (no clocks compared); each answer names the seed it comes from, counts B/Y presses, and
  gives each fresh controller's trigger even before a seed. On the robot it runs on the background CPUs (0-3) at
  nice 10.
- **The source** (`teleop/headset_source.py`, `TeleopCore`'s `headset` kind). Idle and Hold aim where the arms are,
  each gripper at its trigger. Engage freezes that pose, but an arm hanging at the reference rises to its ready pose
  (below); the worker is seeded from the pose aligned to as following begins, and a grip held through the raise must
  be let go first, so nothing jumps. Following takes only answers from the latest seed while they are fresh (0.2 s);
  a stale answer gives no target, so the lifecycle holds, and never the seed again (that would pull the arm back).
  CAD angles become motor positions through the report's scale and offset, a trigger the gripper's position between
  open and closed; a lost hand keeps its gripper's target. B/Y while engaged is Hold (T), once a press.
- **The relay**: Helm's xr socket forwards each checked controller (`hand_sample`) to a running headset session
  (`TeleopSession.hands`, never waiting on a send), and the loop to the worker.
- **Helm**: Teleop's Input switch (GELLO / Headset) beside Mode, remembered in the browser, for Direct and
  Calibrate; a running session keeps its own. With the headset, a card shows each arm's state (Ready, Free,
  Dragging, No controller), what holds a dragged arm back (out of reach, nearly straight, at a limit, held to its
  box or speed), the IK's time and the answer's age. The leaders' trigger card goes; nothing is recorded yet;
  Calibrate's still follows a B/Y hold as it does pedal B. The live confirmations say what the headset does.
- **Jetson timing** (2026-11-02; background CPUs, nice 10, numpy single-threaded): a solve 0.71 ms from the last
  answer, 1.5 ms cold; both arms dragged at 50 Hz, 3.4 ms an update (median), 6.6 ms (99th percentile), 12.4 ms at
  most, none of 1490 over 20 ms (at 100 Hz, 2 over). **One worker serves both arms at the loop's 50 Hz.** For that,
  a solve takes at most 4 iterations an update (was 12) and the Jacobian is built with an explicit cross product.
- **The hanging pose's elbow**: from the reference pose the plain solve leaves the elbow straight or folds it back
  (J3 positive). An update that does not converge with J3 within 0.05 rad of straight solves again from J3 at
  -0.05 (`elbow_bend`: the forearm folded forward, as Minerva's arms fold) and keeps the better answer.
- **Checked offline**:
  - `test_vr_worker.py` (4, the real process among them), `test_headset_source.py` (9);
  - contract `test_headset_mode.py` (3) on the real Rust virtual receiver: Engage raises both hanging arms to the
    ready pose, a raise-and-reach drag lands within 6 mm and a 3 cm circle turns J1 under 0.2 rad, the trigger
    closes, lost hands and B/Y hold, Engage again leaves the arms where they were held;
  - `test_helm_xr.py` (20), `test_helm_teleop_mode.mjs`, `test_helm_pedals.mjs`;
  - Helm's Teleop panel in a local preview, a scripted controller driving the virtual receiver.
- **Found: the hanging pose's swing, in a session.** In that preview a 3 cm sideways circle, 6 cm from the
  reference pose, swings J1 and J5 against each other by up to 1.14 rad, within the 3 rad/s cap (the smallest
  singular value falls to 0 as the hand crosses the shoulder's vertical). From a working pose, forearms forward, the
  same circle moves no joint more than 0.39 rad (J1 0.05). Every session starts at the reference pose, so this is
  an operator's first drag (phase 2's finding).
- **The ready pose** (chosen over the gripper's orientation giving way or slower caps).
  Engage raises an arm hanging at the reference (J2, J3 and J4 within 0.25 rad of it) to its ready pose at the
  alignment speed, as a replay's first frame is aligned to; any other arm stays where it is, so Engage after a hold
  mid-task moves nothing. The pose (`headset_source.READY`, CAD joints): left (0, -0.3, -1.6, -0.7, 0, 0), the right
  mirrored (J2 +0.3). The forearm is folded forward and the gripper level, 28 cm ahead of the shoulder and 24 cm
  above where it hangs, the hands 48 cm apart. The smallest singular value there is 0.16, and J2-J4 sit 0.42 rad or
  more inside the swept limits, so a 6 cm drag any way converges with no joint at a limit. A first choice, 0.22 rad
  from the left J4's limit, stopped a 6 cm lift short. A ready pose outside the receiver's limits refuses the
  session.
  - **Rising**, the forearm swings forward before it lifts: halfway the hand is 18 cm ahead and only 7 cm up. The
    live confirmation asks for the space in front of the hands to be clear.
  - **In the preview**, the scripted circle from the ready pose kept J1 within ±0.09 rad (1.14 before).
  - **The left wrist's travel**: with the gripper level, the left arm stops at J4's limit (-1.12 rad; the right's
    is -1.49) once the hand is about 9 cm above the ready pose; the card says so. Tilting the hand goes higher.
- **Not built yet**: the VR profile with tighter speed caps (AIZEE's 3 rad/s and 0.2 rad lead as the starting
  point, sized against Minerva's).

## Phase 5: Feedback in the headset

- A ghost of the target, the workspace boxes and the clamp line, with the headset's model (`xr_model.js`).
- Later: a controller pulse when the jaws stop on an object, where the GELLO trigger has its wall.

## Phase 6: Rehearse, then live

- Offline on `simulation/arm_plant.py`, then `helm --sim` with the real Quest.
- One arm live in the cage with the operator driving, then both arms under the enforced bound.

## Open

- The tool point's fingertip offset, from the finger meshes.
- The ready pose's raise against the real table and what stands on it: the forearm sweeps forward before it lifts
  (phase 4). Not yet seen on Minerva.
- Whether the Quest's browser resolves `.local` names; until seen, the links use addresses.
