# Feature task: the neck's Feetech servo controller

- **Branch and worktree:** `claude/neck-feetech` in `C:\Users\ltr\Workspace\aizee-neck`.
- **Base commit:** `e0165a0` (`claude/head-model`: the Head V1 export and the
  [head and neck handoff](HEAD_NECK_MODEL.md)).
- **Owner:** Claude (2026-10-25 session).
- **Requests:**
  - "start on the neck controller with the feetech servos". The neck has two Feetech STS3250s on a Waveshare bus
    servo board, the SO-101 arm's adapter.
  - The neck should "have the same limits, teleop and all pipeline that we use for the robstrides", with "tuning
    parameters extracted too".
  - "Make what you need to [get] functional tuning for these servos", under "as much shared architecture as the
    robstrides as possible", and "in Helm, they should fall under the same tabs and menu".
  - "The head gets its own receiver that should be synced with the dual arms and cameras."
  - A Meta Quest Pro steers the head in teleop: "The headset should connect to a webxr service that interfaces with
    Helm (or is a part of Helm, part of the teleop tab). If there is no headset connected, the head stays static at
    the zero positions for now."

## Decisions

The user chose a separate head receiver. The defaults below were stated to the user and stand unless they say
otherwise.

1. **One shared core, its own process.** The head receiver runs the arms' Rust core (leases, limits, stops,
   protocol, recorder) behind a Feetech `Hardware`, as its own process on the Jetson (`bin/head_controller`). Helm
   drives it with the same client code.
2. **Collision.** Two receivers cannot check each other's live motion. The head's whole reachable volume, over its
   swept limits, becomes a fixed keep-out in the arms' collision model, so the head moves freely in its range and
   never meets an arm. The arms lose only the space next to the head. That collision-model change belongs to the
   motion-contract owner. Until then, the head moves only with the arms down and disabled.
3. **Sync.**
   - The arms, head and cameras run on the Jetson's one monotonic clock.
   - One Helm session drives both receivers through the same states.
   - Recordings store the neck as its own timestamped stream beside the arms and cameras. That is a deliberate
     format change, since episodes assert 14 axes today.
4. **Faults.** A fault in either receiver holds both. The head keeps its pose, since a servo holds its last goal.
   Torque is cut only on lost feedback, a servo fault or an e-stop, as for the arms.
5. **Names.** The motors are `head_pan` and `head_tilt` (the robot definition's names). They drive the CAD joints
   `neck_z` and `neck_y` (the URDF's) through the head's directions, as an arm motor drives its CAD joint.

## The servos and the protocol

| | STS3250 |
|---|---|
| Model number | 2825 |
| Supply and torque | 12 V; 50 kg·cm stall (retail listings, no official datasheet found) |
| Position | 12-bit magnetic encoder, 4096 steps a turn |
| Bus | Half-duplex TTL, factory 1 Mbps, factory id 1 |

- **Control table:** the STS/SMS series table, the same one the SO-101's STS3215 uses. EEPROM is addresses 0–39
  and persists across power cycles; writing it needs `Lock` (55) at 0. RAM starts at 40.
- **Homing offset:** Present_Position = actual position − Homing_Offset.
- **Sign-magnitude registers:** `Homing_Offset` (sign in bit 11); goal and present position and velocity
  (bit 15); `Present_Load` (bit 10).
- **Status error bits:** input voltage, angle sensor, overheat, overcurrent, overload.
- **Temperature glitches under torque:** in the head's first live jog (2026-10-31) the pan servo answered 150 °C,
  in valid replies, between 34 °C ones: four times in 2 s while the tilt servo lifted the head, each stopping the
  head at the receiver's 65 °C limit. Limp, 279 reads in 3 s held 33–34 °C at 12.1 V. The head's adapter
  (`feetech::TemperatureCheck`) holds back a reading more than 5 °C from the last one taken until 5 readings in
  a row (50 ms) agree with it, so a real rise still reaches the limit; the servo's own overheat alarm still faults
  at once. The service's status and exit summary (`live-head.log`) count what it held back (`temperatures`).
- **Sources:**
  - LeRobot v0.4.0 [`tables.py`](https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/motors/feetech/tables.py)
    and [`feetech.py`](https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/motors/feetech/feetech.py)
    (the homing convention);
  - rustypot [`sts3215.rs`](https://github.com/pollen-robotics/rustypot/blob/main/src/servo/feetech/sts3215.rs).

## Built so far

| Piece | Where | What it does |
|---|---|---|
| Head receiver | `rust/control_core/src/feetech.rs`, `head_config.rs`, `head_service.rs`, `bin/head_controller.rs` | The core behind `FeetechHardware`, described below |
| Python driver | `neck/feetech.py`, `neck/sim.py`, `neck/probe.py` | The same protocol for tools; a byte-level simulator |
| Setup | `neck/setup.py`, `robot-platform neck ...` | Ids, centring, servo limits, the head's files, described below |
| Derivation | `config/devices/minerva-head.json`, `neck/head.py` | The arms' `derive_receiver`, described below |
| Tune | `helm/tune.py`, the Tune tab | The arms' sweep engine on the head, described below |
| Release | `ops/release.py` | Carries `bin/head_controller` and the head model's URDF, its mesh digests in the asset lock |
| Teleop | `helm/head_teleop.py`, `helm/teleop.py` | The head in the arms' session, described below |
| Headset | `helm/xr.py`, `static/xr.html`, `static/xr.js`, the Teleop tab's Head card (`static/head.js`) | The Quest steering the head, described below |

**Head receiver.** It runs the arms' receiver core behind `FeetechHardware`.

- **Command mapping:**
  - position goes to Goal_Position;
  - kp and kd go to the P and D registers, written with the EEPROM locked so they never persist;
  - zero gains mean torque off, the core's limp Idle;
  - velocity and effort are not sent, since a position servo takes neither.
- **Feedback:** position, speed, load as effort, and temperature. Any alarm bit faults the joint, and a servo that
  reboots while driven reads as dropped.
- **Startup gates:** the servos' model and firmware, then the arms' reference-pose gate, before the socket opens.
  There is no command watchdog: a servo holds its last goal.
- **Modes:** `virtual_bus`, `observe` and `teleop_direct`. The physical bus is pinned by its adapter's
  `/dev/serial/by-id` path.

**Setup.** Every step needs the torque off and exactly the expected servo answering.

- `set-id`, `centre` and `limits` write EEPROM, so each needs `--confirm`. Each verifies what it wrote, undoes a
  centring that does not read back, and locks the EEPROM again.
- `save-reference` and `sweep-limits` only read the servos. They write the head's files beside the arms' servo
  calibration, in the arms' formats.

**Derivation.** The head profile reads as a teleop profile. The head derives through the arms' `derive_receiver`
with collision checks off: the swept range plus overtravel, the approach margins, speed headroom, the hold gains
and the servo calibration all apply unchanged. The receiver works in raw servo radians. Commands come at 25 Hz;
the receiver ramps each over 40 ms, so the servos get 100 Hz goals.

**Tune: a jog, never a sweep.** The user, 2026-10-29: "do not sweep the head. the sweep test for the head should
only be a slow jog to its limits, one at a time from their center reference, because at some angles if the head is
angled all the way down and rotated to the side, it will hit its shoulder." The Tune tab's Head choice runs the
arms' sweep engine in its `jog` mode (`servo_sweep.SweepEngine._jog_joint`) on the head's receiver, with direct
updates and the head's own settings (nothing raised to try):

1. each joint alone, the other held at its reference;
2. from the reference slowly (`characterize_speed`, 0.15 rad/s) to one end of its usable range, held 1 s, back;
3. the same to the other end.

The results say how far each end was reached and the worst lag. Nothing is saved: the head keeps its profile's
settings, and `TuneCore` refuses any other mode on the head (and the jog on an arm).

**Shoulder clearance (coupled limits).** Turned to a side and tilted all the way down, the head reaches its shoulder.
The per-joint limits, the servos' own among them, are a rectangle that holds those corners, so how far down the head
may tilt depends on how far it is turned, as NAO's head pitch range narrows with its yaw (`neck.clearance`):

- Limits → Head's last step: with the servos limp, turn the head to a side and tilt it down until it is just clear of
  the shoulder, Record corner; record each side's end and halfway; Save shoulder clearance. The corners go into the
  limits file (`clearance`: the corners in radians from the reference, the down side, `margin_rad` 0.1), so a new
  reference or new limits drop them.
- The floor (how far down at a pan) is the whole tilt range straight ahead, runs straight to each corner less the
  margin, and never eases off further out. The region allowed only shrinks outward and downward, so Return, which
  moves each joint toward its zero at its own speed, stays in it.
- Head teleop clamps each headset target onto the floor (pan kept) and steps toward it in moves the lifecycle takes
  whole (90 % of its speeds per 40 ms cycle), each clear: the lifecycle moves each joint at its own speed, so one long
  move toward a clear pose could cut through the corner below it. Aligning (Engage), the head holds where it is;
  following, it goes at the alignment speed until it first reaches its target, then at its own speed. A head left
  below the floor rises straight up first.
- Live sessions take the head only once its clearance is saved (`TeleopSession._head_bundle`); virtual ones always.

**Where the head rests.** The user, 2026-10-30: "the head cannot stay straight and ahead. there should be an
"engage" phase for the head as it will have a starting position and resting position where it currently is." Limp,
the head droops. So a live head session or jog starts where it rests:

- Just before the head's receiver starts, Helm reads the servos limp (`neck.head.read_rest`). The receiver's start
  check then takes that pose (`service_config(start=...)`, tolerance 0.05 rad, the head still) in place of the
  reference. A reading more than 0.1 rad outside the saved limits starts nothing.
- Engage raises it: the jog's first move takes each joint from where it rests to its reference at 0.15 rad/s; in
  teleop, Enable holds it where it rests and Engage (T) steps it toward the headset at the alignment speed.
- Return (X), the end of a jog, a stopped jog and a session's close take it back to its reference first, then down
  to where it rested (inside its usable range and clear of the shoulder), then disable it: it is never dropped from
  straight ahead. Each stage is a straight move toward or away from straight ahead, so neither crosses the shoulder
  corner. In teleop that is `HeadTeleop.returning` ('centre', then 'rest'), not the lifecycle's own Shutdown; X with
  the head limp (Idle) just disables it, and H still cuts its torque at once.
- A virtual session has no head to read: it rests where the real head droops, tilted down to the end of its range
  on the shoulder's down side (its saved clearance gives the side), so it starts there and Return comes back there
  (the user, 2026-10-31: it had ended straight ahead, where the real head would fall). With no clearance saved yet
  it rests at its zero.

**Live · Head.** The Teleop tab's Head choice (`helm.head_only.HeadOnlyCore`) runs the head alone under the session's
keys and pedals B and C, steered by the headset. The arms' receivers are never opened: their drives stay as they are,
and Helm's live view keeps reading them. No leaders, bridge, task or arm limits are needed; a replay or a policy
refuses it. A head disabled after it was powered needs a new session, as the arms do.

**Teleop.** A teleop session runs the head's receiver beside the arms', with the same keys and states: I, E, T, X
and H act on both. The head uses the arms' direct lifecycle (`TeleopLifecycle`, `DirectLifecycleSession`), so the
arms' speed caps, alignment and braking near the ends apply. It cycles every 40 ms.

- **Its speed:** it follows at most 0.7 rad/s (`FOLLOW_SHARE`, 0.7 of its profile's 1.0), catching up included;
  with its shoulder clearance saved, its targets step at 0.63 rad/s. Its receiver keeps its own cap, 1.25 rad/s.
  Coming down at 1.0 rad/s, the tilt read 1.30 rad/s, and the head tripped (2026-11-02, `sustained_speed`).

- **Its own process, live:** each exchange with the head's service waits for the service's next 10 ms tick. In the
  arms' 100 Hz loop, a head cycle (two exchanges) took 22 ms of every 40 ms: the arms' commands went out 10 and
  30 ms apart, each 30 ms long, and Enable faulted (2026-10-31, "command_expired · wrong_owner"). A live head now
  cycles in a process of its own (`helm.head_teleop.HeadProcess`, its child `serve_head`): the arms' loop posts it
  their actions and the headset's poses, and reads its last status, and never waits for it (a sender thread and a
  reader thread do the pipe's work). The child moves itself, and so its service, to the background CPUs before it
  builds the head, and cycles it there on its own 40 ms grid (`HeadLoop`). A fault there, or the child dying,
  holds the arms as an inline head's fault does. A virtual head answers at once and cycles on the arms' ticks.
- **Why a process, not a thread** (2026-10-31): on a thread the head shared the arms' interpreter.
  - First, the head's thread ran on the control CPUs beside the arms' loop and receiver. The arms' worst cycles got
    slower: exchanges up to 10.6 ms (2-3 ms alone), and a command came within 2 ms of expiring.
  - Moved to the background CPUs (cameras, the relay, Helm), with the interpreter changing hands after 1 ms instead
    of 5, the faults went on: 15:15 (blamed then on the CPUs' deep idle state, c7, since disabled) and 15:53,
    after 2.4 minutes of TELEOP (`live-20261031T195313Z`). The failing cycle mapped its leader sample on time but
    sent its command 11 ms late, while the arms' receiver ticked regularly. The arms' slow cycles kept time with
    the head's 40 ms cycle (phase-locked, R 0.71; 0.67 in the clean 12-minute session before it, which came within
    1 ms of the same fault). The 1 ms switch interval cannot help a thread holding the interpreter while it is not
    running, as a thread preempted on busy CPUs is.
  - Offline, a head holding its interpreter for 8 ms of each answer slowed 51 of 200 arms ticks past 8 ms on a
    thread (worst 16 ms), and at most 3 in its own process.
  - The arms' process still runs threads beside their loop (the views' sender, the recorder, the head process's
    proxy): beside a live head the interpreter still changes hands after 1 ms.

- **What it follows while the session follows (T):**
  1. the headset, while its latest pose is at most 0.2 s old;
  2. then its last target, held until that pose is 1 s old;
  3. then its zero.

  Without a headset, the head holds at its zero.
- **Faults hold both:**
  - a head fault makes the arms disengage, with a notice;
  - an arms fault sends a powered head back as Return (X) does, to its reference, then down to where it rested,
    and disables it there; the session closes only once it has. Before, the head only held, and closing the
    session cut its torque wherever it was (2026-10-31). H disables it at once.
- **Start-up:** a head that fails to start shows as a head fault, and the arms run on.
- **Receivers:** a virtual session gets a virtual head. A live session gets the physical head only when the head's
  files exist and its bus is pinned.

**Headset (Meta Quest Pro).** Helm serves `/xr`, a WebXR page for the Quest's browser.

- **What it sends:** the headset's orientation in the level `local` space, at most 60 times a second, over
  `/api/helm/xr`. `helm.xr` turns it into yaw (left positive) and pitch (up positive) from a zero, and the running
  session's head takes them: yaw drives `neck_z` and pitch drives `neck_y`, through the head's directions, clamped
  to the usable range.
- **The zero:**
  - Starting head tracking, or recentering the headset (hold the Meta button), sets the zero at the headset's
    heading, level.
  - A controller's trigger pull makes the zero wherever the headset looks, as does Zero headset on the Teleop tab. A
    pinch does not (since phase 3 tracks the hands for the wrist cameras: holding the leaders, they pinch by accident,
    and a zero moves the head); nor do hands point at the screens.
- **Who may steer:**
  - The socket needs a logged-in device, Helm's action token and the same origin.
  - It does not need control: the session that the device in control engages decides whether the head moves.
  - One device's headset steers at a time. Another device is refused while it is connected; the same device
    reconnecting takes its stream back.
  - Zero headset on the Teleop tab is a change, so it needs control.
- **A stalled loop gets no poses:** poses go only to a loop that answered within 0.75 s, and never wait for another
  send. A stalled loop's pipe therefore never fills with poses, which would block Helm's sends, Disable's among them.
- **The Head card,** under the gripper card, says:
  - what the head does: its state, whether it follows the headset, holds or sits at its zero, or its fault;
  - whether a headset tracks: its name, rate, yaw and pitch;
  - without a headset, the address of the headset page.
- **Inside the headset** (`static/xr_scene.js`, three.js, passthrough where the headset has it; since 2026-11-03 the
  status line, the status panel and the bar below gave way to a ring of controls on A or X, and the wrist pictures
  sit on the model's wrists: docs/development/HEADSET_3D_PLAN.md, "The ring of controls"):
  - the head camera's picture on a screen ahead, with a status line under it: the picture's rate and age, and what
    the head does;
  - a bar under that (the user, 2026-10-31: looking around, the picture was somewhere else):
    - Follow head: on, the picture stays in front of the eyes as the head turns, level (its yaw and pitch; roll
      left out, as the robot's head keeps its camera); off, it stays where it is. On by default.
    - 3D (the user, 2026-10-31: "it is very hard to gather a sense of depth from the 2D image"): the head camera's
      depth instead of the flat picture, below. Off by default. While a session drives the head, it stays where the
      head camera saw it in the room, whatever Follow head says.
    - Model: Minerva's model with the world-locked 3D (`static/xr_model.js`), the display model's meshes made coarse
      (`?detail=coarse`, 89 k triangles), its head left out, posed by Helm's guide and placed as the 3D is. On by
      default.
    - Hands: the wrist cameras' pictures above the tracked hands, facing the eyes (`static/xr_hands.js`; the session
      asks for hand tracking; without it, the controllers carry them). On by default; the two streams add about
      1 MB/s to the Quest's Wi-Fi.
    - On a second row, two sliders, 0 to 100 % in 5 % steps: Picture, for the room to show through the picture
      (flat or 3D) and the model; and Hands, the wrist cameras' own (the user, 2026-10-31: theirs set apart).
    - The headset's browser remembers them (`static/xr_view.js`), and where the picture sits while it follows.
  - **The 3D picture** (`static/xr_depth.js`, phase 1 of the plan: depth images to the headset, 3D built there):
    - The head camera service already publishes its depth beside the picture (640×480 at 15 fps; the D435 makes
      it). Only while a headset asks (`?roles=head_depth`), Helm's camera relay on the Jetson takes each sent frame's
      depth: the nearest reading of each 2×2 block, readings outside 0.15–2.5 m dropped, zlib level 1 (about 53 KB
      a frame, 2-6 ms on the background CPUs, where the relay keeps), after the frame's JPEG.
    - The headset inflates and fills it in a worker (`static/xr_fill.js`, phase 2; about 3 ms a frame on the laptop,
      off the headset's drawing), and a grid with a vertex per reading (320×240) is placed on the GPU from the
      camera's intrinsics, coloured from the very picture it came with through the camera's registration (its color
      intrinsics and the depth sensor's pose in the color camera, published by the camera service since `6aeeb56` on
      `claude/head-depth-registration`). Where the depth sees wider than the color camera, or a camera publishes no
      registration, it is grey, lighter nearer.
    - At a depth edge (a neighbour more than 4 % of the depth nearer) the far reading is cut: near objects keep
      their outline, nothing joins near to far.
    - **The fill** (each pixel keeps the pixel its colour comes from, drawn from that one's own reading):
      - the sensor's small dropouts take the farthest reading within 2 pixels;
      - a pixel the colour camera cannot see (beside the depth sensor, it sees past a near object's edge from a
        little to the side) takes its colour from the nearest one along its row on its own surface that it can:
        before, the table beside an object's edge took the object's colour;
      - behind each near object's edge, a background layer, drawn behind it: the far surface carried up to 40 pixels
        under the object as the plane it lies in (straight in inverse depth). From the camera's viewpoint the object
        hides it; leaning, or with the robot's head turned, it fills what the object hid, a little darker (guessed).
    - Behind it all, the colour picture itself, 3 m ahead of the colour camera (past the depth's 2.5 m range,
      `BACKDROP_M`), fills where the depth has no reading: too far, or beyond the background layer's reach (the
      user, 2026-10-31).
    - Helm's own camera cards ask for every camera; `head_depth` goes only to a viewer that names it (they read
      "Invalid camera stream" while a headset showed 3D, 2026-10-31).
    - **World-locked** (phase 2, while a session drives the head): the head's teleop publishes the head's measured
      pose on a local feed (`neck/head_pose.py`, `tcp://127.0.0.1:5596`): every row of the head service's 100 Hz
      record, at the Jetson's wall time, as the neck's CAD angles and the colour camera's pose in the robot frame
      (forward kinematics on the head model, the camera at the `AWG_RGB_CAMERA` centre: the head profile's `camera`).
      The relay stamps each depth frame with the sample nearest its capture (its timestamp less 20 ms; none within
      50 ms, none). In the headset, Helm's zero (the yaw and pitch the operator looked at as they zeroed the head)
      and the eyes' position then anchor the robot's frame in the room: the head camera at its zero is where the
      eyes were, looking where they looked. Each frame then goes where the head camera saw it from: it stays put as
      the eyes turn, a lagging head only delays the newly seen parts, and leaning shows parallax. Zeroing again
      anchors it again. A zero pitched down or up tilts the room by as much, as the head follows the pitch from
      there.
    - Without the head's pose (no session, or a virtual head), its rig is the head camera at the operator's eyes,
      turned with their look while Follow head is on and held where it is while it is off, as before. The panel's
      3D row says which.
  - a status panel to the lower left: Helm's link, tracking, yaw and pitch, the head, the camera, the 3D picture
    (off, waiting, fixed in the room, or around the eyes), and the controls;
  - the grip buttons grab either and move it; let go, it turns upright to face the viewer. Following, the picture
    keeps the place it was let go of, in front of the eyes. A trigger pulled on the bar works its controls (drag along
    the slider); pulled anywhere else, it zeroes.
  - The picture comes over Helm's camera socket asking for the head camera only (`?roles=head`, or `head_depth`
    with its depth), at Helm's camera rate (10 fps). Only the newest frame is decoded, and it dims once a second old.
- **Settings → Headset:**
  - the headset's state and Zero headset, as on the Head card;
  - **Preview in the 3D view:** while no session drives the head, the 3D view's head turns where the headset looks,
    on every device's view, and its badge says so. No motors move. The switch is Helm's, kept in memory and off at
    each start; it is a change, so it needs control. A session that drives the head shows its measured angles
    instead.
  - **What the headset shows:** the headset's screen, status line and status panel, drawn with the headset's own
    canvases (`static/xr_panels.js`) from Helm's state. The head camera streams only while the section is on screen.

**Using it.** The Quest joins Minerva's Wi-Fi.

1. In the Quest's browser, open `https://192.168.8.180:8760/xr`, Minerva's Wi-Fi address, which the certificate
   names.
2. Accept Helm's self-signed certificate once.
3. Log in. Helm names the device "Quest headset" and comes back to `/xr`.
4. Look straight ahead and level, and press Start head tracking. The page asks for passthrough, falling back to VR.
5. The head follows once the session is engaged (T).

**Tests:**

- Python:
  - `tests/unit/test_neck_feetech.py` (13), `tests/unit/test_neck_head.py` (3);
  - `tests/contract/test_helm_tune_head.py` (4), which includes a whole head sweep on the real core;
  - `tests/contract/test_head_teleop.py` (13) and `tests/contract/test_teleop_head.py` (11), the head in a session
    (a live head's process: the arms never wait for it, its fault or death holds them; its pose feed; and its
    speed, under its receiver's cap with room for the tilt's overshoot);
  - `tests/unit/test_head_pose.py` (5): the camera's pose from the neck's angles, and the feed's rows and clock;
  - `tests/unit/test_head_alignment.py` (8): the head camera's alignment on the table's tags (the solve, what the
    head alone cannot tell, the saved mount the feed takes only for its head reference), and
    `tests/unit/test_helm_head_alignment.py` (7), Helm's flow for it (HEADSET_3D_PLAN.md, Alignment);
  - `tests/unit/test_head_loop.py` (4), a live head's cycles on its own grid (in its process);
  - `tests/unit/test_helm_xr.py` (6): the gaze maths, the one-headset rule, the zero, the socket and the zero
    endpoint, and a stalled loop.

  All are in the gate.
- Browser: `tests/ui/test_helm_head.mjs`, the Head card's text, and `tests/ui/test_helm_xr_follow.mjs`, the headset's
  picture on the real three.js (following the head or staying, its bar's switch and slider, a trigger on the bar not
  zeroing, the remembered view), `tests/ui/test_helm_xr_depth.mjs`, the 3D picture (the head_depth packet,
  inflating, the camera's role switch, the shader's placement and colouring, the bar's 3D switch),
  `tests/ui/test_helm_xr_fill.mjs`, the fill (dropouts, the colour camera's hidden pixels, the background layer and
  its plane, the worker), `tests/ui/test_helm_xr_world.mjs`, the world-locked rig (the zero's anchor, a panned
  head, the scene keeping a frame put as the eyes turn, the model placed by it), and `tests/ui/test_helm_xr_model.mjs`,
  the model (coarse meshes, the head left out, the guide's poses, a failed load retried) and the wrist cameras above
  the hands, run by `tests/unit/test_browser_scripts.py`. The coarse meshes: `tests/unit/test_model_viewer.py` and
  `tests/unit/test_helm_on_robot.py`. The relay's
  depth, its head pose stamp and Helm's head_depth role: `tests/unit/test_helm_camera_video.py`.
- Rust: 10 host tests for the Feetech hardware and the head config. The Linux service cross-builds for the Jetson.

## Bring-up on the hardware, with the user

The servo tools run on whichever computer the Waveshare board is plugged into. The board stays on the Jetson in
the end; there the tools need the release (a deploy, with the user's OK).

1. Power the board from the servos' 12 V supply, plug its USB in, and run `robot-platform neck ports`. Note the
   adapter's device, USB serial and its `/dev/serial/by-id/...` path.
2. **One servo on the bus at a time.** Both ship as id 1.
   - Run `robot-platform neck probe --port DEV` and keep the JSON.
   - Run `robot-platform neck set-id --port DEV --to 2 --confirm` for the tilt servo. The pan servo keeps id 1.
3. **The head's setup is in Helm: Limits tab, Head** (`helm.head_setup`, on Helm on Minerva, where the board is).
   The user's rule (2026-10-29): calibrations and reference saves run only through flows in Helm, so they can be
   redone at any time. The servos stay limp throughout; each step can be done again.
   - **Read the head.** A limp reader on the profile's pinned bus (10 Hz, health once a second); the 3D view's head
     follows it, each reading a pose of its own. It stops for any session that needs the motors.
   - **Set the reference…** With the head held straight ahead and level: both servos centred (each reads 2048,
     mid-turn), then `<asset>-head-reference.json` saved. If either fails, both offsets go back. Directions
     checked before carry over for the same servos. Saved limits then belong to the old reference and must be
     swept again.
   - **Directions.** Turn each joint: if the 3D view's head turns the other way, Flip; then Looks right.
   - **Limits.** Turn each joint to both ends and hold there (the arms' recorder), Confirm both, Save head limits:
     `<asset>-head-limits.json`, naming its reference, and each servo's own limits set 0.05 rad outside, a
     backstop under Helm's.
   - Replaced files are kept under the runs folder's `head-setup-*`.
   The CLI's `centre`, `save-reference`, `sweep-limits` and `limits` remain for the bench and tests; on Minerva,
   use Helm.
7. Fill the head profile's `bus` (the by-id path and USB serial), then commit and deploy, with the user's OK.
8. In Limits → Head, record the shoulder corners and save the clearance.
9. In Helm's Tune tab, choose Head and run the jog, virtual first, then live with the user present. The arms stay
   down and disabled.
10. Teleop → Head (Live · Head) with the headset, before the head joins Live · Both.

**Done on Minerva, 2026-10-25 (bench, the head still printing):**

- **The board:** on the Jetson, a CH343 (`1a86:55d3`, USB serial `5A68010635`) at
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A68010635-if00`, on the onboard hub the CAN adapters share. The
  head profile's `bus` is pinned to it.
- **The probe** found two STS3250s (firmware 3.9, 1 Mbaud, 12.1 V, 24-25 °C, torque off, no alarms) at ids 1 and 11.
  Neither was fresh: both carry offsets and limits from earlier use (id 1: offset −219, limits 360..2145; id 11:
  offset 2047, limits 1050..3730). Centring and limits replace them on the assembled head.
- **Which is which:** the user turned the tilt servo; id 1 moved (1783 → 1307 steps), so the tilt was id 1 and the
  pan id 11.
- **Renumbered** with both on the bus, each step with its source id answering, its target id free and the servo
  limp, as `set-id` does: the tilt 1 → 2, then the pan 11 → 1. The bus now holds ids 1 (pan) and 2 (tilt), both
  locked.
- **Found doing it:** an EEPROM write is acknowledged only once stored, after the driver's 20 ms reply window. The id
  write's late acknowledgement (from the old id, `ff ff 0b 02 00 f2`) was read as the reply to the next ping: a
  "malformed reply", although the write had taken. `set-id`, `centre` and `limits` would hit it too. EEPROM writes
  now wait up to 100 ms for their own acknowledgement (`feetech.EEPROM_ACK_S`).
- **First motion (the user asked):** both servos, torque limit 40 %, to their mid-turn reading 2048 (pan from 2224,
  tilt from 1307), held 2 s, then limp; they stayed at 2048. Two surprises:
  - A speed register of 400 gave about 1200 steps/s (there in 0.62 s, not 2 s): the STS3250's speed unit is not
    the simulator's steps/s. The receiver sends goals only, so it does not depend on it.
  - The tilt reached the 40 % torque limit while moving (the pan 10 %), so it was either the start or a load on its
    horn. It held at rest with no load.
- **Second motion, the user watching ("they moved smoothly"):** from 2048, one at a time, the pan ±300 steps and
  back, then the tilt −300 and back, each held 1 s, then limp at 2048. With the speed register at 100, the tilt
  moved at 100 steps/s, but the pan at 280-390 (peaks 550-750). Their registers differ only in id, limits and
  homing offset. The pan's leftover offset (2047) puts its encoder at 4095, the wrap, where it reads 2048: its
  speed estimate may be off across the wrap. Check again once centring has rewritten the offset.

**To verify on the first live run:**

- Gains written with the EEPROM locked take effect: read P back after the receiver's first gain write. They are
  lost at power-off by design.
- The head's directions against the CAD (the sign).
- **The foot switch does not cut the head's 12 V supply** (the user, 2026-10-25). The user wants Disable to stop the
  head and Shutdown to bring it to a stable pose, which is what the lifecycle does:
  - Disable (H) cuts the head's torque at once, in any state.
  - Shutdown (X) brings each joint to its zero (the reference pose: straight ahead, level) at the shutdown speed,
    then disables once it settles.
  - The assembled head does not rest at its zero without torque (it droops), so Shutdown ends where the head
    rested when the session began (see "Where the head rests").

## Next

- **Limits tab:** a stopped Feetech reader in Helm's live link, so the head's range is swept in the Limits tab like
  the arms'. `neck sweep-limits` stands in until then.
- **Minerva Check:** the head's identity and pose checks, and its live sweep.
- **Recording:** the neck's stream beside the arms and cameras.
- **The assembled head:** reference and limits saved in Helm (2026-10-30 02:02Z); then the shoulder corners, the
  Tune tab's jog and a Live · Head session with the headset (steps 8-10).
- **No arm-vs-head collision checks:** the user decided (2026-10-25) the head needs none.

## Open questions

- ~~Which servo is pan and which is tilt?~~ Pan is id 1 and tilt id 2, set on the hardware on 2026-10-25.
- ~~Does the foot switch cut the head's 12 V supply?~~ No (the user, 2026-10-25): Disable is the head's stop.
