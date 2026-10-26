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

**Tune.** The Tune tab has a Head choice beside Left and Right. The arms' sweep engine runs on the head's receiver
with direct updates (no collision envelope):

1. characterise;
2. a P × D ladder in register units;
3. speeds and braking;
4. verify.

Saving writes `<asset>-head-servo.json`, bound to each servo's identity, and the next derivation uses it. The
virtual receiver uses a nominal reference until the head is set up.

**Teleop.** A teleop session runs the head's receiver beside the arms', with the same keys and states: I, E, T, X
and H act on both. The head uses the arms' direct lifecycle (`TeleopLifecycle`, `DirectLifecycleSession`), so the
arms' speed caps, alignment and braking near the ends apply. It cycles every 40 ms.

- **What it follows while the session follows (T):**
  1. the headset, while its latest pose is at most 0.2 s old;
  2. then its last target, held until that pose is 1 s old;
  3. then its zero.

  Without a headset, the head holds at its zero.
- **Faults hold both:**
  - a head fault makes the arms disengage, with a notice;
  - an arms fault disengages the head, and H disables it.
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
  - A trigger pull or a pinch makes the zero wherever the headset looks, as does Zero headset on the Teleop tab.
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
- **Inside the headset** (`static/xr_scene.js`, three.js, passthrough where the headset has it):
  - the head camera's picture on a screen ahead, with a status line under it: the picture's rate and age, and what
    the head does;
  - a status panel to the lower left: Helm's link, tracking, yaw and pitch, the head, the camera, and the controls;
  - the grip buttons grab either and move it; let go, it turns upright to face the viewer. The trigger still zeroes.
  - The picture comes over Helm's camera socket asking for the head camera only (`?roles=head`), at Helm's camera
    rate (10 fps). Only the newest frame is decoded, and it dims once a second old.
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
  - `tests/contract/test_head_teleop.py` (3) and `tests/contract/test_teleop_head.py` (3), the head in a session;
  - `tests/unit/test_helm_xr.py` (6): the gaze maths, the one-headset rule, the zero, the socket and the zero
    endpoint, and a stalled loop.

  All are in the gate.
- Browser: `tests/ui/test_helm_head.mjs`, the Head card's text, run by `tests/unit/test_browser_scripts.py`.
- Rust: 10 host tests for the Feetech hardware and the head config. The Linux service cross-builds for the Jetson.

## Bring-up on the hardware, with the user

The servo tools run on whichever computer the Waveshare board is plugged into. The board stays on the Jetson in
the end; there the tools need the release (a deploy, with the user's OK).

1. Power the board from the servos' 12 V supply, plug its USB in, and run `robot-platform neck ports`. Note the
   adapter's device, USB serial and its `/dev/serial/by-id/...` path.
2. **One servo on the bus at a time.** Both ship as id 1.
   - Run `robot-platform neck probe --port DEV` and keep the JSON.
   - Run `robot-platform neck set-id --port DEV --to 2 --confirm` for the tilt servo. The pan servo keeps id 1.
3. **Both servos on the bus, the head held straight ahead and level, torque off.** For each id, run
   `robot-platform neck centre --port DEV --id N --confirm`. Each then reads 2048 there, mid-turn, so its range
   never wraps.
4. Run `robot-platform neck save-reference --port DEV --directory <operator files> --asset minerva-production-01`.
5. Run `robot-platform neck sweep-limits --port DEV --directory ... --asset ...` and turn each joint slowly to both
   ends by hand.
6. **Optional:** `neck limits --id N --low L --high H --confirm` sets each servo's own limits, a little outside
   the swept range, as a backstop.
7. Fill the head profile's `bus` (the by-id path and USB serial), then commit and deploy, with the user's OK.
8. In Helm's Tune tab, choose Head, then try a virtual sweep, then a live one with the user present. The arms stay
   down and disabled.

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
  - If the assembled head does not rest stably at its zero without torque, give the head a rest pose of its own
    (tilted onto a stop) for Shutdown.

## Next

- **Limits tab:** a stopped Feetech reader in Helm's live link, so the head's range is swept in the Limits tab like
  the arms'. `neck sweep-limits` stands in until then.
- **Minerva Check:** the head's identity and pose checks, and its live sweep.
- **Recording:** the neck's stream beside the arms and cameras.
- **The assembled head:** centre, save-reference and sweep-limits (steps 3-5), then the Tune tab's head sweep.
- **No arm-vs-head collision checks:** the user decided (2026-10-25) the head needs none.

## Open questions

- ~~Which servo is pan and which is tilt?~~ Pan is id 1 and tilt id 2, set on the hardware on 2026-10-25.
- ~~Does the foot switch cut the head's 12 V supply?~~ No (the user, 2026-10-25): Disable is the head's stop.
