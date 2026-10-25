# Feature task: the neck's Feetech servo controller

- **Branch and worktree:** `claude/neck-feetech` in `C:\Users\ltr\Workspace\aizee-neck`.
- **Base commit:** `e0165a0` (`claude/head-model`: the Head V1 export and the
  [head and neck handoff](HEAD_NECK_MODEL.md)).
- **Owner:** Claude (2026-10-25 session).
- **Request:** "start on the neck controller with the feetech servos". The neck has two Feetech STS3250s on a
  Waveshare bus servo board, the SO-101 arm's adapter. The user wants it to "have the same limits, teleop and all
  pipeline that we use for the robstrides", with "tuning parameters extracted too".
- **This first step:** the bus driver, a simulated bus, and read-only tools. Nothing here enables torque, writes
  a register on hardware or moves a servo.

## The servos and the protocol

| | STS3250 |
|---|---|
| Model number | 2825 |
| Supply and torque | 12 V; 50 kg·cm stall (retail listings, no official datasheet found) |
| Position | 12-bit magnetic encoder, 4096 steps a turn |
| Bus | Half-duplex TTL, factory 1 Mbps, factory id 1 |

Every servo ships as id 1, so the two must get distinct ids before they share the bus.

- **Control table:** the STS/SMS series table, the same one the SO-101's STS3215 uses. EEPROM is addresses 0–39
  and persists across power cycles; writing it needs `Lock` (55) at 0. RAM starts at 40.
- **Sign-magnitude registers:** `Homing_Offset` (sign in bit 11); goal and present position and velocity
  (bit 15); `Present_Load` (bit 10).
- **Status error bits:** input voltage, angle sensor, overheat, overcurrent, overload.
- **Sources:**
  - LeRobot v0.4.0 [`motors/feetech/tables.py`](https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/motors/feetech/tables.py)
    for model numbers, encodings and baud codes.
  - rustypot [`servo/feetech/sts3215.rs`](https://github.com/pollen-robotics/rustypot/blob/main/src/servo/feetech/sts3215.rs)
    for the register map and the load sign bit.
  - Feetech's packet format, as implemented by the SO-101 leader in `python/teleop/so101_leader.py`.

## Built so far

- **`src/robot_platform/neck/feetech.py`** builds packets and holds the register table.
  - Every reply is checked strictly. Silence raises `NoReply`, alarm bits raise `ServoAlarm`, and a malformed
    reply raises `FeetechError`. Nothing retries or reconnects.
  - Operations: ping, read, `read_many` (one span), write, `sync_read` and `sync_write`.
  - EEPROM writes need `allow_eeprom=True`.
  - `detect_echo` checks once whether the adapter loops requests back.
- **`src/robot_platform/neck/sim.py`** simulates servos that answer the same bytes.
  - Its motion model slews to the goal within the position limits, speed and acceleration. It is not the real
    servo's dynamics, and its 2900 steps/s no-load speed is a stand-in until measured.
  - A hand can move a torque-off servo. Silence, corrupt checksums and alarm bits can be injected.
  - Every request is logged, so a test can prove a tool only read.
- **`robot-platform neck`** has three read-only commands that send only PING, READ and SYNC_READ:
  - `ports` lists serial ports with their USB ids.
  - `probe --port DEV [--ids 0-20]` prints what each servo that answers holds, as JSON: model, firmware, id,
    baud, limits, offset, mode, P/D/I, torque, lock, position, voltage, temperature and alarms.
  - `watch --port DEV --ids 1,2` prints positions and the range seen so far while you turn the joints by hand
    with the torque off.
- **Tests:** [tests/unit/test_neck_feetech.py](../../tests/unit/test_neck_feetech.py), 9 of them, in the gate.

## First live step (read-only, with the user)

1. Power the Waveshare board from the servos' 12 V supply, then plug its USB into the computer that will run the
   tools.
2. Run `robot-platform neck ports` to find the adapter's device and USB ids.
3. Connect **one servo at a time**. Two servos at the factory id 1 answer together, and their replies collide.
   Run `robot-platform neck probe --port /dev/ttyACM0` and keep the JSON.
4. Run `robot-platform neck watch --port /dev/ttyACM0 --ids 1` and turn that joint by hand to both ends.

The code runs on the Jetson only after a deploy, which needs the user's OK. Before that it runs from a PC with the
adapter plugged in.

## Plan

1. **Ids and safe settings.** These are EEPROM writes and need the user's OK.
   - One servo at a time on the bus: pan gets id 1 and tilt id 2.
   - Write their position limits, torque limit and protection settings, recorded in a neck servo file.
2. **Limits, as for the RobStrides.**
   - A torque-off hand sweep (`watch`) gives each neck joint's range.
   - Store it with the same margin rule as the arms' swept ranges.
   - Set the direction and zero against the CAD pose: head straight ahead and level.
3. **Motion and tuning.**
   - Position mode with speed and acceleration caps.
   - A Tune sweep for the neck: step responses over P, D and acceleration rungs, picking the stiffest rung that
     passes, written to the neck servo file.
   - Rehearsed on the simulator first, then live with the user present.
4. **Helm and teleop.**
   - Neck entries in Limits, Tune and Check.
   - Live telemetry, so the 3D view moves the neck.
   - A teleop input. The OpenRB leader frames already carry joystick values, which could drive the neck.
   - Recording: the neck's state goes into episodes only on purpose, since it changes the dataset.
5. **Collision.** The head goes into the collision model first; that model belongs to the motion-contract owner,
   see [HEAD_NECK_MODEL](HEAD_NECK_MODEL.md). Until then the neck should not move while the arms do under the guard.

## Open questions

- Which servo is pan and which is tilt, and are both still at id 1?
- What should drive the neck in teleop? The leaders' joysticks, Helm controls, the pedals, or something
  automatic such as looking at the active gripper?
