# Teleop latency, end to end

How a leader motion becomes arm motion, hop by hop. The first section is the production path today; the rest is the
2026-10-04 analysis of the guarded 25 Hz path, which explains where each hop's time goes and why the loop was
changed. Each number is labelled **measured** (with what was measured), **recorded** (from a session log), **from the
code**, or **from the profile**. No end-to-end figure has been measured for the production path.

## The production path (direct profile, loop on the Jetson)

```text
leader servos -(Dynamixel sync-read)-> OpenRB board -(USB CDC)-> operator-station sampler (200 Hz, GELLO bridge)
  -(signed UDP request/reply over the tether)-> Jetson loop process (100 Hz, CPUs 4-5): map, lease, move
  -(ZMQ, 127.0.0.1:5590)-> guarded_controller (2.5 ms ticks) -(SocketCAN)-> drive -> joint
```

| Hop | What changed since the guarded analysis | Basis |
|---|---|---|
| Sampler cache | 200 Hz instead of 100 Hz; the bridge runs it in its own process, so Helm moving to the Jetson did not touch this hop | leader sampler rate from `teleop/gello_bridge.py`; timing unchanged by design |
| Link | Same UDP request/reply, bridge to loop; the bridge reaches Helm over SSH but the leader link does not ride it | from the code (`teleop/leader_link.py`) |
| Loop sampling | One 10 ms cycle instead of 40 ms | from the profile: `loop_hz` 100 |
| Loop work | No collision query; direct lifecycle. The arms' tick took 5.6 ms median of the 10 ms cycle in a 12-minute two-arm session with the head | recorded: live session 2026-10-31, see [HELM_ON_JETSON.md](HELM_ON_JETSON.md) |
| Interpolation | None | from the profile: `interpolation_ms` 0 |
| Receiver | Still one request per 2.5 ms tick, applied on the next tick; command lease 150 ms, direct command freshness 30 ms | from the code and profile |
| Servo and arm tracking | Set by each joint's gains; unchanged | recorded: servo sweeps |

Leader-link timing is recorded per session as each ends (report written by Helm; the bridge sends back counts and
timing). In two virtual sessions over the tether, 25,657 requests at 100 Hz went unskipped with one 20 to 50 ms gap, and
the station answered within 0.4 ms (recorded, 2026-10-24). The CPU partition ([JETSON_CPU_SPLIT.md](JETSON_CPU_SPLIT.md))
exists because camera load on shared cores stretched loop cycles past the command lifetime.

## The guarded 25 Hz path (analysis of 2026-10-04)

## The path

```
leader servos ─(Dynamixel sync-read)→ OpenRB board ─(USB CDC)→ PC sampler (100 Hz cache)
  ─(UDP request/reply, tether)→ Jetson teleop loop (25 Hz): map, lease, collision check
  ─(ZMQ, local)→ receiver (400 Hz ticks): interpolate over 40 ms
  ─(SocketCAN, USB-CAN)→ drive (MIT law) → joint
```

| # | Hop | Typical | Worst | Basis |
|---|---|---|---|---|
| 1 | Servo position to board sample: one Dynamixel sync-read of 7 XL330s at 1 Mbaud, each servo replying after its default 500 µs Return Delay Time | 2.5 | 5 | measured: board sample to reply 5 ms |
| 2 | Board to PC: 50-byte USB reply, Windows serial driver | 1 | 2 | measured: poll of both boards p50 6.0, p99 7.9 ms |
| 3 | PC sampler cache: polled at 100 Hz, served from the latest pair | 5 | 10 | from the code (`LatestLeaderInputs`, 1 ms timer resolution) |
| 4 | Loop sampling: the Jetson asks once per 40 ms cycle | 20 | 40 | from the code (`TeleopCore.period_s`) |
| 5 | Link, one way, UDP over the USB tether | 0.5 | 1 | measured: round trip p50 0.9, p99 1.9 ms |
| 6 | Loop work from the leader reply to the move sent: map; renew (two requests); collision offer; lifecycle; collision query; move | 11 | 30 | recorded: live cycle 13 ms typical, 34 ms max; virtual 6.5 / 14.4 ms |
| 7 | Receiver: the move waits for the next tick to be handled, then the tick after to reach the drives | 3.75 | 5 | from the code (`service.rs`: one request per 2.5 ms tick; `handle_deferred` applies on the next tick) |
| 8 | Interpolation: the command ramps from the previous target to the new one over 40 ms, one loop period | 40 | 40 | from the code (`follow_targets`) |
| 9 | CAN to the drive (7 frames per bus per tick) | 0.5 | 1 | from the code |
| 10 | Servo and arm tracking: lag behind the command at speed, with velocity, friction, gravity and acceleration fed forward | 15 | 25 | recorded: sweep lag 0.01–0.035 rad at 0.5–1.5 rad/s |
|   | **Leader motion to arm motion** | **≈ 100 ms** | **≈ 160 ms** | |

Four hops dominate: the loop's sampling (4) and its interpolation (8), both one
40 ms period; the loop's work (6); and the servo's tracking lag (10).

**Age as the loop counts it.** The loop measures a sample's age at about 18 ms,
not the ≈ 9 ms physical age of hops 1–3 and 5. The difference is deliberate
bookkeeping. The PC counts the board's own sample-to-reply time plus the whole
USB request-to-reply time, which counts the request leg twice. The Jetson then
adds the whole link round trip. That bounds age without comparing the two
computers' clocks; the 50 ms age limit is checked against this count.

**The receiver round trips.** `GuardedControllerClient.renew()` is two requests
(offer, renew), so a teleop cycle makes four. The receiver takes one request per
2.5 ms tick, so a request waits 1.25 ms on average. That waiting is most of the
difference between the live cycle (13 ms) and the virtual one (6.5 ms).

## What changes, and what it saves

| Change | Hops | Saves (typical) |
|---|---|---|
| A move that carries a collision verdict also renews the lease with that offer. A cycle with a move makes two requests (offer, move) instead of four; one without a move renews as before. | 6, 7 | ≈ 3–5 ms of the cycle |
| Loop at 50 Hz, interpolation 20 ms | 4, 8 | ≈ 30 ms (10 + 20) |
| PC leader sampler at 200 Hz | 3 | ≈ 2.5 ms |
| Leader firmware: Return Delay Time 0 on every servo (needs a reflash) | 1, and allows a faster sampler | ≈ 2–3.5 ms |

Together that is about 60 ms typical, down from about 100. The 50 Hz loop needs
its cycle p99 under 20 ms, which the halved receiver round trips are for. 100 Hz
(another ≈ 15 ms) needs collision compute p99 under 5 ms and the whole cycle
under 10 ms. Measure it at 50 Hz first.

Not changed here: hop 10, the servos' own lag, is set by the gains each sweep
found, and predicting the leader's motion ahead is left out for now.

**Status (2026-10-04).** The first three are in: a move renews the lease with
its offer, the loop runs at 50 Hz with a 20 ms ramp (the period now comes from
the profile's `interpolation_ms`), and the PC samples the leaders at 200 Hz.
The table above describes the loop before them. The firmware change waits for a
reflash of the leader boards.

Running at 50 Hz changed three things counted in cycles, which are now in time:
an engaged arm rides out 80 ms of missed leader samples (four cycles, two
before); alignment judges settling speed over at least 40 ms (one 20 ms cycle
read the drives' position steps as twice the speed); and a cycle faults the
session only past 35 ms (a slower one starts the next late).

## Two arms in the guarded profile (built 2026-10-05)

This section describes the collision-gated profile. The production direct profile has no collision bound; it sizes nothing
from a motion envelope.

- **One receiver for both arms.** It runs both CAN buses (7 drives each) and
  all 14 motors in one tick, so one collision verdict covers both arms
  (`LiveReceiver` with both buses; the receiver allows two buses only under the
  motion bound below).
- **An enforced arm-against-arm bound** (`collision::Bound::Motion`,
  `MotionEnvelopeCompiler`). The reserve bound sizes each joint's box by its
  *maximum* speed over the verdict's lifetime: too wide to certify one arm
  against the other. The motion bound sizes it from the move commanded: the
  reference the receiver drives (the command in force, then the new ramp), plus
  a tracking allowance that grows with the joint's speed bound (half its tracking
  bound at rest), plus on each side its stop from its speed bound that way
  (1.25 × the commanded speed, falling no faster than its stop acceleration,
  floor 0.5 rad/s). The receiver enforces the allowance and the speed bound
  every tick, and reports the bounds so the client sizes boxes from them.
- **The certification** is one proximity query at the box's centre plus a
  per-part travel bound from each joint axis's distance to the part
  (`JointBoxBound`). The earlier bound charged a gripper part a 2.2 m lever (its
  mesh frame sits at the assembly origin) and certified ±0.011 rad per joint at
  the hanging pose; this one certifies ±0.075 rad there, about 2 ms a box on the
  PC.
- **Blocked moves** are tried at half and a fifth of their step and speed, else
  the arms hold their last target. Within 20 mm of the clearance a move passes
  only if it takes the limiting parts apart, or the arms could rest at its target
  with the whole margin, so held arms can always leave.

What it costs: at rest each hand's box is about 0.15 m, so the hands hold about
30 cm apart (29 cm in the virtual rehearsal, `tests/contract/test_two_arm_teleop.py`).
The tracking allowance and the 0.5 rad/s floor over 40 ms dominate, on lever
arms near a metre. Measured rest errors, right J1's braking (4 rad/s²) and the
left wrist sweeps would tighten it. The Jetson's cost per cycle (one box, up to
four when a move is retried) is still to be measured against the 12 ms compute
budget.
