# Teleop latency, end to end

Analysed 2026-10-04 for two-arm live teleop. This is how a leader motion becomes
arm motion, hop by hop, with each hop's latency. It ends with the changes that
cut it.

**Basis.**
- Measured: leader USB timing from the 100 Hz capture of 2026-09-26
  (`calibration_runs/gello-dual/input-parallel-100hz-20260926.json`), and the
  link round trips from Minerva Check's link test (2026-09-30).
- Recorded: loop cycle times from the device log's live and virtual sessions,
  and servo lag from the servo sweeps.
- From the code: the receiver tick and the interpolation, with the code paths
  cited per hop. Each hop is labelled measured, recorded or from the code.

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

## Two arms

- **One receiver for both arms.** It runs both CAN buses (7 drives each, as
  now per bus) and all 14 motors in one tick, so one collision verdict covers
  both arms.
- **An enforced arm-against-arm bound.** The enforced compiler that exists today
  sizes each joint's box by the joint's *maximum* speed over the verdict's
  lifetime. That box is large even at rest, which is why teleop has run
  advisory checks. The receiver only ever ramps to the last target and then
  holds, so a box can instead be sized from the move actually commanded:
  - from the current command to the new target;
  - plus the joint's tracking bound;
  - plus its stopping distance from the commanded speed.
  
  The box is certified with one proximity query at its centre and per-link
  travel bounds, in the arm-against-arm scope, and the receiver enforces it. A
  move whose box cannot be certified is not sent: both arms hold their last
  certified targets until the leaders move apart.
