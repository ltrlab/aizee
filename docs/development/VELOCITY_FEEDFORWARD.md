# Feature task: opt-in velocity feed-forward for manual-tuning teleop

- Branch and worktree: `claude/velocity-feedforward`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`, on
  `claude/speed-estimate-fix` (stacked; none of the stack is in integration yet)
- Owner: Claude Code, at the operator's request (2026-10-14): "would vel ff help? if so build it", then, shown
  the simulation below, "ship plain feed-forward to try live" (an A/B against position-only)
- Saved evidence: both-arm sessions `~/minerva/evidence/live-20261013T031504Z` (J1 375/9, J2 350/10, J3 320/8) and
  `live-20261014T011541Z` (J1 550/15, J2 600/20, J3 600/11), manual tuning, position-only

## Why

Manual tuning ran position-only: the drives got a target and no velocity, so Kd opposes every motion. Top
following speed is then (Kp x lead - friction) / (Kd + Kp x 10 ms) (TUNING_SPEED_ESTIMATE): J3 was held near
0.7 rad/s at 320/8 and trailed the leader by up to 0.7-1.0 rad. Kd also sets a tracking lag of about Kd/Kp
(20-30 ms). Feeding the command's speed forward makes Kd damp the joint about that speed instead.

Direct teleop first fed one cycle's step forward (2026-10-10, DIRECT_TELEOP.md). With the leaders repeating one
sample in ten, the step dropped to zero and back, and Kd x each jump pulsed the drives about every 100 ms. With
leader firmware 2 the leaders repeated 7 of 85,800 cycles (0.01%) on 2026-10-14.

## What it does

- `receiver.direct.position_only: false` on an unramped direct profile (interpolation 0) is the
  `direct-velocity-feedforward` law (`guarded_config`): arm joints carry `velocity_feedforward`, grippers none.
- The velocity sent is the least-squares slope of the last `FEED_SAMPLES` = 4 slewed commands over their cycle
  times (`teleop.lifecycle._fed_velocities`), within the joint's speed cap. A target that was not slewed (snap,
  hold, lost leader, stop) clears the history and sends none. No friction or acceleration (ramp) feed-forward:
  both pulsed before. Gravity feed-forward is unchanged.
- Manual tuning accepts either direct law (`direct_law`); presets are bound to the law (`binding.control`), so a
  position-only preset is never loaded for feed-forward or back. Helm's panel names the law and estimates top
  speed without the Kd term under feed-forward.
- The receiver accepts manual tuning's in-session gain update under either direct law (it required
  position-only); every other condition stays: same targets, no velocity or ramp sent, every joint settled.
- `config/devices/minerva-teleop-direct-tuning-vff.json`: the tuning profile with only `id`, `position_only` and
  its notes changed. The position-only profile is untouched; which Helm is launched picks the law.
- `scripts/qualify_manual_tuning_loop.py` checks the law's output: position-only none; feed-forward none for
  grippers, within caps, and never a zero for a cycle while a command keeps moving one way.

No bound changed: speed, acceleration, lead, tracking, effort, freshness and watchdog limits are as before.

## Expected behaviour (offline; not physical evidence)

1-DOF simulation (drive PD at 2 kHz, 100 Hz commands seen 3 ms late, 1.5 mrad leader counts, 0.5 Nm friction),
J2 1.33 kg m2 at 600/20, 0.5 rad/s:

| Law | Lag while moving | Overshoot, abrupt stop | Overshoot, 5 rad/s2 stop |
|---|---|---|---|
| Position-only | 21 mrad | 10.5 mrad, 1 swing | 8.8 mrad, 1 swing |
| Velocity feed-forward | 4.5 mrad | 17 mrad, 2 swings | 15 mrad |
| Position-only at Kd 30 | 30 mrad | 5.3 mrad, 0 swings | 4.4 mrad |

Without Kd's drag nothing feeds the braking torque forward, so a stopping arm overshoots by about I x a / Kp.
An inertia x acceleration feed-forward from a filtered leader reference removes most of it in simulation
(2-3 mrad, no swings) but brings the lag back to position-only's; not built. The same position-only J2 case
matched the 2026-10-14 session (6-7 mrad at ~0.35 rad/s).

Rehearsal through the real TeleopCore, lifecycle and Rust receiver on `simulation/arm_plant.py` (servo-file
inertia and friction, gravity 10% over the fed model), right arm, saved gains, ~0.4 rad/s, no receiver trips:

| Joint | Law | Lag | Abrupt stop: overshoot / to 2 mrad | Smooth stop: overshoot |
|---|---|---|---|---|
| J2 600/20 | position-only | 20 mrad | 8.5 mrad, 2 swings / >1.2 s | 3.6 mrad |
| J2 600/20 | feed-forward | 4 mrad | 15 mrad, 1 swing / 330 ms | 6.6 mrad |
| J3 600/11 | position-only | 16 mrad | -1.1 mrad / 20 ms | -0.7 mrad |
| J3 600/11 | feed-forward | 4 mrad | 6.1 mrad / 40 ms | 0.8 mrad |

## Not addressed

- Left and right J1 ring at ~3.5 Hz after their own stops whatever Kp (375 and 550) and Kd (9 and 15): likely
  gearbox or mount compliance, not the servo law.
- During motion about 90% of the 2-4 Hz jiggle is in the leader signal (robot/leader 1.0-1.2); feed-forward
  follows it at least as faithfully.

## Results

- Gate: 713 Python tests, zero failures/skips/xfails, Rust included, on `c184c6d`
  (`build/dev-check/20261014T020754Z-7f46af4c/report.json`), after the ARM64 rebuild.
- Jetson private-vcan manual-tuning qualification of release `20261014T021323Z-c184c6d` (receiver
  `cc82487c`), 125 s of following each, synthetic 90 Hz quantised leaders (about one repeat in ten):

  | Run | Following cycles, all accepted | Misses / skips | Loop p99 / max | Gain writes | Velocity check |
  |---|---|---|---|---|---|
  | Feed-forward, left (`~/minerva/validation/vff-c184c6d-02/tuning-vff-left-125s`) | 13,011 | 0 / 0 | 5.41 / 6.11 ms | 2 | ok, 0 zero drops |
  | Feed-forward, both (`vff-c184c6d-02/tuning-vff-both-125s`) | 13,012 | 0 / 0 | 5.77 / 6.37 ms | 2 | ok, 0 zero drops |
  | Position-only, both (`vff-c184c6d-01/tuning-po-both-125s`) | 13,012 | 0 / 0 | 5.60 / 5.91 ms | 2 | ok (none sent) |

- Route edits (`routes.json`, one per asset and runs folder) are bound to a profile id, so the feed-forward
  profile runs from its own folder, `build/manual-tuning/operator-runs-vff`, holding a copy of the verified
  signs (`copied_from` provenance) and a preset seeded from the position-only one (same gains, re-bound).
  Helm launch config `helm-tuning-vff`; `helm-tuning` stays position-only.
- Deployed: release `20261014T022244Z-c184c6d`, receiver binary identical to the qualified one. Helm relaunched
  on the feed-forward profile: law shown in Motor tuning, preset loaded, all 14 drives answering.
- Not yet run on the arms.

## Correction: the new runs folder lacked the verified directions (2026-10-14)

`operator-runs-vff` was created with the routes and preset only. Helm then made a fresh direction session from
the 2026-09-24 seed, which lacks the operator's verified directions: right J3 and J4 are -1 there, +1 in the
seed. In the first feed-forward session the viewer mirrored right J3 and J4 (reported by the operator), and the
right arm's gravity feed-forward used the seed's pitch chain: J2's term was +6.1 Nm instead of -6.1 Nm with J3 at
1 rad, and up to ~9-12 Nm off elsewhere (about 20 mrad at Kp 600; no trips). The left arm was unaffected.
Fixed by stopping Helm and copying the verified session (`e78c80c8`, identical to `operator-runs`) into
`operator-runs-vff/minerva-production-01-directions/session.json` (the seed one kept as
`seed-session-replaced-20261014.json`); Helm now reports every right-arm direction verified. The two
folders still hold separate copies of directions and route edits, so a change made in one Helm does not reach
the other.

## One folder for both laws (2026-10-14, `185eeb8`)

Route edits are now bound to the digest of the route definitions (`routes_sha256`) as well as the profile id:
profiles with the same routes share one `routes.json` (a file without the digest stays its own profile's). The
feed-forward Helm (`helm-tuning-vff`) runs from `build/manual-tuning/operator-runs` again, sharing the verified
direction session and route edits with the position-only Helm; each law keeps its own preset file there. The
existing `routes.json` got its digest added (original kept as `routes.before-shared-20261014.json`), and the
feed-forward preset was copied in unchanged. `operator-runs-vff` is no longer used; it keeps the first
feed-forward sessions' Helm records. Gate: 714 tests, zero failures/skips/xfails, on `185eeb8`
(`build/dev-check/20261014T024318Z-cdba773d/report.json`). Deployed `20261014T024820Z-185eeb8`, receiver
unchanged (`cc82487c`).
