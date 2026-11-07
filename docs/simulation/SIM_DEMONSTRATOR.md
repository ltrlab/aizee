# Scripted handoff demonstrator

**Status: done and in use.** It generates the simulated demonstrations ([SIM_GENERATION.md](SIM_GENERATION.md)) and shows the task can be done in simulation. Nothing here is proven on the robot. The follow-ups at the end are open planner gaps.

`robot_platform.evaluation.demonstrator.HandoffDemonstrator` does the whole cube task in simulation: the left gripper picks the cube up, hands it to the right, and the right drops it in the bucket. It runs on the measured bench ([SIM_WORKCELL.md](SIM_WORKCELL.md)), through the actual receiver, with driven grippers ([SIM_GRIPPERS.md](SIM_GRIPPERS.md)). It plans from privileged simulation state (the cube's start pose and the workcell), then holds each grasp pose on the receiver's feedback. It exists to generate demonstrations and to show the task can be done; it is not a learned policy.

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy handoff_demonstrator --grippers driven --workcell config/devices/simulation/minerva-production-01-workcell.json --timeout 70
```

## What the arms can do on this bench

Every pose below came out of a reach or clearance check, not a guess. The checks are in `scripts/sim/` (`demonstrator_reach_map.py`, `workcell_reach.py`); `tests/unit/test_demonstrator.py` checks the resulting plan.

**Reach**
- **Straight down is out of reach.** At the 0.512 m table, the fingers cannot point straight down at the cube. On the table they tilt at most about 20 degrees down, and less higher up: 15 degrees 3 cm above the pre-grasp, 10 degrees at 6 cm, level at the lift.
- **Seeded solving.** The reachable set near the table is thin, so IK from random joint seeds rarely lands in it. The demonstrator samples 100,000 poses per arm once (about 10 s) and seeds every fresh solve from the samples nearest its target.

**Clearance**
- **Wrist housing.** It reaches 63 mm to one side of the fingers and 45 mm to the other, from 82 mm behind the pads.
  - With level fingers on the table, either side goes through the table, which is why the grasp tilts.
  - With the narrow side down and a 20 degree tilt, the pads clear the table by 2.4 mm at the grasp. With the wide side down, the housing sinks 34-54 mm into it.
- **Wrist roll.** Putting the narrow side down puts the wrist roll (J5) near +/-pi, at the edge of its range (about +/-3.3 rad). Moves from there run into the limit or spin a whole turn. So:
  - the left keeps the narrow side down from its start through the lift, then rolls over on the way to the handoff, well above the table;
  - the right works with the wide side down (J5 near 0) until it lifts the cube at the handoff, rolls there, and drops with the narrow side down. With the wide side down, the housing hangs 11-48 mm into the bucket's walls at every drop pose tried.
- **Bucket approach.** The bucket is long front to back and narrow across. The right comes to 12 cm in front of the drop point at its height, then slides straight forward over the near wall. A diagonal carry crossed the side wall by 36 mm.

**Command limits**
- **Stopping margin.** The direct receiver refuses an arm joint's command within the stopping margin of either end of its range (0.024-0.051 rad here). The evaluation harness now clips commands into that usable range, as the receiver accepts them. It used to clip to the full range, so a command at the edge aborted the trial (right J5 at 3.2348 against 3.2585 - 0.0239).

## The plan

| Step | Pads | Fingers | Housing side down |
|---|---|---|---|
| Start | left (0.15, -0.30), right (-0.10, -0.20), 12 cm above the table | left forward, right forward-left | left narrow, right wide |
| Above the cube | 6 cm over the pre-grasp | forward, 10 degrees down | narrow |
| Pre-grasp, grasp | 6 cm behind and 2 cm above the cube, then on it | along the normal of a pair of the cube's side faces (the pair facing nearest forward, else the other), 20 degrees down; where that is out of reach, turned up to 20 degrees or tilted 10-15 degrees | narrow |
| Close, lift | cube + 8 cm | level | narrow |
| Climb, only where the move to the handoff would not clear | lift + 10 cm | level | narrow |
| One arm at a time, only where moving together they would meet: the left to the handoff, then the right | | | |
| Handoff | the left holds the cube out at the right's turned grip point, 8.5 mm from (0, -0.30) 12 cm above the table | left forward-right, jaws closing level | wide |
| Right turns where it waits, 6 cm back along its fingers | | right forward-left, J5 a quarter turn: jaws closing vertically | |
| Right comes in and closes; left opens, backs 8 cm straight out, level, then returns to its start | | | |
| Right lifts | handoff + 10 cm | forward-left, 10 degrees up | narrow (rolls here) |
| Over the bucket | its centre, 12 cm above the rim, approached from 12 cm in front | forward, 20 degrees up | narrow |
| Right opens | the cube drops | | |

**How it moves**
- **Straight-line moves.** Each move takes the pads in a straight line while the gripper turns evenly, solved every 1 cm from the last solution. Where an arm cannot follow the line, the move falls back to joint interpolation in 0.02 rad steps. The roll to the handoff and the right's lift do this.
- **Wrist-roll branches.** The start pose is tried on each wrist-roll branch (the same pose a turn apart). Of the plans whose every move clears, the one with the fewest fallbacks, then the shortest, is kept.
- **Clearance.** A plan is used only if every move keeps the robot more than 1 mm from the table, the bucket and the other arm, with each gripper posed where it stops on the cube (0.4 s to check). The move to the handoff interpolates the left's joints while it rolls over, and from some grasps of a turned cube that swing took the pads up to 14 mm into the table. Where it would, the left first climbs 10 cm straight up.
- **Handoff.** Level jaws on perpendicular faces crossed the two grippers' inner fingers behind the cube's corner, 6.5 mm deep once the fingers' paddles collided. In teleop the handoff turns one wrist's J5 a quarter turn, and the plan does the same: the right's jaws close on the cube's top and bottom. The grippers then clear each other by 5 mm holding the cube, and by 25 mm as the right comes in.
  - J5's axis sits off the fingers' centre, so the turn moves the grip point about 9 mm: one way 6.5 mm down, the other 6.2 mm up. The plan turns the way that leaves the grip point higher (6.5 mm lower, the move to the handoff cleared the table by 0.7 mm), and the left brings the cube there.
  - The right turns where it waits. Turning on the way there, its joint path went 7 mm into the bucket.
  - From the cube region's far corner (x 0.04-0.06 m, y -0.497 m), moving to the handoff together swept the left's wrist 4.9 mm through the right's fingertips, and the climb does not separate the arms. Where the move still would not clear after the climb, the left brings the cube to the handoff first, and the right comes after. A trial there succeeds, with the bucket at 52.7 s.
  - After letting go, the left backs straight out, level, then returns to its start. Rising as it backed out, its fingertips went 3.7 mm into the right's upper jaw; left where it backed out to, it was 2.3 mm into the right's way as the right lifted the cube.
- **Grasps.** The jaws open to the cube plus 2 cm each side and close 1 rad past touching, so the drive squeezes with kp x 1 rad.
- **Gripper steps.** A gripper step ends once that gripper's reading has moved less than 0.01 rad over 0.2 s (stalled on the cube, or open), after at least 0.3 s and at most 3 s. A fixed 0.6 s wait let the left open before the right had both jaws on the cube, because the right needs about 1 s to close from open at the receiver's gripper gains. The two grippers then overlapped for 40 ms, short of the 50 ms the scene requires of a handoff, so 4 of 10 trials ended out of sequence. The settle test reads position: the speed reading carries the drives' noise.
- **Holds.** Holds correct toward the pose (at most 0.03 rad) until the arm settles within 0.004 rad or 1.5 s pass.
- **Speed.** Moves are planned at up to 0.5 rad/s per joint in transit and 0.15 rad/s on approaches and lifts. With the smooth start and stop, a transit move peaks at 0.75 rad/s. The right's lift at the handoff, which includes its half-turn roll, runs at transit speed in free space.
- **Pacing.** The direct profile's commands are position-only, and following lets each command lead its joint by at most a bounded lead (the lifecycle's `max_lead`, 0.023-0.087 rad, passed to policies as `TrialContext.lead`). That caps how fast a joint follows: by (kp x lead - friction) / (kd + kp x 10 ms), about 0.23-0.35 rad/s for left J1 and J2 and right J2 and J3.
  - Unpaced, four joints trailed their commands by more than their lead. Right J1 fell 0.5 rad behind while carrying the cube to the bucket, so the arm left the planned path, and the slide over the near wall started with J1 still 0.45 rad short.
  - A move's clock now runs at full rate while every arm joint trails its last planned command by less than half its lead. It slows as one trails further and stops at the full lead.
  - With pacing, every arm joint stays within its lead (right J1 within 0.019 rad), and the nominal trial takes 39.4 s instead of 32.5 s. A single bound in radians for every joint held the soft wrists back and took 59 s.

## Where the cube can be

`scripts/sim/demonstrator_reach_map.py` plans the task for every cube position on a grid and checks each move's clearance. On the placeholder layout, at a 4 cm step:
- The whole task plans and clears by more than 1 mm for x 0-0.28 m at every y from -0.32 to -0.52 m.
- It also plans in the front-right patch near the table's edge: x from 0.12-0.20 m at y -0.28 to -0.20 m.
- Only the front-left corner nearest the robot's centre is out of reach.

**Turned cubes.** A cube looks the same every quarter turn, so the left can take it by either pair of side faces. Planning only, over the cube region at a 6 cm step and yaws from -40 to +40 degrees in 10 degree steps (180 cells; `demonstrator_reach_map.py --x 0.02 0.26 --y -0.50 -0.32 --step 0.06 --yaws -40 -30 -20 -10 0 10 20 30 40`):
- By the pair of faces nearest forward alone, 149 cells planned and cleared, 20 could not be picked (yaws +10 to +40 at small x), and 11 planned too close. Eight of those went into the table on the way to the handoff, up to 14 mm.
- With the second pair of faces, the climb and the clearance check, all 180 plan and clear.

The workcell's cube region was x 0.02-0.26 m, y -0.32 to -0.50 m, inside that area, until the box moved to where the head camera's depth puts it (below).

**On the interim layout (2026-10-18).** The box now stands where the head camera's depth puts it, the tag rectangle 36 cm in front of the hanging arms, and the table with it (SIM_WORKCELL.md, "The robot's interim place"). Over the rectangle at 5 cm cells, the whole task, the release into the box included, plans and clears over its left half (u 25-275 mm, v 25-375 mm: base x 0.05-0.30 m, y -0.385 to -0.735 m); nearer the box most plans come within 1 mm of it, and at the far right the left cannot pick. The workcell's cube region is now x 0.10-0.30 m, y -0.385 to -0.735 m, and its nominal start the near real block's depth-fitted place and turn, (0.1722, -0.4843) m at -37 degrees.

With the bucket moved out of the way, the left picks a 38.1 mm cube and hands it over with every move clear (6 cm map) from x -0.12 to 0.42 m at y -0.36 to -0.66 m, and at its edges down to -0.78 m; yaw 45 degrees reaches a little more than yaw 0. Out there it fails three ways: beyond y -0.80 m it cannot pick; reaching to the right of x -0.18 m, or to the far edge, the left's forearm comes within 1 mm of the table on its way to the handoff or over the cube, or its links of the right's jaws; and near the handoff point (x -0.18 to -0.06 m, y -0.36 to -0.42 m) the handoff does not plan.

## Drops

The right brings the cube over the bucket and lets it go one of the ways `DROPS` lists, tried in order for each wrist-roll branch until a plan clears. The first is the drop above (fingers 20 degrees up, 12 cm over the rim, from 12 cm in front), so every layout it planned keeps its plan:

| Drop | Fingers | Lead to the drop point | Pads over the rim |
|---|---|---|---|
| from the front | forward, 20 degrees up | 12 cm in front | 12 cm |
| lower, from closer in front | forward, 20 degrees up | 6 cm in front | 8 cm |
| fingers level, from the front | forward, level | 10 cm in front | 9 cm |
| fingers tipped down, from the front | forward, 30 degrees down | 8 cm in front | 9 cm |
| from straight above | forward, 20 degrees up | 10 cm above | 8 cm |
| fingers steeply down, from straight above | forward, 60 degrees down | 8 cm above | 8 cm |

Planning-only maps over buckets of the training ranges' corners (130 x 200 x 150 mm to 200 x 300 x 220 mm inside, turned -49.4, -19.4 and 10.6 degrees) at 7.5 cm steps over the table's right half, the block at the nominal start, on the interim layout:
- The release plans and clears for bucket centres x -0.45 to 0.075 m, y -0.47 (-0.55 for the 300 mm long buckets, whose near wall would leave the table) to -0.85 m, except as below. Of the 181 that planned, 129 used the first drop, 29 the level fingers, 16 the lower drop and 7 the drop from above.
- **Not covered: far out.** No drop reaches a centre at y -0.925 m, nor the far corners at y -0.78 to -0.85 m (x -0.45 or 0.075, and x -0.375 for the smallest bucket): beyond the right's reach with any of these fingers.
- **Not covered: tall buckets near the robot.** For walls of 220 mm, no drop reaches centres at y -0.55 m with x -0.30 to -0.075 m, nor (the 200 mm wide bucket) at y -0.625 m with x -0.15 to -0.075 m. The drop points are 8-18 cm over a 732 mm rim, out of the right's reach close in, and the lower drops bring its wrist or forearm 2-7 mm into the bucket's near wall. Buckets of 150-185 mm plan there.
- **Not covered: on the handoff.** A bucket centred within about 10 cm of the handoff point (x about 0, y -0.47 to -0.55 m) is in the way of the handoff.

The training layouts' bucket region stays where the release was mapped; a draw in one of these gaps is refused by the planner and drawn again (SIM_GENERATION.md).

## Blocks that are not cubes

A block may be a cuboid (SIM_WORKCELL.md, `cube.size`). The left takes it across a pair of its side faces, the pair facing nearest straight ahead first as for a cube, but passes over a pair its jaws cannot open across with 5 mm to spare each side (they open to about 97 mm, so every block of the training ranges fits either way). Its jaws stop at that pair's width and the right's at the block's height, and each opens 2 cm past the widest it may close on. The plan records the pair, its width and the block's height (`grasp`, and `planned_grasp` in `describe()`), and the episodes' metadata carries it.

**Short blocks are refused.** The right closes on the block's top and bottom, so a short block brings its jaws close to the left's fingers, and the left backing out of the handoff passes them more closely. At the nominal start (planning only, the block 38.1 mm across): 55 mm tall, the left's retreat clears the right's fingers by 10.9 mm; 48 mm, 7.6 mm; 42 mm, 4.7 mm; 38.1 mm, 2.8 mm; 36 mm, 1.7 mm; 33 mm, 0.3 mm (refused); 30 mm, 1.2 mm into them (refused). Opening the left as wide as it goes before backing out made it worse (0.7 and 2.2 mm into the right's fingers further along), and was not kept. So blocks under about 35 mm tall are refused and drawn again: in the layout trials (SIM_GENERATION.md) every draw refused at the left's retreat had a block 31-36 mm tall, and every block that ran was at least 37 mm tall. The real blocks' tops are 37-39 mm up. The training layouts now draw block heights from 35 mm; the retreat itself is a follow-up ("Follow-ups").

## Why a plan fails

A `PlanningError` lists, per wrist-roll branch and drop, why it failed: a move out of reach (`left lifts: the left gripper cannot reach [...]`) or a move that came within 1 mm (`drop from the front: right carries: <robot geom> comes 0.4 mm from <geom>`). The demonstrator keeps them as `notes`. A generator counts a refused draw with this reason.

Planning the same trial again (the same workcell, start and limits) reuses the plan: a generator plans a drawn layout before it runs it, and the trial's own reset then costs nothing.

## Results

Through the receiver with driven grippers on the measured bench:

- **Nominal cube start (0.15, -0.35):** success. Pickup at 6.3 s, handoff at 23.7 s, bucket settled at 49.2 s, with no clipped commands. Before the quarter-turn take the handoff came at 19.5 s and the bucket at 39.4 s.
- **On the interim layout, the nominal start at the near real block, turned -37 degrees:** success through the receiver with driven grippers (`tests/contract/test_demonstrator.py`), dropping "lower, from closer in front" into the depth-measured box. The far-corner and turned-cube tests keep the earlier placeholder layout they were planned on (`tests/fixtures/minerva_workcell_placeholder_20261017.json`).
- **Default 3x3 grid, first placeholder region:** 5/10 succeeded (Wilson 95% 0.24-0.76).
  - Every trial that planned succeeded.
  - The other five could not plan their pick, at the region's far edge (y -0.41 to -0.42) and near edge (y -0.28), at x 0.08-0.22.
- **Default 3x3 grid, reachable region, with the alternative grasp orientations:** 6/10. The other 4 ended out of sequence: the cube reached the bucket but the scene saw no handoff (see "Gripper steps").
- **The same grid with the gripper settle test:** 10/10 succeeded (Wilson 95% 0.72-1.00), median time to complete 46.1 s (p90 46.8 s). With the right's lift at transit speed, also 10/10, median 32.0 s (p90 32.7 s).
- **The same grid with paced moves:** 10/10 succeeded (Wilson 95% 0.72-1.00), median time to complete 40.8 s (p90 41.8 s). No arm joint trailed its command by more than 0.89 of its lead. Unpaced, five did somewhere on the grid, right J1 by 0.52 rad (21 times its lead).
- **Turned cubes, yaw drawn from +/-45 degrees, 4x4 grid plus the nominal start:** 17/17 succeeded (Wilson 95% 0.82-1.00), median time to complete 40.6 s (p90 43.5 s). The yaws ran from -44.8 to +44.7 degrees, and no joint trailed by more than 0.88 of its lead. `run ... --grid 4x4 --yaw 0.785398`.
- **With the fingers' paddles colliding, the arm-against-arm audit and the quarter-turn take**: the default grid 10/10 (Wilson 95% 0.72-1.00), median time to complete 48.3 s (p90 49.4 s); the turned cubes 17/17 (Wilson 95% 0.82-1.00), median 49.0 s (p90 51.2 s), yaws -44.8 to +44.7 degrees. No commands were clipped, and no arm joint trailed its command by more than 0.89 of its lead. The right's turn and the left's return to its start add about 8 s.

## Follow-ups

Two planner gaps the training layouts work around:

- **The left's retreat past short blocks.** Backing out of the handoff, the left passes within 1 mm of the right's fingers for blocks under about 35 mm tall ("Blocks that are not cubes"), so the training layouts draw heights from 35 mm. Candidates: back the left out along the right's jaws, or open it less before it leaves.
- **The right's turn meets a large held block.** In one layout trial of 30 (attempt 23, seed [2000, 23, 3]), 0.64 s into `right turns` (the right turning its open jaws upright where it waits, before it comes in) a right finger met the 50 x 47 x 48 mm block in the left's grip, and the block forced its jaws open past the gripper's range: the receiver stopped on `right_gripper` `position_above_upper`, 5.2823 rad against 5.2791 (`limit_violation`, 25.15 s). Nothing touched the bucket or the table. The plan's clearance check leaves the held block out, and the right waits 6 cm back along its fingers (`APPROACH`) whatever the block's size. The layout is saved as `tests/fixtures/minerva_layout_take_abort_20261018.json`; `scripts/sim/repro_take_abort.py` runs it through the receiver with driven grippers and says whether it still aborts. Candidates: check the right's turn and approach against the held block, or back the right off by the block's size. Until then a generator drops such an attempt: it ends `source_gap_or_abort` and records no episode.

## Limits

- The robot's place relative to the tags is an interim placeholder, and the table follows it (SIM_WORKCELL.md). A measured place needs its own reach and clearance check (`tests/unit/test_demonstrator.py` runs one), and the cube region and nominal start may move with it.
- Tall buckets near the robot, buckets beyond y -0.85 m and buckets on the handoff point are not released into ("Drops"), blocks under about 35 mm tall are refused at the left's retreat ("Blocks that are not cubes"), and a large held block can meet the right's turning jaws ("Follow-ups").
- The demonstrator reads the cube's start pose, which a learned policy must not see. It starts from a fixed pose above the table, not from the hanging arms, and the harness stages it there.
- The nominal plan's tightest point is the left's fingers backing out 2.5 mm from the right's after letting go; the next is the move to the handoff, 4.6 mm over the table. At the grasp the left's fingers pass 9.4 mm over the table (2.4 mm with the old pad boxes, whose corners reached lower than the finger's rounded tip). Tracking error that brings the robot more than 2 mm into the table, the bucket or the other arm makes the trial ineligible.
- The arms remain kinematic, so the grippers cannot push each other. The scene audits one arm passing into the other instead.
