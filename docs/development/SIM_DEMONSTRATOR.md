# Scripted handoff demonstrator

`robot_platform.evaluation.demonstrator.HandoffDemonstrator` does the whole cube task in simulation: the left gripper picks the cube up, hands it to the right, and the right drops it in the bucket. It runs on the measured bench ([SIM_WORKCELL.md](SIM_WORKCELL.md)), through the actual receiver, with driven grippers ([SIM_GRIPPERS.md](SIM_GRIPPERS.md)). It plans from privileged simulation state (the cube's start pose and the workcell), then holds each grasp pose on the receiver's feedback. It exists to generate demonstrations and to show the task can be done; it is not a learned policy.

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy handoff_demonstrator --grippers driven --workcell config/devices/simulation/minerva-production-01-workcell.json --timeout 70
```

## What the arms can do on this bench

Every pose below came out of a reach or clearance check, not a guess. The scripts that found them are in this branch's history; `tests/unit/test_demonstrator.py` checks the resulting plan.

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
| Pre-grasp, grasp | 6 cm behind and 2 cm above the cube, then on it | forward, 20 degrees down; where that is out of reach, turned up to 20 degrees or tilted 10-15 degrees | narrow |
| Close, lift | cube + 8 cm | level | narrow |
| Handoff | both on the cube at (0, -0.30), 12 cm above the table | left forward-right, right forward-left, jaws on perpendicular faces | wide |
| Right closes, left opens and backs off | | | |
| Right lifts | handoff + 10 cm | forward-left, 10 degrees up | narrow (rolls here) |
| Over the bucket | its centre, 12 cm above the rim, approached from 12 cm in front | forward, 20 degrees up | narrow |
| Right opens | the cube drops | | |

**How it moves**
- **Straight-line moves.** Each move takes the pads in a straight line while the gripper turns evenly, solved every 1 cm from the last solution. Where an arm cannot follow the line, the move falls back to joint interpolation in 0.02 rad steps. The roll to the handoff and the right's lift do this.
- **Wrist-roll branches.** The start pose is tried on each wrist-roll branch (the same pose a turn apart), and the plan with the fewest fallbacks, then the shortest, is kept.
- **Grasps.** The jaws open to the cube plus 2 cm each side and close 1 rad past touching, so the drive squeezes with kp x 1 rad.
- **Gripper steps.** A gripper step ends once that gripper's reading has moved less than 0.01 rad over 0.2 s (stalled on the cube, or open), after at least 0.3 s and at most 3 s. A fixed 0.6 s wait let the left open before the right had both jaws on the cube, because the right needs about 1 s to close from open at the receiver's gripper gains. The two grippers then overlapped for 40 ms, short of the 50 ms the scene requires of a handoff, so 4 of 10 trials ended out of sequence. The settle test reads position: the speed reading carries the drives' noise.
- **Holds.** Holds correct toward the pose (at most 0.03 rad) until the arm settles within 0.004 rad or 1.5 s pass.
- **Speed.** Moves run at up to 0.5 rad/s per joint in transit and 0.15 rad/s on approaches and lifts. The right's lift at the handoff, which includes its half-turn roll, runs at transit speed in free space.

## Where the cube can be

`scripts/sim/demonstrator_reach_map.py` plans the task for every cube position on a grid and checks each move's clearance. On the placeholder layout, at a 4 cm step:
- The whole task plans and clears by more than 1 mm for x 0-0.28 m at every y from -0.32 to -0.52 m.
- It also plans in the front-right patch near the table's edge: x from 0.12-0.20 m at y -0.28 to -0.20 m.
- Only the front-left corner nearest the robot's centre is out of reach.

The workcell's cube region is now x 0.02-0.26 m, y -0.32 to -0.50 m, inside that area. Re-run the map after measuring the bench.

## Results

Through the receiver with driven grippers on the measured bench:

- **Nominal cube start (0.15, -0.35):** success. Pickup at 6.0 s, handoff at 16.7 s, bucket settled at 32.5 s, with no clipped commands. `tests/contract/test_demonstrator.py` runs this trial.
- **Default 3x3 grid, first placeholder region:** 5/10 succeeded (Wilson 95% 0.24-0.76).
  - Every trial that planned succeeded.
  - The other five could not plan their pick, at the region's far edge (y -0.41 to -0.42) and near edge (y -0.28), at x 0.08-0.22.
- **Default 3x3 grid, reachable region, with the alternative grasp orientations:** 6/10. The other 4 ended out of sequence: the cube reached the bucket but the scene saw no handoff (see "Gripper steps").
- **The same grid with the gripper settle test:** 10/10 succeeded (Wilson 95% 0.72-1.00), median time to complete 46.1 s (p90 46.8 s). With the right's lift at transit speed, also 10/10, median 32.0 s (p90 32.7 s).

## Limits

- The bench's positions are still placeholders: the table's position, the bucket's position and height, and the cube region (SIM_WORKCELL.md). A measured layout needs its own reach and clearance check (`tests/unit/test_demonstrator.py` runs one).
- The demonstrator reads the cube's start pose, which a learned policy must not see. It starts from a fixed pose above the table, not from the hanging arms, and the harness stages it there.
- The plan's tightest point is the left pads passing 2.4 mm over the table at the grasp. Tracking error that dips more than 2 mm into the table makes the trial ineligible.
- The arms remain kinematic. The scene does not model gripper-to-gripper contact, and at the handoff the two grippers' pads come within a few millimetres of each other.
