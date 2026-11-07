# The cube handoff in simulation, on the measured bench

**What it is.** The simulated task: a scripted demonstrator picks up a block with the left gripper, hands it to the
right and puts it in the box, through the real native receiver with grippers that stall on the block. Its episodes can
seed sim-plus-real co-training. **Status: done and merged.** Later work extended it: the finger collision and the
quarter-turn take (below), the printed fiducial tags and head-camera depth ([SIM_WORKCELL.md](SIM_WORKCELL.md)),
randomized layouts ([SIM_RANDOMIZED_LAYOUTS.md](SIM_RANDOMIZED_LAYOUTS.md)) and the room look
([SIM_ROOM_LOOK.md](SIM_ROOM_LOOK.md)). Everything here is simulation; none of it is proven on the robot.

## What it replaced

The task scene could not show the task on the real bench, for three reasons:

- its layout was synthetic (a 0.28 m table chosen for top-grasp reach);
- its grippers ignored the cube (squeeze came from penetration, and the reading showed "fully closed" while holding);
- no policy in the harness could complete the task. The frozen fixture never reaches pickup through the calibrated plant:
  `python -m robot_platform.evaluation run --policy fixture_replay --plant armplant` ([MINERVA_EVAL.md](MINERVA_EVAL.md)).

## Behaviour

- With the default synthetic workcell, the scene compiles to the same geometry as before.
- A driven gripper stops on the cube with torque kp x shortfall, clipped to `max_effort`, and pads within 0.5 mm.
- The demonstrator succeeds at the nominal cube start, with no clipped commands. Every planned move clears the table,
  the bucket and the other arm by more than 1 mm, and no arm joint trails its command by more than the lead following
  allows.
- Each finger's paddle collides as convex 5 mm slices of its CAD surface, at most 1.8 mm outside it; its rack and pinion
  mount does not collide. One arm more than 2 mm into the other makes a trial ineligible, as the robot in the table or
  bucket does.
- A trial with `forbidden_robot_robot_penetration` scores `forbidden_penetration`. The frozen fixture replay now ends
  there, since its handoff passes the arms through each other.
- The harness clips commands into the receiver's usable range instead of the full range.
- A generated episode is published only for a success. Disturbances are recorded per trial and undone after it.
- The per-step speed-ups change no result: the nominal trial gives the same events, and the same qpos, qvel and surface
  velocities at every driven step.

The task runs on the direct, collision-disabled profile. The guarded two-arm bound keeps the hands about 30 cm apart,
which a handoff cannot meet.

## Results

- **Cube region.** `scripts/sim/demonstrator_reach_map.py` (4 cm step) finds where the whole task plans with every move
  clearing by more than 1 mm: x 0-0.28 m at every y from -0.32 to -0.52 m, plus a front-right patch near the table's
  edge.
- **Default grid** (the nominal start plus 9 cells, grid seed 0), through the receiver with driven grippers: 10/10
  (Wilson 95% 0.72-1.00) once the gripper settle test and paced moves were in. Before, some handoffs never registered
  (both grippers held the cube for 40 ms where the scene requires 50 ms) and five arm joints trailed their commands by
  more than the lead allows; paced, none did (at most 0.89 of its lead).
- **Turned cubes.** Planning only, yaws -40 to +40 degrees over the region (180 cells): the first pair of faces alone
  plans 149; with the second pair, the climb before the handoff roll and the clearance check, all 180 do. Through the
  receiver, a 4x4 grid plus the nominal start with yaw drawn from +/-45 degrees: 17/17 (Wilson 95% 0.82-1.00).
- **Finger collision and the quarter-turn take.** Motivated by videos in which the cube passed through a finger and the
  grippers' fingers crossed at the handoff. With the paddle-only shape and the right's J5 a quarter turn, the default
  grid is 10/10 (median 48.3 s) and the turned cubes 17/17 (median 49.0 s). The nominal plan's tightest clearance is
  2.5 mm, the left's fingers backing out past the right's. At the region's far corner the arms met on the way to the
  handoff, so there they move one at a time.
- **Speed.** The finger pieces made the scene's per-step Python cost ten times the physics step; a nominal trial took
  208 s of wall time for 49 s of scene time, and after optimization 119 s, bit for bit the same result.
- **Reproduce the grid:**
  ```
  python -m robot_platform.evaluation run --policy handoff_demonstrator --plant armplant --grippers driven \
      --workcell config/devices/simulation/minerva-production-01-workcell.json --timeout 70
  ```

## Notes for training on the generated episodes

- Generated episodes carry `policy` authority on every row. `python/training/minerva_dataset.py` trains on human and
  correction rows by default, so it rejects them ("No valid training frames found"); allowing `policy` explicitly loads
  them. See [SIM_GENERATION.md](SIM_GENERATION.md).
- `python/training/build_lang_cache.py` globs only `episode_*.hdf5` and misses `task_episode_*.hdf5`.
- A checkpoint's `ready_pose`, where `MinervaCheckpointPolicy` starts, is the mean of the training episodes' start poses.
  If the episodes start on two wrist-roll branches a turn apart (left J5 at -3.14 or +3.14 rad), the mean is a pose none
  of them had and a trial staged there can start with the left gripper's pads in the table. A mode or medoid of the
  starts is safe either way.
- Real capture and generated episodes both record at 15 Hz ([DATASET_PIPELINE.md](../data/DATASET_PIPELINE.md)). The
  evaluation harness's checkpoint policy executes one predicted step per `action_dt`, which must equal the recording
  period or the policy moves faster or slower than its demonstrations.

## Not known

- **Workcell placeholders.** The table's position relative to the hanging arms, the box's height, position, orientation
  and wall thickness, the cube's mass and its friction on the pads are not measured; measure them, then re-run the reach
  map and `tests/unit/test_demonstrator.py`.
- **Cameras.** Head D435 extrinsics and the wrist camera mounts are not calibrated. Generation varies them by +/-1 cm and
  up to 2 degrees.
- **Gripper force.** Squeeze is kp x shortfall / 9.2 mm/rad (about 310 N combined at kp 2 and 1.4 rad), not measured. A
  squeeze on a kitchen scale would calibrate it. The drive's inertia and friction are nominal.
- **Scope.** The arms stay kinematic: no robot/table push-back and no gripper-to-gripper contact. The scene audits one arm
  passing into the other instead, and at the handoff the grippers clear each other by about 5 mm. Live `helm --sim` still
  uses kinematic grippers.
- **Whether scripted simulation episodes may train a policy** is a dataset-contract decision. Today's loader excludes them
  unless training allows the `policy` authority explicitly.
