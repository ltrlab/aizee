# Generated demonstrations

`python -m robot_platform.evaluation generate` records the episodes a demonstrator completes in simulation. They use the same format, joints, cameras and clocks as a real capture (format 8), for sim + real co-training.

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation generate --episodes 50 --workcell config/devices/simulation/minerva-production-01-workcell.json --out build/generated/bench-v1
```

## What it does

Each attempt is an evaluation trial: the native receiver, the armplant joints and the contact scene, with driven grippers ([SIM_GRIPPERS.md](SIM_GRIPPERS.md)). By default the demonstrator is `handoff_demonstrator` ([SIM_DEMONSTRATOR.md](SIM_DEMONSTRATOR.md)).

`EpisodeRecorder` samples at `--hz` in scene time (default 15 Hz, near the real capture's 14) and writes through the shared `TaskEpisodeWriter`:
- the receiver's measured and commanded motor coordinates and its efforts;
- the head and both wrist cameras at `--camera-size` (default 320x240, as recorded), rendered with the full CAD visuals (`--visual-mode cad`).

An attempt publishes its episode only if the scene's own events call it a success: pickup, then handoff, then the cube settled in the bucket, and eligible. Anything else is discarded, partial file included.

`manifest.jsonl` lists every attempt: its grid seed, cube start, outcome, the episode it made (if any) and its frame count.

Cube starts come from the evaluation grid over the workcell's cube region, but:
- the seeds start at 1000;
- the nominal start (the evaluation's trial 0) is never generated.

So generated poses never repeat the evaluation grid at its default seed 0. Set `--grid`, `--jitter` and `--yaw` to spread them. When a seed's cells are used up, the next seed draws new jitter in the same cells.

## Appearance

Unless `--same-look` is given, each episode gets its own look from `MinervaTaskScene.randomize_appearance`, seeded by the grid seed, the trial and its cube start:
- the lights' strength x U[1-light, 1+light];
- the workcell's, floor's and cube's colours, each channel x U[1-color, 1+color];
- each camera's place, +/- `--camera-shift` m per axis and a turn of up to `--camera-turn` degrees.

The defaults are 0.3, 0.1, 1 cm and 2 degrees. The values chosen go into the episode's metadata as `appearance`. Only what the cameras see changes: geometry, friction, masses and contacts are untouched (`tests/unit/test_scene_appearance.py`). Camera poses are not calibrated, so spreading them keeps a policy from depending on one guessed view.

## Randomized layouts

The user (2026-10-18): "the sim episodes we train on should vary the size and location of both the bucket and the blocks." `generate --randomize <file>` draws every attempt's layout instead of taking the grid's cube pose:

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation generate --episodes 120 --workcell config/devices/simulation/minerva-production-01-workcell.json --randomize config/devices/simulation/minerva-production-01-randomization.json --out build/generated/random-v1
```

Without `--randomize` nothing changes: the workcell's fixed layout and the grid, for regression. `scripts/sim/generate_parallel.py` passes the option to every worker, and each worker draws from its own seed range.

**The file.** `config/devices/simulation/minerva-production-01-randomization.json` (code: `simulation/layouts.py`). Every range is `{"range": [low, high], "source": ...}` and every region `{"value": [x low, x high, y low, y high]}` or `{"polygon": [[x, y], ...]}`, each source naming a training-randomization design choice of 2026-10-18 and what it is centred on. None is a measurement; re-centre them when the box and blocks are taped and weighed.

| What | Drawn uniformly over | Centred on |
|---|---|---|
| Block length, width, height | 30-55 mm each, on its own: cuboids | the depth check's blocks, about 37 x 52 and 47 x 48 mm across, tops 37-39 mm up (SIM_WORKCELL.md, "The head camera's depth") |
| Block mass | the workcell's mass per volume x the drawn volume (a placeholder: nothing weighed) | |
| Block yaw | -90 to 90 degrees (a cuboid looks the same every half turn) | square to the table |
| Block position | the cube region, a polygon in the base frame: where the left picks a 38.1 mm cube and hands it over with every move clear, the bucket out of the way, on the interim layout (SIM_DEMONSTRATOR.md, "Where the cube can be") | the left's reach |
| Bucket inside width, length | 130-200 mm, 200-300 mm | the depth's 155 x 242 mm |
| Bucket wall height | 150-220 mm | the depth's 189 mm rim |
| Bucket yaw | -49.4 to 10.6 degrees from square to the table | the depth's -19.4 degrees |
| Bucket position | the bucket region, a polygon: where some drop plans for the training sizes' corners (SIM_DEMONSTRATOR.md, "Drops"); its outer walls on the table and 3 cm clear of the block's footprint, drawn again until they are (at most 50 times) | the right's drop reach |
| Table group offset | x and y +/-4 cm, yaw +/-3 degrees about the tag rectangle's middle | the interim robot-to-rectangle placement (SIM_WORKCELL.md, "The robot's interim place") |
| Head camera | +/-2 cm along each of the rectangle's axes, +/-2 degrees about each of its own axes | its depth-anchored pose; the camera is on a temporary mount |

The table offset moves the table, the tags, the head camera calibrated against them, the bucket and the block together: the robot's place relative to the table is what is unknown. The user was told +/-3 cm; the coordinator widened it to +/-4 cm around the interim placement. Since the layout places the head camera, the appearance variation moves only the wrist cameras.

**Planning each draw.** The demonstrator plans every draw before it runs, after a check that the head camera stands clear of the robot (`generate.HeadCameraClearance`: with the arms hanging, no ray through its image meets the robot, and it sits at least 2 cm from it; a D435 is 25 mm deep). A draw that fails either (or one with no bucket place clear of the block) is drawn again, at most `plan_draws` (10) times per attempt, and each rejected draw is counted with its reason. Attempt k of a run with seed S draws from [S, k, draw], so a run repeats exactly.

**What an episode records.** Its metadata's `layout`: the seed, the block's size, mass, yaw and start, the bucket's size, height, yaw and centre (and where each was drawn before the table moved), the table offset and pivot, the head camera's shift, turns and resulting pose, and the rejected draws with their reasons. The scene's capabilities carry the whole drawn workcell, and the demonstrator's description the drop and the grasp it planned (`planned_drop`, `planned_grasp`: which face pair, how wide). The manifest lists each attempt's layout seed, layout and rejections.

**Measured on the CPU** (`scripts/sim/layout_trials.py`: through the receiver with driven grippers, no cameras; coolserver, 6 workers, seed 2000):
- At `0789f74`, with the head-camera check: 30 attempts, each found a layout within its 10 draws. Of 73 draws, 43 were refused (59%): 22 put the head camera within 2 cm of the robot or the robot in its view, 18 planned but came within 1 mm, 3 did not reach. Of the 30 layouts that ran, 29 succeeded (Wilson 95% 0.83-0.99) and one aborted (below). No commands were clipped; the largest arm-joint lag behind its command was 0.065 rad (left J5).
- At `d8240dd`, before the head-camera check: 24 attempts, 39 draws, 15 refused by the planner (38%: 13 came within 1 mm, 2 did not reach); 24 of 24 succeeded (Wilson 95% 0.86-1.00).
- The layouts that ran covered blocks 31-54 x 31-54 x 37-54 mm, buckets 130-197 mm wide, 204-297 mm long and 153-219 mm tall turned -49 to +8 degrees from the table, and table offsets -3.8 to +3.9 cm, -4.0 to +3.8 cm and -2.8 to +2.6 degrees. Most dropped from the front (24 of 30); the lower drop and the level fingers took 3 each.
- Planning took a median 15 s an attempt (at most 82 s, with refusals), a trial 108 s of wall time for 56 s of scene time.
- The abort: a block 50 x 47 x 48 mm at (0.331, -0.481) m, the bucket at (0.052, -0.565) m. The receiver stopped with `limit_violation` at 25.2 s, 1 s into the right's slow approach to take the block (after the pickup at 9.8 s and before the handoff). It repeats on the laptop with the same layout. No arm joint was within 0.1 rad of its range, so the trip was on a speed, effort or gripper reading; which one is not yet read out (`scripts/sim` repro in this worktree's notes, SIM_RANDOMIZED_LAYOUTS.md).
- Results: `build/sim-random/trials-0789f74/` and `trials-d8240dd/` in the `claude/sim-randomized-layouts` worktree (`attempts.jsonl`, `summary.json`).
- **Smoke run on coolserver GPU 0** (EGL, `~/aizee-sim/bin/launch_gen.sh` with `--randomize`, 3 workers, seed 3000, at `0789f74`): 3 episodes from 3 attempts in 206 s, 830-881 frames each, three 320 x 240 CAD views. Their head-camera contact sheets are in `build/sim-random/smoke-random-0789f74/`; the episodes in `~/aizee-sim/gen/smoke-random-0789f74/episodes/`.

**What the draws cannot cover.** The run draws again for:
- blocks under about 35 mm tall: the left backing out of the handoff passes within 1 mm of the right's fingers. Every refusal at that move had a block 31-36 mm tall, and every block that ran was at least 37 mm tall (SIM_DEMONSTRATOR.md, "Blocks that are not cubes"). The real blocks' tops are 37-39 mm up;
- tall buckets (walls near 220 mm) close to the robot, buckets on the handoff point, and buckets or blocks at the regions' far edges (SIM_DEMONSTRATOR.md, "Drops" and "Where the cube can be");
- head cameras within 2 cm of the robot or seeing it: the interim placement leaves the camera 3.5 cm in front of the torso, and a draw moves it up to 4 cm with the table and 2 cm about its pose. Of 400 draws (no planning) 29% were refused so, which holds the table's drawn shift toward the robot to 3.3 cm (`HeadCameraClearance`).

The refusal rate is the cost; the ranges were not shrunk to hide it. In the first smoke run, before the head-camera check, one episode of three had the robot's torso across half of its head view.

## What an episode says about itself

`data_source` simulation, `evidence` synthetic, `authority` policy on every row, and `collection_mode` scripted_demonstrator. The metadata also includes:
- the demonstrator's description;
- the scene's capabilities: the workcell with its placeholders, the gripper drives, the scene hashes;
- the plant's provenance;
- the run configuration and git identity;
- the trial's cube start.

The writer leaves `training_promoted` and `sim2real_validated` false. The writer's `capture_eligible` only says that capture's own checks passed. Whether scripted simulation episodes may train a policy is the dataset contract's decision, not the generator's.

Today's training loader (`python/training/minerva_dataset.py`) trains on human and correction rows by default. It finds the episodes, then rejects them: "No valid training frames found". Allowing the policy authority explicitly (`allowed_authorities=('policy',)`) loads the nominal episode as 658 samples, with all three cameras and 14 joints. `train_minerva.py` has no option for that, so using them stays an explicit decision.

## Measured

- **One episode, nominal cube start.** 659 frames over 43.9 s, 49.5 MB with three 320x240 CAD views (gzip).
  - While a gripper holds the cube, its reading stops at the cube's width: 2.05 rad for the 38.1 mm cube, against 4.24 rad open.
  - The jaw contacts pass from the left gripper to the right at the handoff.
- **Wall time.** About 100 s per attempt on the development laptop, mostly the stepped receiver and the CAD renders.
- **A batch of 120, generated at `355b785`.** Cube yaw drawn from +/-45 degrees, varied look, 4 workers of 30 over a 6x5 grid with full jitter. 120 episodes from 121 attempts in 2.5 h, 5.3 GB.
  - The one other attempt was refused before it moved: no plan cleared the table and bucket for a cube turned 40 degrees at the region's near edge.
  - The episodes start on two wrist-roll branches a turn apart, left J5 at -3.14 rad (85) or +3.14 rad (35): the same pose, recorded as different joint angles.
  - They predate the finger collision and the quarter-turn take ([SIM_DEMONSTRATOR.md](SIM_DEMONSTRATOR.md)): the cube touched each finger only at its 30 mm tip box, and at the handoff the grippers' inner fingers cross behind the cube. Regenerate before training on them.
- **A batch of 120, regenerated at `e8aa16c`** with the fingers' paddles colliding, the arm-against-arm audit and the quarter-turn take, from the same seeds and worker settings as the batch above: 120 episodes from 121 attempts in 3.0 h, 6.0 GB. The far corner that refused before `e8aa16c` now plans (one arm at a time). The one refusal was a cube turned 40 degrees at the region's near-left edge (x 0.046, y -0.323), as in the first batch.

## Many at once

`scripts/sim/generate_parallel.py` runs several generate workers, each with its own receiver, scene, grid seeds (SEED + 100 k) and plant seed, then hard-links their episodes into one folder with one manifest:

```powershell
.venv-dev/Scripts/python.exe scripts/sim/generate_parallel.py --out build/generated/batch --episodes 120 --workers 4 -- --grid 6x5 --jitter 1 --yaw 0.785398
```

Cube poses go row by row through the grid, so give each worker whole passes (episodes per worker = grid cells); a partial pass leaves out the last rows. On the 16 GB development laptop, four workers used most of the memory, and a training run beside them ran it out.

## Limits

Everything in [SIM_WORKCELL.md](SIM_WORKCELL.md) and [SIM_DEMONSTRATOR.md](SIM_DEMONSTRATOR.md) applies:
- the bench's positions are placeholders;
- the camera poses are not calibrated;
- the demonstrator plans from the cube's true start;
- its motion is scripted, not human.

A policy trained only on these episodes learns this demonstrator's style. Human demonstrations from `helm --sim` and the robot are the other half of the mix.

## Tests

- `tests/unit/test_layouts.py`: every range cites its design choice and what it is centred on; a file that cannot be drawn from is refused; a seed draws the same layout and another seed another; 200 draws stay in their ranges and regions, keep the bucket on the table and clear of the block, keep the block's mass a placeholder scaled with its volume, and move the table group as one with the head camera about its calibrated pose; a polygon region is drawn from uniformly.
- `tests/contract/test_generate_layouts.py` (through the actual receiver): a refused draw is drawn again and counted; the trial runs on the layout's own scene; a recorded episode carries the drawn layout, and the look's variation leaves the head camera where the layout put it; `generate` with layouts lists each attempt's layout and rejections.

`tests/contract/test_generate.py` (in the gate), through the actual receiver:
- a trial that fails leaves no episode and no partial file;
- the recorder publishes a trial that succeeded with every field a real capture has. That includes 14 axes, three camera roles, increasing receiver times, finite efforts and policy authority.
