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
| Block length, width | 30-55 mm each, on its own: cuboids | the depth check's blocks, about 37 x 52 and 47 x 48 mm across (SIM_WORKCELL.md, "The head camera's depth") |
| Block height | 35-55 mm (v3; shorter blocks fail the left's retreat, SIM_DEMONSTRATOR.md, "Follow-ups") | the real blocks' tops, 37-39 mm up |
| Block mass | the workcell's mass per volume x the drawn volume (a placeholder: nothing weighed) | |
| Block yaw | -90 to 90 degrees (a cuboid looks the same every half turn) | square to the table |
| Block position | the cube region, a polygon in the base frame: where the left picks a 38.1 mm cube and hands it over with every move clear, the bucket out of the way, on the interim layout (SIM_DEMONSTRATOR.md, "Where the cube can be") | the left's reach |
| Bucket inside width, length | 130-200 mm, 200-300 mm | the depth's 155 x 242 mm |
| Bucket wall height | 150-220 mm | the depth's 189 mm rim |
| Bucket yaw | -49.4 to 10.6 degrees from square to the table | the depth's -19.4 degrees |
| Bucket position | the bucket region, x -0.17 to 0.075 m and y -0.57 to -0.72 m in the base frame (v3): the part of the drop-mapped region where the head camera sees a drawn bucket's opening most often and the demonstrator plans; its outer walls on the table and 3 cm clear of the block's footprint, drawn again until they are (at most 50 times) | the head camera's view, which holds the depth-measured box at (-0.155, -0.587) m |
| Table group offset | x and y +/-4 cm, yaw +/-3 degrees about the tag rectangle's middle | the interim robot-to-rectangle placement (SIM_WORKCELL.md, "The robot's interim place") |
| Head camera | +/-2 cm along each of the rectangle's axes, +/-2 degrees about each of its own axes | its depth-anchored pose; the camera is on a temporary mount |
| Robot grey | 0.19-0.25 on each channel (v3.1) | the user's look's 0.22 (MINERVA_TASK_SCENE.md, "Room and look") |
| Room tone | x 0.92-1.08 on each channel of the room backdrop, each drawn on its own (v3.1) | 1: the backdrop as painted |
| Floor tone | x 0.90-1.10 on each channel of the floor plane's wood (v3.1) | 1: the wood as painted |
| Light warmth | 3900-5100 K, at the lights' same brightness (v3.1) | the user's look's 4500 K |
| Head view (a check, not a draw) | every accepted layout's head image holds the bucket's whole opening: each corner of its inside rim at least 5 px inside 320 x 240 (10 px at the camera's 640 x 480), the layout's own camera pose applied (v3; `head_view.bucket_opening`) | the depth-measured box passes, about 9 px from the image's right edge |

The table offset moves the table, the tags, the head camera calibrated against them, the bucket and the block together: the robot's place relative to the table is what is unknown. The user was told +/-3 cm; the coordinator widened it to +/-4 cm around the interim placement. Since the layout places the head camera, the appearance variation moves only the wrist cameras.

**Planning each draw.** The demonstrator plans every draw before it runs, after the checks of `generate.layout_checks`: that the head camera stands clear of the robot (`HeadCameraClearance`: with the arms hanging, no ray through its image meets the robot, and it sits at least 2 cm from it; a D435 is 25 mm deep), and, when the file asks for it, that its image holds the bucket's whole opening (`BucketInView`, `workcell.bucket_opening_pixels`). A draw that fails any of them (or one with no bucket place clear of the block) is drawn again, at most `plan_draws` (10) times per attempt, and each rejected draw is counted with its reason. Attempt k of a run with seed S draws from [S, k, draw], so a run repeats exactly.

**What an episode records.** Its metadata's `layout`: the seed, the block's size, mass, yaw and start, the bucket's size, height, yaw and centre (and where each was drawn before the table moved), the table offset and pivot, the head camera's shift, turns and resulting pose, the drawn look (`look`: the robot's grey, the room's and the floor's tones, the lights' kelvin; also the scene capabilities' `look`), and the rejected draws with their reasons. The scene's capabilities carry the whole drawn workcell, and the demonstrator's description the drop and the grasp it planned (`planned_drop`, `planned_grasp`: which face pair, how wide). The manifest lists each attempt's layout seed, layout and rejections.

**The v3 layouts** (the coordinator's decisions, accepted by the user, 2026-10-18):
- **The bucket's opening in the head image.** Every accepted layout's head image holds the bucket's whole opening, with 5 px to spare at 320 x 240. The margin is kept under the depth-measured box's own: it sits about 9 px from the image's right edge, and a wider margin would leave the real layout out.
- **The bucket region kept to the head view.** Over the earlier region only 17% of drawn buckets showed their whole opening at 10 px (4000 draws, no planning). A map over its part right of x -0.30 m with every draw planned (360 draws from seed 7000, `layout_refusals.py map --plan-all`; `build/sim-random/refusals-v3/candidates-plan-all.jsonl` in this worktree) picked x -0.17 to 0.075 m, y -0.57 to -0.72 m: there 75% of draws kept the head camera clear, 68% showed the opening with 5 px to spare, 67% planned, and 35% passed all three. Right of it the opening leaves the image's right edge, beyond it the top edge, and nearer the robot the bucket is in the handoff's way.
- **Blocks 35-55 mm tall;** footprints stay 30-55 mm. The retreat under 35 mm is a follow-up (SIM_DEMONSTRATOR.md, "Follow-ups").
- **The robot's interim place stays 0.36 m from the rectangle,** with its cost: a table drawn more than about 2 cm toward the robot puts the head camera against it and is refused, so such tables are a third as dense in the episodes as in the draws.
- **The take abort** (below) is a follow-up with a saved reproduction (SIM_DEMONSTRATOR.md, "Follow-ups"); a generator drops such an attempt.
- **Measured** (`layout_refusals.py map`, planning only, 240 draws from seed 5000 on the v3 files; `build/sim-random/refusals-v3/` in this worktree): 34% accepted (Wilson 95% 0.28-0.40). 28% put the head camera against the robot or the robot in its view, 20% showed less than the bucket's whole opening, 12% planned but came within 1 mm (mostly the left's way to the handoff through the bucket, or the right unable to carry over it), 2% did not reach, and 4% found no bucket place clear of the block. Of the draws past the camera checks, 70% planned.
- **Bias** (density in the episodes against the uniform draws): the table moved over 2 cm toward the robot 0.4x; blocks beyond y -0.65 m 0.5x (the bucket is then in the left's way); bucket centres right of x -0.13 m 0.5x and walls over 200 mm 0.7x (the opening leaves the image); blocks nearer than y -0.53 m 1.5x and walls under 170 mm 1.4x. Block footprints, bucket width, length and yaw, and the table's yaw and shift across come out about even.

**Measured on the CPU before v3** (`scripts/sim/layout_trials.py`: through the receiver with driven grippers, no cameras; coolserver, 6 workers, seed 2000):
- At `0789f74`, with the head-camera check: 30 attempts, each found a layout within its 10 draws. Of 73 draws, 43 were refused (59%): 22 put the head camera within 2 cm of the robot or the robot in its view, 18 planned but came within 1 mm, 3 did not reach. Of the 30 layouts that ran, 29 succeeded (Wilson 95% 0.83-0.99) and one aborted (below). No commands were clipped; the largest arm-joint lag behind its command was 0.065 rad (left J5).
- At `d8240dd`, before the head-camera check: 24 attempts, 39 draws, 15 refused by the planner (38%: 13 came within 1 mm, 2 did not reach); 24 of 24 succeeded (Wilson 95% 0.86-1.00).
- The layouts that ran covered blocks 31-54 x 31-54 x 37-54 mm, buckets 130-197 mm wide, 204-297 mm long and 153-219 mm tall turned -49 to +8 degrees from the table, and table offsets -3.8 to +3.9 cm, -4.0 to +3.8 cm and -2.8 to +2.6 degrees. Most dropped from the front (24 of 30); the lower drop and the level fingers took 3 each.
- Planning took a median 15 s an attempt (at most 82 s, with refusals), a trial 108 s of wall time for 56 s of scene time.
- The abort: a block 50 x 47 x 48 mm at (0.331, -0.481) m, the bucket at (0.052, -0.565) m; it repeats with the same layout. The receiver stopped with `limit_violation` at 25.15 s, on the right gripper's `position_above_upper`: 5.2823 rad against its 5.2791 rad limit. 0.64 s into `right turns` (the right turning its open jaws upright where it waits, before it comes in), one of its fingers met the block in the left's grip, and the block forced the jaws open: the drive's reading rose from 4.798 rad, where it had held the jaws open, past the limit in 0.25 s (the scene's drive hinge has no range of its own; it follows the jaws through a stiff but soft coupling). Nothing touched the bucket or the table. The plan's clearance check leaves the held block out, and the right waits 6 cm back along its fingers (`APPROACH`) whatever the block's size. Not fixed here; the candidates are checking the right's turn and approach against the held block, or backing it off by the block's size.
- Results: `build/sim-random/trials-0789f74/` and `trials-d8240dd/` in the `claude/sim-randomized-layouts` worktree (`attempts.jsonl`, `summary.json`).
- **Smoke run on coolserver GPU 0** (EGL, `~/aizee-sim/bin/launch_gen.sh` with `--randomize`, 3 workers, seed 3000, at `0789f74`): 3 episodes from 3 attempts in 206 s, 830-881 frames each, three 320 x 240 CAD views. Their head-camera contact sheets are in `build/sim-random/smoke-random-0789f74/`; the episodes in `~/aizee-sim/gen/smoke-random-0789f74/episodes/`.

**Where the refusals fell before v3.** `scripts/sim/layout_refusals.py map` draws layouts as `generate --randomize` does and puts each through the head-camera check and the demonstrator's planning, without a trial; `report` gives the tables. 240 draws from seed 5000 at `0789f74` (coolserver, 6 workers; `build/sim-random/refusals-0789f74/` in this worktree): 42% were accepted (Wilson 95% 0.36-0.49; the trials above refused 59%). 30% put the head camera within 2 cm of the robot or the robot in its view, 22% planned but came within 1 mm, 5% did not reach, and none ran out of bucket places. An attempt keeps its first accepted draw, so a generated set's layouts are the drawn ones weighted by their chance of acceptance. Against the uniform draws, that weight is:
- **Table moved toward the robot: 0.3x.** A head camera moved more than 2 cm toward the robot was refused 83% of the time; one moved away, never (1.2-1.5x). The camera moves with the table group and by its own shift; the interim placement leaves it 3.5 cm in front of the torso. A table moved away is refused by the planner more often (57% of those past the camera check, against 27-32%), so a table near its interim place is the densest (1.5x).
- **Blocks under 34 mm tall: none.** 30-34 mm tall planned 0 times in 23, 34-36 mm 2 in 8, 36-46 mm 49 in 71 and 46-55 mm 51 in 66. Up to 35 mm, the left backing out of the handoff passes within 1 mm of the right's fingers (SIM_DEMONSTRATOR.md, "Blocks that are not cubes"). The real blocks' tops are 37-39 mm up.
- **A bucket beside the handoff: 0.2x.** Centres drawn at x -0.15 to 0.05 m and y -0.59 to -0.47 m planned 3 times in 23 (the left's way to the handoff runs through the bucket, or the right cannot carry over it), against 83-89% at x -0.31 to -0.15 m.
- **Blocks far out: 0.7x.** Drawn beyond y -0.65 m, 20 planned in 47: the left's arm comes within 1 mm of the table or the bucket on its way, or cannot lift the block. Nearer, 28 in 53 at x below 0.13 m (0.9x; there the left cannot always climb before the handoff) and 54 in 68 above it (1.3x).
- **About even (0.8-1.2x, within the counts' noise):** the bucket's width, length, height and yaw, the block's footprint, the table's yaw and its shift across. Tall buckets near the robot do not plan (SIM_DEMONSTRATOR.md, "Drops"), but that corner is small: of the 168 draws past the camera check, 2 had walls over 210 mm and a centre at x -0.31 to -0.07 m nearer than y -0.59 m, and neither planned.

**What the head camera saw before v3.** `layout_refusals.py view` projects the bucket and the block through the head camera at 320 x 240 (fovy 42.65 degrees, so 55 degrees across) with the arms hanging. The workcell file's own box, where the depth put it, is wholly in view, as in the real frame (`build/sim-random/head-interim-0.36.png`). Of the map's 102 accepted draws, the whole bucket is in view in 21%, at least half of its walls' corners in 74% and none in 3%; the middle of its opening in 61, and the block in 100. The openings out of view are past the image's right edge, the robot's right: for buckets drawn right of x -0.25 m the opening is in view in 14 of 53 (none of the 23 also nearer than y -0.65 m), left of it in 47 of 49. So about 4 episodes in 10 start without the head camera showing where the block goes; two of the smoke run's three sheets are such. The user chose both for v3 (above).

The refusal rates are the cost; the ranges were not shrunk to hide them. Weighting each episode by the inverse of its layout's acceptance, or drawing the thin parts more often, would even the set out; neither is done here. Most accepted layouts drop from the front (75 of 102); the lower drop took 14, the level fingers 12, from above 1. In the first smoke run, before the head-camera check, one episode of three had the robot's torso across half of its head view.

**The room look (v3.1).** The user approved the dark grey robot and the house room (MINERVA_TASK_SCENE.md, "Room and look") for a fine-tune of v3's last checkpoint, v3.1 (2026-10-18). The randomization file's `look` varies it per layout, narrowly and centred on the user's look (the four rows above). It is drawn after everything else, so a seed draws the same table, bucket, block and head camera as without it: v3.1's layouts are v3's for the same seeds, in the new look. The drawn workcell carries the look to its scene. A file without `look` keeps the user's look for every layout; the existing appearance variation (lights' strength, colours, wrist cameras) runs on top as before.

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
