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

## What an episode says about itself

`data_source` simulation, `evidence` synthetic, `authority` policy on every row, and `collection_mode` scripted_demonstrator. The metadata also includes:
- the demonstrator's description;
- the scene's capabilities: the workcell with its placeholders, the gripper drives, the scene hashes;
- the plant's provenance;
- the run configuration and git identity;
- the trial's cube start.

The writer leaves `training_promoted` and `sim2real_validated` false. The writer's `capture_eligible` only says that capture's own checks passed. Whether scripted simulation episodes may train a policy is the dataset contract's decision, not the generator's.

## Measured

- **One episode, nominal cube start.** 659 frames over 43.9 s, 49.5 MB with three 320x240 CAD views (gzip).
  - While a gripper holds the cube, its reading stops at the cube's width: 2.05 rad for the 38.1 mm cube, against 4.24 rad open.
  - The jaw contacts pass from the left gripper to the right at the handoff.
- **Wall time.** About 100 s per attempt on the development laptop, mostly the stepped receiver and the CAD renders.

## Limits

Everything in [SIM_WORKCELL.md](SIM_WORKCELL.md) and [SIM_DEMONSTRATOR.md](SIM_DEMONSTRATOR.md) applies:
- the bench's positions are placeholders;
- the camera poses are not calibrated;
- the demonstrator plans from the cube's true start;
- its motion is scripted, not human.

A policy trained only on these episodes learns this demonstrator's style. Human demonstrations from `helm --sim` and the robot are the other half of the mix.

## Tests

`tests/contract/test_generate.py` (in the gate), through the actual receiver:
- a trial that fails leaves no episode and no partial file;
- the recorder publishes a trial that succeeded with every field a real capture has. That includes 14 axes, three camera roles, increasing receiver times, finite efforts and policy authority.
