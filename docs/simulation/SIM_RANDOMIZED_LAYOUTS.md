# Randomized training layouts

**What it is.** `python -m robot_platform.evaluation generate --randomize <file>` draws a different layout for every
generated episode: block size and pose, bucket size and pose, table placement and the head camera, from one sourced
randomization file (`config/devices/simulation/minerva-production-01-randomization.json`). The demonstrator plans each
draw and the generator draws again when it cannot. **Status: done and merged.** The draw and its measured rejection rates
are in [SIM_GENERATION.md](SIM_GENERATION.md), "Randomized layouts"; this page keeps what it changed and what is open.

## Why

- With the placeholder `robot_offsets` the calibrated head camera sat inside the robot's torso (155 of 169 rays through
  its image met the torso), and on the depth-measured workcell the demonstrator found no plan.
- Generated episodes all shared one table, box and block: `generate` only moved the cube over a grid.
- The head camera showed the middle of the bucket's opening in only 61 of 102 accepted layouts and its whole opening in
  21%.

## What it changed

- `simulation/layouts.py` and the randomization file (new); `generate --randomize`; `draw_layout`, `HeadCameraClearance`,
  `BucketInView`, `LayoutChecks`, `layout_checks`; `EpisodeRecorder.annotate` records the layout in the episode metadata.
- A cuboid block (`Workcell.cube_size/block`), the table's yaw (`table_yaw`, `on_table`/`table_frame`), and a robot offset
  that clears the torso with margin (an interim placeholder, 0.325 m and 0.36 m).
- The demonstrator tries five more drops before refusing a layout (`DROPS`), and splits the handoff (`_to_handoff`,
  `_drop`).
- The default workcell's robot offset, table, nominal start and cube region are the interim layout; `helm --sim`, the
  harness and generation without `--randomize` all use it.
- With `--randomize`, a draw is refused unless its head image holds the bucket's whole opening with 5 px to spare at
  320 x 240; buckets are drawn only where the head camera sees them (x -0.17 to 0.075 m, y -0.57 to -0.72 m) and blocks
  are 35-55 mm tall.
- Scripts in `scripts/sim/`: `trial_trace.py`, `layout_trials.py`, `layout_refusals.py`, `repro_take_abort.py`.

## Bounds the tests hold

- Layouts are reproducible from their seed, stay inside their ranges and regions, keep the bucket on the table and 3 cm
  clear of the block, and move the table group rigidly. An episode records its layout.
- A drawn layout runs only if the demonstrator plans it, its head camera stands clear of the robot, and its head image
  holds the bucket's whole opening.
- The nominal trial is bit for bit the same across these changes.
- Tests: `test_layouts.py`, `test_generate_layouts.py`, plus additions to `test_workcell.py` and `test_demonstrator.py`;
  fixtures `tests/fixtures/minerva_workcell_placeholder_20261017.json` and `minerva_layout_take_abort_20261018.json`.

## Not known

- **The robot's place.** `robot_offsets` is an interim placeholder; tape numbers or the depth registration
  ([SIM_WORKCELL.md](SIM_WORKCELL.md)) replace it, and then the cube region, nominal start and both randomization
  regions need re-planning.
- **The blocks and the box.** Size, mass and friction; the ranges re-centre on them when measured.
- **Coverage and bias.** 66% of draws are refused and drawn again, unevenly: 28% put the head camera against the robot,
  20% show less than the bucket's whole opening, 14% do not plan, 4% find no bucket place. Against uniform draws, a set
  has 0.4x of the tables moved over 2 cm toward the robot, 0.5x of the blocks beyond y -0.65 m, 0.5x of the bucket
  centres right of x -0.13 m and 0.7x of the walls over 200 mm.
- **The head view's margin.** The real box sits about 9 px inside the head image's right edge at 320 x 240, so the
  required margin is 5 px; a calibration error of a degree or two moves it by 5-10 px. The robot's arms are seen from a
  camera whose place on the robot is a guess; the wrist cameras are uncalibrated.
- **One trial in 30 aborted** ([SIM_DEMONSTRATOR.md](SIM_DEMONSTRATOR.md), "Follow-ups"): turning its open jaws upright
  beside a held 50 mm block, the right forced its jaws past their range and the receiver tripped on the right gripper's
  position limit. Not fixed; `scripts/sim/repro_take_abort.py` reproduces it, and a generator drops such an attempt.
- **Blocks under 35 mm** are no longer drawn; the left's retreat past them is a planner follow-up.
