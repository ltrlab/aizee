# Training server scripts

These scripts run on the training server (coolserver: Ubuntu 24.04, two RTX 3090s). They generate simulation
episodes, train and evaluate policies, and train LeRobot ACT on real demonstrations. They are not part of the
`robot_platform` package and never run on the robot. Until 2026-11-01 they lived only in `~/aizee-sim/bin` on the
server; this folder is now their source.

## The server's folder

Everything lives under `$AIZEE_SIM` (default `~/aizee-sim`), and nothing is installed system-wide. Copy this folder's
files into `$AIZEE_SIM/bin`: the launchers call their helpers there.

| Path | Contents |
| --- | --- |
| `bin/` | These scripts, plus `control_harness` and `librobot_collision.so` built for x86-64 Linux (below). |
| `tree-<commit>/` | Repository source trees at a commit, for generation and evaluation (`sim_env.sh` points `PYTHONPATH` at one). |
| `train-<commit>/` | A copy of the training source (`python/training`) that `sim_train.py` runs. |
| `.venv`, `.venv-train`, `.venv-lerobot` | Simulation; the in-house trainer (torch); LeRobot (`setup_lerobot.sh`). |
| `gen/`, `ckpt/`, `logs/`, `data/`, `export/`, `eval/`, `tb/` | Generated episodes, checkpoints, logs, datasets, exports, evaluations, TensorBoard. |

## Real demonstrations: LeRobot and ACT

1. Export kept episodes on any machine with the repository: `python -m robot_platform.data.lerobot_export`
   ([DATASET_PIPELINE.md](../../docs/development/DATASET_PIPELINE.md), [EPISODE_FORMAT.md](../../docs/EPISODE_FORMAT.md)).
2. `setup_lerobot.sh` builds `.venv-lerobot`: LeRobot at openpi's pinned revision `0cf8648` (dataset format v2.1), with
   the av, datasets and huggingface-hub versions that revision needs.
3. `check_lerobot_dataset.py <dataset> <source episode>` loads the export with LeRobot itself and compares sampled frames
   with the source episode: timestamps, state, action, format 9's extras and decoded video quality.
4. `train_act.sh` trains ACT in tmux; its defaults are the 2026-10-22 smoke run's.
5. `python -m robot_platform.data.lerobot_policy export` turns a checkpoint into the TorchScript policy the robot loads.

## Simulation: generation, training and evaluation

These need a source tree from the simulation line (`claude/sim-room-look` and its ancestors), which has
`python -m robot_platform.evaluation generate`, the scripted demonstrator and the layout sampler. The current
integration branch does not have them yet.

| Script | Does |
| --- | --- |
| `sim_env.sh` | Sourced: the source tree, native binaries, MuJoCo on GPU 0 through EGL, one BLAS thread per process. |
| `launch_gen.sh` | Starts a generation run in tmux: `gen_parallel.py` with disjoint seeds per worker. |
| `gen_parallel.py` | Runs several `evaluation generate` workers and gathers their episodes into one folder and manifest. |
| `gen_status.sh` | A generation run's progress. |
| `launch_train.sh` | Starts `sim_train.py` in tmux on one GPU. |
| `sim_train.py` | Runs `python/training/train_minerva.py` on generated episodes, with per-step timing. A pipeline proof, never a robot policy. |
| `chain_train.sh`, `chain_finetune.sh` | Wait for a generation (and a base run) to finish, then train or fine-tune. |
| `tb_sync.py`, `usage_sampler.py` | TensorBoard from a run's per-epoch records; CPU and GPU use per session. |
| `eval_v3.py` | Closed-loop evaluation of checkpoints on held-out randomized layouts, with the repository's harness. |
| `export_ema_fp16.py`, `export_obs.py` | A checkpoint's EMA weights in FP16, and one real observation, for the Jetson inference timing test. |

## Native binaries

The simulation and the contract tests use the Rust receiver and collision library built for the server. Build them
from the same commit as the source tree, with the repository's Rust toolchain (1.98.1) under `$AIZEE_SIM/rust`:

```bash
cargo build --locked -p robot_control --bin control_harness --release
cargo build --locked -p robot_collision --release
```

Copy `target/release/control_harness` and `target/release/librobot_collision.so` into `$AIZEE_SIM/bin`.
`sim_env.sh` points `ROBOT_CONTROL_HARNESS` and `ROBOT_COLLISION_LIBRARY` at them. A build stays valid for another
commit while `git rev-parse <commit>:rust` and `<commit>:Cargo.lock` are unchanged. The 2026-10-18 build (from
`01e2198`) passed the Rust tests and 20 contract test files on the server.

## Not kept here

- `lerobot_policy_draft.py`, `export_lerobot_policy.py` and `check_lerobot_policy.py` were drafts of what is now
  `src/robot_platform/data/lerobot_policy.py`.
- `v31_watch.sh` watched one pair of runs (v3 and v3.1).
- `BUILD_INFO.json` recorded the 2026-10-18 build; its commands and checks are in "Native binaries" above.
