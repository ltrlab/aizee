# Learning Pipeline

End-to-end guide for collecting demonstrations, training an ACT policy, validating it offline, and deploying it on the robot.

## Overview

```
1. Calibrate        so101_calibrate.py          (one-time, per leader arm)
2. Collect demos    collect_demo.py             (record HDF5 episodes via SO-101 leader)
3. Train            train.py                    (ACT model on collected episodes)
4. Evaluate         evaluate_policy.py          (offline replay through model, compare to GT)
5. Deploy           act_policy_node.py          (live 20 Hz inference loop on Jetson)
```

## Prerequisites

| Component | Purpose |
|---|---|
| Jetson running `aizee-motor-control-rover` | CAN bus motor control + ZMQ telemetry |
| Both arm cameras streaming | Left `:5563`, right `:5564` (USB D435 on Jetson) |
| SO-101 leader arm | USB serial, for teleoperation during recording |
| `pip install -r requirements.txt` | h5py, torch, rerun-sdk, etc. |
| `config/so101_calibration.json` | Joint-to-joint mapping (see Calibration below) |

---

## 1. Calibration

The SO-101 leader arm must be calibrated once so its joint angles map correctly to the AIZEE arm joints. Re-calibrate if the leader is physically modified or the mapping drifts.

```bash
python python/scripts/so101_calibrate.py --port COM4
```

This walks through each joint interactively and writes `config/so101_calibration.json`. The file contains per-joint:

| Field | Meaning |
|---|---|
| `id` | Dynamixel servo ID on the SO-101 |
| `aizee` | Corresponding AIZEE joint name |
| `rad_min` / `rad_max` | Joint travel range in radians |
| `zero_offset` | Radians subtracted before direction multiply |
| `direction` | `+1` or `-1` sign flip between SO-101 and AIZEE conventions |

**SO-101 → AIZEE joint mapping:**

| SO-101 | AIZEE |
|---|---|
| shoulder_pan | swivel |
| shoulder_lift | gantry_base |
| elbow_flex | gantry_mid |
| wrist_flex | gantry_end |
| wrist_yaw | wrist_pitch |
| wrist_roll | wrist_roll |
| gripper | gripper |

**Quick check:** after calibrating, use `python python/scripts/so101_teleop.py --port COM4` to verify the mapping looks correct before recording episodes.

---

## 2. Data Collection

### Starting the recorder

```bash
# Full command with SO-101 leader arm
python python/scripts/collect_demo.py --port COM4

# Specify output directory
python python/scripts/collect_demo.py --port COM4 --output-dir episodes/pick_task

# Dry-run (no files saved)
python python/scripts/collect_demo.py --port COM4 --dry-run

# Without leader arm (hold mode only — useful for testing cameras)
python python/scripts/collect_demo.py
```

All CLI arguments:

| Argument | Default | Description |
|---|---|---|
| `--port` | None | SO-101 serial port (e.g. `COM4`, `/dev/ttyACM0`). Without it, leader tracking is disabled. |
| `--baud` | 1000000 | SO-101 baud rate |
| `--calib` | `config/so101_calibration.json` | Calibration file |
| `--cmd` | from `teleop.yaml` | ZMQ command endpoint |
| `--telem` | from `teleop.yaml` | ZMQ telemetry endpoint |
| `--cam-left` | `tcp://192.168.0.27:5563` | Left arm camera |
| `--cam-right` | `tcp://192.168.0.27:5564` | Right arm camera |
| `--output-dir` | `episodes` | Output directory |
| `--max-steps` | 10000 | Max frames per episode (at 20 Hz = 500 s) |
| `--image-size` | `240x320` | Image resolution H×W |
| `--max-delta` | 0.3 | Per-step safety clamp (rad) |
| `--dry-run` | off | Don't save anything |

### Controls

| Key | Action |
|---|---|
| **E** | Enable motors — starts TRACKING (with leader) or HOLD (without) |
| **I** | Idle — enable with zero torque, arm floats freely |
| **H** | Toggle TRACKING ↔ HOLD |
| **R** | Toggle recording on/off |
| **X** | Soft shutdown — hold 1 s, ramp to zero, disable |
| **Z** | Capture current SO-101 pose as zero reference |
| **M** | Mirror — set zero offset so current leader pose → current arm pose |
| **Q** | Quit |

Gamepad: A=enable, B=shutdown, Start=hold, Back=quit.

### Recording workflow

1. **Position the arm** — press **I** to idle (arm floats), physically place it at the task starting pose.
2. **Enable tracking** — press **E**. The arm now follows the SO-101 leader.
3. **Start recording** — press **R**. The status line shows `[REC]` and the frame counter.
4. **Perform the task** — move the leader through the desired motion.
5. **Stop recording** — press **R** again. The episode is auto-saved as `episode_XXXX.hdf5`.
6. **Repeat** — record as many episodes as needed. Files are auto-numbered.

Recording samples at **20 Hz** (the main control loop runs at 30 Hz but sub-samples). A frame is only captured when both cameras are fresh (< 500 ms old) and telemetry is available. Stale frames are dropped and counted.

Camera images are decoded in a background thread so JPEG parsing doesn't delay motor commands. Episode saving is also backgrounded so gzip compression doesn't block the UI.

### Episode HDF5 format (format_version=2)

```
episode_XXXX.hdf5
├── attrs:
│     hz=20
│     format_version=2
│     arm_joints="swivel,gantry_base,gantry_mid,gantry_end,wrist_pitch,wrist_roll,gripper"
│     action_space="absolute"
├── observations/
│   ├── qpos          float32  [T, 7]    actual motor positions (swivel = column 0)
│   ├── qcmd          float32  [T, 7]    commanded positions
│   ├── torques       float32  [T, 7]    motor torques
│   └── images/
│       ├── left      uint8    [T, 240, 320, 3]
│       └── right     uint8    [T, 240, 320, 3]
├── actions           float32  [T, 7]    = qcmd[1:] padded (next-step target)
└── timestamps/
    ├── telem         float64  [T]
    ├── camera_left   float64  [T]
    └── camera_right  float64  [T]
```

**Actions** are derived as one-step-ahead commanded positions: `actions[t] = qcmd[t+1]`. This is what the controller actually targeted next, not the raw leader position — avoids gravity sag artifacts.

**Swivel is column 0** of every 7-dim vector (qpos, qcmd, torques, actions). This lets the policy learn a single joint vector end-to-end instead of treating the swivel as a separate channel that must be wired up at deploy time.

### Inspecting episodes

```bash
# Rerun visualization (offline, no hardware needed)
python python/scripts/view_episode.py episodes/episode_0000.hdf5

# Save to .rrd for later viewing
python python/scripts/view_episode.py episodes/episode_0000.hdf5 --save episode.rrd
```

### Tips for good demonstrations

- **Consistency** — start and end each episode at the same pose if possible.
- **Slow, smooth motions** — jerky leader movements produce noisy actions.
- **10-30 episodes** is a reasonable starting point for simple tasks; complex tasks may need 50+.
- **Verify cameras** — check `view_episode.py` output to confirm both cameras captured the workspace clearly.
- **Discard bad episodes** — simply delete the `.hdf5` file before training.

---

## 3. Training

### Basic training

```bash
python python/training/train.py --data-dir episodes/ --output-dir checkpoints/
```

### Full argument reference

| Argument | Default | Description |
|---|---|---|
| `--data-dir` | **required** | Directory containing `episode_*.hdf5` |
| `--output-dir` | `checkpoints` | Checkpoint output directory |
| `--epochs` | 200 | Total training epochs |
| `--batch-size` | 32 | Batch size |
| `--chunk-size` | 32 | Number of future actions predicted per sample |
| `--lr` | 1e-4 | Learning rate (non-backbone) |
| `--lr-backbone` | 1e-5 | Learning rate for ResNet18 backbone (10× lower) |
| `--weight-decay` | 1e-4 | AdamW weight decay |
| `--kl-weight` | 10.0 | KL divergence weight |
| `--d-model` | 256 | Transformer hidden dimension |
| `--dim-feedforward` | 2048 | Transformer FFN dimension |
| `--z-dim` | 32 | CVAE latent dimension |
| `--nhead` | 8 | Attention heads |
| `--num-encoder-layers` | 4 | CVAE encoder layers |
| `--num-decoder-layers` | 7 | DETR decoder layers |
| `--state-mode` | `qpos_qcmd` | State layout: `qpos`, `qpos_qcmd`, `qpos_qcmd_tq` |
| `--action-mode` | `relative` | `absolute` = predict joint targets; `relative` = predict `(target − qpos)` |
| `--augment` | off | Enable train-time image augmentation (geometric crop + color jitter) |
| `--val-fraction` | 0.15 | Fraction of episodes held out for validation (0 disables) |
| `--val-seed` | 0 | Seed for train/val episode split |
| `--device` | cuda/cpu | PyTorch device |
| `--num-workers` | 4 | DataLoader workers |
| `--save-every` | 10 | Save periodic checkpoint every N epochs |
| `--resume` | off | Resume from latest periodic checkpoint in output-dir |
| `--cache` | off | Cache all episodes in RAM (faster, more memory) |

### What happens during training

1. **Train/val split** — episodes are split by file (never within an episode). Validation uses the training set's statistics so normalization is identical.
2. **Dataset init** — all episodes are discovered, a flat `(episode, timestep)` index is built, and normalization statistics (per-joint mean/std for qpos, qcmd, torques, absolute-actions, and relative-actions; per-joint min/max of absolute and relative actions for safety clamping) are computed across the full training subset. Per-episode start poses are also captured so deploy-time can pick the closest one.
3. **Each sample** — at index `(ep, t)`: load `qpos[t]`, both images at `t`, and `actions[t:t+chunk_size]`. Normalize everything (z-score for positions/actions, ImageNet for images). Build the state vector based on `state_mode`. If `--action-mode relative`, each action is converted to `(action − qpos[t])` before normalization. If `--augment`, train images get geometric crop + per-camera color jitter.
4. **Forward pass** — CVAE encodes `(qpos, actions)` → latent `z`, decoder predicts action chunk from `(images, state, z)`. Loss = L1 on actions + `kl_weight × KL`.
5. **Optimizer** — AdamW with two param groups (backbone gets `lr_backbone`), gradient clipping at `max_norm=0.1`, cosine annealing to `lr × 0.01`.
6. **Checkpoints** — periodic saves every `save_every` epochs as `act_epoch_XXXX.pt`. The checkpoint with the lowest validation total-loss is additionally saved to `act_best.pt`.

### Checkpoint contents

Each `.pt` file contains everything needed for inference:

```python
{
    "epoch": int,
    "best_val": float,
    "model_state_dict": ...,
    "optimizer_state_dict": ...,
    "scheduler_state_dict": ...,
    "dataset_stats": {
        "qpos_mean", "qpos_std",              # [7] each
        "qcmd_mean", "qcmd_std",
        "torque_mean", "torque_std",
        "action_mean", "action_std",           # absolute-action stats
        "action_min", "action_max",            # absolute-action range (clamp)
        "rel_action_mean", "rel_action_std",   # delta-action stats
        "rel_action_min", "rel_action_max",    # delta-action range (clamp)
        "start_poses",                         # [N, 7] — every episode's starting qpos
        "ready_pose",                          # [7] — mean of start_poses (legacy fallback)
    },
    "config": {
        "chunk_size", "d_model", "dim_feedforward", "z_dim",
        "nhead", "num_encoder_layers", "num_decoder_layers",
        "kl_weight",
        "num_joints",     # 7 = swivel + 6 arm joints
        "state_mode",     # "qpos" | "qpos_qcmd" | "qpos_qcmd_tq"
        "state_dim",      # num_joints × (1|2|3), derived from state_mode
        "action_mode",    # "absolute" | "relative"
    },
    "train_loss": {"l1": float, "kl": float, "total": float},
    "val_loss":   {"l1": float, "kl": float, "total": float},   # null if val_fraction=0
}
```

`dataset_stats` is embedded in the checkpoint so the inference node doesn't need the original episodes.

### Resuming training

```bash
python python/training/train.py --data-dir episodes/ --output-dir checkpoints/ --resume
```

Finds the latest `act_epoch_*.pt` in the output directory and continues from `epoch + 1`. `best_val` is preserved so the best-checkpoint tracking picks up correctly.

### Training tips

- **Start with defaults** — `state_mode=qpos_qcmd`, `action_mode=relative`, `chunk_size=32`. Relative actions generalize much better on small datasets because the model only has to learn motion shapes, not absolute poses.
- **Turn on `--augment`** whenever you have fewer than ~100 episodes. Geometric crop + color jitter is cheap insurance against overfitting to lighting / framing.
- **Watch `val/total` in TensorBoard** — if it diverges from `train/total`, you're overfitting (add more episodes or augmentation). The `act_best.pt` saved at the lowest `val/total` is usually the best one to deploy.
- **`--cache`** speeds up training significantly if episodes fit in RAM. Each 500-frame episode with images is ~264 MB.
- **Train for 200-400 epochs** for most tasks. Check `act_best.pt` with `evaluate_policy.py` before committing to longer runs.

---

## 4. Offline Evaluation

Before deploying on hardware, validate that the policy learned the task by replaying episodes through the model in open-loop.

### Basic usage

```bash
# Single episode
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/act_epoch_0100.pt \
    --episode episodes/episode_0000.hdf5

# All episodes in a directory
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/act_epoch_0100.pt \
    --episode-dir episodes/

# With temporal ensemble (matches live inference behavior)
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/act_epoch_0100.pt \
    --episode-dir episodes/ --ensemble

# Fast mode (no camera images in Rerun)
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/act_epoch_0100.pt \
    --episode-dir episodes/ --no-images

# Save results
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/act_epoch_0100.pt \
    --episode-dir episodes/ --save eval.rrd --csv eval.csv
```

### All arguments

| Argument | Default | Description |
|---|---|---|
| `--checkpoint` | **required** | Path to `.pt` checkpoint |
| `--episode` | — | One or more HDF5 files (mutually exclusive with `--episode-dir`) |
| `--episode-dir` | — | Directory of `episode_*.hdf5` files |
| `--device` | cuda/cpu | PyTorch device |
| `--ensemble` | off | Enable temporal ensemble |
| `--ensemble-steps` | 16 | Past chunks for ensemble |
| `--no-images` | off | Skip camera images in Rerun |
| `--speed` | 0 | Playback speed (0 = as fast as possible) |
| `--save` | None | Save `.rrd` instead of spawning viewer |
| `--csv` | None | Export per-episode stats to CSV |

### What it does

For each frame in each episode:
1. Feeds ground-truth observations (qpos, images, qcmd, torques) through the model.
2. Gets predicted action (first action from chunk, or temporal ensemble).
3. Compares to ground-truth action via L1 error.

This is **open-loop** — ground truth observations are fed at every step, not the model's own predictions. This isolates prediction quality from compounding error.

### Rerun visualization

The viewer shows:
- **Left/right camera images** from the episode (unless `--no-images`)
- **GT vs predicted actions** overlaid per joint (amber = GT, green = predicted)
- **Per-joint L1 error** over time (red)
- **Inference time** in milliseconds

### Console output

```
Episode                                  Mean L1   Max L1   Inf (ms)
---------------------------------------- -------- -------- ----------
  episode_0000                            0.0342   0.1234       12.3
  episode_0001                            0.0298   0.0987       11.8

Joint          Mean L1   Max L1
-------------- -------- --------
swivel           0.0098   0.0312
gantry_base      0.0123   0.0456
gantry_mid       0.0234   0.0789
gantry_end       0.0189   0.0567
wrist_pitch      0.0145   0.0345
wrist_roll       0.0087   0.0234
gripper          0.0156   0.0489
OVERALL          0.0147   0.0789
```

### Interpreting results

- **Mean L1 < 0.05 rad** across all joints — good, worth trying on hardware.
- **Mean L1 0.05–0.10 rad** — mediocre, might work for coarse tasks but expect tracking errors.
- **Mean L1 > 0.10 rad** — poor, needs more training epochs or more/better demonstrations.
- **High error on specific joints** — the model may be under-constrained for that joint's motion. Check if demonstrations have enough variety for that joint.
- Compare **with and without `--ensemble`** to see if temporal smoothing improves things.

---

## 5. Deployment

### Dry-run first

Always test with `--dry-run` before sending commands to hardware:

```bash
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/act_best.pt \
    --dry-run
```

This runs the full inference pipeline (subscribes to telemetry + cameras, runs the model at 20 Hz) but does **not** send any motor commands. Verify on the console that:
- All three sources (telem, left cam, right cam) become ready.
- Inference time is well under 80 ms.
- Predicted positions look physically reasonable.

### Live deployment

```bash
# Default (Jetson endpoints, CUDA) — usually deploy act_best.pt
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/act_best.pt

# With a larger ensemble window
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/act_best.pt \
    --ensemble-steps 20

# CPU-only (slower, use on dev machine for testing)
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/act_best.pt \
    --device cpu

# Tighter velocity guard for both arm and swivel
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/act_best.pt \
    --max-delta 0.15 --max-delta-swivel 0.08
```

### All arguments

| Argument | Default | Description |
|---|---|---|
| `--checkpoint` | **required** | Path to `.pt` checkpoint |
| `--telem` | `tcp://localhost:5556` | ZMQ telemetry SUB endpoint |
| `--cam-left` | `tcp://localhost:5563` | Left arm camera SUB endpoint |
| `--cam-right` | `tcp://localhost:5564` | Right arm camera SUB endpoint |
| `--cmd` | `tcp://localhost:5555` | ZMQ command PUSH endpoint |
| `--device` | cuda/cpu | PyTorch device |
| `--dry-run` | off | Inference only, no commands sent |
| `--ensemble-steps` | 16 | Past chunks for ensemble (0 = disable) |
| `--max-delta` | 0.3 | Max arm-joint delta per step in rad (at 20 Hz: 0.3 = 6 rad/s) |
| `--max-delta-swivel` | 0.15 | Max swivel delta per step in rad (at 20 Hz: 0.15 = 3 rad/s) |
| `--ramp-speed` | 1.5 | Ramp speed to ready pose in rad/s |
| `--no-rerun` | off | Disable Rerun visualization |

### Safety features

| Feature | Details |
|---|---|
| **Source readiness** | Waits for all 3 streams (telemetry, left cam, right cam) before sending any command. |
| **Closest-start ready pose** | Before inference, the node picks the training-set start pose nearest to the arm's current position and ramps to it, so the first observation is on-distribution. |
| **Staleness check** | Skips the tick if any source is > 200 ms old. |
| **Position bounds** | Absolute mode clamps actions to `[action_min, action_max]`. Relative mode clamps `(action − qpos)` to `[rel_action_min, rel_action_max]`. Both ranges come from the training data. |
| **Delta guard** | Clamps `|action − current_qpos|` per joint per step; swivel uses a tighter limit than the arm joints. |
| **Latency warning** | Warns if inference takes > 80 ms (motor watchdog is 100 ms). |

### Command dispatch

The policy outputs a 7-dim action each tick. The node splits it into two ZMQ messages every step:

- `{"type": "swivel", "position": action[0], ...}` — routes to the swivel base controller.
- `{"type": "arm_joints", "positions": action[1:7], ...}` — routes to the 6-DOF gantry/wrist controller.

This matches the original firmware interface; the policy-level 7-DOF vector is purely a training-side convenience so the model learns whole-body coordination.

### Important warnings

- **Do NOT run `teleop.py` and `act_policy_node.py` simultaneously.** Both push to `:5555`. Interleaved commands are dangerous.
- **`state_mode` / `action_mode` must match training.** The checkpoint carries these in `config` and the node uses them automatically. For old checkpoints without these fields, the node falls back to `state_mode=qpos_qcmd`, `action_mode=absolute`.
- **Arm gains** are loaded from `config/teleop.yaml` (the `gantry.kp` and `gantry.kd` values). Make sure these match what was used during data collection. Swivel gains (`SWIVEL_KP`, `SWIVEL_KD`) are defined in `record_replay.py`.

---

## 6. Episode Replay (without the model)

For debugging or verifying episodes, you can replay a recorded episode directly on hardware (sending the recorded commands without any model):

```bash
# Live replay
python python/scripts/episode_replay_live.py episodes/episode_0000.hdf5

# Dry-run (no commands sent)
python python/scripts/episode_replay_live.py episodes/episode_0000.hdf5 --dry-run

# Move to start pose first, then play
python python/scripts/episode_replay_live.py episodes/episode_0000.hdf5 --goto-start

# Loop indefinitely
python python/scripts/episode_replay_live.py episodes/episode_0000.hdf5 --loop

# Half speed
python python/scripts/episode_replay_live.py episodes/episode_0000.hdf5 --speed 0.5
```

Controls: SPACE = play/pause, R = restart, X = abort + shutdown, Q = quit.

The replay prefers `observations/qcmd` (commanded positions) over `observations/qpos` (actual positions) when available, since commanded positions avoid gravity sag.

---

## Typical Workflow

```
# 1. One-time: calibrate the leader arm
python python/scripts/so101_calibrate.py --port COM4

# 2. Collect 20 demonstrations of the task
python python/scripts/collect_demo.py --port COM4 --output-dir episodes/pick_cup

# 3. Inspect a few episodes visually
python python/scripts/view_episode.py episodes/pick_cup/episode_0005.hdf5

# 4. Train for 300 epochs with augmentation + 15% validation
python python/training/train.py \
    --data-dir episodes/pick_cup \
    --output-dir checkpoints/pick_cup \
    --epochs 300 --augment --val-fraction 0.15

# 5. Evaluate the best-val checkpoint offline
python python/scripts/evaluate_policy.py \
    --checkpoint checkpoints/pick_cup/act_best.pt \
    --episode-dir episodes/pick_cup \
    --ensemble --csv eval_pick_cup.csv

# 6. If Mean L1 looks good, dry-run on hardware
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/pick_cup/act_best.pt \
    --dry-run

# 7. Deploy live
python python/nodes/act_policy_node.py \
    --checkpoint checkpoints/pick_cup/act_best.pt
```
