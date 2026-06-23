# AIZEE Learning Pipeline — Research & Execution Plan (V2)

Status: **Draft, learning-dev branch**, 2026-04-30
Author: Claude (revising user-supplied plan after literature review)
Scope: forward-looking research plan for the AIZEE manipulator's learning stack.

This document supersedes the architectural sections of an earlier draft titled "Embodied Foundation Model: Full-Stack Research & Execution Plan." That draft assumed a wearable-data-collection greenfield project styled after Generalist AI's GEN-1 / Physical Intelligence's pi0 / Sunday Robotics' ACT-1. **AIZEE is none of those things**: it is a single mobile manipulator with a working ACT pipeline, ~40 teleoperated demonstrations, and a single-GPU training budget. The original plan also rested on a false architectural unification of those three systems — see [§1.3](#13-what-the-original-plan-got-wrong). This V2 plan is sized for the project we have.

The companion document [LEARNING_PIPELINE.md](../LEARNING_PIPELINE.md) is the operational guide ("how to run training and deployment today"). This document is "where we are going next."

---

## 1. Where we are

### 1.1 Current baseline (production)

- **Robot:** AIZEE mobile manipulator. 7-DOF effective control: `swivel + gantry_base + gantry_mid + gantry_end + wrist_pitch + wrist_roll + gripper`. Base wheels separate (100 Hz loop). Arm joints at 400 Hz (CAN-bus limited).
- **Sensing:** 1× ELP UVC gripper camera + 1× Intel RealSense scene camera (USB to Jetson, RGB used for policy; scene cam also provides depth). The stereo D435 wrist pair and the 4× D455 PoE rover cameras have been retired. Motor torques available.
- **Demos:** Teleoperation via SO-101 leader arm or OpenRB-150 + Dynamixel XL330 leader. Recorded as HDF5 at 20 Hz with `qpos`, `qcmd`, `torques`, `images/{gripper,scene}`. ~40 episodes on disk today.
- **Policy:** ACT (Zhao et al., 2023). ResNet18 (FrozenBN, ImageNet-pretrained) + DETR-style transformer decoder + CVAE prior. `chunk_size=32`, `d_model=256`, `z_dim=32`. Trained with L1 + KL. ~30–50M params.
- **Inference:** [act_policy_node.py](../../python/nodes/act_policy_node.py) runs at 20 Hz on Jetson with temporal ensemble over the last 16 chunks. Latency budget < 80 ms (motor watchdog is 100 ms).
- **Action space:** 7-DOF joint positions, mode `absolute` or `relative` (`relative` recommended for small datasets — model learns motion shapes rather than absolute poses).
- **Safety:** position bounds, per-step delta clamps (tighter on swivel), source-readiness gating, latency warnings.

### 1.2 What the user-supplied plan assumed that doesn't apply

| Plan assumption | AIZEE reality |
|---|---|
| Wearable handheld gripper for data collection (UMI lineage) | We use teleop on the actual robot. No SLAM pipeline, no fisheye undistortion, no operator scaling to 500k hours. |
| Cross-embodiment transfer is the goal | One specific robot; one specific gripper. No transfer needed. |
| Action space is 8-D end-effector deltas + gripper width + force | Action space is 7-D joint positions. EE-relative is **possible** but requires a fast IK on the gantry chain — that's a project, not free. |
| 7B-parameter model trained from scratch on 500k hours | Single GPU; ~40 episodes; ~20k frames total. Training a 7B model from scratch needs ~50–100k H100-hours and ~1B+ frames. Off by ~3 orders of magnitude. |
| Force/contact sensing on fingertips | We have motor torques (gripper motor current/torque). Useful proxy, not a tactile array. |
| ImageNet pretraining is harmful, train ViT from scratch | At our data scale, ImageNet pretraining is **strictly helpful**. The ACT recipe already uses it. The "train ViT from scratch" advice is conditional on having Generalist-scale data. |
| 30 Hz control with 80 forward passes per cycle (Harmonic Reasoning) | 20 Hz ACT inference today. 80 forward passes of even a 50M-param model on Jetson Orin Nano in 25 ms is unrealistic without heavy quantization and custom kernels. |

### 1.3 What the original plan got wrong

I asked a research agent to fact-check the plan's central claim — that "energy-based implicit transformer with harmonic trajectory optimization" is the convergent architecture of GEN-1, pi0, and ACT-1. **It isn't.** Findings, briefly:

1. **pi0 is flow matching**, explicitly. arXiv 2410.24164 (Black et al., Oct 2024) is literally titled "pi0: A Vision-Language-Action Flow Model for General Robot Control." pi0.5 (arXiv 2504.16054, Apr 2025) is the same flow-matching backbone with co-training on heterogeneous data. Weights are open at `Physical-Intelligence/openpi`. Calling it "energy-based implicit" is wrong.
2. **GEN-1's "Harmonic Reasoning" is real, but is not a Fourier basis or trajectory optimization technique.** Generalist describes it as an **asynchronous continuous-time perception/action interleaving** that removes the System-1/System-2 split — the model "thinks and acts" simultaneously. There is no public claim it is energy-based. There is no arXiv paper for GEN-0 or GEN-1; only blog posts. Architectural specifics (param count, loss, exact attention pattern) are undisclosed.
3. **Sunday Robotics' ACT-1 is unpublished.** The system is real, the company is real (Tony Zhao + Cheng Chi, ~$1.15B valuation), and they advertise "ACT-1 foundation model" in PR. But there is no paper. Founders' history (ACT, Diffusion Policy) makes a diffusion-style head a reasonable inductive guess, not a known fact. (Note: not to be confused with Adept's ACT-1 web-browser agent — different system, different company.)
4. **Implicit BC + InfoNCE doesn't scale to action dim > ~5–7.** Florence et al. 2021 showed strong results on small-action-dim tasks. Diffusion Policy (Chi, RSS 2023) explicitly outperforms IBC across their benchmarks. By 2026, IBC is not the competitive default. Our action dim is 7 — right at the IBC failure boundary.
5. **Harmonic / Fourier-basis action chunks exist in the literature** (Fourier Flow Matching, FCNet, classical DMPs/ProMPs/ProDMPs) but are a small niche, **not** what GEN-1, pi0, or ACT-1 use.

This V2 plan keeps the parts of the original that are useful (action chunking, receding horizon execution, conservative offline RL post-training, the harmonic basis as an *optional* smoothness regularizer) and replaces the load-bearing claims with documented, scale-appropriate techniques.

---

## 2. Goals

User decisions (2026-04-30) shape the plan: primary goal is **robust task performance**, V1 evaluation is **one narrow well-scoped task**, demo collection cadence is **fast (200+ demos in 1–2 days)**, and the kept research arms are **harmonic basis, EE-space + IK adapter, and RL post-training**. A third overhead camera is explicitly out of scope (precedent: Generalist's deployments work without one — wrist-mounted cameras carry the workload).

### 2.1 Primary goal (V1: ~6 weeks)

Replace the ACT (CVAE + L1) head with a **Diffusion Policy** head, on a fresh **single-task ~200-episode dataset** collected in parallel with the implementation. Demonstrate ≥ ACT performance on that task and meaningfully better behavior on at least one of: out-of-distribution starts, multi-modal action distributions, or recovery from perturbations. This is the same upgrade path ALOHA Unleashed (CoRL 2024) took.

The 200-demo scale is comfortably in Diffusion Policy's documented strong regime — well above the ~40 we have today, well below ALOHA Unleashed's 26k. With one well-scoped task, this is the highest-probability path to a working autonomous demo.

### 2.2 Secondary goals (V2: pick after V1 lands, ~weeks 7–12)

V2 picks from the research arms below based on V1 results. The kept arms (in expected priority order, to revisit when V1 finishes):

1. **RL post-training** — Cal-QL or RLPD on top of V1 to push beyond demonstration speed and improve perturbation recovery on the chosen task.
2. **Harmonic / Fourier action-chunk parameterization** — as a smoothness regularizer on the V1 diffusion head. Cheap ablation, ~1 week.
3. **EE-space actions + IK adapter** — build URDF-based fast IK on the gantry chain so the policy can output end-effector deltas. Required if VLA fine-tuning is ever attempted; also useful for cleaner action-space comparisons.

VLA fine-tuning (OpenVLA-OFT, pi0 via openpi) is **demoted to optional research**, not a planned V2 path. The user's priority on robust task performance + the demo-cadence answer (we can collect data, we don't need pretrained semantics) makes the cost/benefit unfavorable. Keep the EE/IK arm warm in case this changes.

### 2.3 Cheap wins to do alongside V1

- Add motor-torque and gripper-width as explicit policy inputs (`state_mode=qpos_qcmd_tq`). Already in HDF5; one-line training flag. The gripper torque is a contact-pressure proxy that may be load-bearing for grasp tasks.

### 2.4 Explicit non-goals

- A wearable data-collection device. We have a robot; we use it.
- A 7B foundation model trained from scratch.
- "Cross-embodiment transfer" — we have one robot.
- Energy-based BC with InfoNCE. Documented to fail at our action dim.
- Custom paged attention / int4 quantization / inference-time optimization with 80 forward passes per cycle. Out of budget for a 1–2 person project on a single Jetson.
- A third overhead camera. Generalist's published demos do not require one; wrist-mounted cameras suffice. We keep the bandwidth budget for V2 research arms instead.
- VLA fine-tuning (OpenVLA-OFT, pi0) as a primary V2 path. Reconsider if V1 plateaus and we can't close the gap with data + RL.

---

## 3. V1 — Diffusion Policy upgrade

### 3.0 Step 0 — Pick the task and collect 200 demos (parallel with implementation)

The user's answer locks two things: V1 evaluates on **one well-scoped task**, and we can collect **~200 demos in 1–2 focused sessions**. This step happens in parallel with the implementation work in §3.2–§3.6 — the diffusion head doesn't need the new data to be written and unit-tested.

Task-selection criteria (pick one):
- **Bounded workspace**: the entire interaction fits comfortably in the visible region of both wrist cameras.
- **One nominal strategy** but with **multi-modal demonstration freedom**: e.g. "pick up the cup and place it in the bin" — the human can grasp from multiple angles, but success is binary. Multi-modality is exactly where Diffusion Policy beats ACT, so the task should permit it. A task with one rigid optimal trajectory is a worse showcase.
- **Cheap reset**: we will run 20+ trials per evaluation; manual reset of a cluttered scene burns hours.
- **Mechanically benign**: nothing fragile, nothing that punishes a 6-rad/s overshoot. We are still tuning safety margins.
- **Recordable failure modes**: we want to be able to label success/failure clearly from the camera stream (helpful for any future RL reward).

Strong default candidate: **single-object pick-and-place on a marked target**. Specific cup or block, marked target zone on the table, reset = put the object back at one of N starting positions. This task hits all five criteria, gives multi-modality (multiple grasp poses) without being chaotic, and is the same task family ACT/Diffusion comparisons are typically published on, which makes our result interpretable against the literature.

Collection protocol (1–2 days):
- Use [collect_demo.py](../../python/scripts/collect_demo.py) on the SO-101 (or OpenRB-150) leader.
- Vary object starting position across a defined grid (e.g. 5×5 grid on a marked region, ~8 demos per cell). Distractor objects optional in V1 — add them later if generalization plateaus.
- Vary lighting and incidental scene changes naturally, but keep the **target zone, table surface, and gripper unchanged** during collection.
- Save into a dedicated `episodes/{task_name}/` directory so we can train independently of the existing mixed `episodes/` set.
- Spot-check every ~25 demos with [view_episode.py](../../python/scripts/view_episode.py) — one bad calibration drift can poison the dataset.
- Quality target: ≥95% of recorded episodes pass a manual review for "the task was actually completed." Discard the rest before training.

Once the new dataset is on disk, the existing 38 mixed episodes become a useful **ACT-baseline-vs-Diffusion-baseline** comparison set for the offline evaluation in §3.7 — they cover broader behavior and let us check that V1 hasn't simply overfit to the new task.

### 3.1 Why diffusion first

For ~200 demonstrations on a single robot and a single narrow task, Diffusion Policy is the documented practical default in 2026. Reasons:
- Strong at this data scale — robust at small data (recent analysis suggests it largely "memorizes a latent action lookup table" and is OOD-robust whenever a near-neighbor observation exists in training), and improves cleanly with more demos. 200 demos on one task is squarely in its strong regime.
- Stable training, well-supported tooling (LeRobot has a reference implementation), apples-to-apples comparable to our existing ACT.
- Lowest delta from current code: same vision backbone, same chunk size, same control loop. The change is local to [act_model.py](../../python/training/act_model.py), [train.py](../../python/training/train.py), and the action-sampling code in the inference node.
- Documented success on the same upgrade path: ALOHA Unleashed (CoRL 2024) is exactly this — ACT (CVAE+L1) → Diffusion Policy transformer head, on broadly similar hardware.

Flow matching (pi0-style) is ~equivalent in quality with simpler objective and faster inference, but the openpi stack is heavier and the win over diffusion at our scale is small. Not part of this plan.

### 3.2 Architecture

Keep:
- ResNet18 (FrozenBN, ImageNet-pretrained) image encoder, shared across left/right cameras. *Verify by ablation* whether `pretrained=True` still wins at the ~40-episode scale; current ACT defaults say yes.
- State encoder MLP (2 layers, `state_dim → d_model`) for `[qpos, qcmd]` (and optionally torques — see §3.6).
- Concatenated context tokens: `[img_left_tokens, img_right_tokens, state_token]`. ~80 spatial tokens per camera at 240×320 input.

Replace:
- The CVAE encoder + DETR decoder + L1 action head, with a **conditional diffusion head** that denoises a chunk of `chunk_size × num_joints` actions.

Two reasonable diffusion variants — choose one and ablate the other after V1 lands:

**Variant A — CNN-based 1D diffusion (Chi et al. RSS 2023, "Diffusion Policy"):**
- 1D U-Net over the action-chunk time axis, FiLM-conditioned on the encoded observation token sequence (pooled or via cross-attention adapter).
- Standard DDPM/DDIM noise schedule. Default 100 training steps, 16 inference DDIM steps. Tune inference steps for 20 Hz budget.
- Pros: simpler, faster to train, fewer params (~10–20M for the diffusion head).
- Cons: no transformer cross-attention to per-token observations — has to compress observations into a global conditioning vector or a small set of tokens.

**Variant B — Transformer diffusion (ALOHA Unleashed / "Diffusion Policy Transformer"):**
- Transformer decoder over learned action-chunk queries (one per timestep, like our current ACT decoder). Cross-attends to context tokens. Predicts noise.
- Pros: cleanly replaces our existing DETR decoder; preserves per-token observation conditioning.
- Cons: slightly more params, slower per step.

**Recommendation: start with Variant B.** The change to our codebase is minimal — replace `ACTDecoder.forward` to also accept a `(noisy_action_chunk, timestep_emb)` and predict noise instead of clean actions. The existing chunk-size = 32 query embedding becomes a noisy-action-chunk projection. This is a single-file change to [act_model.py](../../python/training/act_model.py).

### 3.3 Loss

- L2 (or L1) on predicted noise vs. true noise. Keep weighting simple — no KL term, no auxiliary losses in V1.
- (Optional) Action range clamp at sample time, matching what `act_policy_node` already does. Useful for safety; doesn't change training.

### 3.4 Sampling at inference

DDIM with K steps (K=8–16 to start). Each step is one transformer forward pass. Budget at 20 Hz: 50 ms total per cycle, ~30 ms for inference (current ACT uses ~12 ms on Jetson). With K=16 steps and the same backbone, expect ~16× the per-cycle inference time of ACT — back-of-envelope ~190 ms, **over budget**.

Mitigations, in order of preference:
1. K=8 or fewer DDIM steps. Diffusion Policy reports usable performance at K=10; flow-matching variants work at K=4. Test reconstruction error vs. K offline before deploying.
2. **Action-chunk receding horizon** is already in place — we execute the first M=5–8 of the 32-step chunk and only re-plan once per chunk-step. This means we run inference once per ~250 ms (4 Hz), not per 50 ms. 16 DDIM steps in 250 ms is plausible.
3. Distill to a 1-step student (consistency model / shortcut model) once V1 is working. Defer; this is a follow-up, not blocking.
4. Quantize to int8 (TensorRT). Defer.

Concretely, for V1 we should **reduce inference-call frequency**, not chase 20 Hz model invocations. Current ACT essentially also does this (chunk_size=32 with ensemble window 16 means model is called every step but the ensemble smooths). Diffusion Policy's published recipe calls the model once per ~8–16 control steps and executes the predicted chunk open-loop until the next call.

### 3.5 Training

Re-use [train.py](../../python/training/train.py)'s harness — episode iteration, normalization stats, train/val split, augmentation toggle, checkpointing, TensorBoard. Replace the inner forward call. Keep:
- AdamW, separate LR for backbone (1e-5) vs head (1e-4)
- Cosine annealing
- Gradient clip 0.1 (this is unusually tight; verify it doesn't choke diffusion training — diffusion typically uses 1.0)
- Per-joint normalization stats from `dataset_stats`
- Augmentation: geometric crop + per-camera color jitter (already implemented; should use it always at our data scale)

Hyperparameter sweep (small, 4–8 runs):
- Diffusion steps: {50, 100} train / {8, 16} inference
- Action mode: {`absolute`, `relative`}; expect `relative` to win (matches current ACT finding)
- Backbone unfreeze epoch: {0, 20} (warm up the head before unlocking the vision backbone)
- Optional: `pretrained_encoder` {True, False}

### 3.6 Wider state vector (cheap win, do alongside V1)

The HDF5 already stores `qpos`, `qcmd`, `torques` per timestep. The current `state_mode=qpos_qcmd` ignores torques. Try `state_mode=qpos_qcmd_tq` — the gripper motor's torque is a contact-pressure proxy that may be load-bearing for grasp tasks. This is a one-line training flag and a checkpoint config change.

### 3.7 Evaluation

Re-use the existing offline harness in [evaluate_policy.py](../../python/scripts/evaluate_policy.py): open-loop replay through the model, per-joint L1 vs. ground truth, with and without temporal ensemble. Acceptance gate before deploying live:
- Mean L1 ≤ ACT baseline within 10%, on held-out validation episodes.
- Mean L1 ≤ 0.05 rad across all joints.
- Inference latency ≤ chunk-step budget (≤ 250 ms per chunk re-plan at chunk_step=8, 20 Hz).

Then dry-run on hardware (`--dry-run` in `act_policy_node.py`), then live with conservative `--max-delta` (start at 0.15 rad/step, half of training-time clamp). Compare:
- Success rate on the same 20 trials per task
- Cycle time
- Smoothness (RMS jerk, third derivative of `qpos`)
- Recovery from one specific perturbation (e.g., experimenter slides the target object 5 cm mid-task)

### 3.8 Codebase changes for V1

Concrete file-level diff:

| File | Change |
|---|---|
| [python/training/act_model.py](../../python/training/act_model.py) | Add `DiffusionPolicy` class alongside `ACTPolicy`. Re-uses `ImageEncoder`, `StateEncoder`. Replaces `CVAEEncoder` and the L1 action head. Add a small timestep-embedding MLP. |
| [python/training/train.py](../../python/training/train.py) | Add `--policy {act,diffusion}` flag. For diffusion: sample timestep, add noise to action chunk, predict noise, MSE loss. No KL term. |
| [python/training/dataset.py](../../python/training/dataset.py) | No change — the (state, images, action_chunk) tuple is the same. |
| [python/nodes/act_policy_node.py](../../python/nodes/act_policy_node.py) | Add diffusion-sampling path; bump default chunk-step from 1 to 8 (re-plan less often). Keep all safety clamps. |
| [python/scripts/evaluate_policy.py](../../python/scripts/evaluate_policy.py) | Detect policy type from checkpoint config and dispatch. |
| docs/LEARNING_PIPELINE.md | After V1 ships, add a "Diffusion Policy" section parallel to the existing ACT section. |

The checkpoint format already has a `config` dict — add a `policy_type` field and version-bump `format_version` so old ACT checkpoints still load.

---

## 4. V2 — pick a research arm after V1 lands

V2 is decided on V1 results. The shortlist (per user's stretch-arm picks): RL post-training, harmonic action-chunk parameterization, EE-space + IK adapter. Each addresses a different V1 failure mode, so the V1 result tells us which to start.

| If V1 result is… | Pick this arm | Reasoning |
|---|---|---|
| Diffusion beats ACT cleanly but is jerky / smoothness-limited | **Harmonic basis (§5.1)** | Cheap (~1 week), directly targets jerk by construction. No new data needed. |
| Diffusion is at parity with ACT in success but slow / brittle to perturbations | **RL post-training (§5.2)** | Speed bonus + perturbation recovery are exactly what RL post-training delivers when imitation has plateaued. |
| Diffusion gives clean comparison but joint-space limits transfer / scope of future work | **EE-space + IK adapter (§5.3)** | One-time engineering investment; opens the door to clean comparisons against EE-space policies and to future VLA fine-tuning if priorities change. |
| Diffusion fails to materially improve over ACT | Re-collect with more multi-modality / more environmental variation, then re-evaluate. Don't add a research arm yet — fix the data first. | If V1 doesn't beat ACT, it's almost always the data, not the architecture. |

The arms are described in §5. The expected default, absent surprises, is to attempt **harmonic basis first** (cheap, low-risk, fast feedback) and **RL post-training second** (the higher-impact arm, but costlier and riskier).

---

## 5. Research arms (V2 candidates)

### 5.1 Harmonic / Fourier action-chunk parameterization

Represent each `chunk_size × 7` action chunk as a truncated Fourier series rather than raw timestep values. For chunk size 32 and K=10 harmonics, this is 7 × (1 + 2K) = 147 coefficients, vs. 32 × 7 = 224 raw values. The point is not compression — it's:
- **Smoothness by construction.** Low-K representations cannot produce jerky motion.
- **Time-axis rescaling.** Execute the same coefficients at variable speed without re-planning.

This is **Fourier Flow Matching** territory (Liu et al., NeurIPS 2024) and the classical **ProDMP** lineage. It is **not** what frontier labs (GEN-1, pi0, ACT-1) use, as far as anyone has published — so this is genuine research, not best-practice imitation. That cuts both ways: real upside if it works, no template to copy.

Experiment design (~1 week on top of V1):
1. Build a fixed orthogonal Fourier basis matrix `Φ` (shape `[chunk_size, 1+2K]`) and its left-inverse `Φ⁺`.
2. During training: project ground-truth action chunks to coefficients (`c = Φ⁺ a`), train the diffusion head to denoise in coefficient space, decode back via `a = Φ c`. Loss is still MSE on noise — just in coefficient space.
3. Sweep K ∈ {5, 10, 20}. K=10 is likely too low for sharp grasp actions (gripper close is high-frequency); a per-joint K (e.g. K=20 on the gripper, K=10 elsewhere) is worth trying if a uniform K creates a clear gripper-failure signature.
4. Compare to the V1 (raw-action) baseline on:
   - Success rate (must not regress)
   - RMS jerk (third derivative of `qpos`) — the metric this arm is designed to improve
   - Cycle time (should at least not get worse)
   - Optional: time-axis rescaling — execute the same coefficients at 0.7× and 1.3× speed, see whether success rate degrades gracefully.

Risk: K too low may kill the gripper's fast close. Risk mitigation: per-joint K or skip the harmonic projection on the gripper channel entirely — the basis change is per-dimension, so this is trivial.

### 5.2 Conservative offline-to-online RL post-training

The honest 2026 read on Cal-QL: it works in benchmarks but real-robot adoption is limited and there are documented failure modes (initial performance drop online, struggles when offline data isn't retained during fine-tuning). **RLPD-style hybrid online RL** (use offline demos as part of the online replay buffer at every gradient step) is a more robust real-robot recipe and has been demonstrated on real manipulators in 2024–2025 papers.

Recommended formulation if we attempt this:
- **Reward**: sparse binary task-success (auto-detected from camera or simple geometric check on the target zone), augmented by a small smoothness penalty (jerk) and a hard penalty on torque spikes.
- **Speed bonus**: a small reward on `1/completion_time` to push beyond demonstration speed. Tune carefully — this is the lever that produces "GEN-1-style 3× speedup" results, but it's also the lever that produces hardware-damaging velocity-maxing if mis-weighted.
- **KL penalty** to the V1 Diffusion Policy action distribution to keep behavior close to the imitation baseline (the same regularizer used in RLHF for language models). This is the single most important guardrail — without it, online RL can rapidly depart from anything we trained for.
- **Replay buffer** seeded with the 200 demonstrations from V1, re-sampled at every gradient step (RLPD recipe).

What we need to build before starting:
1. **Auto-reset.** A reliable reset behavior that returns the workspace to a known starting condition without human intervention. For pick-and-place this is "robot places the object back at a sampled grid cell." This is its own subproject — possibly the largest engineering item in V2.5.
2. **Success detector.** Camera-based geometric check (object in target zone) is fastest. Add a torque-spike detector for failed grasps.
3. **Hard safety envelope.** Workspace box, joint-limit buffers, max torque per joint, max velocity per joint. Implemented in the Rust motor-control layer, not the Python policy. Already partially in place; needs an explicit RL-mode tightening.
4. **Breakage budget.** Even with the envelope, expect some hardware wear — gripper rubber, cables, motor brushes. Don't run RL on the only working motor of a given type without spares on hand.

Expected runtime: **at least 1–2 weeks of robot-time** for a meaningful comparison vs. V1, assuming 4–8 hours/day of supervised autonomous operation. RL is the highest-risk, highest-reward arm in V2. Defer until V1 is rock-solid and the auto-reset infrastructure exists.

### 5.3 EE-space actions + URDF-based IK adapter

The current policy outputs joint-space targets. EE-space (end-effector pose deltas + gripper width delta) has three benefits:
- Cleaner action representation for tasks that are intrinsically EE-defined (insertion, alignment, surface-following).
- Decouples policy learning from kinematics — if the gantry chain changes (e.g. wrist_pitch is replaced), an EE-space policy is unaffected; a joint-space policy needs retraining or a remapping.
- Required (or strongly preferred) by every open VLA stack — keeps the door open to OpenVLA-OFT / pi0 fine-tuning if priorities shift.

What this entails:
1. **Forward kinematics** from the URDF in [urdf/](../../urdf/). Straightforward — pinocchio or ikfast or urdfpy. Use it to compute EE pose from `qpos` at every demo timestep, store as an additional dataset field.
2. **Inverse kinematics** that is fast enough to run inline in the inference loop (target ≤ 1 ms per call). On a 6-DOF arm without redundancy this is analytical; on a 7-DOF (gantry + wrist) chain there's nullspace freedom. Options: ikfast (analytical when possible), TRAC-IK (fast numerical), Pinocchio's quadprog-based IK. We have a redundant chain (gantry has 3-DOF planar + 3-DOF wrist + 1-DOF gripper); ikfast may not solve it cleanly. Plan for TRAC-IK or Pinocchio.
3. **Action representation**: EE pose as 6D (position + 6D rotation per Zhou et al. — avoids Euler/quat singularities) + gripper width = 7 values. Same dimensionality as joint-space, but different semantics.
4. **Training**: keep V1 architecture; change only the action target. Train both joint-space and EE-space versions on the same demos for a controlled comparison.
5. **Inference loop change**: per chunk-step, run IK on the predicted EE chunk, convert to joint targets, feed to the existing motor-control interface. Latency budget tightens by the IK call cost (~1 ms × chunk-step length); should still fit.
6. **Safety**: existing joint-limit clamps still apply post-IK. Add an explicit EE-workspace box for an additional layer.

Expected effort: 2–3 weeks. The IK is the critical-path item; everything else is incremental.

If V1 doesn't show a smoothness or kinematics failure mode that EE-space would address, this arm is genuinely optional — joint-space is fine for many tasks. Keep it warm; promote to V2 only if the V1 result motivates it (e.g. if V1 struggles on contact-aligned subtasks).

---

## 6. Risk register

Sized for actual project risks, not borrowed from the original plan.

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| 1 | Diffusion sampling latency exceeds the per-chunk control budget on Jetson Orin Nano | Medium | High | Reduce inference frequency (re-plan every 8–16 control steps). Reduce DDIM steps. Profile early — do not commit to V1 deploy until offline latency is measured. |
| 2 | V1 doesn't beat the ACT baseline | Low–Medium | Medium | Acceptable outcome at parity. If clearly worse, re-collect with more multi-modality before adding research arms — the cause is almost always the data, not the architecture. |
| 3 | Diffusion training is unstable with `grad_clip=0.1` (current default for ACT) | Medium | Low | Bump to 1.0 for diffusion; ablate. |
| 4 | The 200-demo collection session reveals hardware/calibration drift, poisoning the dataset | Medium | High | Spot-check every 25 demos with view_episode.py. Re-zero/re-mirror the leader at session boundaries. Discard any episodes that fail manual review. |
| 5 | The chosen V1 task lacks multi-modality, blunting the comparison vs. ACT | Medium | Medium | Pick a task where the human can grasp/place from multiple angles. If V1 looks like parity for this reason, add multi-modal demonstrations explicitly (different grasp strategies for the same start). |
| 6 | Real-robot RL (V2.5) damages the robot during exploration | High *if attempted* | High | Don't attempt without a hardened safety envelope, working auto-reset, and a breakage budget. Use RLPD (hybrid online + offline replay) and a KL penalty to V1; never run online RL without that regularizer. |
| 7 | The harmonic basis arm kills high-frequency components of the gripper action and reduces grasp success | Medium | Low (it's an optional arm) | Per-joint K, or skip the gripper joint in the harmonic projection. |
| 8 | Auto-reset is harder than expected, blocking the RL arm | Medium | Medium | The auto-reset subproject is on the V2.5 critical path. Scope it explicitly; budget 1–2 weeks for it as a precondition to RL. |
| 9 | The IK on the redundant gantry chain is non-trivial; nullspace handling produces inconsistent joint configurations between chunk-steps | Medium *if EE-arm attempted* | Medium | Use TRAC-IK or Pinocchio; seed each IK call with the previous joint solution to keep continuity in nullspace. |

The original plan listed risks for wearable-device SLAM and a 7B model that don't apply here. Risk #6 is the one borrowed from the original plan that *does* apply.

---

## 7. Timeline

Honest, single-developer estimates. V1.0 (task selection + demo collection) and V1.1 (diffusion head implementation) run in parallel because they're independent.

| Phase | Duration | Milestone |
|---|---|---|
| V1.0 — Pick task, collect ~200 demos | 1–2 days (operator time) over a week | `episodes/{task_name}/` populated; ≥95% manual-review pass rate |
| V1.1 — Diffusion head implementation (parallel with V1.0) | 1–2 weeks | `--policy diffusion` flag works in train.py; checkpoint loads in evaluate_policy.py |
| V1.2 — Hyperparameter sweep + offline eval | 1 week | Best diffusion checkpoint within 10% of ACT mean L1 on held-out validation |
| V1.3 — Latency profiling + chunk-step tuning | 3 days | Per-chunk inference latency budget verified on Jetson |
| V1.4 — Hardware dry-run + live deployment | 1 week | 20-trial success-rate comparison ACT vs. Diffusion on the V1 task |
| V1.5 — Comparison report | 2 days | Document filed at `docs/learning/V1_RESULTS.md`; includes success rate, cycle time, RMS jerk, qualitative perturbation behavior |
| **V1 total** | **~5–6 weeks** | |
| V2 — pick a research arm | (decision based on V1.5) | Per the §4 table: harmonic basis, RL post-training, or EE-space + IK |
| V2 execution | 1–4 weeks (harmonic) / 3–6 weeks (RL) / 2–3 weeks (EE-space) | Per arm |
| **First V2 result** | **~7–12 weeks from start** | |

Compare to the original plan's "12–15 months to first autonomous deployment" — that timeline assumed greenfield wearable hardware, 50k–500k hours of crowd data, and a 7B model. None of which apply.

---

## 8. Required reading (revised)

The original plan's reading list was good for the founder-genealogy / frontier-lab angle but heavy on speculative architectural inferences. The list below is sized for the V1/V2 path.

**Primary, blocking:**
- Chi et al. (2023). *Diffusion Policy: Visuomotor Policy Learning via Action Diffusion.* RSS 2023. — V1 architecture.
- Zhao et al. (2023). *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ALOHA / ACT).* RSS 2023. — Our current baseline.
- Zhao et al. (2024). *ALOHA Unleashed: A Simple Recipe for Robot Dexterity.* CoRL 2024. — Concrete example of the ACT → Diffusion upgrade.

**For V2 research arms (read when picking the arm):**
- Liu et al. (2024). *Fourier Flow Matching.* NeurIPS 2024. — If picking the harmonic basis arm.
- Ball et al. (2023). *Efficient Online Reinforcement Learning with Offline Data (RLPD).* — If picking the RL arm. The most robust real-robot recipe in the recent literature.
- Nakamoto et al. (2023). *Cal-QL: Calibrated Offline RL Pre-Training for Efficient Online Fine-Tuning.* NeurIPS 2023. — Read alongside RLPD; understand the failure modes before choosing between them.
- Zhou et al. (2019). *On the Continuity of Rotation Representations in Neural Networks.* — If picking the EE-space arm; defines the 6D rotation representation we should use.
- Beeson & Ames (2015). *TRAC-IK: An Improved Inverse Kinematics Solver.* — If picking the EE-space arm; a likely IK solver pick.

**Background, contextual:**
- Florence et al. (2022). *Implicit Behavioral Cloning.* ICLR 2023. — Read to understand why we are *not* using EBM/InfoNCE at our action dim.
- Black et al. (2024). *pi0: A Vision-Language-Action Flow Model for General Robot Control.* arXiv 2410.24164. — Worth reading once for the flow-matching framing, even though we're not using pi0.

**Skip unless priorities change:**
- OpenVLA-OFT and pi0.5 papers — relevant only if VLA fine-tuning is reinstated as a V2 path.
- Anything on UMI / wearable data collection — not our setting.
- Florence's Implicit BC follow-ups — superseded for our action dim.
- PaLM-E / RT-2 — closed; not actionable.
- Generalist GEN-0/GEN-1 blogs — interesting but architecturally undisclosed; nothing to copy from.

---

## 9. Decisions made (2026-04-30)

The structuring questions in the previous draft of §9 were answered by the user before this revision:

| Question | Decision |
|---|---|
| Primary goal of the next 3–6 months | **Robust task performance.** V1 = Diffusion Policy on AIZEE; V2 = research arms on top of V1. Frontier-lab-style architectural novelty is explicitly *not* the goal. |
| Task scope for V1 evaluation | **One narrow, well-scoped task.** Strong default candidate: single-object pick-and-place on a marked target (see §3.0). |
| Demo collection cadence | **Fast — 200+ demos in 1–2 focused sessions.** This unlocks aggressive data scale-up as a baseline rather than as a separate phase. |
| Stretch arms kept on the books | **Harmonic basis, EE-space + IK adapter, RL post-training.** Third overhead camera explicitly *out* — Generalist's deployments work without one, and we'd rather spend the engineering on the kept arms. |

Open items that should be re-examined as V1 progresses:

1. **Specific task choice.** §3.0 names a strong default (pick-and-place on a marked target), but the user should pick the actual physical object/zone before V1.0 demo collection starts. This is the one decision still owed.
2. **Whether to keep VLA fine-tuning on the books at all.** The current plan demotes it to "skip unless priorities change." If V1 plateaus and none of the three research arms close the gap, reconsider — OpenVLA-OFT remains the documented 2025 SOTA fine-tune recipe.
3. **What "robust" means quantitatively.** Concrete success-rate target on the V1 task (e.g. ≥95% over 100 trials) should be set when the task is chosen, before V1 evaluation begins. Without a target, "robust" is a ratchet that never closes.
4. **Whether the V2 arm picked from §4 is sequential or parallel.** Default: sequential (one arm at a time). Harmonic basis is cheap enough to run in parallel with another, if the operator wants to.

---

*This is a working hypothesis. Treat it as a baseline to be revised as V1 lands and the comparison data comes back.*
