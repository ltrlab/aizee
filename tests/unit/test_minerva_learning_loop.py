"""End-to-end unit tests for the 14-DoF TaskEpisodeWriter -> MinervaEpisodeDataset ->
train_minerva -> MinervaCheckpointPolicy & minerva_policy_node learning loop.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
for _p in (str(REPO), str(REPO / "src")):
    if _p in sys.path:
        sys.path.remove(_p)
    sys.path.insert(0, _p)

torch = pytest.importorskip("torch")

from python.common.minerva_constants import MINERVA_JOINTS
from python.training.flow_matching_head import FlowMatchingActionHead
from python.training.minerva_dataset import (
    MinervaEpisodeDataset,
    denormalize_actions,
    discover_minerva_episodes,
    split_episodes,
)
from python.training.minerva_model import MinervaPolicy
from python.training.train_minerva import EMA, main as train_minerva_main
from robot_platform.control import RobotSession
from robot_platform.data.task_capture import CAMERAS, TaskEpisodeWriter
from robot_platform.evaluation.policies import MinervaCheckpointPolicy, load_policy
from robot_platform.evaluation.runner import TrialContext
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings


JOINTS_14 = tuple(MINERVA_JOINTS[:14])


def _write_task_episode(
    directory: Path,
    *,
    num_frames: int = 12,
    authorities: list[str] | None = None,
    training_eligibles: list[bool] | None = None,
    authority_default: str = "human",
    image_hw: tuple[int, int] = (240, 320),
    finalize: bool = True,
) -> Path:
    if authority_default == "scripted_fixture":
        metadata = {
            "data_source": "simulation",
            "evidence": "synthetic",
            "run_id": "fixture_run",
            "collection_mode": "scripted_fixture",
            "task": "pick_place_red_cube",
            "instruction": "pick up the red block with the left arm",
        }
        real = False
    else:
        metadata = {
            "data_source": "real",
            "task": "pick_place_red_cube",
            "instruction": "pick up the red block with the left arm",
        }
        real = True

    writer = TaskEpisodeWriter(directory, JOINTS_14, metadata, max_pending=64)
    h, w = image_hw
    for step in range(num_frames):
        qpos = np.full(14, 0.01 * step, dtype=np.float32)
        qcmd = np.full(14, 0.01 * step + 0.005, dtype=np.float32)
        images = {
            cam: np.full((h, w, 3), (step * 17 + idx * 31) % 255, dtype=np.uint8)
            for idx, cam in enumerate(CAMERAS)
        }
        auth = authorities[step] if authorities is not None else authority_default
        elig = training_eligibles[step] if training_eligibles is not None else True
        writer.append(
            q=qpos,
            qcmd=qcmd,
            source_time=100.0 + step * 0.05,
            scene_time=None if real else (step + 1) * 0.05,
            images=images,
            camera_times={cam: (200.0 if real else 0.05) + step * 0.05 for cam in CAMERAS},
            task_state={
                "training_eligible": elig,
                "receiver_stamp": step * 50,
                "camera_provenance": {cam: {"kind": "fixture"} for cam in CAMERAS},
            },
            authority=auth,
        )
    if finalize:
        return writer.finish()
    writer.abort("incomplete test")
    return writer.path


def test_discover_and_dataset_segment_boundaries_and_authority_filtering(tmp_path: Path):
    # Frame authorities in TaskEpisodeWriter:
    # TaskEpisodeWriter sets action_valid[t] = (t+1 < N) and (authority[t] == authority[t+1]) and training_eligible[t].
    # Let's construct:
    #   0..4: "human" -> action_valid[0..3] = True, action_valid[4] = False (transition to policy at 5)
    #   5..6: "policy" -> excluded from BC targets even when action_valid[5] = True!
    #   7..11: "correction" -> action_valid[7..10] = True, action_valid[11] = False (terminal frame)
    authorities = [
        "human", "human", "human", "human", "human",
        "policy", "policy",
        "correction", "correction", "correction", "correction", "correction",
    ]
    ep_real = _write_task_episode(
        tmp_path,
        num_frames=12,
        authorities=authorities,
    )
    # Also write an aborted/incomplete episode and a scripted_fixture episode
    _write_task_episode(tmp_path, num_frames=6, finalize=False)
    _write_task_episode(
        tmp_path,
        num_frames=6,
        authority_default="scripted_fixture",
    )

    discovered_default = discover_minerva_episodes(tmp_path, include_ineligible=False)
    assert discovered_default == [ep_real]

    discovered_with_fixture = discover_minerva_episodes(tmp_path, include_ineligible=True)
    assert len(discovered_with_fixture) == 2

    ds = MinervaEpisodeDataset(
        discovered_default,
        chunk_size=4,
        state_mode="qpos_qcmd",
        action_mode="absolute",
        future_offset=2,
        correction_weight=3.5,
    )
    assert ds.num_joints == 14
    assert ds.joint_names == JOINTS_14
    assert ds.image_shapes == {cam: (240, 320, 3) for cam in CAMERAS}

    # Valid frame indices are [0, 1, 2, 3] (human) and [7, 8, 9, 10] (correction).
    # Frame 4 (human->policy boundary), frames 5..6 (policy), and frame 11 (terminal) are excluded!
    indexed_ts = [t for _, t in ds._index]
    assert indexed_ts == [0, 1, 2, 3, 7, 8, 9, 10]

    # Check IWR sample weights: 1.0 for human, 3.5 for correction
    expected_weights = [1.0, 1.0, 1.0, 1.0, 3.5, 3.5, 3.5, 3.5]
    np.testing.assert_allclose(ds.sample_weights, expected_weights)

    # Check segment boundary clamping:
    # At index 2 (frame t=2), the valid human segment ends at t=4 (last valid frame is t=3).
    # Note TaskEpisodeWriter stores actions[t] = qcmd[t+1]:
    #   actions[2] = qcmd[3] = 0.035
    #   actions[3] = qcmd[4] = 0.045
    # So a chunk of size 4 starting at t=2 must clamp at t=3 (0.045) and NEVER bleed into t>=4!
    obs_t2, actions_norm_t2 = ds[2]
    actions_raw = denormalize_actions(actions_norm_t2.numpy(), ds.dataset_stats, "absolute")
    expected_t2_steps = [0.035, 0.045, 0.045, 0.045]
    for step_idx, expected_val in enumerate(expected_t2_steps):
        np.testing.assert_allclose(actions_raw[step_idx], np.full(14, expected_val), atol=1e-5)

    # Also check JEPA future_images at t=3 (index 3): t + future_offset = 5 would cross into "policy" (frame 5),
    # so t_fut clamps to seg_end = 4 (the terminal observation of the human segment, matching t=2 + 2 = 4).
    obs_t3, _ = ds[3]
    for cam in CAMERAS:
        torch.testing.assert_close(obs_t3["future_images"][cam], obs_t2["future_images"][cam])

    # NaN torques must not poison stats in qpos_qcmd mode
    assert np.isfinite(ds.dataset_stats["torque_mean"]).all()
    assert np.isfinite(ds.dataset_stats["torque_std"]).all()

    # Requesting qpos_qcmd_tq on an episode with NaN torques must raise ValueError
    with pytest.raises(ValueError, match="torques"):
        MinervaEpisodeDataset(
            discovered_default,
            chunk_size=4,
            state_mode="qpos_qcmd_tq",
            action_mode="absolute",
        )


def test_flow_matching_rtc_prefix_inpainting_and_eval_dropout():
    torch.manual_seed(0)
    policy = MinervaPolicy(
        num_joints=14,
        chunk_size=6,
        d_model=64,
        state_dim=28,
        lang_dim=0,
        nhead=4,
        head_layers=1,
        head_ff=128,
        flow_steps=4,
        pretrained_encoder=False,
    )
    policy.eval()

    state = torch.zeros(1, 28)
    images = {cam: torch.zeros(1, 3, 240, 320) for cam in CAMERAS}
    actions = torch.zeros(1, 6, 14)

    # In eval() mode, vision token dropout must be disabled so repeated forward passes are deterministic
    torch.manual_seed(42)
    out1 = policy(state, images, actions)
    torch.manual_seed(42)
    out2 = policy(state, images, actions)
    torch.testing.assert_close(out1["flow"], out2["flow"])
    torch.testing.assert_close(out1["total"], out2["total"])

    # RTC prefix inpainting: when velocity is identically zero (zero-init output layer),
    # x_t[:P] is guided along (1 - t)*x_0 + t*prefix at each step.
    head = FlowMatchingActionHead(
        chunk_size=6, action_dim=14, d_model=64, cond_dim=28, nhead=4, num_layers=1, dim_feedforward=128
    )
    head.eval()
    prefix = torch.full((1, 2, 14), 0.75)
    gen1 = torch.Generator().manual_seed(123)
    sampled_with_prefix = head.sample(
        torch.zeros(1, 28),
        torch.zeros(1, 4, 64),
        num_steps=4,
        prefix_actions=prefix,
        generator=gen1,
    )
    gen2 = torch.Generator().manual_seed(123)
    sampled_no_prefix = head.sample(
        torch.zeros(1, 28),
        torch.zeros(1, 4, 64),
        num_steps=4,
        generator=gen2,
    )
    assert sampled_with_prefix.shape == (1, 6, 14)
    # Suffix (steps 2..5) uses identical noise and zero velocity, so it matches
    torch.testing.assert_close(sampled_with_prefix[:, 2:, :], sampled_no_prefix[:, 2:, :])
    # Prefix (steps 0..1) is pulled toward prefix=0.75
    assert not torch.allclose(sampled_with_prefix[:, :2, :], sampled_no_prefix[:, :2, :])


def test_ema_warmup_and_average_parameters_context():
    model = torch.nn.Linear(4, 2, bias=False)
    torch.nn.init.constant_(model.weight, 1.0)
    ema = EMA(model, decay=0.9999)
    assert ema.num_updates == 0

    # Mutate model weights to 2.0 and take one EMA update; warmup decay at step 1 is (1+1)/(10+1) = 2/11
    torch.nn.init.constant_(model.weight, 2.0)
    ema.update(model)
    assert ema.num_updates == 1
    expected_shadow = (2.0 / 11.0) * 1.0 + (9.0 / 11.0) * 2.0
    np.testing.assert_allclose(ema.shadow["weight"].numpy(), expected_shadow, rtol=1e-5)

    # Context manager swaps in EMA shadow weights and restores raw weights on exit
    with ema.average_parameters(model):
        np.testing.assert_allclose(model.weight.detach().numpy(), expected_shadow, rtol=1e-5)
    np.testing.assert_allclose(model.weight.detach().numpy(), 2.0, rtol=1e-5)


def test_end_to_end_14dof_train_eval_and_policy_node_compatibility(tmp_path: Path):
    data_dir = tmp_path / "episodes"
    ckpt_dir = tmp_path / "checkpoints"
    for _ in range(3):
        _write_task_episode(
            data_dir,
            num_frames=10,
            authorities=["human"] * 5 + ["correction"] * 5,
        )

    train_paths, val_paths = split_episodes(str(data_dir), 0.34, seed=42)
    assert len(train_paths) == 2
    assert len(val_paths) == 1

    train_minerva_main([
        "--data-dir", str(data_dir),
        "--output-dir", str(ckpt_dir),
        "--epochs", "2",
        "--batch-size", "2",
        "--chunk-size", "4",
        "--d-model", "64",
        "--nhead", "4",
        "--head-layers", "1",
        "--head-ff", "128",
        "--flow-steps", "2",
        "--val-fraction", "0.34",
        "--save-every", "1",
        "--num-workers", "0",
        "--correction-weight", "2.0",
        "--no-pretrained-encoder",
        "--future-offset", "0",
        "--no-tensorboard",
        "--device", "cpu",
    ])

    best_pt = ckpt_dir / "minerva_best.pt"
    epoch2_pt = ckpt_dir / "minerva_epoch_0002.pt"
    assert best_pt.is_file()
    assert epoch2_pt.is_file()

    # Verify checkpoint metadata
    ckpt = torch.load(best_pt, map_location="cpu", weights_only=False)
    assert ckpt["config"]["num_joints"] == 14
    assert tuple(ckpt["config"]["joint_names"]) == JOINTS_14
    assert ckpt["config"]["image_shapes"] == {cam: [240, 320, 3] for cam in CAMERAS}
    assert ckpt["ema_num_updates"] > 0

    # Verify resume works cleanly
    train_minerva_main([
        "--data-dir", str(data_dir),
        "--output-dir", str(ckpt_dir),
        "--resume", str(epoch2_pt),
        "--epochs", "3",
        "--batch-size", "2",
        "--chunk-size", "4",
        "--d-model", "64",
        "--nhead", "4",
        "--head-layers", "1",
        "--head-ff", "128",
        "--flow-steps", "2",
        "--val-fraction", "0.34",
        "--save-every", "1",
        "--num-workers", "0",
        "--no-pretrained-encoder",
        "--future-offset", "0",
        "--no-tensorboard",
        "--device", "cpu",
    ])
    assert (ckpt_dir / "minerva_epoch_0003.pt").is_file()

    # Load into evaluation harness via load_policy("checkpoint:...")
    eval_policy = load_policy(f"checkpoint:{best_pt}")
    assert isinstance(eval_policy, MinervaCheckpointPolicy)
    ctx = TrialContext(
        index=0,
        cube_start={"cell": "nominal"},
        joint_names=JOINTS_14,
        joint_map={},
        lower=np.full(14, -3.14),
        upper=np.full(14, 3.14),
        home=np.zeros(14),
        period_s=0.05,
        plant="ideal",
    )
    eval_policy.reset(ctx)

    # Pass 640x480 images (sim scene resolution) to verify exact Image.reduce(2) -> 320x240 downsampling
    obs = {
        "t": 0.0,
        "q": np.zeros(14, dtype=np.float64),
        "qcmd": np.zeros(14, dtype=np.float64),
        "dq": np.zeros(14, dtype=np.float64),
        "effort": np.full(14, np.nan, dtype=np.float64),
        "images": {cam: np.zeros((480, 640, 3), dtype=np.uint8) for cam in CAMERAS},
    }
    chunk1 = eval_policy(obs)
    assert chunk1.shape == (4, 14)
    assert np.isfinite(chunk1).all()

    # Second call exercises RTC prefix inpainting from the unexecuted suffix of chunk1
    obs["t"] = 0.1
    chunk2 = eval_policy(obs)
    assert chunk2.shape == (4, 14)
    assert np.isfinite(chunk2).all()

    # Verify compatibility with minerva_policy_node.py functions
    spec = importlib.util.spec_from_file_location("tested_minerva_policy", REPO / "python/nodes/minerva_policy_node.py")
    policy_node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(policy_node)

    loaded_policy, loaded_stats, loaded_cfg = policy_node.load_minerva_checkpoint(str(best_pt), torch.device("cpu"))
    assert loaded_cfg["num_joints"] == 14
    robot_spec = load_robot("minerva")
    session = RobotSession(robot_spec, load_settings(robot_spec, overrides={"host": "localhost"}), max_age_s=0.2)
    for side in ("left", "right"):
        session.ingest_telemetry(
            side,
            {"motors": {name: {"position": 0.01} for name in session.arms[side].joints}},
            received_at=10.0,
        )
    measured = policy_node.model_observation(session.observation(now=10.05), "qpos_qcmd", num_joints=14)
    assert measured is not None
    qpos_14, torques_14 = measured
    assert qpos_14.shape == (14,)
    assert torques_14 is None
    full_target = policy_node.policy_target(chunk1[0], session)
    assert full_target.shape == (17,)
    np.testing.assert_array_equal(full_target[14:], np.zeros(3, dtype=np.float32))
