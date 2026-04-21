"""
tests/ml/test_pipeline.py — Offline pytest suite for the ACT pipeline.

No hardware, no ZMQ. Covers:
  - Synthetic HDF5 episode generation (format_version=2, 7-DOF)
  - EpisodeDataset loading, shapes, normalization, chunk-padding (absolute + relative)
  - Train/val split helper
  - ACTPolicy training forward (loss, gradients)
  - ACTPolicy inference (shapes, no NaN)
  - apply_safety_limits: absolute and relative modes, delta clamp
  - Single-epoch training smoke test

Run from repo root:
    pytest tests/ml/test_pipeline.py -v
    pytest tests/ml/test_pipeline.py -v -k "safety"   # just safety tests
"""

from pathlib import Path

import h5py
import numpy as np
import pytest
import torch

from python.training.dataset import EpisodeDataset, split_episodes
from python.training.act_model import ACTPolicy

from python.nodes.act_policy_node import (
    apply_safety_limits, POLICY_JOINTS, NUM_POLICY_JOINTS,
)

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------
NUM_JOINTS = NUM_POLICY_JOINTS   # 7
IMG_H, IMG_W = 240, 320
CHUNK_SIZE = 10

TINY_MODEL = dict(
    chunk_size=CHUNK_SIZE,
    d_model=64,
    z_dim=8,
    nhead=4,
    num_encoder_layers=1,
    num_decoder_layers=1,
    pretrained_encoder=False,
    num_joints=NUM_JOINTS,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_episode(path: Path, T: int, seed: int = 0):
    """Write a single synthetic episode HDF5 file in format_version=2."""
    rng = np.random.default_rng(seed)
    qpos = rng.uniform(-1.0, 1.0, (T, NUM_JOINTS)).astype(np.float32)
    qcmd = qpos + rng.normal(0, 0.01, qpos.shape).astype(np.float32)
    torques = rng.uniform(-0.5, 0.5, (T, NUM_JOINTS)).astype(np.float32)
    actions = np.concatenate([qpos[1:], qpos[-1:]], axis=0).astype(np.float32)
    left = rng.integers(0, 255, (T, IMG_H, IMG_W, 3), dtype=np.uint8)
    right = rng.integers(0, 255, (T, IMG_H, IMG_W, 3), dtype=np.uint8)

    with h5py.File(path, "w") as f:
        f.attrs["hz"] = 20
        f.attrs["format_version"] = 2
        f.attrs["arm_joints"] = ",".join(POLICY_JOINTS)
        f.attrs["action_space"] = "absolute"
        obs = f.create_group("observations")
        obs.create_dataset("qpos", data=qpos)
        obs.create_dataset("qcmd", data=qcmd)
        obs.create_dataset("torques", data=torques)
        imgs = obs.create_group("images")
        imgs.create_dataset("left", data=left, chunks=(1, IMG_H, IMG_W, 3))
        imgs.create_dataset("right", data=right, chunks=(1, IMG_H, IMG_W, 3))
        f.create_dataset("actions", data=actions)


@pytest.fixture(scope="module")
def episodes_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("episodes")
    lengths = [30, 50, 45]
    for i, T in enumerate(lengths):
        _write_episode(d / f"episode_{i:04d}.hdf5", T, seed=i)
    return d, lengths


@pytest.fixture(scope="module")
def dataset(episodes_dir):
    d, _ = episodes_dir
    # absolute mode for roundtrip tests; state_mode=qpos so state_dim == num_joints
    return EpisodeDataset(
        str(d), chunk_size=CHUNK_SIZE, state_mode="qpos", action_mode="absolute",
    )


# ---------------------------------------------------------------------------
# Dataset tests
# ---------------------------------------------------------------------------

class TestEpisodeDataset:

    def test_length(self, episodes_dir, dataset):
        _, lengths = episodes_dir
        assert len(dataset) == sum(lengths)

    def test_num_episodes(self, episodes_dir, dataset):
        _, lengths = episodes_dir
        assert dataset.num_episodes == len(lengths)

    def test_num_joints_inferred(self, dataset):
        assert dataset.num_joints == NUM_JOINTS

    def test_item_shapes(self, dataset):
        obs, chunk = dataset[0]
        assert obs["qpos"].shape == (NUM_JOINTS,)
        assert obs["state"].shape == (NUM_JOINTS,)  # state_mode=qpos
        assert obs["images"]["left"].shape == (3, IMG_H, IMG_W)
        assert obs["images"]["right"].shape == (3, IMG_H, IMG_W)
        assert chunk.shape == (CHUNK_SIZE, NUM_JOINTS)

    def test_state_mode_qpos_qcmd_shape(self, episodes_dir):
        d, _ = episodes_dir
        ds = EpisodeDataset(
            str(d), chunk_size=CHUNK_SIZE, state_mode="qpos_qcmd", action_mode="absolute",
        )
        obs, _ = ds[0]
        assert obs["state"].shape == (2 * NUM_JOINTS,)

    def test_state_mode_qpos_qcmd_tq_shape(self, episodes_dir):
        d, _ = episodes_dir
        ds = EpisodeDataset(
            str(d), chunk_size=CHUNK_SIZE, state_mode="qpos_qcmd_tq", action_mode="absolute",
        )
        obs, _ = ds[0]
        assert obs["state"].shape == (3 * NUM_JOINTS,)

    def test_item_dtypes(self, dataset):
        obs, chunk = dataset[0]
        assert obs["qpos"].dtype == torch.float32
        assert obs["images"]["left"].dtype == torch.float32
        assert chunk.dtype == torch.float32

    def test_qpos_normalization_range(self, dataset):
        obs, _ = dataset[0]
        assert obs["qpos"].abs().max() < 10.0

    def test_chunk_padding_at_end(self, episodes_dir):
        d, lengths = episodes_dir
        ds = EpisodeDataset(
            str(d), chunk_size=20, state_mode="qpos", action_mode="absolute",
        )
        last_t_ep0 = lengths[0] - 1
        _, chunk = ds[last_t_ep0]
        assert torch.allclose(chunk[0], chunk[-1], atol=1e-5)

    def test_dataset_stats_shape(self, dataset):
        stats = dataset.dataset_stats
        for key in ("qpos_mean", "qpos_std", "action_min", "action_max",
                    "rel_action_min", "rel_action_max", "ready_pose"):
            assert key in stats
            assert stats[key].shape == (NUM_JOINTS,)
        assert stats["start_poses"].shape == (3, NUM_JOINTS)

    def test_denormalize_roundtrip_absolute(self, dataset):
        with h5py.File(dataset._episode_paths[0], "r") as f:
            raw_actions = f["actions"][:5]
        normed = dataset.normalize_actions(raw_actions)
        recovered = dataset.denormalize_actions(normed)
        np.testing.assert_allclose(recovered, raw_actions, atol=1e-5)

    def test_denormalize_roundtrip_relative(self, episodes_dir):
        d, _ = episodes_dir
        ds = EpisodeDataset(
            str(d), chunk_size=CHUNK_SIZE, state_mode="qpos", action_mode="relative",
        )
        with h5py.File(ds._episode_paths[0], "r") as f:
            raw_actions = f["actions"][:5]
            anchor = f["observations/qpos"][0]
        normed = ds.normalize_actions(raw_actions, qpos=anchor)
        recovered = ds.denormalize_actions(normed, qpos=anchor)
        np.testing.assert_allclose(recovered, raw_actions, atol=1e-5)

    def test_cache_gives_same_results(self, episodes_dir):
        d, _ = episodes_dir
        ds_nocache = EpisodeDataset(
            str(d), chunk_size=CHUNK_SIZE, state_mode="qpos", action_mode="absolute",
            cache=False,
        )
        ds_cached = EpisodeDataset(
            str(d), chunk_size=CHUNK_SIZE, state_mode="qpos", action_mode="absolute",
            cache=True,
        )
        obs_nc, chunk_nc = ds_nocache[5]
        obs_c, chunk_c = ds_cached[5]
        assert torch.allclose(obs_nc["qpos"], obs_c["qpos"])
        assert torch.allclose(chunk_nc, chunk_c)


class TestSplitEpisodes:

    def test_returns_all_paths(self, episodes_dir):
        d, lengths = episodes_dir
        train, val = split_episodes(str(d), val_fraction=0.34, seed=0)
        assert len(train) + len(val) == len(lengths)

    def test_no_val_when_fraction_zero(self, episodes_dir):
        d, _ = episodes_dir
        train, val = split_episodes(str(d), val_fraction=0.0, seed=0)
        assert len(val) == 0
        assert len(train) > 0

    def test_split_is_deterministic(self, episodes_dir):
        d, _ = episodes_dir
        a_train, a_val = split_episodes(str(d), val_fraction=0.34, seed=42)
        b_train, b_val = split_episodes(str(d), val_fraction=0.34, seed=42)
        assert a_train == b_train
        assert a_val == b_val

    def test_never_empty_train(self, episodes_dir):
        d, _ = episodes_dir
        train, _ = split_episodes(str(d), val_fraction=0.99, seed=0)
        assert len(train) >= 1


# ---------------------------------------------------------------------------
# ACTPolicy tests
# ---------------------------------------------------------------------------

class TestACTPolicy:

    @pytest.fixture(scope="class")
    def policy(self):
        return ACTPolicy(state_dim=NUM_JOINTS, **TINY_MODEL)

    def test_training_forward_keys(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        state = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, state, imgs, imgs, actions)
        assert set(loss_dict.keys()) == {"l1", "kl", "total"}

    def test_training_forward_no_nan(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        state = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, state, imgs, imgs, actions)
        for k, v in loss_dict.items():
            assert not torch.isnan(v), f"NaN in {k} loss"

    def test_training_loss_positive(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        state = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, state, imgs, imgs, actions)
        assert loss_dict["l1"].item() >= 0.0
        assert loss_dict["kl"].item() >= 0.0

    def test_gradient_flow(self):
        p = ACTPolicy(state_dim=NUM_JOINTS, **TINY_MODEL)
        qpos = torch.randn(2, NUM_JOINTS)
        state = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss = p(qpos, state, imgs, imgs, actions)["total"]
        loss.backward()
        grads_missing = [
            name for name, param in p.named_parameters()
            if param.requires_grad and param.grad is None
        ]
        assert len(grads_missing) == 0, f"No gradient for: {grads_missing[:5]}"

    def test_inference_shape(self, policy):
        policy.eval()
        qpos = torch.randn(1, NUM_JOINTS)
        state = torch.randn(1, NUM_JOINTS)
        imgs = torch.randn(1, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, state, imgs, imgs)
        assert chunk.shape == (1, CHUNK_SIZE, NUM_JOINTS)

    def test_inference_no_nan(self, policy):
        policy.eval()
        qpos = torch.randn(1, NUM_JOINTS)
        state = torch.randn(1, NUM_JOINTS)
        imgs = torch.randn(1, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, state, imgs, imgs)
        assert not torch.isnan(chunk).any()

    def test_inference_batch(self, policy):
        policy.eval()
        B = 4
        qpos = torch.randn(B, NUM_JOINTS)
        state = torch.randn(B, NUM_JOINTS)
        imgs = torch.randn(B, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, state, imgs, imgs)
        assert chunk.shape == (B, CHUNK_SIZE, NUM_JOINTS)

    def test_training_requires_actions(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        state = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        with pytest.raises(AssertionError):
            policy(qpos, state, imgs, imgs, actions=None)


# ---------------------------------------------------------------------------
# Safety limits tests
# ---------------------------------------------------------------------------

class TestSafetyLimits:

    @pytest.fixture
    def stats(self):
        """Dataset stats with known bounds for easy assertions."""
        return {
            "action_min":      np.full(NUM_JOINTS, -1.0, dtype=np.float32),
            "action_max":      np.full(NUM_JOINTS,  1.0, dtype=np.float32),
            "rel_action_min":  np.full(NUM_JOINTS, -0.5, dtype=np.float32),
            "rel_action_max":  np.full(NUM_JOINTS,  0.5, dtype=np.float32),
        }

    def test_no_clamp_when_safe(self, stats):
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.full(NUM_JOINTS, 0.02, dtype=np.float32)
        out, delta_flags = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=0.05, max_delta_swivel=0.05,
        )
        np.testing.assert_allclose(out, action, atol=1e-6)
        assert not delta_flags.any()

    def test_absolute_upper_clamp(self, stats):
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        action[0] = 2.0     # swivel over the abs max
        action[1] = 0.5     # within range
        out, _ = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=10.0, max_delta_swivel=10.0,
        )
        assert out[0] == pytest.approx(1.0)
        assert out[1] == pytest.approx(0.5)

    def test_absolute_lower_clamp(self, stats):
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        action[0] = -2.0
        out, _ = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=10.0, max_delta_swivel=10.0,
        )
        assert out[0] == pytest.approx(-1.0)

    def test_delta_clamp_arm(self, stats):
        """Jump on an arm joint larger than max_delta_arm is clipped."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        action[1] = 0.3     # arm joint 1 requests a 0.3 jump
        out, delta_flags = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=0.05, max_delta_swivel=0.05,
        )
        assert out[1] == pytest.approx(0.05, abs=1e-6)
        assert delta_flags[1]
        assert not delta_flags[0]   # swivel not flagged

    def test_delta_clamp_swivel_separate_limit(self, stats):
        """Swivel uses max_delta_swivel, not max_delta_arm."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        action[0] = 0.20    # swivel requests 0.20
        out, delta_flags = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=0.30, max_delta_swivel=0.08,
        )
        assert out[0] == pytest.approx(0.08, abs=1e-6)
        assert delta_flags[0]

    def test_relative_mode_clamps_delta_range(self, stats):
        """In relative mode, (action - qpos) is clamped to rel_action_min/max."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        action[1] = 0.9     # delta = 0.9 > rel_action_max (0.5)
        out, _ = apply_safety_limits(
            action, qpos, stats, action_mode="relative",
            max_delta_arm=10.0, max_delta_swivel=10.0,
        )
        assert out[1] == pytest.approx(0.5, abs=1e-6)

    def test_output_dtype(self, stats):
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        out, _ = apply_safety_limits(
            action, qpos, stats, action_mode="absolute",
            max_delta_arm=0.05, max_delta_swivel=0.05,
        )
        assert out.dtype == np.float32


# ---------------------------------------------------------------------------
# Training smoke test
# ---------------------------------------------------------------------------

class TestTrainSmoke:

    def test_one_batch(self, dataset):
        from torch.utils.data import DataLoader

        def collate(batch):
            obs_list, act_list = zip(*batch)
            return (
                {
                    "qpos": torch.stack([o["qpos"] for o in obs_list]),
                    "state": torch.stack([o["state"] for o in obs_list]),
                    "images": {
                        "left":  torch.stack([o["images"]["left"]  for o in obs_list]),
                        "right": torch.stack([o["images"]["right"] for o in obs_list]),
                    },
                },
                torch.stack(act_list),
            )

        loader = DataLoader(dataset, batch_size=2, collate_fn=collate, shuffle=False)
        obs, actions = next(iter(loader))

        policy = ACTPolicy(state_dim=dataset.state_dim, **TINY_MODEL)
        optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-4)

        policy.train()
        optimizer.zero_grad()
        loss_dict = policy(
            obs["qpos"], obs["state"],
            obs["images"]["left"], obs["images"]["right"],
            actions,
        )
        loss_dict["total"].backward()
        optimizer.step()

        assert loss_dict["total"].item() > 0
        assert not torch.isnan(loss_dict["total"])

    def test_loss_decreases_on_overfit(self):
        B = 4
        qpos = torch.randn(B, NUM_JOINTS)
        state = torch.randn(B, NUM_JOINTS)
        imgs = torch.randn(B, 3, IMG_H, IMG_W)
        actions = torch.randn(B, CHUNK_SIZE, NUM_JOINTS)

        policy = ACTPolicy(state_dim=NUM_JOINTS, **TINY_MODEL)
        optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-3)

        losses = []
        for _ in range(30):
            optimizer.zero_grad()
            loss = policy(qpos, state, imgs, imgs, actions)["total"]
            loss.backward()
            optimizer.step()
            losses.append(loss.item())

        assert losses[-1] < losses[0], \
            f"Loss did not decrease: {losses[0]:.4f} → {losses[-1]:.4f}"
