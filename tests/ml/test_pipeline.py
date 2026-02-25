"""
tests/ml/test_pipeline.py — Offline pytest suite for the ACT pipeline.

No hardware, no ZMQ. Covers:
  - Synthetic HDF5 episode generation
  - EpisodeDataset loading, shapes, normalization, chunk-padding
  - ACTPolicy training forward (loss, gradients)
  - ACTPolicy inference (shapes, no NaN)
  - apply_safety_limits: absolute bounds + per-step delta clamp
  - Single-epoch training smoke test

Run from repo root:
    pytest tests/ml/test_pipeline.py -v
    pytest tests/ml/test_pipeline.py -v -k "safety"   # just safety tests
"""

import io
from pathlib import Path

import h5py
import numpy as np
import pytest
import torch

# ---------------------------------------------------------------------------
# Repo-root import helpers (conftest.py already added root to sys.path)
# ---------------------------------------------------------------------------

from python.training.dataset import EpisodeDataset
from python.training.act_model import ACTPolicy

# Import apply_safety_limits directly from act_policy_node.
# The module has top-level zmq/torch imports; those packages are all present
# in the dev environment (requirements.txt + requirements_training.txt).
from python.nodes.act_policy_node import apply_safety_limits, ARM_JOINTS

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------
NUM_JOINTS = 6
IMG_H, IMG_W = 240, 320
CHUNK_SIZE = 10   # small for fast tests

# Tiny model dims so every test runs in <5 s on CPU
TINY_MODEL = dict(
    chunk_size=CHUNK_SIZE,
    d_model=64,
    z_dim=8,
    nhead=4,
    num_encoder_layers=1,
    num_decoder_layers=1,
    pretrained_encoder=False,   # no network download
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_episode(path: Path, T: int, seed: int = 0):
    """Write a single synthetic episode HDF5 file."""
    rng = np.random.default_rng(seed)
    qpos = rng.uniform(-1.0, 1.0, (T, NUM_JOINTS)).astype(np.float32)
    # action[t] = qpos[t+1], last repeated — same convention as collect_demo.py
    actions = np.concatenate([qpos[1:], qpos[-1:]], axis=0).astype(np.float32)
    left = rng.integers(0, 255, (T, IMG_H, IMG_W, 3), dtype=np.uint8)
    right = rng.integers(0, 255, (T, IMG_H, IMG_W, 3), dtype=np.uint8)

    with h5py.File(path, "w") as f:
        f.attrs["hz"] = 20
        f.attrs["arm_joints"] = ",".join(ARM_JOINTS)
        obs = f.create_group("observations")
        obs.create_dataset("qpos", data=qpos)
        imgs = obs.create_group("images")
        imgs.create_dataset("left", data=left, chunks=(1, IMG_H, IMG_W, 3))
        imgs.create_dataset("right", data=right, chunks=(1, IMG_H, IMG_W, 3))
        f.create_dataset("actions", data=actions)


@pytest.fixture(scope="module")
def episodes_dir(tmp_path_factory):
    """Three episodes of different lengths: 30, 50, 45 steps."""
    d = tmp_path_factory.mktemp("episodes")
    lengths = [30, 50, 45]
    for i, T in enumerate(lengths):
        _write_episode(d / f"episode_{i:04d}.hdf5", T, seed=i)
    return d, lengths


@pytest.fixture(scope="module")
def dataset(episodes_dir):
    d, _ = episodes_dir
    return EpisodeDataset(str(d), chunk_size=CHUNK_SIZE)


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

    def test_item_shapes(self, dataset):
        obs, chunk = dataset[0]
        assert obs["qpos"].shape == (NUM_JOINTS,)
        assert obs["images"]["left"].shape == (3, IMG_H, IMG_W)
        assert obs["images"]["right"].shape == (3, IMG_H, IMG_W)
        assert chunk.shape == (CHUNK_SIZE, NUM_JOINTS)

    def test_item_dtypes(self, dataset):
        obs, chunk = dataset[0]
        assert obs["qpos"].dtype == torch.float32
        assert obs["images"]["left"].dtype == torch.float32
        assert chunk.dtype == torch.float32

    def test_qpos_normalization_range(self, dataset):
        """After z-score normalization, qpos should be roughly in [-5, 5]."""
        obs, _ = dataset[0]
        assert obs["qpos"].abs().max() < 10.0

    def test_action_normalization_range(self, dataset):
        """Actions are min-max normalized to [-1, 1]."""
        for i in range(min(50, len(dataset))):
            _, chunk = dataset[i]
            assert chunk.min() >= -1.0 - 1e-5
            assert chunk.max() <= 1.0 + 1e-5

    def test_chunk_padding_at_end(self, episodes_dir):
        """The last timestep of an episode should have a padded chunk."""
        d, lengths = episodes_dir
        ds = EpisodeDataset(str(d), chunk_size=20)
        # Find the sample at the very last timestep of the first episode
        last_t_ep0 = lengths[0] - 1
        obs, chunk = ds[last_t_ep0]
        # All padded rows should be identical to the last real action
        assert torch.allclose(chunk[0], chunk[-1], atol=1e-5), \
            "Padded action rows should all equal the last real action"

    def test_dataset_stats_shape(self, dataset):
        stats = dataset.dataset_stats
        for key in ("qpos_mean", "qpos_std", "action_min", "action_max", "action_range"):
            assert key in stats
            assert stats[key].shape == (NUM_JOINTS,)

    def test_denormalize_roundtrip(self, dataset):
        """normalize then denormalize should recover original actions."""
        with h5py.File(dataset._episode_paths[0], "r") as f:
            raw_actions = f["actions"][:5]
        normed = dataset.normalize_actions(raw_actions)
        recovered = dataset.denormalize_actions(normed)
        np.testing.assert_allclose(recovered, raw_actions, atol=1e-5)

    def test_cache_gives_same_results(self, episodes_dir):
        """Cached and non-cached datasets return identical items."""
        d, _ = episodes_dir
        ds_nocache = EpisodeDataset(str(d), chunk_size=CHUNK_SIZE, cache=False)
        ds_cached = EpisodeDataset(str(d), chunk_size=CHUNK_SIZE, cache=True)
        obs_nc, chunk_nc = ds_nocache[5]
        obs_c, chunk_c = ds_cached[5]
        assert torch.allclose(obs_nc["qpos"], obs_c["qpos"])
        assert torch.allclose(chunk_nc, chunk_c)


# ---------------------------------------------------------------------------
# ACTPolicy tests
# ---------------------------------------------------------------------------

class TestACTPolicy:

    @pytest.fixture(scope="class")
    def policy(self):
        return ACTPolicy(**TINY_MODEL)

    def test_training_forward_keys(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, imgs, imgs, actions)
        assert set(loss_dict.keys()) == {"l1", "kl", "total"}

    def test_training_forward_no_nan(self, policy):
        qpos = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, imgs, imgs, actions)
        for k, v in loss_dict.items():
            assert not torch.isnan(v), f"NaN in {k} loss"

    def test_training_loss_positive(self, policy):
        """L1 and KL losses should be non-negative."""
        qpos = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss_dict = policy(qpos, imgs, imgs, actions)
        assert loss_dict["l1"].item() >= 0.0
        assert loss_dict["kl"].item() >= 0.0

    def test_gradient_flow(self, policy):
        """Backward pass should produce gradients in all parameter groups."""
        # Use a fresh policy so we don't pollute the class-scoped one
        p = ACTPolicy(**TINY_MODEL)
        qpos = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        actions = torch.randn(2, CHUNK_SIZE, NUM_JOINTS)
        loss = p(qpos, imgs, imgs, actions)["total"]
        loss.backward()
        grads_missing = [
            name for name, param in p.named_parameters()
            if param.requires_grad and param.grad is None
        ]
        assert len(grads_missing) == 0, \
            f"No gradient for: {grads_missing[:5]}"

    def test_inference_shape(self, policy):
        policy.eval()
        qpos = torch.randn(1, NUM_JOINTS)
        imgs = torch.randn(1, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, imgs, imgs)
        assert chunk.shape == (1, CHUNK_SIZE, NUM_JOINTS)

    def test_inference_no_nan(self, policy):
        policy.eval()
        qpos = torch.randn(1, NUM_JOINTS)
        imgs = torch.randn(1, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, imgs, imgs)
        assert not torch.isnan(chunk).any()

    def test_inference_batch(self, policy):
        """Inference should work with batch size > 1."""
        policy.eval()
        B = 4
        qpos = torch.randn(B, NUM_JOINTS)
        imgs = torch.randn(B, 3, IMG_H, IMG_W)
        with torch.no_grad():
            chunk = policy.select_action(qpos, imgs, imgs)
        assert chunk.shape == (B, CHUNK_SIZE, NUM_JOINTS)

    def test_training_requires_actions(self, policy):
        """Calling forward() without actions should raise."""
        qpos = torch.randn(2, NUM_JOINTS)
        imgs = torch.randn(2, 3, IMG_H, IMG_W)
        with pytest.raises(AssertionError):
            policy(qpos, imgs, imgs, actions=None)


# ---------------------------------------------------------------------------
# Safety limits tests
# ---------------------------------------------------------------------------

class TestSafetyLimits:

    @pytest.fixture
    def stats(self):
        """Dataset stats with known bounds for easy assertions."""
        return {
            "action_min":   np.array([-1.0] * NUM_JOINTS, dtype=np.float32),
            "action_max":   np.array([+1.0] * NUM_JOINTS, dtype=np.float32),
            "action_range": np.array([2.0]  * NUM_JOINTS, dtype=np.float32),
        }

    def test_no_clamp_when_safe(self, stats):
        """Small action within bounds and small delta should pass through unchanged."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.full(NUM_JOINTS, 0.02, dtype=np.float32)
        out, delta_flags = apply_safety_limits(action, qpos, stats, max_delta=0.05)
        np.testing.assert_allclose(out, action, atol=1e-6)
        assert not delta_flags.any()

    def test_absolute_upper_clamp(self, stats):
        """Position above action_max is clamped down."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.array([2.0, 0.5, -0.5, 0.0, 0.0, 0.0], dtype=np.float32)
        out, _ = apply_safety_limits(action, qpos, stats, max_delta=10.0)
        assert out[0] == pytest.approx(1.0)    # clamped to max
        assert out[1] == pytest.approx(0.5)    # unchanged
        assert out[2] == pytest.approx(-0.5)   # unchanged

    def test_absolute_lower_clamp(self, stats):
        """Position below action_min is clamped up."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.array([-2.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        out, _ = apply_safety_limits(action, qpos, stats, max_delta=10.0)
        assert out[0] == pytest.approx(-1.0)

    def test_delta_clamp_positive(self, stats):
        """Jump larger than max_delta is clipped to current + max_delta."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        max_delta = 0.05
        out, delta_flags = apply_safety_limits(action, qpos, stats, max_delta=max_delta)
        assert out[0] == pytest.approx(max_delta, abs=1e-6)
        assert delta_flags[0]            # joint 0 was delta-clamped
        assert not delta_flags[1:].any() # other joints not clamped

    def test_delta_clamp_negative(self, stats):
        """Negative jump larger than max_delta is clipped to current - max_delta."""
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.array([-0.3, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        max_delta = 0.05
        out, delta_flags = apply_safety_limits(action, qpos, stats, max_delta=max_delta)
        assert out[0] == pytest.approx(-max_delta, abs=1e-6)
        assert delta_flags[0]

    def test_both_layers_active(self, stats):
        """Action outside abs bounds AND large delta: both limits engage."""
        qpos = np.array([0.8, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        action = np.array([1.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        max_delta = 0.05
        out, delta_flags = apply_safety_limits(action, qpos, stats, max_delta=max_delta)
        # Layer 1: clip 1.5 → 1.0
        # Layer 2: delta = 1.0 - 0.8 = 0.2, clamped to 0.05 → out = 0.85
        assert out[0] == pytest.approx(0.8 + max_delta, abs=1e-5)
        assert delta_flags[0]

    def test_output_dtype(self, stats):
        qpos = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_JOINTS, dtype=np.float32)
        out, _ = apply_safety_limits(action, qpos, stats, max_delta=0.05)
        assert out.dtype == np.float32


# ---------------------------------------------------------------------------
# Training smoke test
# ---------------------------------------------------------------------------

class TestTrainSmoke:
    """Verify one forward+backward step of the training loop doesn't crash."""

    def test_one_batch(self, dataset):
        from torch.utils.data import DataLoader

        def collate(batch):
            obs_list, act_list = zip(*batch)
            return (
                {
                    "qpos": torch.stack([o["qpos"] for o in obs_list]),
                    "images": {
                        "left":  torch.stack([o["images"]["left"]  for o in obs_list]),
                        "right": torch.stack([o["images"]["right"] for o in obs_list]),
                    },
                },
                torch.stack(act_list),
            )

        loader = DataLoader(dataset, batch_size=2, collate_fn=collate, shuffle=False)
        obs, actions = next(iter(loader))

        policy = ACTPolicy(**TINY_MODEL)
        optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-4)

        policy.train()
        optimizer.zero_grad()
        loss_dict = policy(
            obs["qpos"],
            obs["images"]["left"],
            obs["images"]["right"],
            actions,
        )
        loss_dict["total"].backward()
        optimizer.step()

        # Loss decreased from random init is not guaranteed in 1 step,
        # but it must be a finite number > 0
        assert loss_dict["total"].item() > 0
        assert not torch.isnan(loss_dict["total"])

    def test_loss_decreases_on_overfit(self):
        """Policy should overfit a single repeated batch (loss decreasing)."""
        # 4 identical samples — policy should memorize them
        B = 4
        qpos = torch.randn(B, NUM_JOINTS)
        imgs = torch.randn(B, 3, IMG_H, IMG_W)
        actions = torch.randn(B, CHUNK_SIZE, NUM_JOINTS)

        policy = ACTPolicy(**TINY_MODEL)
        optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-3)

        losses = []
        for _ in range(30):
            optimizer.zero_grad()
            loss = policy(qpos, imgs, imgs, actions)["total"]
            loss.backward()
            optimizer.step()
            losses.append(loss.item())

        # Loss at step 30 should be lower than at step 1
        assert losses[-1] < losses[0], \
            f"Loss did not decrease: {losses[0]:.4f} → {losses[-1]:.4f}"
