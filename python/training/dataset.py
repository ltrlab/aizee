"""
dataset.py — PyTorch Dataset over AIZEE HDF5 demonstration episodes.

Each HDF5 file has the schema:
    /observations/qpos           float32 [T, 6]
    /observations/images/left    uint8   [T, 240, 320, 3]
    /observations/images/right   uint8   [T, 240, 320, 3]
    /actions                     float32 [T, 6]
    attrs: hz=20, arm_joints="gantry_base,..."

Usage:
    dataset = EpisodeDataset("episodes/", chunk_size=100)
    obs, action_chunk = dataset[0]
    # obs["qpos"]             → [6]        float32 tensor (normalized)
    # obs["images"]["left"]   → [3,240,320] float32 tensor (ImageNet norm)
    # obs["images"]["right"]  → [3,240,320] float32 tensor (ImageNet norm)
    # action_chunk            → [chunk_size, 6] float32 tensor (normalized)
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

# ImageNet normalization constants
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class EpisodeDataset(Dataset):
    """Flat-indexed dataset over all (episode, timestep) pairs.

    Args:
        data_dir: Directory containing episode_XXXX.hdf5 files.
        chunk_size: Number of future actions to return per sample.
        cache: If True, load all HDF5 data into RAM at init.
    """

    def __init__(
        self,
        data_dir: str,
        chunk_size: int = 100,
        cache: bool = False,
    ):
        self.data_dir = Path(data_dir)
        self.chunk_size = chunk_size
        self.cache = cache

        # Discover episode files
        self._episode_paths: List[Path] = sorted(
            self.data_dir.glob("episode_*.hdf5")
        )
        if len(self._episode_paths) == 0:
            raise FileNotFoundError(
                f"No episode_*.hdf5 files found in {self.data_dir}"
            )

        # Build flat index: list of (episode_idx, timestep)
        self._index: List[Tuple[int, int]] = []
        self._episode_lengths: List[int] = []

        for ep_idx, path in enumerate(self._episode_paths):
            with h5py.File(path, "r") as f:
                T = f["observations/qpos"].shape[0]
            self._episode_lengths.append(T)
            for t in range(T):
                self._index.append((ep_idx, t))

        # Compute normalization statistics over all episodes
        self.dataset_stats = self._compute_stats()

        # Optional: load everything into RAM
        self._cache_data: Optional[List[Dict]] = None
        if cache:
            self._load_cache()

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def _compute_stats(self) -> Dict:
        """Compute per-joint mean/std for qpos and min/max for actions."""
        all_qpos: List[np.ndarray] = []
        all_actions: List[np.ndarray] = []

        for path in self._episode_paths:
            with h5py.File(path, "r") as f:
                all_qpos.append(f["observations/qpos"][:])
                all_actions.append(f["actions"][:])

        qpos_cat = np.concatenate(all_qpos, axis=0)   # [N, 6]
        act_cat = np.concatenate(all_actions, axis=0)  # [N, 6]

        qpos_mean = qpos_cat.mean(axis=0).astype(np.float32)
        qpos_std = qpos_cat.std(axis=0).astype(np.float32)
        # Avoid division by zero
        qpos_std = np.maximum(qpos_std, 1e-6)

        action_min = act_cat.min(axis=0).astype(np.float32)
        action_max = act_cat.max(axis=0).astype(np.float32)
        action_range = np.maximum(action_max - action_min, 1e-6)

        return {
            "qpos_mean": qpos_mean,
            "qpos_std": qpos_std,
            "action_min": action_min,
            "action_max": action_max,
            "action_range": action_range,
        }

    # ------------------------------------------------------------------
    # Cache
    # ------------------------------------------------------------------

    def _load_cache(self):
        """Load all episode data into RAM."""
        self._cache_data = []
        for path in self._episode_paths:
            with h5py.File(path, "r") as f:
                self._cache_data.append({
                    "qpos": f["observations/qpos"][:],
                    "left": f["observations/images/left"][:],
                    "right": f["observations/images/right"][:],
                    "actions": f["actions"][:],
                })

    def _read_episode(self, ep_idx: int) -> Dict:
        """Return episode data dict, from cache or HDF5."""
        if self._cache_data is not None:
            return self._cache_data[ep_idx]
        path = self._episode_paths[ep_idx]
        with h5py.File(path, "r") as f:
            return {
                "qpos": f["observations/qpos"][:],
                "left": f["observations/images/left"][:],
                "right": f["observations/images/right"][:],
                "actions": f["actions"][:],
            }

    # ------------------------------------------------------------------
    # Normalization helpers
    # ------------------------------------------------------------------

    def normalize_qpos(self, qpos: np.ndarray) -> np.ndarray:
        """Normalize qpos by per-joint mean/std."""
        return (qpos - self.dataset_stats["qpos_mean"]) / self.dataset_stats["qpos_std"]

    def normalize_actions(self, actions: np.ndarray) -> np.ndarray:
        """Normalize actions to [-1, 1] per joint via min/max scaling."""
        mn = self.dataset_stats["action_min"]
        rng = self.dataset_stats["action_range"]
        return 2.0 * (actions - mn) / rng - 1.0

    def denormalize_actions(self, actions: np.ndarray) -> np.ndarray:
        """Invert normalize_actions."""
        mn = self.dataset_stats["action_min"]
        rng = self.dataset_stats["action_range"]
        return (actions + 1.0) / 2.0 * rng + mn

    @staticmethod
    def normalize_image(img: np.ndarray) -> np.ndarray:
        """Convert uint8 [H,W,3] → float32 [3,H,W] ImageNet normalized."""
        x = img.astype(np.float32) / 255.0  # [H,W,3]
        x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
        return x.transpose(2, 0, 1)  # [3,H,W]

    # ------------------------------------------------------------------
    # Dataset interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, idx: int) -> Tuple[Dict, torch.Tensor]:
        ep_idx, t = self._index[idx]
        ep = self._read_episode(ep_idx)
        T = ep["qpos"].shape[0]

        # Normalize qpos
        qpos = self.normalize_qpos(ep["qpos"][t])  # [6]

        # Normalize images
        left = self.normalize_image(ep["left"][t])   # [3,240,320]
        right = self.normalize_image(ep["right"][t])  # [3,240,320]

        # Action chunk: pad with last action if near episode end
        t_end = min(t + self.chunk_size, T)
        chunk = ep["actions"][t:t_end]  # [k, 6], k ≤ chunk_size
        if chunk.shape[0] < self.chunk_size:
            pad = np.tile(chunk[-1:], (self.chunk_size - chunk.shape[0], 1))
            chunk = np.concatenate([chunk, pad], axis=0)  # [chunk_size, 6]
        chunk = self.normalize_actions(chunk)

        obs = {
            "qpos": torch.from_numpy(qpos),
            "images": {
                "left": torch.from_numpy(left),
                "right": torch.from_numpy(right),
            },
        }
        action_chunk = torch.from_numpy(chunk.astype(np.float32))
        return obs, action_chunk

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @property
    def num_episodes(self) -> int:
        return len(self._episode_paths)

    def stats_as_tensors(self) -> Dict[str, torch.Tensor]:
        """Return dataset_stats with numpy arrays converted to float tensors."""
        return {k: torch.from_numpy(v) for k, v in self.dataset_stats.items()}
