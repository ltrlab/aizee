"""
dataset.py — PyTorch Dataset over AIZEE HDF5 demonstration episodes.

Each HDF5 file has the schema:
    /observations/qpos           float32 [T, 6]
    /observations/qcmd           float32 [T, 6]   (optional — commanded positions)
    /observations/torques        float32 [T, 6]   (optional — motor torques)
    /observations/images/left    uint8   [T, 240, 320, 3]
    /observations/images/right   uint8   [T, 240, 320, 3]
    /actions                     float32 [T, 6]
    attrs: hz=20, arm_joints="gantry_base,..."

Usage:
    dataset = EpisodeDataset("episodes/", chunk_size=100, state_dim=12)
    obs, action_chunk = dataset[0]
    # obs["qpos"]             → [6]          float32 tensor (normalized)
    # obs["state"]            → [state_dim]  float32 tensor (normalized)
    # obs["images"]["left"]   → [3,240,320]  float32 tensor (ImageNet norm)
    # obs["images"]["right"]  → [3,240,320]  float32 tensor (ImageNet norm)
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
        state_dim: Dimension of the state vector fed to the decoder.
            6  = qpos only (backward compatible)
            12 = [qpos, qcmd]
            18 = [qpos, qcmd, torques]
    """

    def __init__(
        self,
        data_dir: str,
        chunk_size: int = 100,
        cache: bool = False,
        state_dim: int = 6,
    ):
        self.data_dir = Path(data_dir)
        self.chunk_size = chunk_size
        self.cache = cache
        self.state_dim = state_dim

        assert state_dim in (6, 12, 18), f"state_dim must be 6, 12, or 18, got {state_dim}"

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
        """Compute per-joint mean/std for qpos, qcmd, torques and actions."""
        all_qpos: List[np.ndarray] = []
        all_actions: List[np.ndarray] = []
        all_qcmd: List[np.ndarray] = []
        all_torques: List[np.ndarray] = []

        for path in self._episode_paths:
            with h5py.File(path, "r") as f:
                qpos = f["observations/qpos"][:]
                all_qpos.append(qpos)
                all_actions.append(f["actions"][:])
                # qcmd: fall back to qpos if not present (zero compliance)
                if "observations/qcmd" in f:
                    all_qcmd.append(f["observations/qcmd"][:])
                else:
                    all_qcmd.append(qpos.copy())
                # torques: zero-fill if not present
                if "observations/torques" in f:
                    all_torques.append(f["observations/torques"][:])
                else:
                    all_torques.append(np.zeros_like(qpos))

        qpos_cat = np.concatenate(all_qpos, axis=0)       # [N, 6]
        act_cat = np.concatenate(all_actions, axis=0)      # [N, 6]
        qcmd_cat = np.concatenate(all_qcmd, axis=0)        # [N, 6]
        torque_cat = np.concatenate(all_torques, axis=0)    # [N, 6]

        qpos_mean = qpos_cat.mean(axis=0).astype(np.float32)
        qpos_std = qpos_cat.std(axis=0).astype(np.float32)
        qpos_std = np.maximum(qpos_std, 1e-6)

        qcmd_mean = qcmd_cat.mean(axis=0).astype(np.float32)
        qcmd_std = qcmd_cat.std(axis=0).astype(np.float32)
        qcmd_std = np.maximum(qcmd_std, 1e-6)

        torque_mean = torque_cat.mean(axis=0).astype(np.float32)
        torque_std = torque_cat.std(axis=0).astype(np.float32)
        torque_std = np.maximum(torque_std, 1e-6)

        # Z-score normalization for actions (matches original ACT)
        action_mean = act_cat.mean(axis=0).astype(np.float32)
        action_std = act_cat.std(axis=0).astype(np.float32)
        action_std = np.maximum(action_std, 0.01)

        # Keep min/max for safety clamping in act_policy_node.py
        action_min = act_cat.min(axis=0).astype(np.float32)
        action_max = act_cat.max(axis=0).astype(np.float32)

        return {
            "qpos_mean": qpos_mean,
            "qpos_std": qpos_std,
            "qcmd_mean": qcmd_mean,
            "qcmd_std": qcmd_std,
            "torque_mean": torque_mean,
            "torque_std": torque_std,
            "action_mean": action_mean,
            "action_std": action_std,
            "action_min": action_min,
            "action_max": action_max,
        }

    # ------------------------------------------------------------------
    # Cache
    # ------------------------------------------------------------------

    def _load_cache(self):
        """Load all episode data into RAM."""
        self._cache_data = []
        for path in self._episode_paths:
            with h5py.File(path, "r") as f:
                qpos = f["observations/qpos"][:]
                ep = {
                    "qpos": qpos,
                    "left": f["observations/images/left"][:],
                    "right": f["observations/images/right"][:],
                    "actions": f["actions"][:],
                    "qcmd": f["observations/qcmd"][:] if "observations/qcmd" in f else qpos.copy(),
                    "torques": f["observations/torques"][:] if "observations/torques" in f else np.zeros_like(qpos),
                }
                self._cache_data.append(ep)

    def _read_episode(self, ep_idx: int) -> Dict:
        """Return episode data dict, from cache or HDF5."""
        if self._cache_data is not None:
            return self._cache_data[ep_idx]
        path = self._episode_paths[ep_idx]
        with h5py.File(path, "r") as f:
            qpos = f["observations/qpos"][:]
            return {
                "qpos": qpos,
                "left": f["observations/images/left"][:],
                "right": f["observations/images/right"][:],
                "actions": f["actions"][:],
                "qcmd": f["observations/qcmd"][:] if "observations/qcmd" in f else qpos.copy(),
                "torques": f["observations/torques"][:] if "observations/torques" in f else np.zeros_like(qpos),
            }

    # ------------------------------------------------------------------
    # Normalization helpers
    # ------------------------------------------------------------------

    def normalize_qpos(self, qpos: np.ndarray) -> np.ndarray:
        """Normalize qpos by per-joint mean/std."""
        return (qpos - self.dataset_stats["qpos_mean"]) / self.dataset_stats["qpos_std"]

    def normalize_qcmd(self, qcmd: np.ndarray) -> np.ndarray:
        """Normalize qcmd by per-joint mean/std."""
        return (qcmd - self.dataset_stats["qcmd_mean"]) / self.dataset_stats["qcmd_std"]

    def normalize_torques(self, torques: np.ndarray) -> np.ndarray:
        """Normalize torques by per-joint mean/std."""
        return (torques - self.dataset_stats["torque_mean"]) / self.dataset_stats["torque_std"]

    def normalize_actions(self, actions: np.ndarray) -> np.ndarray:
        """Normalize actions via z-score (mean/std), matching original ACT."""
        return (actions - self.dataset_stats["action_mean"]) / self.dataset_stats["action_std"]

    def denormalize_actions(self, actions: np.ndarray) -> np.ndarray:
        """Invert normalize_actions."""
        return actions * self.dataset_stats["action_std"] + self.dataset_stats["action_mean"]

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
        T = self._episode_lengths[ep_idx]
        t_end = min(t + self.chunk_size, T)

        # Read only the single timestep (and action slice) needed.
        if self._cache_data is not None:
            ep = self._cache_data[ep_idx]
            qpos_raw  = ep["qpos"][t]
            left_raw  = ep["left"][t]
            right_raw = ep["right"][t]
            chunk_raw = ep["actions"][t:t_end]
            qcmd_raw  = ep["qcmd"][t]
            torque_raw = ep["torques"][t]
        else:
            path = self._episode_paths[ep_idx]
            with h5py.File(path, "r") as f:
                qpos_raw  = f["observations/qpos"][t]
                left_raw  = f["observations/images/left"][t]
                right_raw = f["observations/images/right"][t]
                chunk_raw = f["actions"][t:t_end]
                if "observations/qcmd" in f:
                    qcmd_raw = f["observations/qcmd"][t]
                else:
                    qcmd_raw = qpos_raw.copy()
                if "observations/torques" in f:
                    torque_raw = f["observations/torques"][t]
                else:
                    torque_raw = np.zeros_like(qpos_raw)

        # Normalize qpos (always [6] — used by CVAE encoder)
        qpos = self.normalize_qpos(qpos_raw)  # [6]

        # Build extended state vector based on state_dim
        if self.state_dim == 6:
            state = qpos.copy()
        elif self.state_dim == 12:
            qcmd = self.normalize_qcmd(qcmd_raw)
            state = np.concatenate([qpos, qcmd])  # [12]
        else:  # 18
            qcmd = self.normalize_qcmd(qcmd_raw)
            torques = self.normalize_torques(torque_raw)
            state = np.concatenate([qpos, qcmd, torques])  # [18]

        # Normalize images
        left  = self.normalize_image(left_raw)   # [3,240,320]
        right = self.normalize_image(right_raw)  # [3,240,320]

        # Action chunk: pad with last action if near episode end
        chunk = chunk_raw
        if chunk.shape[0] < self.chunk_size:
            pad = np.tile(chunk[-1:], (self.chunk_size - chunk.shape[0], 1))
            chunk = np.concatenate([chunk, pad], axis=0)  # [chunk_size, 6]
        chunk = self.normalize_actions(chunk)

        obs = {
            "qpos": torch.from_numpy(qpos),
            "state": torch.from_numpy(state.astype(np.float32)),
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
