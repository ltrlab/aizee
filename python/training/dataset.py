"""
dataset.py — PyTorch Dataset over AIZEE HDF5 demonstration episodes.

Each HDF5 file (format_version=4) has the schema:
    /observations/qpos             float32 [T, J]   (J = 7: swivel + 6 arm joints)
    /observations/qcmd             float32 [T, J]   (optional — commanded positions)
    /observations/torques          float32 [T, J]   (optional — motor torques)
    /observations/images/gripper   uint8   [T, 768, 1024, 3]  (single ELP UVC cam)
    /actions                       float32 [T, J]
    attrs: hz=20, arm_joints="swivel,gantry_base,...", action_space="absolute"

The previous stereo D435 schema (format_version<=3, 240x320 frames) wrote
`/observations/images/left` and `/observations/images/right` — that data is
no longer compatible with this loader; collect new episodes on the new
single-camera rig (which captures and stores 1024x768).

Usage:
    dataset = EpisodeDataset("episodes/", chunk_size=32, state_mode="qpos_qcmd",
                             action_mode="relative", augment=True)
    obs, action_chunk = dataset[0]
    # obs["qpos"]                 → [J]           float32 tensor (normalized)
    # obs["state"]                → [J*k]         float32 tensor (normalized, k=1/2/3)
    # obs["images"]["gripper"]    → [3,768,1024]  float32 tensor (ImageNet norm)
    # action_chunk                → [chunk_size, J] float32 tensor (normalized)

Action modes:
    "absolute" — predict absolute joint targets (original ACT).
    "relative" — predict (target - current_qpos); inference adds current qpos back.
                 Typically generalizes much better on small datasets because the
                 policy only has to learn motion shapes, not absolute poses.
"""

from __future__ import annotations

import random
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import h5py
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

# ImageNet normalization constants
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

STATE_MODES = ("qpos", "qpos_qcmd", "qpos_qcmd_tq")
ACTION_MODES = ("absolute", "relative")


def _state_mode_k(mode: str) -> int:
    """Number of per-joint feature blocks concatenated to form state."""
    if mode == "qpos":         return 1
    if mode == "qpos_qcmd":    return 2
    if mode == "qpos_qcmd_tq": return 3
    raise ValueError(f"unknown state_mode: {mode!r}; expected one of {STATE_MODES}")


class EpisodeDataset(Dataset):
    """Flat-indexed dataset over all (episode, timestep) pairs.

    Args:
        data_dir: Directory containing episode_XXXX.hdf5 files.
        chunk_size: Number of future actions to return per sample.
        state_mode: What to pack into the state vector fed to the decoder.
            "qpos"         — qpos only (1 × J)
            "qpos_qcmd"    — qpos + qcmd (2 × J)
            "qpos_qcmd_tq" — qpos + qcmd + torques (3 × J)
        action_mode: "absolute" or "relative". See module docstring.
        augment: If True, apply train-time image + small state noise.
                 Leave off for validation and offline eval.
        cache: If True, load all HDF5 data into RAM at init.
        episode_paths: Explicit list of paths (overrides data_dir glob). Used
                       for train/val splitting — pass the split's paths here
                       and reuse dataset_stats from the training subset via
                       the `dataset_stats` arg.
        dataset_stats: If provided, skip stat computation and use these. Used
                       for the validation set so its normalization matches the
                       training set exactly.
        future_offset: If > 0, the sample dict also contains
                       obs["future_images"]["gripper"] at index
                       t + future_offset (clamped to T-1 near episode end).
                       Used by the JEPA world-model objective in ACT-JEPA —
                       the predictor learns to predict the future image
                       representation from the current one.
    """

    def __init__(
        self,
        data_dir: Optional[str] = None,
        chunk_size: int = 32,
        state_mode: str = "qpos_qcmd",
        action_mode: str = "relative",
        augment: bool = False,
        cache: bool = False,
        episode_paths: Optional[List[Path]] = None,
        dataset_stats: Optional[Dict] = None,
        future_offset: int = 0,
    ):
        if state_mode not in STATE_MODES:
            raise ValueError(f"state_mode must be in {STATE_MODES}, got {state_mode!r}")
        if action_mode not in ACTION_MODES:
            raise ValueError(f"action_mode must be in {ACTION_MODES}, got {action_mode!r}")

        self.chunk_size = chunk_size
        self.state_mode = state_mode
        self.action_mode = action_mode
        self.augment = augment
        self.cache = cache
        self.future_offset = max(0, int(future_offset))
        self._k = _state_mode_k(state_mode)

        # Resolve episode paths
        if episode_paths is not None:
            self._episode_paths: List[Path] = list(episode_paths)
        else:
            if data_dir is None:
                raise ValueError("either data_dir or episode_paths must be provided")
            self._episode_paths = sorted(Path(data_dir).glob("episode_*.hdf5"))
        if len(self._episode_paths) == 0:
            raise FileNotFoundError("no episode_*.hdf5 files found")

        # Infer num_joints from first episode
        with h5py.File(self._episode_paths[0], "r") as f:
            qpos0 = f["observations/qpos"]
            self.num_joints = int(qpos0.shape[1])
        self.state_dim = self.num_joints * self._k
        self.action_dim = self.num_joints

        # Build flat index and episode lengths
        self._episode_lengths: List[int] = []
        self._index: List[Tuple[int, int]] = []
        for ep_idx, path in enumerate(self._episode_paths):
            with h5py.File(path, "r") as f:
                T = f["observations/qpos"].shape[0]
                assert f["observations/qpos"].shape[1] == self.num_joints, (
                    f"{path.name}: qpos has {f['observations/qpos'].shape[1]} "
                    f"joints, expected {self.num_joints}"
                )
            self._episode_lengths.append(T)
            for t in range(T):
                self._index.append((ep_idx, t))

        # Stats
        if dataset_stats is not None:
            self.dataset_stats = dataset_stats
        else:
            self.dataset_stats = self._compute_stats()

        # Optional in-memory cache
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
        all_rel_actions: List[np.ndarray] = []
        start_poses: List[np.ndarray] = []

        for path in self._episode_paths:
            with h5py.File(path, "r") as f:
                qpos = f["observations/qpos"][:]
                actions = f["actions"][:]
                qcmd = f["observations/qcmd"][:] if "observations/qcmd" in f else qpos.copy()
                torques = (f["observations/torques"][:]
                           if "observations/torques" in f else np.zeros_like(qpos))

            all_qpos.append(qpos)
            all_actions.append(actions)
            all_qcmd.append(qcmd)
            all_torques.append(torques)
            start_poses.append(qpos[0])

            # For relative actions we normalize per-joint deltas (target − current qpos)
            # over the chunk horizon. Use qpos as the anchor since that's what the
            # policy sees at inference.
            T = qpos.shape[0]
            horizons = min(self.chunk_size, T)
            for t in range(T):
                end = min(t + horizons, T)
                all_rel_actions.append(actions[t:end] - qpos[t:t + 1])

        qpos_cat = np.concatenate(all_qpos, axis=0)
        act_cat = np.concatenate(all_actions, axis=0)
        qcmd_cat = np.concatenate(all_qcmd, axis=0)
        torque_cat = np.concatenate(all_torques, axis=0)
        rel_cat = np.concatenate(all_rel_actions, axis=0)

        def _mean_std(x: np.ndarray, min_std: float = 1e-6) -> Tuple[np.ndarray, np.ndarray]:
            m = x.mean(axis=0).astype(np.float32)
            s = np.maximum(x.std(axis=0).astype(np.float32), min_std)
            return m, s

        qpos_mean, qpos_std = _mean_std(qpos_cat)
        qcmd_mean, qcmd_std = _mean_std(qcmd_cat)
        torque_mean, torque_std = _mean_std(torque_cat)
        action_mean, action_std = _mean_std(act_cat, min_std=0.01)
        rel_mean, rel_std = _mean_std(rel_cat, min_std=0.005)

        # Keep min/max for safety clamping in act_policy_node.py
        action_min = act_cat.min(axis=0).astype(np.float32)
        action_max = act_cat.max(axis=0).astype(np.float32)
        # Per-joint range of relative actions — used as a conservative deploy clamp
        rel_min = rel_cat.min(axis=0).astype(np.float32)
        rel_max = rel_cat.max(axis=0).astype(np.float32)

        start_arr = np.stack(start_poses)
        return {
            "qpos_mean": qpos_mean, "qpos_std": qpos_std,
            "qcmd_mean": qcmd_mean, "qcmd_std": qcmd_std,
            "torque_mean": torque_mean, "torque_std": torque_std,
            "action_mean": action_mean, "action_std": action_std,
            "action_min": action_min, "action_max": action_max,
            "rel_action_mean": rel_mean, "rel_action_std": rel_std,
            "rel_action_min": rel_min, "rel_action_max": rel_max,
            # Representative start poses — policy ramps to the closest one at deploy
            "start_poses": start_arr.astype(np.float32),
            "ready_pose": start_arr.mean(axis=0).astype(np.float32),
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
                self._cache_data.append({
                    "qpos": qpos,
                    "gripper": f["observations/images/gripper"][:],
                    "actions": f["actions"][:],
                    "qcmd": (f["observations/qcmd"][:]
                             if "observations/qcmd" in f else qpos.copy()),
                    "torques": (f["observations/torques"][:]
                                if "observations/torques" in f else np.zeros_like(qpos)),
                })

    def _read_frame(self, ep_idx: int, t: int, t_end: int) -> Dict:
        """Return per-frame arrays for (ep_idx, t) plus action chunk [t:t_end].

        If `future_offset > 0`, also returns `future_gripper` at index
        `min(t + future_offset, T-1)` for the JEPA target encoder.
        """
        T = self._episode_lengths[ep_idx]
        t_future = min(t + self.future_offset, T - 1) if self.future_offset > 0 else None

        if self._cache_data is not None:
            ep = self._cache_data[ep_idx]
            out = {
                "qpos": ep["qpos"][t],
                "gripper": ep["gripper"][t],
                "actions": ep["actions"][t:t_end],
                "qcmd": ep["qcmd"][t],
                "torques": ep["torques"][t],
            }
            if t_future is not None:
                out["future_gripper"] = ep["gripper"][t_future]
            return out
        path = self._episode_paths[ep_idx]
        with h5py.File(path, "r") as f:
            qpos = f["observations/qpos"][t]
            out = {
                "qpos": qpos,
                "gripper": f["observations/images/gripper"][t],
                "actions": f["actions"][t:t_end],
                "qcmd": f["observations/qcmd"][t] if "observations/qcmd" in f else qpos.copy(),
                "torques": (f["observations/torques"][t]
                            if "observations/torques" in f else np.zeros_like(qpos)),
            }
            if t_future is not None:
                out["future_gripper"] = f["observations/images/gripper"][t_future]
        return out

    # ------------------------------------------------------------------
    # Normalization
    # ------------------------------------------------------------------

    def _norm(self, x: np.ndarray, mean_key: str, std_key: str) -> np.ndarray:
        return (x - self.dataset_stats[mean_key]) / self.dataset_stats[std_key]

    def normalize_qpos(self, qpos):    return self._norm(qpos, "qpos_mean", "qpos_std")
    def normalize_qcmd(self, qcmd):    return self._norm(qcmd, "qcmd_mean", "qcmd_std")
    def normalize_torques(self, tq):   return self._norm(tq, "torque_mean", "torque_std")

    def normalize_actions(self, actions: np.ndarray, qpos: Optional[np.ndarray] = None) -> np.ndarray:
        """Normalize target actions according to the configured action_mode.

        For "relative" mode, qpos is the anchor (current joint position) — each
        chunk step is converted to (action - qpos) before normalization.
        """
        if self.action_mode == "absolute":
            return self._norm(actions, "action_mean", "action_std")
        if qpos is None:
            raise ValueError("qpos anchor required for relative action mode")
        rel = actions - qpos[None, :]
        return self._norm(rel, "rel_action_mean", "rel_action_std")

    def denormalize_actions(
        self, norm_actions: np.ndarray, qpos: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Invert normalize_actions → absolute joint positions."""
        if self.action_mode == "absolute":
            return norm_actions * self.dataset_stats["action_std"] + self.dataset_stats["action_mean"]
        if qpos is None:
            raise ValueError("qpos anchor required for relative action mode")
        rel = norm_actions * self.dataset_stats["rel_action_std"] + self.dataset_stats["rel_action_mean"]
        # Broadcast qpos across chunk dim if needed
        if rel.ndim == 2 and qpos.ndim == 1:
            return rel + qpos[None, :]
        return rel + qpos

    # ------------------------------------------------------------------
    # Image augmentation
    # ------------------------------------------------------------------

    @staticmethod
    def _imagenet_normalize(img_u8: np.ndarray) -> np.ndarray:
        """uint8 [H,W,3] → float32 [3,H,W] ImageNet normalized."""
        x = img_u8.astype(np.float32) / 255.0
        x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
        return x.transpose(2, 0, 1)

    # Preserve the original static-method name for back-compat with act_policy_node.py
    normalize_image = _imagenet_normalize

    def _augment_and_normalize(
        self, gripper_u8: np.ndarray, rng: random.Random
    ) -> np.ndarray:
        """Train-time augmentation for the gripper camera."""
        out = self._augment_batch_and_normalize([gripper_u8], rng)
        return out[0]

    def _augment_batch_and_normalize(
        self, imgs_u8: List[np.ndarray], rng: random.Random
    ) -> List[np.ndarray]:
        """Augment a batch of co-registered images.

        - SHARED geometric crop across every image in the batch (so current
          and future frames stay spatially consistent for the JEPA prediction
          task — a different crop would corrupt the prediction objective).
        - INDEPENDENT color jitter and blur per image (simulates frame-to-
          frame exposure / WB drift).
        - ImageNet normalization at the end.
        """
        assert len(imgs_u8) > 0
        H, W = imgs_u8[0].shape[:2]

        scale = rng.uniform(0.88, 1.00)
        ar = rng.uniform(0.95, 1.05)
        crop_h = int(round(H * scale))
        crop_w = int(round(H * scale * ar * (W / H)))
        crop_h = max(1, min(H, crop_h))
        crop_w = max(1, min(W, crop_w))
        top = rng.randint(0, H - crop_h)
        left = rng.randint(0, W - crop_w)

        def geom(img: np.ndarray) -> np.ndarray:
            cropped = img[top:top + crop_h, left:left + crop_w]
            t = torch.from_numpy(cropped).permute(2, 0, 1).unsqueeze(0).float()
            t = F.interpolate(t, size=(H, W), mode="bilinear", align_corners=False)
            return t.squeeze(0).permute(1, 2, 0).clamp(0, 255).to(torch.uint8).numpy()

        def color_jitter(img_u8: np.ndarray) -> np.ndarray:
            img = img_u8.astype(np.float32) / 255.0
            img = img * rng.uniform(0.80, 1.20)
            m = img.mean()
            img = (img - m) * rng.uniform(0.80, 1.20) + m
            gray = img.mean(axis=2, keepdims=True)
            img = gray + (img - gray) * rng.uniform(0.80, 1.20)
            shift = rng.uniform(-0.05, 0.05)
            img = img + np.array([shift, 0, -shift], dtype=np.float32)[None, None, :]
            return np.clip(img * 255.0, 0, 255).astype(np.uint8)

        def maybe_blur(img: np.ndarray) -> np.ndarray:
            if rng.random() >= 0.1:
                return img
            t = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float()
            t = F.avg_pool2d(t, kernel_size=3, stride=1, padding=1)
            return t.squeeze(0).permute(1, 2, 0).clamp(0, 255).to(torch.uint8).numpy()

        outs: List[np.ndarray] = []
        for img in imgs_u8:
            x = geom(img)
            if rng.random() < 0.8:
                x = color_jitter(x)
            x = maybe_blur(x)
            outs.append(self._imagenet_normalize(x))
        return outs

    # ------------------------------------------------------------------
    # Dataset interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, idx: int) -> Tuple[Dict, torch.Tensor]:
        ep_idx, t = self._index[idx]
        T = self._episode_lengths[ep_idx]
        t_end = min(t + self.chunk_size, T)

        frame = self._read_frame(ep_idx, t, t_end)
        qpos_raw = frame["qpos"]
        qcmd_raw = frame["qcmd"]
        torque_raw = frame["torques"]
        chunk_raw = frame["actions"]
        gripper_raw = frame["gripper"]

        # State vector
        qpos_n = self.normalize_qpos(qpos_raw).astype(np.float32)
        if self.state_mode == "qpos":
            state = qpos_n
        elif self.state_mode == "qpos_qcmd":
            state = np.concatenate([qpos_n, self.normalize_qcmd(qcmd_raw)]).astype(np.float32)
        else:  # qpos_qcmd_tq
            state = np.concatenate([
                qpos_n,
                self.normalize_qcmd(qcmd_raw),
                self.normalize_torques(torque_raw),
            ]).astype(np.float32)

        # Future-frame (JEPA target). Only present when future_offset > 0.
        future_gripper_raw = frame.get("future_gripper")
        has_future = future_gripper_raw is not None

        # Image augmentation / normalization. When future frame is present
        # the same geometric crop is shared across current + future so the
        # JEPA prediction task remains spatially consistent.
        if self.augment:
            rng = random.Random((ep_idx * 1_000_003 + t) ^ torch.randint(0, 2**31 - 1, (1,)).item())
            if has_future:
                gripper_n, fgrip_n = self._augment_batch_and_normalize(
                    [gripper_raw, future_gripper_raw], rng
                )
            else:
                gripper_n = self._augment_and_normalize(gripper_raw, rng)
                fgrip_n = None
        else:
            gripper_n = self._imagenet_normalize(gripper_raw)
            fgrip_n = self._imagenet_normalize(future_gripper_raw) if has_future else None

        # Action chunk — pad with last action if near episode end
        chunk = chunk_raw.astype(np.float32)
        if chunk.shape[0] < self.chunk_size:
            pad = np.tile(chunk[-1:], (self.chunk_size - chunk.shape[0], 1))
            chunk = np.concatenate([chunk, pad], axis=0)

        chunk_n = self.normalize_actions(chunk, qpos=qpos_raw).astype(np.float32)

        obs = {
            "qpos": torch.from_numpy(qpos_n),
            "state": torch.from_numpy(state),
            "images": {
                "gripper": torch.from_numpy(gripper_n),
            },
        }
        if has_future:
            obs["future_images"] = {
                "gripper": torch.from_numpy(fgrip_n),
            }
        return obs, torch.from_numpy(chunk_n)

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @property
    def num_episodes(self) -> int:
        return len(self._episode_paths)

    @property
    def episode_paths(self) -> List[Path]:
        return list(self._episode_paths)

    def stats_as_tensors(self) -> Dict[str, torch.Tensor]:
        """Return dataset_stats with numpy arrays converted to float tensors."""
        return {k: torch.from_numpy(v) if isinstance(v, np.ndarray) else v
                for k, v in self.dataset_stats.items()}


def split_episodes(
    data_dir: str,
    val_fraction: float,
    seed: int = 0,
    episode_min: Optional[int] = None,
    episode_max: Optional[int] = None,
) -> Tuple[List[Path], List[Path]]:
    """Split episode files into train/val sets by episode (never within an episode).

    A deterministic shuffle of the sorted file list is used so re-running with
    the same seed reproduces the split. At least one episode is kept in each
    subset when val_fraction > 0.

    `episode_min` / `episode_max` filter by the integer suffix in the filename
    (`episode_0068.hdf5` -> 68), inclusive on both ends.
    """
    paths = sorted(Path(data_dir).glob("episode_*.hdf5"))
    if episode_min is not None or episode_max is not None:
        lo = episode_min if episode_min is not None else -1
        hi = episode_max if episode_max is not None else 10**9
        def _idx(p: Path) -> int:
            return int(p.stem.split("_")[-1])
        paths = [p for p in paths if lo <= _idx(p) <= hi]
    if len(paths) == 0:
        raise FileNotFoundError(f"no episode_*.hdf5 files found in {data_dir}")
    if val_fraction <= 0:
        return paths, []

    rng = random.Random(seed)
    shuffled = paths[:]
    rng.shuffle(shuffled)
    n_val = max(1, int(round(len(shuffled) * val_fraction)))
    n_val = min(n_val, len(shuffled) - 1)   # keep at least one for train
    val = shuffled[:n_val]
    train = shuffled[n_val:]
    # Return in original file-order for readable logs
    return sorted(train), sorted(val)
