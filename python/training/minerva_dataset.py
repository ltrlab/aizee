"""
minerva_dataset.py — PyTorch Dataset over Minerva bimanual HDF5 episodes.

Extends the AIZEE single-camera dataset to Minerva's 3-camera, 17-DoF,
language-conditioned setting. Episode HDF5 schema (format_version >= 6):

    /observations/qpos                 float32 [T, 17]
    /observations/qcmd                 float32 [T, 17]   (optional)
    /observations/torques              float32 [T, 17]   (optional)
    /observations/images/left_wrist    uint8   [T, H, W, 3]
    /observations/images/right_wrist   uint8   [T, H, W, 3]
    /observations/images/head          uint8   [T, Hs, Ws, 3]
    /actions                           float32 [T, 17]
    attrs: hz, minerva_joints="left_arm_j1,...", action_space="absolute",
           language_instruction="pick up the red block", task_id=3

Key differences vs. the ACT dataset:
  - THREE camera streams (each with its own future frame for the JEPA target).
  - PERCENTILE [-1, 1] action normalization (2nd/98th pct) — robust to teleop
    outliers and the natural range for the flow-matching head. (mean/std is
    kept for the proprioceptive state.)
  - a per-episode task STRING → cached pooled language embedding (language.py),
    with optional paraphrase augmentation at train time.

Returned sample:
    obs = {
      "state":     [state_dim]              float32  (normalized qpos[/qcmd[/tq]])
      "images":    {cam: [3,H,W] float32}   ImageNet-normalized
      "future_images": {cam: [3,H,W]}       (only if future_offset > 0)
      "language":  [lang_dim] float32       (only if a conditioner is attached)
    }
    action_chunk = [chunk_size, 17] float32 (normalized)
"""

from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import h5py
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

from python.training.language import ParaphraseTable, TextConditioner

_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

STATE_MODES = ('qpos', 'qpos_qcmd', 'qpos_qcmd_tq')
ACTION_MODES = ('absolute', 'relative')
CAMERAS = ('left_wrist', 'right_wrist', 'head')


def _state_mode_k(mode: str) -> int:
    return {'qpos': 1, 'qpos_qcmd': 2, 'qpos_qcmd_tq': 3}[mode]


def parse_segments(seg_raw) -> list:
    """Parse the HDF5 `segments` attr (JSON) -> [{start,end,label}, ...] with
    half-open [start,end) frame ranges. Returns [] if absent/malformed."""
    if seg_raw is None:
        return []
    try:
        if isinstance(seg_raw, bytes):
            seg_raw = seg_raw.decode('utf-8')
        segs = json.loads(seg_raw)
        return [s for s in segs if isinstance(s, dict) and {'start', 'end', 'label'} <= set(s)]
    except Exception:
        return []


# ---------------------------------------------------------------------------
# Normalization helpers (module-level so the inference node can reuse them)
# ---------------------------------------------------------------------------


def imagenet_normalize(img_u8: np.ndarray) -> np.ndarray:
    """uint8 [H,W,3] -> float32 [3,H,W] ImageNet normalized."""
    x = img_u8.astype(np.float32) / 255.0
    x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
    return x.transpose(2, 0, 1)


def pct_normalize(a: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Map [lo, hi] -> [-1, 1] per-dimension."""
    rng = np.maximum(hi - lo, 1e-6)
    return (2.0 * (a - lo) / rng - 1.0).astype(np.float32)


def pct_denormalize(n: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Inverse of pct_normalize."""
    rng = np.maximum(hi - lo, 1e-6)
    return ((n + 1.0) * 0.5 * rng + lo).astype(np.float32)


def normalize_actions(
    actions: np.ndarray,
    stats: dict,
    action_mode: str,
    qpos: Optional[np.ndarray] = None,
) -> np.ndarray:
    if action_mode == 'absolute':
        return pct_normalize(actions, stats['action_lo'], stats['action_hi'])
    if qpos is None:
        raise ValueError('qpos anchor required for relative action mode')
    rel = actions - qpos[None, :]
    return pct_normalize(rel, stats['rel_action_lo'], stats['rel_action_hi'])


def denormalize_actions(
    norm: np.ndarray,
    stats: dict,
    action_mode: str,
    qpos: Optional[np.ndarray] = None,
) -> np.ndarray:
    if action_mode == 'absolute':
        return pct_denormalize(norm, stats['action_lo'], stats['action_hi'])
    if qpos is None:
        raise ValueError('qpos anchor required for relative action mode')
    rel = pct_denormalize(norm, stats['rel_action_lo'], stats['rel_action_hi'])
    if rel.ndim == 2 and qpos.ndim == 1:
        return rel + qpos[None, :]
    return rel + qpos


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------


class MinervaEpisodeDataset(Dataset):
    def __init__(
        self,
        episode_paths: List[Path],
        *,
        chunk_size: int = 32,
        state_mode: str = 'qpos_qcmd',
        action_mode: str = 'absolute',
        augment: bool = False,
        future_offset: int = 0,
        dataset_stats: Optional[Dict] = None,
        conditioner: Optional[TextConditioner] = None,
        paraphrases: Optional[ParaphraseTable] = None,
        allowed_authorities: Tuple[str, ...] = ('human', 'correction'),
        include_fixture: bool = False,
        correction_weight: float = 1.0,
    ):
        if state_mode not in STATE_MODES:
            raise ValueError(f'state_mode must be in {STATE_MODES}')
        if action_mode not in ACTION_MODES:
            raise ValueError(f'action_mode must be in {ACTION_MODES}')
        if correction_weight <= 0 or not np.isfinite(correction_weight):
            raise ValueError('correction_weight must be positive and finite')
        self.chunk_size = chunk_size
        self.state_mode = state_mode
        self.action_mode = action_mode
        self.augment = augment
        self.future_offset = max(0, int(future_offset))
        self._k = _state_mode_k(state_mode)
        self.conditioner = conditioner
        self.paraphrases = paraphrases or ParaphraseTable()
        auth_set = set(allowed_authorities)
        if include_fixture:
            auth_set.add('scripted_fixture')
        self.allowed_authorities = tuple(sorted(auth_set))
        self.correction_weight = float(correction_weight)

        self._paths = [Path(p) for p in episode_paths]
        if not self._paths:
            raise FileNotFoundError('no Minerva episode files provided')

        with h5py.File(self._paths[0], 'r') as f:
            self.num_joints = int(f['observations/qpos'].shape[1])
            raw_joints = f.attrs.get('minerva_joints', '')
            if isinstance(raw_joints, bytes):
                raw_joints = raw_joints.decode('utf-8')
            parsed_joints = tuple(j.strip() for j in str(raw_joints).split(',') if j.strip())
            self.joint_names: Tuple[str, ...] = (
                parsed_joints
                if len(parsed_joints) == self.num_joints
                else tuple(f'joint_{i}' for i in range(self.num_joints))
            )
            self._cameras = [c for c in CAMERAS if f'observations/images/{c}' in f]
            if not self._cameras:
                raise ValueError(f'{self._paths[0].name}: no known camera streams found')
            self.image_shapes: Dict[str, Tuple[int, int, int]] = {
                c: tuple(int(d) for d in f[f'observations/images/{c}'].shape[1:]) for c in self._cameras
            }
        self.state_dim = self.num_joints * self._k
        self.action_dim = self.num_joints

        # Flat (episode, timestep) index + contiguous segment bounds + task strings.
        self._lengths: List[int] = []
        self._index: List[Tuple[int, int]] = []
        self._seg_ends: List[int] = []
        self._authorities: List[str] = []
        self._ep_valid_masks: List[np.ndarray] = []
        self._task_strings: List[str] = []
        self._segments: List[list] = []

        for ei, p in enumerate(self._paths):
            with h5py.File(p, 'r') as f:
                if 'complete' in f.attrs and not bool(f.attrs['complete']):
                    raise ValueError(f'{p.name}: incomplete episode cannot be used for training')
                qpos_ds = f['observations/qpos']
                T, J = int(qpos_ds.shape[0]), int(qpos_ds.shape[1])
                if J != self.num_joints:
                    raise ValueError(f'{p.name}: num_joints={J} does not match dataset num_joints={self.num_joints}')
                ep_joints_raw = f.attrs.get('minerva_joints', '')
                if isinstance(ep_joints_raw, bytes):
                    ep_joints_raw = ep_joints_raw.decode('utf-8')
                ep_joints = tuple(j.strip() for j in str(ep_joints_raw).split(',') if j.strip())
                if ep_joints and self.joint_names and ep_joints != self.joint_names:
                    raise ValueError(f'{p.name}: joint order {ep_joints} does not match {self.joint_names}')
                for c in self._cameras:
                    if f'observations/images/{c}' not in f:
                        raise ValueError(f'{p.name}: missing required camera stream {c!r}')
                    c_shape = tuple(int(d) for d in f[f'observations/images/{c}'].shape[1:])
                    if c_shape != self.image_shapes[c]:
                        raise ValueError(f'{p.name}: camera {c!r} shape {c_shape} != {self.image_shapes[c]}')

                lang = f.attrs.get('language_instruction', f.attrs.get('task_tag', ''))
                if isinstance(lang, bytes):
                    lang = lang.decode('utf-8')
                seg_raw = f.attrs.get('segments')
                self._task_strings.append(str(lang))
                self._segments.append(parse_segments(seg_raw))
                self._lengths.append(T)

                if 'action_valid' in f:
                    act_valid = np.asarray(f['action_valid'][:], dtype=bool)
                elif f.attrs.get('requires_action_valid_mask', False):
                    raise ValueError(f'{p.name}: requires_action_valid_mask is set but action_valid is missing')
                else:
                    act_valid = np.ones(T, dtype=bool)

                if 'authority' in f:
                    raw_auth = f['authority'][:]
                    auth_list = [a.decode('utf-8') if isinstance(a, bytes) else str(a) for a in raw_auth]
                else:
                    auth_list = ['human'] * T

                if self.state_mode == 'qpos_qcmd_tq':
                    if 'observations/torques' not in f:
                        raise ValueError(f"{p.name}: state_mode='qpos_qcmd_tq' requires observations/torques")
                    tq_arr = f['observations/torques'][:]
                    tq_valid = (
                        np.asarray(f['observations/torque_valid'][:], dtype=bool)
                        if 'observations/torque_valid' in f
                        else np.isfinite(tq_arr)
                    )
                else:
                    tq_valid = None

            valid_mask = np.zeros(T, dtype=bool)
            for t in range(T):
                if not act_valid[t] or auth_list[t] not in auth_set:
                    continue
                if tq_valid is not None and not np.all(tq_valid[t]):
                    raise ValueError(f"{p.name}: frame {t} has invalid/NaN torques under state_mode='qpos_qcmd_tq'")
                valid_mask[t] = True
            self._ep_valid_masks.append(valid_mask)

            for t in range(T):
                if not valid_mask[t]:
                    continue
                label_t = self._label_at(ei, t)
                auth_t = auth_list[t]
                seg_end = t + 1
                while (
                    seg_end < T
                    and valid_mask[seg_end]
                    and auth_list[seg_end] == auth_t
                    and self._label_at(ei, seg_end) == label_t
                ):
                    seg_end += 1
                self._index.append((ei, t))
                self._seg_ends.append(seg_end)
                self._authorities.append(auth_t)

        if not self._index:
            raise ValueError(
                f'No valid training frames found across {len(self._paths)} episodes '
                f'for allowed_authorities={self.allowed_authorities}'
            )

        self.sample_weights = np.array(
            [self.correction_weight if a == 'correction' else 1.0 for a in self._authorities],
            dtype=np.float64,
        )
        self.dataset_stats = dataset_stats or self._compute_stats()

    # -- stats -------------------------------------------------------------
    def _compute_stats(self) -> Dict:
        J = self.num_joints
        qpos_all, qcmd_all, tq_all, act_all, rel_all, starts = [], [], [], [], [], []
        for ei, p in enumerate(self._paths):
            valid_mask = self._ep_valid_masks[ei]
            if not np.any(valid_mask):
                continue
            with h5py.File(p, 'r') as f:
                qpos = f['observations/qpos'][:]
                actions = f['actions'][:]
                # Only REAL fields feed the stats — never zero/qpos placeholders,
                # which would pull mean->0 and inflate std on a mixed dataset.
                qcmd = f['observations/qcmd'][:] if 'observations/qcmd' in f else None
                tq = f['observations/torques'][:] if 'observations/torques' in f else None
                tq_val = (
                    np.asarray(f['observations/torque_valid'][:], dtype=bool)
                    if 'observations/torque_valid' in f
                    else (np.isfinite(tq) if tq is not None else None)
                )
            first_valid = int(np.flatnonzero(valid_mask)[0])
            qpos_all.append(qpos[valid_mask])
            act_all.append(actions[valid_mask])
            starts.append(qpos[first_valid])
            if qcmd is not None:
                qcmd_all.append(qcmd[valid_mask])
            if tq is not None and tq_val is not None:
                finite_rows = valid_mask & np.all(tq_val & np.isfinite(tq), axis=1)
                if np.any(finite_rows):
                    tq_all.append(tq[finite_rows])

        # Compute relative action stats over the exact bounded segments indexed for training.
        by_ep: Dict[int, List[Tuple[int, int]]] = {}
        for (ei, t), seg_end in zip(self._index, self._seg_ends):
            by_ep.setdefault(ei, []).append((t, seg_end))
        for ei, spans in by_ep.items():
            with h5py.File(self._paths[ei], 'r') as f:
                qpos = f['observations/qpos'][:]
                actions = f['actions'][:]
            for t, seg_end in spans:
                t_end = min(t + self.chunk_size, seg_end)
                rel_all.append(actions[t:t_end] - qpos[t : t + 1])

        def mean_std(xs, floor=1e-6):
            if not xs:  # channel absent everywhere -> identity norm
                return np.zeros(J, np.float32), np.ones(J, np.float32)
            x = np.concatenate(xs, axis=0)
            return x.mean(0).astype(np.float32), np.maximum(x.std(0), floor).astype(np.float32)

        def pct(x, floor=1e-3):
            x = np.concatenate(x, axis=0)
            lo = np.percentile(x, 2, axis=0).astype(np.float32)
            hi = np.percentile(x, 98, axis=0).astype(np.float32)
            hi = np.maximum(hi, lo + floor)
            return lo, hi

        qpos_mean, qpos_std = mean_std(qpos_all)
        qcmd_mean, qcmd_std = mean_std(qcmd_all)
        tq_mean, tq_std = mean_std(tq_all)
        act_lo, act_hi = pct(act_all)
        rel_lo, rel_hi = pct(rel_all)
        starts = np.stack(starts).astype(np.float32)
        return {
            'qpos_mean': qpos_mean,
            'qpos_std': qpos_std,
            'qcmd_mean': qcmd_mean,
            'qcmd_std': qcmd_std,
            'torque_mean': tq_mean,
            'torque_std': tq_std,
            'action_lo': act_lo,
            'action_hi': act_hi,
            'rel_action_lo': rel_lo,
            'rel_action_hi': rel_hi,
            'start_poses': starts,
            'ready_pose': starts.mean(0).astype(np.float32),
        }

    # -- image aug ---------------------------------------------------------
    def _aug_pair(self, imgs_u8: List[np.ndarray], rng: random.Random) -> List[np.ndarray]:
        """Shared geometric crop across the frames in `imgs_u8` (current+future of
        ONE camera, so the JEPA prediction stays spatially consistent) + independent
        color jitter, then ImageNet normalize."""
        H, W = imgs_u8[0].shape[:2]
        scale = rng.uniform(0.88, 1.0)
        ar = rng.uniform(0.95, 1.05)
        crop_h = max(1, min(H, int(round(H * scale))))
        crop_w = max(1, min(W, int(round(H * scale * ar * (W / H)))))
        top = rng.randint(0, H - crop_h)
        left = rng.randint(0, W - crop_w)

        def geom(img):
            c = img[top : top + crop_h, left : left + crop_w]
            t = torch.from_numpy(c).permute(2, 0, 1).unsqueeze(0).float()
            t = F.interpolate(t, size=(H, W), mode='bilinear', align_corners=False)
            return t.squeeze(0).permute(1, 2, 0).clamp(0, 255).to(torch.uint8).numpy()

        def jitter(img_u8):
            img = img_u8.astype(np.float32) / 255.0
            img = img * rng.uniform(0.8, 1.2)
            m = img.mean()
            img = (img - m) * rng.uniform(0.8, 1.2) + m
            g = img.mean(2, keepdims=True)
            img = g + (img - g) * rng.uniform(0.8, 1.2)
            return np.clip(img * 255.0, 0, 255).astype(np.uint8)

        out = []
        for img in imgs_u8:
            x = geom(img)
            if rng.random() < 0.8:
                x = jitter(x)
            out.append(imagenet_normalize(x))
        return out

    # -- interface ---------------------------------------------------------
    def _label_at(self, ei: int, t: int) -> str:
        """Per-frame action label: the segment covering frame t, else the
        episode-level instruction."""
        for s in self._segments[ei]:
            if int(s['start']) <= t < int(s['end']):
                return str(s['label'])
        return self._task_strings[ei]

    def __len__(self) -> int:
        return len(self._index)

    def _read(self, f, cam: str, t: int) -> np.ndarray:
        return f[f'observations/images/{cam}'][t]

    def __getitem__(self, idx: int):
        ei, t = self._index[idx]
        seg_end = self._seg_ends[idx]
        T = self._lengths[ei]
        t_end = min(t + self.chunk_size, seg_end)
        # Future JEPA observation stays strictly inside the same valid authority/task segment.
        t_fut = min(t + self.future_offset, min(seg_end, T - 1)) if self.future_offset > 0 else None
        rng = random.Random((ei * 1_000_003 + t) ^ torch.randint(0, 2**31 - 1, (1,)).item())

        with h5py.File(self._paths[ei], 'r') as f:
            qpos = f['observations/qpos'][t].astype(np.float32)
            qcmd = (f['observations/qcmd'][t] if 'observations/qcmd' in f else qpos).astype(np.float32)
            if self.state_mode == 'qpos_qcmd_tq':
                tq = (f['observations/torques'][t] if 'observations/torques' in f else np.zeros_like(qpos)).astype(
                    np.float32
                )
            else:
                tq = None
            chunk = f['actions'][t:t_end].astype(np.float32)
            cur = {c: self._read(f, c, t) for c in self._cameras}
            fut = {c: self._read(f, c, t_fut) for c in self._cameras} if t_fut is not None else None

        # State (per-dim mean/std)
        s = self.dataset_stats
        qpos_n = (qpos - s['qpos_mean']) / s['qpos_std']
        if self.state_mode == 'qpos':
            state = qpos_n
        elif self.state_mode == 'qpos_qcmd':
            state = np.concatenate([qpos_n, (qcmd - s['qcmd_mean']) / s['qcmd_std']])
        else:
            state = np.concatenate(
                [
                    qpos_n,
                    (qcmd - s['qcmd_mean']) / s['qcmd_std'],
                    (tq - s['torque_mean']) / s['torque_std'],
                ]
            )

        # Images (per-camera shared current/future crop)
        images, future_images = {}, {}
        for c in self._cameras:
            if fut is not None:
                if self.augment:
                    cn, fn = self._aug_pair([cur[c], fut[c]], rng)
                else:
                    cn, fn = imagenet_normalize(cur[c]), imagenet_normalize(fut[c])
                images[c] = torch.from_numpy(cn)
                future_images[c] = torch.from_numpy(fn)
            else:
                cn = self._aug_pair([cur[c]], rng)[0] if self.augment else imagenet_normalize(cur[c])
                images[c] = torch.from_numpy(cn)

        # Action chunk (pad tail at segment boundary, percentile-normalize)
        if chunk.shape[0] < self.chunk_size:
            chunk = np.concatenate([chunk, np.tile(chunk[-1:], (self.chunk_size - chunk.shape[0], 1))], axis=0)
        chunk_n = normalize_actions(chunk, s, self.action_mode, qpos=qpos)

        obs: Dict = {'state': torch.from_numpy(state.astype(np.float32)), 'images': images}
        if fut is not None:
            obs['future_images'] = future_images

        # Language (per-frame segment label; optional paraphrase augmentation)
        if self.conditioner is not None:
            task = self._label_at(ei, t)
            if self.augment and task:
                task = self.paraphrases.sample(task, np.random.default_rng(rng.randint(0, 2**31 - 1)))
            vec = self.conditioner.get(task) if task else np.zeros(self.conditioner.embed_dim, np.float32)
            obs['language'] = torch.from_numpy(vec.astype(np.float32))

        return obs, torch.from_numpy(chunk_n)

    @property
    def cameras(self) -> List[str]:
        return list(self._cameras)


def collate_minerva(batch):
    """Collate MinervaEpisodeDataset samples into batched tensors."""
    obs_list, action_list = zip(*batch)
    cams = list(obs_list[0]['images'].keys())
    out: Dict = {
        'state': torch.stack([o['state'] for o in obs_list]),
        'images': {c: torch.stack([o['images'][c] for o in obs_list]) for c in cams},
    }
    if 'future_images' in obs_list[0]:
        out['future_images'] = {c: torch.stack([o['future_images'][c] for o in obs_list]) for c in cams}
    if 'language' in obs_list[0]:
        out['language'] = torch.stack([o['language'] for o in obs_list])
    return out, torch.stack(action_list)


def discover_minerva_episodes(data_dir: str | Path, *, include_ineligible: bool = False) -> List[Path]:
    """Discover completed `task_episode_*.hdf5` files, or legacy `episode_*.hdf5` files."""
    root = Path(data_dir)
    task_paths = sorted(p for p in root.glob('task_episode_*.hdf5') if not p.name.endswith('.partial.hdf5'))
    if task_paths:
        eligible_paths = []
        for p in task_paths:
            with h5py.File(p, 'r') as f:
                if 'complete' in f.attrs and not bool(f.attrs['complete']):
                    continue
                if not include_ineligible and 'capture_eligible' in f.attrs and not bool(f.attrs['capture_eligible']):
                    continue
                eligible_paths.append(p)
        if not eligible_paths:
            raise FileNotFoundError(
                f'found {len(task_paths)} task_episode_*.hdf5 in {data_dir}, '
                f'but none are complete and eligible (pass include_ineligible=True for fixtures)'
            )
        return eligible_paths
    legacy_paths = sorted(root.glob('episode_*.hdf5'))
    if not legacy_paths:
        raise FileNotFoundError(f'no task_episode_*.hdf5 or episode_*.hdf5 in {data_dir}')
    return legacy_paths


def split_episodes(
    data_dir: str,
    val_fraction: float,
    seed: int = 0,
    *,
    include_ineligible: bool = False,
):
    paths = discover_minerva_episodes(data_dir, include_ineligible=include_ineligible)
    if val_fraction <= 0 or len(paths) < 2:
        return paths, []
    rng = random.Random(seed)
    shuf = paths[:]
    rng.shuffle(shuf)
    n_val = min(max(1, round(len(shuf) * val_fraction)), len(shuf) - 1)
    return sorted(shuf[n_val:]), sorted(shuf[:n_val])


__all__ = [
    'MinervaEpisodeDataset',
    'collate_minerva',
    'discover_minerva_episodes',
    'split_episodes',
    'normalize_actions',
    'denormalize_actions',
    'imagenet_normalize',
    'pct_normalize',
    'pct_denormalize',
    'STATE_MODES',
    'ACTION_MODES',
    'CAMERAS',
]
