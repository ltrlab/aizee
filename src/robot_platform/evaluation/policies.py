"""Reference policies that validate the harness. None of them is a learned or capable policy.

- fixture_replay: the frozen offline-IK contact trajectory (tests/fixtures/minerva_task_contact_path.json)
  replayed open loop from its own prepositioned start. It reaches the task only where the cube sits
  where the fixture expects it, and only as far as the plant tracks it.
- hold_still: holds the receiver's initial (hanging) pose. Must score 0, every trial a timeout.
- penetrating: starts at the fixture's start and drives the left gripper down through the table.
  Must be ineligible (forbidden penetration).
"""
from __future__ import annotations

import hashlib
import importlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/minerva_task_contact_path.json'


def fixture_model_coordinates(fixture):
    """{fixture joint name: (model joint, model coordinate per fixture unit)}: the fixture's own explicit map
    (arm joints are their model joints; a gripper value times the fixture-only scale is its slide)."""
    scale = fixture['gripper_test_scale_m_per_motor_rad']
    return {name: (name, 1.) if 'arm' in name else (f"{name.split('_')[0]}_gripper_finger_a_slide", scale)
            for name in fixture['joint_names']}


def fixture_trajectory(joint_names, joint_map, fixture=None):
    """The fixture's waypoints interpolated exactly as its regression replays them, as (rows, motor coordinates in
    `joint_names` order) through the receiver's motor->model `joint_map`, and the row period (s)."""
    fixture = fixture if isinstance(fixture, dict) else json.loads(Path(fixture or FIXTURE).read_text(encoding='utf-8'))
    model = fixture_model_coordinates(fixture)
    by_motor = {row['actuator']: (joint, row) for joint, row in joint_map.items()}
    if set(joint_names) != set(fixture['joint_names']):
        raise ValueError('The fixture and the receiver name different joints')
    columns = []
    for name in joint_names:
        joint, row = by_motor[name]
        if joint != model[name][0]:
            raise ValueError(f'{name} drives {joint}, but the fixture moves {model[name][0]}')
        columns.append((fixture['joint_names'].index(name), model[name][1], float(row['scale']), float(row['offset'])))

    def motor(values):
        return np.array([(values[i] * unit - offset) / scale for i, unit, scale, offset in columns])
    waypoints = fixture['waypoints']
    q = motor(waypoints[0]['q'])
    rows = [q]
    for point in waypoints[1:]:
        target = motor(point['q'])
        rows += [q + (target - q) * (k + 1) / point['steps'] for k in range(point['steps'])]
        q = target
    return np.array(rows), float(fixture['step_s'])


class FixtureReplayPolicy:
    name = 'fixture_replay'

    def __init__(self, fixture=FIXTURE):
        self.fixture = Path(fixture)
        self.rows = None

    def reset(self, context):
        self.rows, self.action_dt = fixture_trajectory(context.joint_names, context.joint_map, self.fixture)

    def start_pose(self, context):
        return self.rows[0]

    def __call__(self, observation):
        # Open loop: the rest of the trajectory from now; past its end, its last row held.
        index = min(int(round(observation['t'] / self.action_dt)), len(self.rows) - 1)
        return self.rows[index:]

    def describe(self):
        return {'kind': 'open-loop replay', 'fixture': str(self.fixture),
                'sha256': hashlib.sha256(self.fixture.read_bytes()).hexdigest(),
                'rows': None if self.rows is None else len(self.rows),
                'note': 'frozen offline IK; prepositioned start; no feedback'}


class HoldStillPolicy:
    name = 'hold_still'

    def __init__(self, pose=None):
        self.pose = None if pose is None else np.asarray(pose, dtype=float)
        self.held = None

    def reset(self, context):
        self.held = context.home.copy() if self.pose is None else self.pose.copy()

    def start_pose(self, context):
        return self.held

    def __call__(self, observation):
        return self.held[None, :]

    def describe(self):
        return {'kind': 'hold the start pose', 'pose': 'receiver initial pose (arms hanging)' if self.pose is None
                else self.pose.tolist()}


class PenetratingPolicy:
    """From the fixture's prepositioned start (gripper open just above the table), lower the left shoulder pitch
    (left_arm_j2) by `depth_rad` over `duration_s` and hold: the jaw pads go down into the table."""
    name = 'penetrating'
    action_dt = .05

    def __init__(self, depth_rad=.3, duration_s=1., joint='left_arm_j2', fixture=FIXTURE):
        self.depth_rad, self.duration_s, self.joint, self.fixture = depth_rad, duration_s, joint, Path(fixture)
        self.rows = None

    def reset(self, context):
        rows, _ = fixture_trajectory(context.joint_names, context.joint_map, self.fixture)
        start = rows[0]
        end = start.copy()
        end[list(context.joint_names).index(self.joint)] -= self.depth_rad
        steps = max(1, round(self.duration_s / self.action_dt))
        self.rows = np.array([start + (end - start) * k / steps for k in range(steps + 1)])

    def start_pose(self, context):
        return self.rows[0]

    def __call__(self, observation):
        index = min(int(round(observation['t'] / self.action_dt)), len(self.rows) - 1)
        return self.rows[index:]

    def describe(self):
        return {'kind': 'deliberate table penetration (negative control)', 'joint': self.joint,
                'depth_rad': self.depth_rad, 'duration_s': self.duration_s}


class MinervaCheckpointPolicy:
    """Closed-loop adapter for a trained MinervaPolicy checkpoint (.pt).

    Supports both 14-DoF task-capture checkpoints (direct receiver joint order)
    and legacy 17-DoF checkpoints (mapping the 14 active arm/gripper slots and
    verifying inactive head/lift predictions are zero). Applies the checkpoint's
    stored image shape transform (including exact 2x integer reduction) and
    optional Real-Time Chunking (RTC) prefix inpainting across chunk boundaries.
    """

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        device: str = 'cpu',
        use_ema: bool = True,
        execute_steps: int | None = None,
        action_dt: float = 0.05,
        use_rtc: bool = True,
        seed: int = 0,
    ):
        import sys
        import torch
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from python.training.minerva_dataset import (
            denormalize_actions, imagenet_normalize, normalize_actions,
        )
        from python.training.minerva_model import MinervaPolicy

        self._torch = torch
        self._denormalize_actions = denormalize_actions
        self._normalize_actions = normalize_actions
        self._imagenet_normalize = imagenet_normalize

        self.checkpoint_path = Path(checkpoint_path)
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(f'Minerva checkpoint not found: {self.checkpoint_path}')
        self.sha256 = hashlib.sha256(self.checkpoint_path.read_bytes()).hexdigest()
        self.device = torch.device(device)
        self.use_ema = bool(use_ema)
        self.use_rtc = bool(use_rtc)
        self.seed = int(seed)

        ckpt = torch.load(self.checkpoint_path, map_location=self.device, weights_only=False)
        cfg = ckpt['config']
        self.config = dict(cfg)
        self.stats = {
            k: (np.asarray(v, dtype=np.float32) if isinstance(v, list) else v)
            for k, v in ckpt['dataset_stats'].items()
        }
        self.num_joints = int(cfg['num_joints'])
        self.chunk_size = int(cfg['chunk_size'])
        self.state_mode = str(cfg['state_mode'])
        self.action_mode = str(cfg['action_mode'])
        self.flow_steps = int(cfg.get('flow_steps', 8))
        self.camera_roles = tuple(cfg.get('cameras', ('left_wrist', 'right_wrist', 'head')))
        self.image_shapes = {
            c: tuple(int(x) for x in shape)
            for c, shape in (cfg.get('image_shapes') or {}).items()
        }
        self.ckpt_joint_names = tuple(cfg.get('joint_names') or ())
        self.action_dt = float(action_dt)
        self.execute_steps = int(execute_steps) if execute_steps is not None else max(1, self.chunk_size // 2)
        self.name = f'minerva:{self.checkpoint_path.stem}'

        self.model = MinervaPolicy(
            num_joints=self.num_joints,
            chunk_size=self.chunk_size,
            d_model=int(cfg['d_model']),
            state_dim=int(cfg['state_dim']),
            lang_dim=int(cfg.get('lang_dim', 0)),
            nhead=int(cfg['nhead']),
            head_layers=int(cfg['head_layers']),
            head_ff=int(cfg['head_ff']),
            flow_steps=self.flow_steps,
            pretrained_encoder=False,
        ).to(self.device)

        ema_state = ckpt.get('ema_state_dict') if self.use_ema and 'ema_state_dict' in ckpt else None
        if ema_state is not None:
            merged = dict(ckpt['model_state_dict'])
            merged.update(ema_state)
            self.model.load_state_dict(merged, strict=True)
        else:
            self.model.load_state_dict(ckpt['model_state_dict'], strict=True)
        self.model.eval()

        self._active_indices: np.ndarray | None = None
        self._prev_chunk: np.ndarray | None = None
        self._gen = torch.Generator(device=self.device)

    def reset(self, context):
        names = tuple(context.joint_names)
        if self.num_joints == len(names):
            if self.ckpt_joint_names and self.ckpt_joint_names != names:
                raise ValueError(
                    f'Checkpoint joint order {self.ckpt_joint_names} != plant joint order {names}'
                )
            self._active_indices = np.arange(len(names), dtype=int)
        elif self.num_joints == 17 and len(names) == 14:
            from python.common.minerva_constants import MINERVA_JOINTS
            if tuple(MINERVA_JOINTS[:14]) != names:
                raise ValueError(f'17-DoF checkpoint active prefix {MINERVA_JOINTS[:14]} != {names}')
            self._active_indices = np.arange(14, dtype=int)
        else:
            raise ValueError(
                f'Incompatible checkpoint num_joints={self.num_joints} for {len(names)}-axis plant'
            )
        self._prev_chunk = None
        self._gen.manual_seed(self.seed * 1000 + int(context.index))

    def start_pose(self, context):
        if 'ready_pose' in self.stats and self._active_indices is not None:
            ready = np.asarray(self.stats['ready_pose'], dtype=float)[self._active_indices]
            if np.all(np.isfinite(ready)):
                return np.clip(ready, context.lower, context.upper)
        return context.home.copy()

    def _prepare_image(self, cam: str, img: np.ndarray) -> np.ndarray:
        from PIL import Image
        arr = np.asarray(img, dtype=np.uint8)
        target_shape = self.image_shapes.get(cam)
        if target_shape is not None and arr.shape[:2] != target_shape[:2]:
            th, tw = target_shape[0], target_shape[1]
            h, w = arr.shape[:2]
            pil_img = Image.fromarray(arr)
            if h == th * 2 and w == tw * 2:
                pil_img = pil_img.reduce(2)
            else:
                pil_img = pil_img.resize((tw, th), getattr(Image, 'Resampling', Image).BOX)
            arr = np.asarray(pil_img, dtype=np.uint8)
        return self._imagenet_normalize(arr)

    def _expand_to_model_joints(self, vec14: np.ndarray) -> np.ndarray:
        vec14 = np.asarray(vec14, dtype=np.float32)
        if self.num_joints == len(vec14):
            return vec14
        out = np.zeros(self.num_joints, dtype=np.float32)
        out[self._active_indices] = vec14
        return out

    def __call__(self, observation):
        torch = self._torch
        q = self._expand_to_model_joints(observation['q'])
        qcmd = self._expand_to_model_joints(observation.get('qcmd', observation['q']))
        s = self.stats
        qpos_n = (q - s['qpos_mean']) / s['qpos_std']
        if self.state_mode == 'qpos':
            state = qpos_n
        elif self.state_mode == 'qpos_qcmd':
            qcmd_n = (qcmd - s['qcmd_mean']) / s['qcmd_std']
            state = np.concatenate([qpos_n, qcmd_n])
        elif self.state_mode == 'qpos_qcmd_tq':
            effort = observation.get('effort')
            if effort is None or not np.all(np.isfinite(effort)):
                raise ValueError("state_mode='qpos_qcmd_tq' requires finite effort feedback")
            tq = self._expand_to_model_joints(effort)
            qcmd_n = (qcmd - s['qcmd_mean']) / s['qcmd_std']
            tq_n = (tq - s['torque_mean']) / s['torque_std']
            state = np.concatenate([qpos_n, qcmd_n, tq_n])
        else:
            raise ValueError(f'Unsupported state_mode: {self.state_mode!r}')

        obs_images = observation.get('images') or {}
        images_t = {}
        for cam in self.camera_roles:
            if cam not in obs_images:
                raise ValueError(f'Policy requires camera {cam!r}, missing from observation')
            norm_img = self._prepare_image(cam, obs_images[cam])
            images_t[cam] = torch.from_numpy(norm_img).unsqueeze(0).to(self.device)

        state_t = torch.from_numpy(state.astype(np.float32)).unsqueeze(0).to(self.device)

        prefix_t = None
        if (self.use_rtc and self._prev_chunk is not None
                and 0 < self.execute_steps < self.chunk_size):
            overlap = self._prev_chunk[self.execute_steps:]
            if len(overlap) > 0:
                overlap_n = self._normalize_actions(overlap, s, self.action_mode, qpos=q)
                prefix_t = torch.from_numpy(overlap_n).unsqueeze(0).to(self.device)

        chunk_n = self.model.select_action(
            state_t, images_t, num_steps=self.flow_steps,
            prefix_actions=prefix_t, generator=self._gen,
        )[0].cpu().numpy()
        chunk_full = self._denormalize_actions(chunk_n, s, self.action_mode, qpos=q)
        self._prev_chunk = chunk_full.copy()

        if self.num_joints == 17:
            inactive = np.abs(chunk_full[:, 14:])
            if np.any(inactive > 1e-4):
                raise ValueError('17-DoF policy predicted non-zero motion on inactive head/lift axes')
        return chunk_full[:, self._active_indices].astype(float)

    def describe(self):
        return {
            'kind': 'MinervaPolicy flow-matching checkpoint',
            'checkpoint': str(self.checkpoint_path),
            'sha256': self.sha256,
            'num_joints': self.num_joints,
            'chunk_size': self.chunk_size,
            'execute_steps': self.execute_steps,
            'action_dt': self.action_dt,
            'state_mode': self.state_mode,
            'action_mode': self.action_mode,
            'flow_steps': self.flow_steps,
            'use_ema': self.use_ema,
            'use_rtc': self.use_rtc,
            'camera_roles': list(self.camera_roles),
            'image_shapes': {c: list(s) for c, s in self.image_shapes.items()},
        }


def _handoff_demonstrator():
    from .demonstrator import HandoffDemonstrator
    return HandoffDemonstrator()


REFERENCE = {'fixture_replay': FixtureReplayPolicy, 'hold_still': HoldStillPolicy, 'penetrating': PenetratingPolicy,
             'handoff_demonstrator': _handoff_demonstrator}


def load_policy(spec):
    """A reference policy by name, 'checkpoint:<path>', or 'package.module:factory'."""
    if spec in REFERENCE:
        return REFERENCE[spec]()
    if spec.startswith('checkpoint:'):
        return MinervaCheckpointPolicy(spec.partition(':')[2])
    module, sep, attribute = spec.partition(':')
    if not sep:
        raise ValueError(f"policy is one of {sorted(REFERENCE)}, 'checkpoint:<path>', or 'module:factory'")
    return getattr(importlib.import_module(module), attribute)()
