"""inference.py — shared obs-build / normalize / denorm / forward pipeline.

Extracted from act_policy_node.py so the live inference node, the offline
episode visualizer, and any future evaluation script all use the same code
path. Keeping a single implementation guarantees the visualizer's predicted
chunk is byte-equivalent to what the live policy would emit for the same obs.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from python.training.act_model import ACTPolicy


_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# State mode → per-joint feature block count.
_STATE_MODE_K = {"qpos": 1, "qpos_qcmd": 2, "qpos_qcmd_tq": 3}


def normalize_image(img: np.ndarray) -> np.ndarray:
    """uint8 [H,W,3] → float32 [3,H,W] ImageNet normalized."""
    x = img.astype(np.float32) / 255.0
    x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
    return x.transpose(2, 0, 1)


def normalize_qpos(qpos, stats):    return (qpos - stats["qpos_mean"])   / stats["qpos_std"]
def normalize_qcmd(qcmd, stats):    return (qcmd - stats["qcmd_mean"])   / stats["qcmd_std"]
def normalize_torques(tq,  stats):  return (tq   - stats["torque_mean"]) / stats["torque_std"]


def denormalize_actions(
    actions: np.ndarray, stats: dict, action_mode: str, qpos: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Inverse of the training normalization → absolute joint positions.

    actions shape: [chunk_size, J] or [J]
    """
    if action_mode == "absolute":
        return actions * stats["action_std"] + stats["action_mean"]
    if qpos is None:
        raise ValueError("qpos anchor required for relative action mode")
    deltas = actions * stats["rel_action_std"] + stats["rel_action_mean"]
    if deltas.ndim == 2 and qpos.ndim == 1:
        return deltas + qpos[None, :]
    return deltas + qpos


def build_state_vector(
    qpos_norm: np.ndarray,
    state_mode: str,
    qcmd_raw: Optional[np.ndarray],
    qpos_raw: np.ndarray,
    torques_raw: Optional[np.ndarray],
    stats: dict,
    num_joints: int,
) -> np.ndarray:
    """Build the extended state vector for inference.

    `qcmd_raw` is the unnormalized command source: at inference time this is
    typically the previous predicted action (closed-loop bootstrap), but
    callers may pass `qpos_raw` for a recorded-qcmd source.
    """
    if state_mode == "qpos":
        return qpos_norm.copy()

    src = qcmd_raw if qcmd_raw is not None else qpos_raw
    qcmd_norm = normalize_qcmd(src, stats)

    if state_mode == "qpos_qcmd":
        return np.concatenate([qpos_norm, qcmd_norm])

    tq_raw = torques_raw if torques_raw is not None else np.zeros(num_joints, dtype=np.float32)
    tq_norm = normalize_torques(tq_raw, stats)
    return np.concatenate([qpos_norm, qcmd_norm, tq_norm])


def load_checkpoint(
    path: str, device: torch.device, *, strict: bool = False,
) -> Tuple[ACTPolicy, dict, dict]:
    """Load a checkpoint into a plain ACTPolicy.

    `strict=False` by default so JEPA-trained checkpoints (which have extra
    `jepa_predictor.*` keys that don't exist on ACTPolicy) load cleanly — those
    modules are training-only and are not needed for action prediction.

    Returns (policy, dataset_stats, config). Config is annotated with the
    derived num_joints / state_mode / state_dim / action_mode fields.
    """
    ckpt = torch.load(path, map_location=device, weights_only=False)
    config = ckpt["config"]
    dataset_stats = ckpt["dataset_stats"]

    for k in dataset_stats:
        v = dataset_stats[k]
        if hasattr(v, "numpy"):
            dataset_stats[k] = v.cpu().numpy().astype(np.float32)
        elif hasattr(v, "astype"):
            dataset_stats[k] = v.astype(np.float32)

    num_joints  = config.get("num_joints", 7)
    state_mode  = config.get("state_mode", "qpos_qcmd")
    state_dim   = config.get("state_dim", num_joints * _STATE_MODE_K.get(state_mode, 1))
    action_mode = config.get("action_mode", "absolute")

    policy = ACTPolicy(
        chunk_size=config["chunk_size"],
        d_model=config["d_model"],
        dim_feedforward=config.get("dim_feedforward", 2048),
        z_dim=config["z_dim"],
        nhead=config["nhead"],
        num_encoder_layers=config["num_encoder_layers"],
        num_decoder_layers=config["num_decoder_layers"],
        kl_weight=config.get("kl_weight", 10.0),
        pretrained_encoder=False,
        num_joints=num_joints,
        state_dim=state_dim,
    ).to(device)

    policy.load_state_dict(ckpt["model_state_dict"], strict=strict)
    policy.eval()

    config["num_joints"]  = num_joints
    config["state_mode"]  = state_mode
    config["state_dim"]   = state_dim
    config["action_mode"] = action_mode

    return policy, dataset_stats, config


@torch.no_grad()
def predict_chunk(
    policy: ACTPolicy,
    qpos_raw: np.ndarray,
    gripper_img: np.ndarray,
    *,
    dataset_stats: dict,
    config: dict,
    qcmd_raw: Optional[np.ndarray] = None,
    torques_raw: Optional[np.ndarray] = None,
    device: Optional[torch.device] = None,
) -> np.ndarray:
    """Run one forward pass and return an absolute-position action chunk.

    Args:
        qpos_raw:     [J] float32, unnormalized joint positions.
        gripper_img:  uint8 [H, W, 3], the gripper camera frame.
        qcmd_raw:     [J] unnormalized — the qcmd source. For closed-loop live
                      inference pass the previous predicted action; for
                      recorded-qcmd offline replay pass the recorded qcmd[t].
                      None falls back to qpos_raw (safe first-step bootstrap).
        torques_raw:  [J], required only when state_mode == 'qpos_qcmd_tq'.

    Returns:
        action_chunk: [chunk_size, J] float32 in absolute-position coordinates.
    """
    if device is None:
        device = next(policy.parameters()).device

    qpos_norm = normalize_qpos(qpos_raw, dataset_stats)
    state_vec = build_state_vector(
        qpos_norm,
        config["state_mode"],
        qcmd_raw,
        qpos_raw,
        torques_raw,
        dataset_stats,
        config["num_joints"],
    )
    img_norm = normalize_image(gripper_img)

    qpos_t  = torch.from_numpy(qpos_norm).unsqueeze(0).to(device)
    state_t = torch.from_numpy(state_vec).unsqueeze(0).to(device)
    img_t   = torch.from_numpy(img_norm).unsqueeze(0).to(device)

    chunk = policy.select_action(qpos_t, state_t, img_t)   # [1, chunk_size, J]
    pred_norm = chunk[0].detach().cpu().numpy()
    return denormalize_actions(pred_norm, dataset_stats, config["action_mode"], qpos=qpos_raw)
