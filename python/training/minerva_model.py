"""
minerva_model.py — MinervaPolicy: bimanual, 3-camera, language-conditioned
flow-matching policy with a training-only JEPA/SIGReg representation auxiliary.

Architecture (see the Minerva design brief):

  VISION (inference path):
    - ResNet18 (ImageNet, FrozenBatchNorm) encoders. ONE shared backbone for
      the two structurally-identical wrist cameras, a SEPARATE backbone for the
      head RealSense. Each camera's feature map is flattened to tokens, given a
      2D sin-cos spatial position embedding + a learned CAMERA-IDENTITY embedding
      (mandatory once the wrist backbone is shared), and all camera tokens are
      concatenated into one "memory" sequence the action head cross-attends over.
    - training-time CAMERA DROPOUT (independently mask a whole stream) kills the
      documented "lean on the wide head view, ignore the wrists" shortcut.

  LANGUAGE (inference path):
    - a frozen, pre-cached pooled task embedding (see language.py) enters BOTH
      as a prepended memory token (Octo task-token style) AND, concatenated with
      the proprioceptive state, as the AdaLN conditioning vector of the head.

  ACTION HEAD (inference path):
    - FlowMatchingActionHead: generates the [B, K, 17] action chunk by
      integrating a learned velocity field (~8 Euler steps). Replaces ACT's
      CVAE + DETR decoder + L1 head.

  JEPA AUXILIARY (training only — stripped at inference):
    - a per-camera, ACTION-CONDITIONED predictor forecasts future image tokens
      in latent space (target encoder = the same encoder under no_grad; LeJEPA
      no-EMA recipe), and SIGReg pushes a SEPARATE projection of the context
      tokens toward an isotropic Gaussian. The separate projection head keeps the
      isotropy pressure off the features the flow head conditions on.

Loss:  total = flow_mse + lambda_obs * jepa_smooth_l1 + lambda_reg * sigreg

Inference cost equals: 3 ResNet18 forwards + K-token transformer * num_steps.
The JEPA predictor / SIGReg / target-encoder pass never run at inference.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import torch
import torch.nn as nn
import torch.nn.functional as F

from python.training.act_model import ImageEncoder, StateEncoder
from python.training.flow_matching_head import FlowMatchingActionHead
from python.training.sigreg import SIGReg

# Canonical camera order → identity-embedding index.
_CAMERAS: List[str] = ["left_wrist", "right_wrist", "head"]
_WRIST_CAMERAS = ("left_wrist", "right_wrist")


# ---------------------------------------------------------------------------
# 2D sin-cos spatial position embedding
# ---------------------------------------------------------------------------

def build_2d_sincos_pos_embed(h: int, w: int, dim: int, device, dtype) -> torch.Tensor:
    """Return a [h*w, dim] 2D sin-cos position embedding (half for rows, half cols)."""
    assert dim % 4 == 0, "d_model must be divisible by 4 for 2D sin-cos pos embed"
    quarter = dim // 4
    omega = torch.arange(quarter, device=device, dtype=torch.float32) / quarter
    omega = 1.0 / (10000.0 ** omega)                       # [quarter]
    y = torch.arange(h, device=device, dtype=torch.float32)
    x = torch.arange(w, device=device, dtype=torch.float32)
    yy, xx = torch.meshgrid(y, x, indexing="ij")           # [h, w]
    yy = yy.reshape(-1)[:, None] * omega[None, :]          # [h*w, quarter]
    xx = xx.reshape(-1)[:, None] * omega[None, :]
    pe = torch.cat([yy.sin(), yy.cos(), xx.sin(), xx.cos()], dim=1)  # [h*w, dim]
    return pe.to(dtype)


# ---------------------------------------------------------------------------
# Action-conditioned JEPA predictor (training only)
# ---------------------------------------------------------------------------

class ActionConditionedJEPAPredictor(nn.Module):
    """Predict future image tokens from current tokens, conditioned on the action
    chunk. Action-conditioning (V-JEPA 2-AC style) forces the encoder to retain
    controllable-dynamics information — exactly what a manipulation policy needs."""

    def __init__(self, d_model: int, action_dim: int, chunk_size: int,
                 nhead: int = 8, num_layers: int = 3, dim_feedforward: int = 1024,
                 dropout: float = 0.1):
        super().__init__()
        self.future_embed = nn.Parameter(torch.zeros(1, 1, d_model))
        nn.init.normal_(self.future_embed, std=0.02)
        # Pool the (normalized) action chunk to one conditioning vector.
        self.action_pool = nn.Sequential(
            nn.Linear(action_dim * chunk_size, d_model), nn.GELU(),
            nn.Linear(d_model, d_model),
        )
        layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead, dim_feedforward=dim_feedforward,
            dropout=dropout, batch_first=True,
        )
        self.transformer = nn.TransformerEncoder(layer, num_layers=num_layers)

    def forward(self, ctx_tokens: torch.Tensor, actions: torch.Tensor) -> torch.Tensor:
        """ctx_tokens: [B, N, d]  actions: [B, K, A] -> predicted future [B, N, d]."""
        B = ctx_tokens.size(0)
        a = self.action_pool(actions.reshape(B, -1)).unsqueeze(1)   # [B, 1, d]
        x = ctx_tokens + self.future_embed + a                      # broadcast cond
        return self.transformer(x)


# ---------------------------------------------------------------------------
# MinervaPolicy
# ---------------------------------------------------------------------------

class MinervaPolicy(nn.Module):
    def __init__(
        self,
        *,
        num_joints: int = 17,
        chunk_size: int = 32,
        d_model: int = 512,
        state_dim: int = 34,            # e.g. qpos+qcmd = 2*17
        lang_dim: int = 0,              # 0 disables language conditioning
        nhead: int = 8,
        head_layers: int = 6,
        head_ff: int = 2048,
        dropout: float = 0.1,
        pretrained_encoder: bool = True,
        camera_dropout: float = 0.15,
        flow_steps: int = 8,
        # JEPA auxiliary
        lambda_obs: float = 0.3,
        lambda_reg: float = 0.05,
        predictor_layers: int = 3,
        sigreg_slices: int = 1024,
        sigreg_points: int = 17,
    ):
        super().__init__()
        assert d_model % 4 == 0
        self.num_joints = num_joints
        self.chunk_size = chunk_size
        self.d_model = d_model
        self.state_dim = state_dim
        self.lang_dim = lang_dim
        self.camera_dropout = camera_dropout
        self.flow_steps = flow_steps
        self.lambda_obs = lambda_obs
        self.lambda_reg = lambda_reg

        # --- vision: shared wrist encoder + separate head encoder ---
        self.wrist_encoder = ImageEncoder(pretrained=pretrained_encoder)
        self.head_encoder = ImageEncoder(pretrained=pretrained_encoder)
        self.img_proj = nn.Linear(self.wrist_encoder.out_channels, d_model)  # 512 -> d
        self.camera_id_embed = nn.Embedding(len(_CAMERAS), d_model)

        # --- language ---
        self.lang_mem_proj = nn.Linear(lang_dim, d_model) if lang_dim > 0 else None

        # --- action head (conditioned on state + language) ---
        cond_dim = state_dim + lang_dim
        self.flow_head = FlowMatchingActionHead(
            action_dim=num_joints, chunk_size=chunk_size, d_model=d_model,
            cond_dim=cond_dim, nhead=nhead, num_layers=head_layers,
            dim_feedforward=head_ff, dropout=dropout,
        )

        # --- JEPA auxiliary (training only) ---
        self.jepa_predictor = ActionConditionedJEPAPredictor(
            d_model=d_model, action_dim=num_joints, chunk_size=chunk_size,
            nhead=nhead, num_layers=predictor_layers,
        )
        self.sigreg_proj = nn.Linear(d_model, d_model)  # separate projection head
        self.sigreg = SIGReg(num_slices=sigreg_slices, num_points=sigreg_points,
                             resample_slices=True)

    # ------------------------------------------------------------------
    # Encoding
    # ------------------------------------------------------------------
    def _encode_one(self, img: torch.Tensor, cam_name: str) -> torch.Tensor:
        """img: [B,3,H,W] -> tokens [B, N, d] with 2D spatial PE + camera-id."""
        encoder = self.wrist_encoder if cam_name in _WRIST_CAMERAS else self.head_encoder
        feat = encoder(img)                                # [B, 512, Hp, Wp]
        B, C, Hp, Wp = feat.shape
        tokens = feat.permute(0, 2, 3, 1).reshape(B, Hp * Wp, C)
        tokens = self.img_proj(tokens)                     # [B, N, d]
        pe = build_2d_sincos_pos_embed(Hp, Wp, self.d_model, tokens.device, tokens.dtype)
        cam_idx = torch.tensor(_CAMERAS.index(cam_name), device=tokens.device)
        cam_vec = self.camera_id_embed(cam_idx)            # [d]
        return tokens + pe.unsqueeze(0) + cam_vec.view(1, 1, -1)

    def _build_memory(
        self,
        images: Dict[str, torch.Tensor],
        language: Optional[torch.Tensor],
        *,
        apply_dropout: bool,
    ):
        """Assemble the cross-attention memory + key-padding mask.

        Returns (memory [B,S,d], key_padding_mask [B,S] with True=ignore,
                 per_cam_tokens dict{cam: [B,N,d]}).
        """
        per_cam = {cam: self._encode_one(images[cam], cam) for cam in _CAMERAS if cam in images}
        cams = list(per_cam.keys())
        B = per_cam[cams[0]].size(0)
        device = per_cam[cams[0]].device

        seq: List[torch.Tensor] = []
        mask: List[torch.Tensor] = []

        # Language task token (never masked → always ≥1 valid key).
        if self.lang_mem_proj is not None and language is not None:
            seq.append(self.lang_mem_proj(language).unsqueeze(1))     # [B,1,d]
            mask.append(torch.zeros(B, 1, dtype=torch.bool, device=device))

        # Per-camera dropout flags: [B, num_cams] True = drop this stream.
        if apply_dropout and self.camera_dropout > 0:
            drop = torch.rand(B, len(cams), device=device) < self.camera_dropout
            # Never drop every camera for a sample — force-keep one if all dropped.
            all_dropped = drop.all(dim=1)
            if all_dropped.any():
                keep = torch.randint(0, len(cams), (int(all_dropped.sum()),), device=device)
                drop[all_dropped, keep] = False
        else:
            drop = torch.zeros(B, len(cams), dtype=torch.bool, device=device)

        for ci, cam in enumerate(cams):
            toks = per_cam[cam]                                       # [B,N,d]
            seq.append(toks)
            n = toks.size(1)
            mask.append(drop[:, ci:ci + 1].expand(B, n))             # [B,N]

        memory = torch.cat(seq, dim=1)                               # [B,S,d]
        key_padding_mask = torch.cat(mask, dim=1)                    # [B,S]
        return memory, key_padding_mask, per_cam

    def _ensure_language(self, language, state):
        """A lang_dim>0 model must ALWAYS receive a lang_dim-wide vector; when
        none is supplied, condition on zeros (matches training on empty task
        strings) so the cond vector and the memory language-token stay consistent
        with the cond_proj / lang_mem_proj widths instead of crashing."""
        if self.lang_dim > 0 and language is None:
            return state.new_zeros(state.size(0), self.lang_dim)
        return language

    def _cond(self, state: torch.Tensor, language: Optional[torch.Tensor]) -> torch.Tensor:
        if self.lang_dim > 0:
            return torch.cat([state, language], dim=-1)
        return state

    # ------------------------------------------------------------------
    # Training forward
    # ------------------------------------------------------------------
    def forward(
        self,
        state: torch.Tensor,                       # [B, state_dim] normalized
        images: Dict[str, torch.Tensor],           # {cam: [B,3,H,W]}
        actions: torch.Tensor,                     # [B, K, num_joints] normalized
        language: Optional[torch.Tensor] = None,   # [B, lang_dim] or None
        future_images: Optional[Dict[str, torch.Tensor]] = None,
    ) -> Dict[str, torch.Tensor]:
        language = self._ensure_language(language, state)
        memory, kpm, per_cam = self._build_memory(images, language, apply_dropout=True)
        cond = self._cond(state, language)

        flow_loss = self.flow_head.flow_loss(actions, cond, memory, kpm)

        # JEPA world-model auxiliary (only when future frames supplied).
        if future_images is not None:
            obs_losses, reg_losses = [], []
            for cam in per_cam:
                if cam not in future_images:
                    continue
                with torch.no_grad():
                    fut = self._encode_one(future_images[cam], cam)  # target, no grad
                pred = self.jepa_predictor(per_cam[cam], actions)
                obs_losses.append(F.smooth_l1_loss(pred, fut))
                reg_losses.append(self.sigreg(
                    self.sigreg_proj(per_cam[cam]).reshape(-1, self.d_model)
                ))
            obs_loss = torch.stack(obs_losses).mean() if obs_losses else flow_loss * 0.0
            reg_loss = torch.stack(reg_losses).mean() if reg_losses else flow_loss * 0.0
        else:
            obs_loss = flow_loss * 0.0
            reg_loss = flow_loss * 0.0

        total = flow_loss + self.lambda_obs * obs_loss + self.lambda_reg * reg_loss
        return {"flow": flow_loss, "obs": obs_loss, "reg": reg_loss, "total": total}

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------
    @torch.no_grad()
    def select_action(
        self,
        state: torch.Tensor,
        images: Dict[str, torch.Tensor],
        language: Optional[torch.Tensor] = None,
        *,
        num_steps: Optional[int] = None,
    ) -> torch.Tensor:
        """Return a normalized action chunk [B, K, num_joints]. No dropout, no JEPA."""
        language = self._ensure_language(language, state)
        memory, kpm, _ = self._build_memory(images, language, apply_dropout=False)
        cond = self._cond(state, language)
        return self.flow_head.sample(
            cond, memory, num_steps=num_steps or self.flow_steps,
            memory_key_padding_mask=kpm,
        )

    # ------------------------------------------------------------------
    # Optimizer param groups (backbone gets a lower LR, like ACT)
    # ------------------------------------------------------------------
    def backbone_parameters(self) -> List[nn.Parameter]:
        return list(self.wrist_encoder.parameters()) + list(self.head_encoder.parameters())

    def non_backbone_parameters(self) -> List[nn.Parameter]:
        bb = {id(p) for p in self.backbone_parameters()}
        return [p for p in self.parameters() if id(p) not in bb]


__all__ = ["MinervaPolicy", "ActionConditionedJEPAPredictor", "build_2d_sincos_pos_embed"]
