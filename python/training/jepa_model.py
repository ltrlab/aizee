"""
jepa_model.py — ACT-JEPA: ACT with a self-supervised world-model objective.

Builds on the existing ACTPolicy by adding three training-time-only modules:

    1. A *target encoder* (functionally the context encoder evaluated under
       no_grad — no EMA, no separate weights, following LeJEPA's
       heuristics-free recipe).
    2. A *JEPA predictor* — a small transformer that takes the current
       per-camera image tokens and predicts the same tokens at a future
       timestep, in latent space.
    3. A *SIGReg* regularizer that pushes context-token embeddings toward
       an isotropic Gaussian, preventing representation collapse without
       stop-gradient / EMA / centering heuristics.

At inference, only the ACT components are used (context encoder, decoder,
action head), so the runtime cost on the Jetson Orin is identical to
standard ACT.

Loss:
    total = action_l1 + lambda_obs * jepa_l1 + lambda_reg * sigreg
            + kl_weight * cvae_kl     (inherited from ACTPolicy)

References:
    - ACT-JEPA: Vujinović & Kovačević, "ACT-JEPA: Joint-Embedding Predictive
      Architecture Improves Imitation Learning" (2025).
    - LeJEPA: Balestriero & LeCun, "LeJEPA: Provable and Scalable
      Self-Supervised Learning Without the Heuristics" (2025).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

from python.training.act_model import (
    ACTDecoder,
    ACTPolicy,
    CVAEEncoder,
    ImageEncoder,
    PositionalEncoding,
    StateEncoder,
)
from python.training.sigreg import SIGReg


# ---------------------------------------------------------------------------
# Predictor
# ---------------------------------------------------------------------------

class JEPAPredictor(nn.Module):
    """Transformer predictor: current image tokens → future image tokens.

    Operates entirely in the projected `d_model` latent space (i.e., after
    img_proj in ACTPolicy). A learned future-time embedding is added to
    each token so the same architecture can predict at any horizon.

    Input/output shape: [B, N_tokens, d_model].
    """

    def __init__(
        self,
        d_model: int = 256,
        nhead: int = 8,
        num_layers: int = 4,
        dim_feedforward: int = 1024,
        dropout: float = 0.1,
        max_tokens: int = 512,
    ):
        super().__init__()
        self.d_model = d_model

        # Positional embedding (sinusoidal) — same scheme as the CVAE encoder
        self.pos_enc = PositionalEncoding(d_model, max_len=max_tokens, dropout=dropout)

        # Learned "future timestep" embedding broadcast across all tokens
        self.future_embed = nn.Parameter(torch.zeros(1, 1, d_model))
        nn.init.normal_(self.future_embed, std=0.02)

        layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            batch_first=True,
        )
        self.transformer = nn.TransformerEncoder(layer, num_layers=num_layers)

    def forward(self, ctx_tokens: torch.Tensor) -> torch.Tensor:
        x = self.pos_enc(ctx_tokens) + self.future_embed
        return self.transformer(x)


# ---------------------------------------------------------------------------
# Full ACT-JEPA policy
# ---------------------------------------------------------------------------

class ACTJEPAPolicy(ACTPolicy):
    """ACT + JEPA world-model objective.

    Subclasses ACTPolicy so the inference path (`select_action`, weight
    loading, parameter groups) is unchanged. Adds a JEPA predictor and a
    SIGReg regularizer used only at training time.

    Args (new vs. ACTPolicy):
        predictor_layers, predictor_heads, predictor_ff:
            Transformer hyperparams for the JEPA predictor.
        lambda_obs:
            Weight on the latent-space prediction loss.
        lambda_reg:
            Weight on the SIGReg regularization loss.
        sigreg_slices, sigreg_points:
            SIGReg hyperparams (see sigreg.py).
        target_no_grad:
            If True (default) the target encoder is the context encoder run
            under torch.no_grad() — no separate weights, no EMA, no
            stop-gradient hacks. Set False to let gradients flow through the
            target path as well (LeJEPA shows this is stable thanks to
            SIGReg).
    """

    def __init__(
        self,
        *,
        chunk_size: int = 100,
        d_model: int = 256,
        dim_feedforward: int = 2048,
        z_dim: int = 32,
        nhead: int = 8,
        num_encoder_layers: int = 4,
        num_decoder_layers: int = 7,
        dropout: float = 0.1,
        kl_weight: float = 10.0,
        pretrained_encoder: bool = True,
        num_joints: int = 7,
        state_dim: int = 14,
        # JEPA-specific
        predictor_layers: int = 4,
        predictor_heads: int = 8,
        predictor_ff: int = 1024,
        lambda_obs: float = 0.5,
        lambda_reg: float = 0.05,
        sigreg_slices: int = 1024,
        sigreg_points: int = 17,
        target_no_grad: bool = True,
    ):
        super().__init__(
            chunk_size=chunk_size,
            d_model=d_model,
            dim_feedforward=dim_feedforward,
            z_dim=z_dim,
            nhead=nhead,
            num_encoder_layers=num_encoder_layers,
            num_decoder_layers=num_decoder_layers,
            dropout=dropout,
            kl_weight=kl_weight,
            pretrained_encoder=pretrained_encoder,
            num_joints=num_joints,
            state_dim=state_dim,
        )
        self.lambda_obs = lambda_obs
        self.lambda_reg = lambda_reg
        self.target_no_grad = target_no_grad

        self.jepa_predictor = JEPAPredictor(
            d_model=d_model,
            nhead=predictor_heads,
            num_layers=predictor_layers,
            dim_feedforward=predictor_ff,
            dropout=dropout,
        )

        self.sigreg = SIGReg(
            num_slices=sigreg_slices,
            num_points=sigreg_points,
            resample_slices=True,
        )

    # ------------------------------------------------------------------
    # Parameter groups — predictor + sigreg follow the "non-backbone" LR
    # ------------------------------------------------------------------

    def jepa_parameters(self) -> List[nn.Parameter]:
        """Predictor parameters (training-time only)."""
        return list(self.jepa_predictor.parameters())

    # ------------------------------------------------------------------
    # Forward
    # ------------------------------------------------------------------

    def forward(
        self,
        qpos: torch.Tensor,
        state: torch.Tensor,
        images_left: torch.Tensor,
        images_right: torch.Tensor,
        actions: Optional[torch.Tensor] = None,
        future_images_left: Optional[torch.Tensor] = None,
        future_images_right: Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:
        """Training forward pass.

        Computes the full ACT loss (L1 + KL) plus the JEPA observation loss
        and SIGReg regularizer when future images are supplied.

        When future_images_* are None, falls back to the plain ACT loss
        with zero observation/sigreg losses (useful for warm-up / ablation).
        """
        assert actions is not None, "actions required for training forward pass"

        # === Shared image encoding (context) ===
        ctx_tokens = self._encode_images(images_left, images_right)  # [B, N, d_model]

        # === ACT action path (existing) ===
        mu, log_var = self.cvae_encoder(actions, qpos)
        z = self._reparameterize(mu, log_var)

        state_token = self.state_encoder(state).unsqueeze(1)
        z_token = self.z_proj(z).unsqueeze(1)
        memory = torch.cat([ctx_tokens, state_token, z_token], dim=1)
        decoded = self.decoder(memory)
        pred_actions = self.action_head(decoded)

        l1_loss = F.l1_loss(pred_actions, actions)
        kl_loss = -0.5 * torch.mean(1 + log_var - mu.pow(2) - log_var.exp())

        # === JEPA world-model path (training-only) ===
        if future_images_left is not None and future_images_right is not None:
            target_ctx = torch.no_grad() if self.target_no_grad else _NullCtx()
            with target_ctx:
                future_tokens = self._encode_images(
                    future_images_left, future_images_right
                )  # [B, N, d_model]

            pred_future = self.jepa_predictor(ctx_tokens)
            # Smooth-L1 in latent space; less sensitive to outlier tokens than L1
            obs_loss = F.smooth_l1_loss(pred_future, future_tokens)

            # SIGReg on flattened context tokens
            reg_loss = self.sigreg(ctx_tokens.reshape(-1, self.d_model))
        else:
            obs_loss = ctx_tokens.sum() * 0.0
            reg_loss = ctx_tokens.sum() * 0.0

        total = (
            l1_loss
            + self.kl_weight * kl_loss
            + self.lambda_obs * obs_loss
            + self.lambda_reg * reg_loss
        )

        return {
            "l1": l1_loss,
            "kl": kl_loss,
            "obs": obs_loss,
            "reg": reg_loss,
            "total": total,
        }


class _NullCtx:
    """Trivial context manager used when target_no_grad is disabled."""
    def __enter__(self): return self
    def __exit__(self, *a): return False


__all__ = ["ACTJEPAPolicy", "JEPAPredictor"]
