"""
flow_matching_head.py — conditional flow-matching action head for Minerva.

Replaces ACT's CVAE + DETR-decoder + L1 action path with a generative
flow-matching head that models the (multimodal) distribution over action
chunks. It keeps ACT's decoder *shape* — learned-length action-chunk tokens
that cross-attend to a multi-camera "memory" — but instead of regressing the
action directly it regresses a velocity field and integrates it at inference.

Why flow matching (not epsilon-DDPM):
  - velocity regression is a plain MSE objective (stable, no noise schedule to
    tune), and samples in very few Euler steps (GR00T N1.5 ships 4; pi0 uses
    flow matching) — the deciding factor for a ~20 Hz Jetson Orin loop.
  - composes cleanly with the JEPA/SIGReg auxiliary (no CVAE latent to fight).

Conditioning (RDT/DiT topology):
  - the noisy action-chunk tokens are the MAIN sequence (self-attention lets
    the two arms + head + lift coordinate across the chunk),
  - a pooled conditioning vector (flow-time t  +  proprio state  +  pooled
    language) drives AdaLN-Zero modulation of every block,
  - the multi-camera token grid (+ optional prepended language token) enters
    only via cross-attention.

Rectified/OT linear path:
    x_t = (1 - t) * noise + t * x1,     target velocity  v* = x1 - noise
    train:  minimise  || v_theta(x_t, t, cond, memory) - v* ||^2
    sample: x_0 ~ N(0, I);  x_{k+1} = x_k + (1/N) * v_theta(x_k, t_k, ...)

Everything here is inference-relevant (unlike the JEPA aux) — it is the head
that runs on the robot.
"""

from __future__ import annotations

import math
from typing import Optional

import torch
import torch.nn as nn


# ---------------------------------------------------------------------------
# Timestep (flow-time) embedding
# ---------------------------------------------------------------------------

class FlowTimeEmbedding(nn.Module):
    """Sinusoidal embedding of the continuous flow-time t in [0, 1] -> d_model."""

    def __init__(self, d_model: int, max_period: float = 10000.0):
        super().__init__()
        self.d_model = d_model
        self.max_period = max_period
        self.mlp = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.SiLU(),
            nn.Linear(d_model, d_model),
        )

    def forward(self, t: torch.Tensor) -> torch.Tensor:
        """t: [B] in [0,1] -> [B, d_model]."""
        half = self.d_model // 2
        freqs = torch.exp(
            -math.log(self.max_period)
            * torch.arange(half, device=t.device, dtype=torch.float32) / half
        )
        args = t.float().unsqueeze(1) * freqs.unsqueeze(0)  # [B, half]
        emb = torch.cat([torch.cos(args), torch.sin(args)], dim=1)
        if self.d_model % 2:  # pad odd d_model
            emb = torch.cat([emb, torch.zeros_like(emb[:, :1])], dim=1)
        return self.mlp(emb)


def _modulate(x: torch.Tensor, shift: torch.Tensor, scale: torch.Tensor) -> torch.Tensor:
    """AdaLN modulation: x * (1 + scale) + shift.  x:[B,T,d]  shift/scale:[B,d]."""
    return x * (1 + scale.unsqueeze(1)) + shift.unsqueeze(1)


# ---------------------------------------------------------------------------
# DiT block: AdaLN-Zero self-attn + gated cross-attn + AdaLN-Zero MLP
# ---------------------------------------------------------------------------

class AdaLNDiTBlock(nn.Module):
    """One denoiser block.

    - self-attention over the action-chunk tokens (AdaLN-Zero modulated),
    - cross-attention into the camera/language memory (gated),
    - feed-forward MLP (AdaLN-Zero modulated).

    All residual gates are zero-initialised (AdaLN-Zero) so the block starts as
    the identity and training is stable from step 0.
    """

    def __init__(self, d_model: int, nhead: int, dim_feedforward: int, dropout: float = 0.0):
        super().__init__()
        self.norm1 = nn.LayerNorm(d_model, elementwise_affine=False)
        self.self_attn = nn.MultiheadAttention(d_model, nhead, dropout=dropout, batch_first=True)

        self.norm2 = nn.LayerNorm(d_model, elementwise_affine=False)
        self.cross_attn = nn.MultiheadAttention(d_model, nhead, dropout=dropout, batch_first=True)

        self.norm3 = nn.LayerNorm(d_model, elementwise_affine=False)
        self.mlp = nn.Sequential(
            nn.Linear(d_model, dim_feedforward),
            nn.GELU(),
            nn.Linear(dim_feedforward, d_model),
        )

        # AdaLN-Zero: produce 7 modulation vectors from the conditioning vector.
        #   self-attn: shift, scale, gate ; cross-attn: gate ; mlp: shift, scale, gate
        self.ada = nn.Sequential(nn.SiLU(), nn.Linear(d_model, 7 * d_model))
        nn.init.zeros_(self.ada[-1].weight)
        nn.init.zeros_(self.ada[-1].bias)

    def forward(
        self,
        x: torch.Tensor,               # [B, K, d]
        cond: torch.Tensor,            # [B, d]
        memory: torch.Tensor,          # [B, S, d]
        memory_key_padding_mask: Optional[torch.Tensor] = None,  # [B, S] True=pad
    ) -> torch.Tensor:
        (shift_sa, scale_sa, gate_sa,
         gate_ca,
         shift_mlp, scale_mlp, gate_mlp) = self.ada(cond).chunk(7, dim=-1)

        h = _modulate(self.norm1(x), shift_sa, scale_sa)
        sa, _ = self.self_attn(h, h, h, need_weights=False)
        x = x + gate_sa.unsqueeze(1) * sa

        h = self.norm2(x)
        ca, _ = self.cross_attn(
            h, memory, memory,
            key_padding_mask=memory_key_padding_mask, need_weights=False,
        )
        x = x + gate_ca.unsqueeze(1) * ca

        h = _modulate(self.norm3(x), shift_mlp, scale_mlp)
        x = x + gate_mlp.unsqueeze(1) * self.mlp(h)
        return x


# ---------------------------------------------------------------------------
# Full flow-matching action head
# ---------------------------------------------------------------------------

class FlowMatchingActionHead(nn.Module):
    """Conditional flow-matching head predicting an action chunk [B, K, A].

    Args:
        action_dim: A (=17 for Minerva).
        chunk_size: K, number of future actions per chunk.
        d_model: transformer width (memory must already be projected to d_model).
        cond_dim: dimensionality of the external conditioning vector supplied by
            the policy (e.g. normalized state + pooled language). Projected to
            d_model and summed with the flow-time embedding.
        nhead, num_layers, dim_feedforward, dropout: transformer hyperparams.
    """

    def __init__(
        self,
        action_dim: int,
        chunk_size: int,
        d_model: int = 512,
        cond_dim: int = 0,
        nhead: int = 8,
        num_layers: int = 6,
        dim_feedforward: int = 2048,
        dropout: float = 0.0,
    ):
        super().__init__()
        self.action_dim = action_dim
        self.chunk_size = chunk_size
        self.d_model = d_model

        self.action_in = nn.Linear(action_dim, d_model)
        self.pos_embed = nn.Parameter(torch.zeros(1, chunk_size, d_model))
        nn.init.normal_(self.pos_embed, std=0.02)

        self.time_embed = FlowTimeEmbedding(d_model)
        self.cond_proj = nn.Linear(cond_dim, d_model) if cond_dim > 0 else None

        self.blocks = nn.ModuleList([
            AdaLNDiTBlock(d_model, nhead, dim_feedforward, dropout)
            for _ in range(num_layers)
        ])

        # Final AdaLN-Zero + zero-init output projection (velocity starts at 0).
        self.norm_out = nn.LayerNorm(d_model, elementwise_affine=False)
        self.ada_out = nn.Sequential(nn.SiLU(), nn.Linear(d_model, 2 * d_model))
        nn.init.zeros_(self.ada_out[-1].weight)
        nn.init.zeros_(self.ada_out[-1].bias)
        self.action_out = nn.Linear(d_model, action_dim)
        nn.init.zeros_(self.action_out.weight)
        nn.init.zeros_(self.action_out.bias)

    # -- conditioning ------------------------------------------------------
    def _cond_vector(self, t: torch.Tensor, cond: Optional[torch.Tensor]) -> torch.Tensor:
        c = self.time_embed(t)                       # [B, d]
        if self.cond_proj is not None and cond is not None:
            c = c + self.cond_proj(cond)
        return c

    # -- velocity field ----------------------------------------------------
    def forward(
        self,
        noisy_actions: torch.Tensor,                 # [B, K, A]
        t: torch.Tensor,                             # [B] in [0,1]
        cond: Optional[torch.Tensor],                # [B, cond_dim] or None
        memory: torch.Tensor,                        # [B, S, d_model]
        memory_key_padding_mask: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """Predict the flow velocity v_theta(x_t, t, cond, memory) -> [B, K, A]."""
        c = self._cond_vector(t, cond)
        x = self.action_in(noisy_actions) + self.pos_embed
        for blk in self.blocks:
            x = blk(x, c, memory, memory_key_padding_mask)
        shift, scale = self.ada_out(c).chunk(2, dim=-1)
        x = _modulate(self.norm_out(x), shift, scale)
        return self.action_out(x)

    # -- training loss -----------------------------------------------------
    def flow_loss(
        self,
        clean_actions: torch.Tensor,                 # [B, K, A] (normalized)
        cond: Optional[torch.Tensor],
        memory: torch.Tensor,
        memory_key_padding_mask: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """Rectified-flow / OT-path MSE loss on the velocity field."""
        B = clean_actions.size(0)
        noise = torch.randn_like(clean_actions)
        t = torch.rand(B, device=clean_actions.device)
        t_ = t.view(B, 1, 1)
        x_t = (1.0 - t_) * noise + t_ * clean_actions
        target_v = clean_actions - noise
        pred_v = self.forward(x_t, t, cond, memory, memory_key_padding_mask)
        return torch.mean((pred_v - target_v) ** 2)

    # -- sampling ----------------------------------------------------------
    @torch.no_grad()
    def sample(
        self,
        cond: Optional[torch.Tensor],
        memory: torch.Tensor,
        *,
        num_steps: int = 8,
        memory_key_padding_mask: Optional[torch.Tensor] = None,
        generator: Optional[torch.Generator] = None,
    ) -> torch.Tensor:
        """Integrate the velocity field with forward Euler -> action chunk [B, K, A]."""
        B = memory.size(0)
        device = memory.device
        x = torch.randn(
            B, self.chunk_size, self.action_dim, device=device, generator=generator,
        )
        dt = 1.0 / num_steps
        for k in range(num_steps):
            t = torch.full((B,), k * dt, device=device)
            v = self.forward(x, t, cond, memory, memory_key_padding_mask)
            x = x + dt * v
        return x


__all__ = ["FlowMatchingActionHead", "AdaLNDiTBlock", "FlowTimeEmbedding"]
