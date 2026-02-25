"""
act_model.py — Self-contained ACT (Action Chunking with Transformers) implementation.

Based on the original ACT paper (Zhao et al., 2023).
No dependency on lerobot or external ACT libraries.

Architecture:
  - Image encoder: ResNet18 per camera → feature maps
  - State encoder: 2-layer MLP (6 → 512) for qpos
  - CVAE encoder (training only): transformer encodes full action sequence + qpos
    → latent z (dim=32)
  - Transformer decoder: DETR-style with learned chunk_size queries
  - Action head: linear 512 → 6 per query

Forward signatures:
    # Training
    loss_dict = policy(qpos, images_left, images_right, actions)
    # → {"l1": tensor, "kl": tensor, "total": tensor}

    # Inference
    action_chunk = policy.select_action(qpos, images_left, images_right)
    # → [B, chunk_size, 6]
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights


# ---------------------------------------------------------------------------
# Positional encoding
# ---------------------------------------------------------------------------

class PositionalEncoding(nn.Module):
    """Standard sinusoidal positional encoding for sequences."""

    def __init__(self, d_model: int, max_len: int = 512, dropout: float = 0.1):
        super().__init__()
        self.dropout = nn.Dropout(dropout)
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(max_len).unsqueeze(1).float()
        div = torch.exp(
            torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model)
        )
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        self.register_buffer("pe", pe.unsqueeze(0))  # [1, max_len, d_model]

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: [B, T, d_model]"""
        x = x + self.pe[:, : x.size(1)]
        return self.dropout(x)


# ---------------------------------------------------------------------------
# Image encoder
# ---------------------------------------------------------------------------

class ImageEncoder(nn.Module):
    """ResNet18 backbone — strips avgpool and fc, returns feature map.

    Output shape: [B, 512, H', W'] where H' = ceil(H/32), W' = ceil(W/32).
    For 240x320 input: [B, 512, 8, 10] → 80 spatial tokens.
    """

    def __init__(self, pretrained: bool = True):
        super().__init__()
        weights = ResNet18_Weights.DEFAULT if pretrained else None
        backbone = resnet18(weights=weights)
        # Keep everything up to (and including) layer4
        self.backbone = nn.Sequential(
            backbone.conv1,
            backbone.bn1,
            backbone.relu,
            backbone.maxpool,
            backbone.layer1,
            backbone.layer2,
            backbone.layer3,
            backbone.layer4,
        )
        self.out_channels = 512

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: [B, 3, H, W] → [B, 512, H', W']"""
        return self.backbone(x)


# ---------------------------------------------------------------------------
# State encoder
# ---------------------------------------------------------------------------

class StateEncoder(nn.Module):
    """2-layer MLP embedding qpos → d_model vector."""

    def __init__(self, input_dim: int = 6, d_model: int = 512):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, d_model),
            nn.ReLU(),
            nn.Linear(d_model, d_model),
        )

    def forward(self, qpos: torch.Tensor) -> torch.Tensor:
        """qpos: [B, input_dim] → [B, d_model]"""
        return self.net(qpos)


# ---------------------------------------------------------------------------
# CVAE encoder (training only)
# ---------------------------------------------------------------------------

class CVAEEncoder(nn.Module):
    """Small transformer that encodes (action_chunk, qpos) → (mu, log_var).

    Used during training to produce the latent z.
    At inference, z ~ N(0, I).
    """

    def __init__(
        self,
        action_dim: int = 6,
        chunk_size: int = 100,
        d_model: int = 512,
        z_dim: int = 32,
        nhead: int = 8,
        num_layers: int = 4,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.chunk_size = chunk_size
        self.d_model = d_model
        self.z_dim = z_dim

        # Project action sequence to d_model
        self.action_proj = nn.Linear(action_dim, d_model)
        # Project qpos to d_model (reused as CLS token)
        self.qpos_proj = nn.Linear(6, d_model)

        self.pos_enc = PositionalEncoding(d_model, max_len=chunk_size + 2, dropout=dropout)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead,
            dim_feedforward=d_model * 4,
            dropout=dropout, batch_first=True,
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # Project CLS token → mu, log_var
        self.mu_proj = nn.Linear(d_model, z_dim)
        self.logvar_proj = nn.Linear(d_model, z_dim)

    def forward(
        self, actions: torch.Tensor, qpos: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        actions: [B, chunk_size, 6]
        qpos:    [B, 6]
        Returns: mu [B, z_dim], log_var [B, z_dim]
        """
        B = actions.size(0)

        # CLS token from qpos
        cls = self.qpos_proj(qpos).unsqueeze(1)  # [B, 1, d_model]

        # Action tokens
        act_tokens = self.action_proj(actions)    # [B, chunk_size, d_model]

        # Concat [CLS | actions]
        seq = torch.cat([cls, act_tokens], dim=1)  # [B, chunk_size+1, d_model]
        seq = self.pos_enc(seq)

        out = self.transformer(seq)   # [B, chunk_size+1, d_model]
        cls_out = out[:, 0]           # [B, d_model]

        mu = self.mu_proj(cls_out)
        log_var = self.logvar_proj(cls_out)
        return mu, log_var


# ---------------------------------------------------------------------------
# DETR-style transformer decoder
# ---------------------------------------------------------------------------

class ACTDecoder(nn.Module):
    """DETR-style decoder with learned query embeddings.

    Queries: learned [chunk_size, d_model] embeddings.
    Keys/values: concatenated context tokens (image features + state + z).
    """

    def __init__(
        self,
        chunk_size: int = 100,
        d_model: int = 512,
        nhead: int = 8,
        num_layers: int = 7,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.chunk_size = chunk_size
        self.d_model = d_model

        # Learned query embeddings
        self.query_embed = nn.Embedding(chunk_size, d_model)

        decoder_layer = nn.TransformerDecoderLayer(
            d_model=d_model, nhead=nhead,
            dim_feedforward=d_model * 4,
            dropout=dropout, batch_first=True,
        )
        self.transformer = nn.TransformerDecoder(decoder_layer, num_layers=num_layers)

    def forward(
        self, memory: torch.Tensor
    ) -> torch.Tensor:
        """
        memory: [B, S, d_model]  — context tokens
        Returns: [B, chunk_size, d_model]
        """
        B = memory.size(0)
        queries = self.query_embed.weight.unsqueeze(0).expand(B, -1, -1)
        out = self.transformer(queries, memory)
        return out


# ---------------------------------------------------------------------------
# Full ACT policy
# ---------------------------------------------------------------------------

class ACTPolicy(nn.Module):
    """ACT: Action Chunking with Transformers.

    Args:
        chunk_size: Number of actions to predict per forward pass.
        d_model: Transformer hidden dimension.
        z_dim: CVAE latent dimension.
        kl_weight: Weight for KL term in ELBO loss.
        pretrained_encoder: Use pretrained ResNet18 weights.
    """

    def __init__(
        self,
        chunk_size: int = 100,
        d_model: int = 512,
        z_dim: int = 32,
        nhead: int = 8,
        num_encoder_layers: int = 4,
        num_decoder_layers: int = 7,
        dropout: float = 0.1,
        kl_weight: float = 10.0,
        pretrained_encoder: bool = True,
    ):
        super().__init__()
        self.chunk_size = chunk_size
        self.d_model = d_model
        self.z_dim = z_dim
        self.kl_weight = kl_weight

        # Encoders
        self.img_encoder_left = ImageEncoder(pretrained=pretrained_encoder)
        self.img_encoder_right = ImageEncoder(pretrained=pretrained_encoder)
        self.state_encoder = StateEncoder(input_dim=6, d_model=d_model)

        # Project image feature maps (512 channels) → d_model
        self.img_proj = nn.Linear(self.img_encoder_left.out_channels, d_model)

        # Project z → d_model
        self.z_proj = nn.Linear(z_dim, d_model)

        # CVAE encoder (used during training)
        self.cvae_encoder = CVAEEncoder(
            action_dim=6,
            chunk_size=chunk_size,
            d_model=d_model,
            z_dim=z_dim,
            nhead=nhead,
            num_layers=num_encoder_layers,
            dropout=dropout,
        )

        # DETR decoder
        self.decoder = ACTDecoder(
            chunk_size=chunk_size,
            d_model=d_model,
            nhead=nhead,
            num_layers=num_decoder_layers,
            dropout=dropout,
        )

        # Action head
        self.action_head = nn.Linear(d_model, 6)

        self._init_weights()

    def _init_weights(self):
        """Initialize non-pretrained weights."""
        for module in [
            self.img_proj, self.z_proj, self.state_encoder,
            self.action_head, self.cvae_encoder.action_proj,
            self.cvae_encoder.qpos_proj,
            self.cvae_encoder.mu_proj, self.cvae_encoder.logvar_proj,
        ]:
            if isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def _encode_images(
        self, images_left: torch.Tensor, images_right: torch.Tensor
    ) -> torch.Tensor:
        """Encode both camera images into a flat sequence of tokens.

        images_left/right: [B, 3, H, W]
        Returns: [B, 2*H'*W', d_model]
        """
        feat_l = self.img_encoder_left(images_left)    # [B, 512, H', W']
        feat_r = self.img_encoder_right(images_right)  # [B, 512, H', W']

        B, C, Hp, Wp = feat_l.shape
        # Flatten spatial → tokens
        feat_l = feat_l.permute(0, 2, 3, 1).reshape(B, Hp * Wp, C)  # [B, H'W', 512]
        feat_r = feat_r.permute(0, 2, 3, 1).reshape(B, Hp * Wp, C)

        # Project to d_model
        feat_l = self.img_proj(feat_l)  # [B, H'W', d_model]
        feat_r = self.img_proj(feat_r)

        return torch.cat([feat_l, feat_r], dim=1)  # [B, 2*H'W', d_model]

    def _reparameterize(
        self, mu: torch.Tensor, log_var: torch.Tensor
    ) -> torch.Tensor:
        """Reparameterization trick: z = mu + eps * std."""
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        return mu + eps * std

    def _build_memory(
        self,
        qpos: torch.Tensor,
        images_left: torch.Tensor,
        images_right: torch.Tensor,
        z: torch.Tensor,
    ) -> torch.Tensor:
        """Assemble context (memory) tokens for the decoder.

        Returns: [B, S, d_model] where S = 2*H'*W' + 2
        """
        img_tokens = self._encode_images(images_left, images_right)  # [B, 2*H'W', d_model]
        state_token = self.state_encoder(qpos).unsqueeze(1)            # [B, 1, d_model]
        z_token = self.z_proj(z).unsqueeze(1)                         # [B, 1, d_model]
        return torch.cat([img_tokens, state_token, z_token], dim=1)   # [B, S, d_model]

    def forward(
        self,
        qpos: torch.Tensor,
        images_left: torch.Tensor,
        images_right: torch.Tensor,
        actions: Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:
        """Training forward pass.

        Args:
            qpos: [B, 6] normalized joint positions
            images_left: [B, 3, H, W] ImageNet-normalized
            images_right: [B, 3, H, W] ImageNet-normalized
            actions: [B, chunk_size, 6] normalized target actions

        Returns:
            dict with keys "l1", "kl", "total" (all scalar tensors)
        """
        assert actions is not None, "actions required for training forward pass"

        # CVAE encode → latent z
        mu, log_var = self.cvae_encoder(actions, qpos)
        z = self._reparameterize(mu, log_var)

        # Build memory and decode
        memory = self._build_memory(qpos, images_left, images_right, z)
        decoded = self.decoder(memory)               # [B, chunk_size, d_model]
        pred_actions = self.action_head(decoded)     # [B, chunk_size, 6]

        # L1 reconstruction loss
        l1_loss = F.l1_loss(pred_actions, actions)

        # KL divergence: -0.5 * sum(1 + log_var - mu^2 - exp(log_var))
        kl_loss = -0.5 * torch.mean(1 + log_var - mu.pow(2) - log_var.exp())

        total_loss = l1_loss + self.kl_weight * kl_loss

        return {"l1": l1_loss, "kl": kl_loss, "total": total_loss}

    @torch.no_grad()
    def select_action(
        self,
        qpos: torch.Tensor,
        images_left: torch.Tensor,
        images_right: torch.Tensor,
    ) -> torch.Tensor:
        """Inference forward pass. z sampled from prior N(0, I).

        Args:
            qpos: [B, 6]
            images_left: [B, 3, H, W]
            images_right: [B, 3, H, W]

        Returns:
            action_chunk: [B, chunk_size, 6]
        """
        B = qpos.size(0)
        z = torch.randn(B, self.z_dim, device=qpos.device, dtype=qpos.dtype)

        memory = self._build_memory(qpos, images_left, images_right, z)
        decoded = self.decoder(memory)
        return self.action_head(decoded)  # [B, chunk_size, 6]
