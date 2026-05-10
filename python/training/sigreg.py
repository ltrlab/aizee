"""
sigreg.py — Sketched Isotropic Gaussian Regularization (LeJEPA).

Self-contained ~50-line implementation of the LeJEPA regularizer
(Balestriero & LeCun, 2025). The idea: an embedding distribution
minimizing downstream prediction risk is provably isotropic Gaussian,
so we push embeddings toward N(0, I) by:

  1. Drawing `num_slices` random unit directions.
  2. Projecting embeddings onto each direction (1-D marginals).
  3. Standardizing each marginal (zero-mean / unit-std) so the test is
     scale-free.
  4. Comparing each marginal to N(0, 1) with an Epps-Pulley-style
     empirical-characteristic-function (ECF) test on a Gauss-Hermite grid.

This eliminates the EMA / stop-gradient / centering heuristics that
make traditional JEPAs brittle.

Usage:
    sigreg = SIGReg(num_slices=1024, num_points=17)
    loss = sigreg(embeddings)             # embeddings: [N, D]
    loss.backward()
"""

from __future__ import annotations

import math

import torch
import torch.nn as nn
import torch.nn.functional as F


class SIGReg(nn.Module):
    """Sketched Isotropic Gaussian Regularization.

    Args:
        num_slices: Number of random 1-D projections per forward pass.
            More slices → tighter Monte-Carlo estimate; cost is linear.
        num_points: Number of ECF evaluation points (Epps-Pulley grid).
        t_min, t_max: ECF grid range (in standard-deviation units of N(0,1)).
        resample_slices: If True, draw a fresh slice basis each forward
            (recommended — adds stochasticity, prevents over-fitting any
            fixed basis). If False, use a registered buffer (deterministic).
        eps: Std floor for the per-slice standardization.
    """

    def __init__(
        self,
        num_slices: int = 1024,
        num_points: int = 17,
        t_min: float = 0.1,
        t_max: float = 3.0,
        resample_slices: bool = True,
        eps: float = 1e-6,
    ):
        super().__init__()
        self.num_slices = num_slices
        self.num_points = num_points
        self.resample_slices = resample_slices
        self.eps = eps

        # ECF grid — Epps-Pulley uses a Gaussian-weighted grid over t.
        # We use a uniform grid in [t_min, t_max] for simplicity; the
        # Gaussian weighting is implicit via the target φ(t) = exp(-t²/2).
        t = torch.linspace(t_min, t_max, num_points)
        self.register_buffer("t_grid", t)                       # [M]
        self.register_buffer("phi_target", torch.exp(-0.5 * t.pow(2)))  # [M]

        if not resample_slices:
            v = torch.randn(1, num_slices)  # placeholder; correct D set lazily
            self.register_buffer("_slices", v)
        else:
            self._slices = None

    def _get_slices(self, dim: int, device, dtype) -> torch.Tensor:
        if self.resample_slices:
            v = torch.randn(dim, self.num_slices, device=device, dtype=dtype)
            return F.normalize(v, dim=0)
        # Lazy resize-or-reinit if D changed
        if self._slices.shape != (dim, self.num_slices):
            v = torch.randn(dim, self.num_slices, device=device, dtype=dtype)
            self._slices = F.normalize(v, dim=0)
        return self._slices

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        x: [N, D] — embeddings. N must be > 1.
        Returns: scalar loss ≥ 0. Zero when projections are perfectly N(0,1).
        """
        if x.dim() != 2:
            x = x.reshape(-1, x.shape[-1])
        N, D = x.shape
        if N < 2:
            return x.sum() * 0.0  # no-op, keeps graph alive

        # Random unit projections — shape [D, S]
        V = self._get_slices(D, x.device, x.dtype)
        proj = x @ V                                            # [N, S]

        # Per-slice standardization → marginals have empirical mean 0, std 1
        proj = (proj - proj.mean(dim=0, keepdim=True)) / (
            proj.std(dim=0, unbiased=False, keepdim=True) + self.eps
        )

        # Empirical characteristic function at grid points t_grid
        # ECF(t) = (1/N) Σ exp(i t Y)  → cos + i sin parts
        t = self.t_grid.to(x.dtype)                             # [M]
        # angles: [N, S, M]
        angles = proj.unsqueeze(-1) * t.view(1, 1, -1)
        ecf_re = angles.cos().mean(dim=0)                       # [S, M]
        ecf_im = angles.sin().mean(dim=0)                       # [S, M]

        # Target ECF of N(0, 1) is real: exp(-t²/2). Imaginary part target = 0.
        phi = self.phi_target.to(x.dtype).view(1, -1)           # [1, M]
        loss = (ecf_re - phi).pow(2) + ecf_im.pow(2)            # [S, M]
        return loss.mean()


__all__ = ["SIGReg"]
