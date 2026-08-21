"""
test_minerva_smoke.py — end-to-end plumbing test for the Minerva policy.

Exercises the full pipeline on tiny synthetic data (no real robot, no network,
no sentence-transformers) so the wiring is runtime-verified before real episodes
exist:

    save_minerva_episode (v6 writer)
      -> MinervaEpisodeDataset (+ hash-fallback language cache)
      -> collate_minerva -> MinervaPolicy.forward (flow + JEPA + SIGReg) -> backward/step
      -> MinervaPolicy.select_action (flow sampling)
      -> regression: select_action with language=None on a lang_dim>0 model must NOT crash
         (guards the shape bug the code review caught).

Run directly:   python python/tests/ml/test_minerva_smoke.py
Or via pytest:  pytest python/tests/ml/test_minerva_smoke.py
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

# aizee/ (for `python.*`) and aizee/python/ (for `common.*`) on the path.
# test file: aizee/python/tests/ml/test_minerva_smoke.py -> parents[3] == aizee/
_AIZEE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_AIZEE))
sys.path.insert(0, str(_AIZEE / "python"))

from python.scripts.collect_demo_app.minerva_recording import save_minerva_episode
from python.training.language import TextConditioner
from python.training.minerva_dataset import MinervaEpisodeDataset, collate_minerva
from python.training.minerva_model import MinervaPolicy
from torch.utils.data import DataLoader

_INSTRUCTIONS = ["pick up the red block", "place the cube in the bin"]
_SIZES = {"left_wrist": (96, 96), "right_wrist": (96, 96), "head": (64, 96)}  # (H, W)


def _write_episode(out_dir: Path, seed: int, instr: str, T: int = 10):
    rng = np.random.default_rng(seed)
    qpos = [rng.standard_normal(17).astype(np.float32) * 0.1 for _ in range(T)]
    qcmd = [q + rng.standard_normal(17).astype(np.float32) * 0.01 for q in qpos]
    tq = [rng.standard_normal(17).astype(np.float32) * 0.05 for _ in range(T)]
    cams = {
        name: [rng.integers(0, 255, (h, w, 3), dtype=np.uint8) for _ in range(T)]
        for name, (h, w) in _SIZES.items()
    }
    save_minerva_episode(out_dir, qpos, cams, qcmd_buf=qcmd, torque_buf=tq,
                         language_instruction=instr, task_id=seed)


def run_smoke() -> None:
    torch.manual_seed(0)
    tmp = Path(tempfile.mkdtemp(prefix="minerva_smoke_"))
    for i, instr in enumerate(_INSTRUCTIONS):
        _write_episode(tmp, seed=i, instr=instr)

    # Language cache via the dependency-free hash fallback (384-d).
    conditioner = TextConditioner(model_name="hash")
    conditioner.build_cache(_INSTRUCTIONS)

    ds = MinervaEpisodeDataset(
        sorted(tmp.glob("episode_*.hdf5")),
        chunk_size=8, future_offset=8, augment=True, action_mode="absolute",
        conditioner=conditioner,
    )
    assert ds.num_joints == 17, ds.num_joints
    assert set(ds.cameras) == set(_SIZES), ds.cameras
    loader = DataLoader(ds, batch_size=2, shuffle=True, collate_fn=collate_minerva)

    policy = MinervaPolicy(
        num_joints=17, chunk_size=8, d_model=64, state_dim=ds.state_dim,
        lang_dim=conditioner.embed_dim, nhead=4, head_layers=2, head_ff=128,
        camera_dropout=0.15, flow_steps=4, predictor_layers=2, sigreg_slices=64,
        pretrained_encoder=False,   # no weight download in CI
    )
    opt = torch.optim.AdamW(policy.parameters(), lr=1e-4)

    obs, actions = next(iter(loader))
    images = obs["images"]
    future = obs["future_images"]

    # --- training step ---
    policy.train()
    loss = policy(obs["state"], images, actions, language=obs["language"], future_images=future)
    for k in ("flow", "obs", "reg", "total"):
        assert k in loss and torch.isfinite(loss[k]), (k, loss.get(k))
    assert loss["obs"].item() > 0, "JEPA obs loss should be active with future frames"
    assert loss["total"].requires_grad
    opt.zero_grad(); loss["total"].backward(); opt.step()

    # --- inference ---
    policy.eval()
    b1 = {"state": obs["state"][:1], "images": {c: v[:1] for c, v in images.items()}}
    with torch.no_grad():
        chunk = policy.select_action(b1["state"], b1["images"],
                                     language=obs["language"][:1], num_steps=4)
    assert chunk.shape == (1, 8, 17), chunk.shape
    assert torch.isfinite(chunk).all()

    # --- regression: language=None on a lang_dim>0 model must not crash ---
    with torch.no_grad():
        chunk_nolang = policy.select_action(b1["state"], b1["images"],
                                            language=None, num_steps=4)
    assert chunk_nolang.shape == (1, 8, 17), chunk_nolang.shape
    assert torch.isfinite(chunk_nolang).all()

    print(f"SMOKE PASS — loss(flow={loss['flow']:.3f} obs={loss['obs']:.3f} "
          f"reg={loss['reg']:.3f}) chunk={tuple(chunk.shape)} "
          f"params={sum(p.numel() for p in policy.parameters()):,}")


def test_minerva_smoke():
    run_smoke()


if __name__ == "__main__":
    run_smoke()
