#!/usr/bin/env bash
# The splat trainer's venv in ~/minerva-splat on the operator station's Ubuntu (WSL2), installed from the wheels
# download_wheels.ps1 fetched on the Windows side (WSL runs bridged there, without a network). Nothing system-wide.
# Ends with a tiny rasterization on the GPU: gsplat's prebuilt CUDA kernels load.
#
#     bash install_wsl.sh [wheels folder]      (default: the Windows profile's minerva-splat\wheels)
set -euo pipefail
ROOT="$HOME/minerva-splat"
WHEELS="${1:-/mnt/c/Users/598Jo/minerva-splat/wheels}"
mkdir -p "$ROOT"
cd "$ROOT"
[ -x venv/bin/python ] || python3 -m venv venv
. venv/bin/activate
pip install --no-index --find-links "$WHEELS" --quiet --upgrade pip wheel
pip install --no-index --find-links "$WHEELS" --quiet "torch==2.4.1+cu124" gsplat "numpy<2" pillow plyfile tqdm
python - <<'EOF'
import time
import torch, gsplat
from gsplat import rasterization
print('torch', torch.__version__, 'cuda', torch.version.cuda, 'available', torch.cuda.is_available(),
      torch.cuda.get_device_name(0))
print('gsplat', gsplat.__version__)
n = 1000
means = torch.randn(n, 3, device='cuda') * .3 + torch.tensor([0., 0., 3.], device='cuda')
quats = torch.nn.functional.normalize(torch.randn(n, 4, device='cuda'), dim=-1)
scales = torch.full((n, 3), .03, device='cuda')
opacities = torch.full((n,), .8, device='cuda')
colors = torch.rand(n, 3, device='cuda')
K = torch.tensor([[[300., 0, 160], [0, 300., 120], [0, 0, 1]]], device='cuda')
started = time.time()
image, alpha, info = rasterization(means, quats, scales, opacities, colors, torch.eye(4, device='cuda')[None], K,
                                   320, 240)
torch.cuda.synchronize()
print('rasterized', tuple(image.shape), 'covered', round(float((alpha > .5).float().mean()), 3),
      'in', round(time.time() - started, 2), 's')
EOF
du -sh "$ROOT/venv"
