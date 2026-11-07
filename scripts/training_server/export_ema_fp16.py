"""Export a MinervaPolicy checkpoint's EMA weights (inference parts only) as FP16, with its config and dataset stats,
for the Jetson inference timing test. CPU only, memory-mapped: the 1 GB checkpoint is never read into RAM whole.

usage: python export_ema_fp16.py <checkpoint.pt> <out.pt>
"""

import hashlib
import json
import os
import sys

os.environ.setdefault('CUDA_VISIBLE_DEVICES', '')
import torch  # noqa: E402

TRAINING_ONLY = ('jepa_predictor.', 'sigreg_proj.', 'sigreg.')

src, dst = sys.argv[1], sys.argv[2]
ckpt = torch.load(src, map_location='cpu', mmap=True, weights_only=False)
ema = ckpt['ema_state_dict']
kept = {k: v.detach().to(torch.float16).clone() for k, v in ema.items() if not k.startswith(TRAINING_ONLY)}
dropped = sorted({k.split('.')[0] for k in ema if k.startswith(TRAINING_ONLY)})
out = {
    'ema_fp16': kept,
    'config': ckpt['config'],
    'dataset_stats': ckpt['dataset_stats'],
    'epoch': ckpt['epoch'],
    'ema_num_updates': ckpt.get('ema_num_updates'),
    'source': os.path.abspath(src),
}
os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
torch.save(out, dst)
digest = hashlib.sha256(open(dst, 'rb').read()).hexdigest()
print(
    json.dumps(
        {
            'epoch': ckpt['epoch'],
            'ema_num_updates': ckpt.get('ema_num_updates'),
            'tensors': len(kept),
            'params': sum(v.numel() for v in kept.values()),
            'dropped_prefixes': dropped,
            'bytes': os.path.getsize(dst),
            'sha256': digest,
            'config': ckpt['config'],
        }
    )
)
