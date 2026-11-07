"""One real observation and its demonstrator chunk from a v3 training episode, built by the training dataset itself
(normalized state, ImageNet-normalized images, normalized actions), for checking the policy's output on the Jetson.
CPU only.

usage: python export_obs.py <training source root> <weights export.pt> <episode.hdf5> <out.pt>
"""
import json
import os
import sys
from pathlib import Path

os.environ['CUDA_VISIBLE_DEVICES'] = ''
sys.path.insert(0, sys.argv[1])
import numpy as np  # noqa: E402
import torch  # noqa: E402

from python.training.minerva_dataset import MinervaEpisodeDataset  # noqa: E402

blob = torch.load(sys.argv[2], map_location='cpu', weights_only=False)
episode, out = Path(sys.argv[3]), sys.argv[4]
cfg = blob['config']
stats = {k: (np.asarray(v, dtype=np.float32) if isinstance(v, list) else v) for k, v in blob['dataset_stats'].items()}
dataset = MinervaEpisodeDataset([episode], chunk_size=cfg['chunk_size'], state_mode=cfg['state_mode'],
                                action_mode=cfg['action_mode'], dataset_stats=stats, allowed_authorities=('policy',))
index = len(dataset) // 3
obs, chunk = dataset[index]
torch.save({'state': obs['state'], 'images': obs['images'], 'actions': chunk, 'episode': episode.name,
            'index': index, 'frames': len(dataset)}, out)
print(json.dumps({'episode': episode.name, 'index': index, 'frames': len(dataset), 'state': list(obs['state'].shape),
                  'images': {k: list(v.shape) for k, v in obs['images'].items()}, 'actions': list(chunk.shape),
                  'bytes': os.path.getsize(out)}))
