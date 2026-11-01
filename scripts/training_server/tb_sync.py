"""Feed TensorBoard from the training wrapper's per-epoch records (ckpt/<run>/timing.jsonl): the same per-epoch
train/ and val/ loss scalars train_minerva writes when TensorBoard is on, plus step time and GPU memory. Polls and adds
each new epoch once. Also writes a fixed comparison run from a JSON of per-epoch losses (v2 on the laptop).

    python tb_sync.py --run train-v3 [--compare v2.json] [--every 60]"""
import argparse
import json
import os
import time
from pathlib import Path

from torch.utils.tensorboard import SummaryWriter

ROOT = Path(os.environ.get('AIZEE_SIM', Path.home() / 'aizee-sim'))
p = argparse.ArgumentParser()
p.add_argument('--run', required=True)
p.add_argument('--compare', type=Path)
p.add_argument('--every', type=float, default=60.)
a = p.parse_args()

if a.compare:
    out = ROOT / 'tb' / a.compare.stem
    if not out.exists():
        w = SummaryWriter(str(out))
        for row in json.loads(a.compare.read_text()):
            for phase in ('train', 'val'):
                for k, v in row[phase].items():
                    w.add_scalar(f'{phase}/{k}', v, row['epoch'])
        w.close()

timing = ROOT / 'ckpt' / a.run / 'timing.jsonl'
writer = SummaryWriter(str(ROOT / 'tb' / a.run))
done = set()
while True:
    if timing.exists():
        for line in timing.read_text().splitlines():
            try:
                r = json.loads(line)
            except ValueError:
                continue
            key = (r.get('epoch'), r.get('phase'))
            if key in done or r.get('phase') not in ('train', 'val') or not r.get('loss'):
                continue
            done.add(key)
            epoch, phase = r['epoch'], r['phase']
            for k, v in r['loss'].items():
                writer.add_scalar(f'{phase}/{k}', v, epoch)
            writer.add_scalar(f'timing/{phase}_s_per_step', r.get('s_per_step', 0.), epoch)
            writer.add_scalar(f'timing/{phase}_epoch_minutes', r.get('wall_s', 0.) / 60, epoch)
            if phase == 'train':
                writer.add_scalar('gpu/max_allocated_gib', r.get('gpu_max_allocated_gib', 0.), epoch)
            writer.flush()
    time.sleep(a.every)
