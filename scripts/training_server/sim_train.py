"""Train MinervaPolicy on generated simulation episodes on coolserver: a sim-only pipeline proof, never a robot policy.

Linux port of the laptop's sim_train.py. Runs Gemini's python/training/train_minerva.py unchanged, with one explicit
opt-in: the dataset also trains on rows with 'policy' authority, which is how the generator labels the scripted
demonstrator's commands. By default the loader trains on human and correction rows only. A sidecar (sim_training.json)
beside the checkpoints says what the data was.

Adds timing only: every SIM_TRAIN_LOG_EVERY steps (default 25) it prints the mean step time, the time spent waiting for
data and the GPU memory high-water mark, and after each epoch it appends a line to timing.jsonl in the output folder.

Usage (with $AIZEE_SIM/.venv-train/bin/python):
    python sim_train.py --data-dir <episodes> --output-dir <checkpoints> [train_minerva options]
AIZEE_TRAIN_SRC names the copied training source (default $AIZEE_SIM/train-0a4d8ef, which has a SOURCE.json).

A fine-tune keeps its starting checkpoint's normalization: with SIM_TRAIN_STATS_FROM=<checkpoint>, the training set
uses that checkpoint's dataset_stats instead of computing them from the new episodes (train_minerva computes them from
the training episodes and does not restore them on --resume), so the inputs and outputs stay on the old scale; the
validation set and every saved checkpoint carry the same stats.
"""
import json
import os
from pathlib import Path
import socket
import sys
import time

ROOT = Path(os.environ.get('AIZEE_TRAIN_SRC', Path(os.environ.get('AIZEE_SIM', Path.home() / 'aizee-sim')) / 'train-0a4d8ef'))
sys.path.insert(0, str(ROOT))

import python.training.train_minerva as train  # noqa: E402

torch = train.torch
LOG_EVERY = int(os.environ.get('SIM_TRAIN_LOG_EVERY', '25'))
STATS_FROM = os.environ.get('SIM_TRAIN_STATS_FROM') or None
STATS = None
if STATS_FROM:
    import numpy as np
    _stats = torch.load(STATS_FROM, map_location='cpu', weights_only=False, mmap=True)['dataset_stats']
    STATS = {k: (np.asarray(v, dtype=np.float32) if isinstance(v, (list, tuple)) else v) for k, v in _stats.items()}
    del _stats


class GeneratedEpisodes(train.MinervaEpisodeDataset):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('allowed_authorities', ('policy',))
        if STATS is not None and kwargs.get('dataset_stats') is None:
            kwargs['dataset_stats'] = STATS
        super().__init__(*args, **kwargs)


train.MinervaEpisodeDataset = GeneratedEpisodes

# A resumed run loads its checkpoint (model, EMA and optimizer, about 1 GB) onto the GPU and keeps it referenced for
# the whole run; every state copies into the live tensors anyway, so load it into host memory instead (kept from the
# laptop wrapper, where the 8 GB GPU ran out of room on resume, 2026-10-17).
_load = torch.load


def _load_on_host(*args, **kwargs):
    kwargs['map_location'] = 'cpu'
    return _load(*args, **kwargs)


torch.load = _load_on_host


def _gib(value):
    return round(value / 2**30, 2)


class _Timed:
    """Wraps a DataLoader: time waiting for each batch, and time from handing it over to the next request (the step:
    run_epoch reads each loss with .item(), which waits for the GPU, before it asks for the next batch)."""

    def __init__(self, loader, label):
        self.loader, self.label = loader, label
        self.steps, self.wait, self.work, self.wall = 0, 0., 0., 0.

    def __len__(self):
        return len(self.loader)

    def __iter__(self):
        started = mark = time.perf_counter()
        total = len(self.loader)
        for batch in self.loader:
            got = time.perf_counter()
            self.wait += got - mark
            yield batch
            mark = time.perf_counter()
            self.work += mark - got
            self.steps += 1
            if LOG_EVERY and (self.steps % LOG_EVERY == 0 or self.steps == total):
                print(f'  [{self.label}] step {self.steps}/{total}: {(mark - started) / self.steps:.3f} s/step '
                      f'(data wait {self.wait / self.steps:.3f} s){self._memory()}', flush=True)
        self.wall = time.perf_counter() - started

    @staticmethod
    def _memory():
        if not torch.cuda.is_available():
            return ''
        return (f', GPU max allocated {_gib(torch.cuda.max_memory_allocated())} GiB, '
                f'reserved {_gib(torch.cuda.max_memory_reserved())} GiB')


_run_epoch = train.run_epoch
_epoch = {'n': 0, 'out': None}


def _timed_run_epoch(policy, loader, device, has_lang, **kwargs):
    label = 'train' if kwargs.get('optimizer') is not None else 'val'
    if label == 'train':
        _epoch['n'] += 1
    timed = _Timed(loader, label)
    result = _run_epoch(policy, timed, device, has_lang, **kwargs)
    line = {'epoch': _epoch['n'], 'phase': label, 'steps': timed.steps, 'wall_s': round(timed.wall, 2),
            's_per_step': round(timed.wall / max(timed.steps, 1), 4),
            'data_wait_s_per_step': round(timed.wait / max(timed.steps, 1), 4), 'loss': result,
            'gpu_max_allocated_gib': _gib(torch.cuda.max_memory_allocated()) if torch.cuda.is_available() else None,
            'gpu_max_reserved_gib': _gib(torch.cuda.max_memory_reserved()) if torch.cuda.is_available() else None,
            'at': time.strftime('%Y-%m-%dT%H:%M:%S')}
    print(f"  [{label}] epoch {_epoch['n']}: {timed.steps} steps in {timed.wall:.1f} s", flush=True)
    if _epoch['out'] is not None:
        with open(_epoch['out'] / 'timing.jsonl', 'a', encoding='utf-8') as f:
            f.write(json.dumps(line) + '\n')
    return result


train.run_epoch = _timed_run_epoch


def _sidecar(argv):
    args = train.parse_args(argv)
    data = Path(args.data_dir)
    manifest = data.parent / 'manifest.jsonl'
    source = ROOT / 'SOURCE.json'
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    _epoch['out'] = out
    gpu = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
    (out / 'sim_training.json').write_text(json.dumps({
        'purpose': 'sim-only pipeline proof; not promoted, never for the robot',
        'data_dir': str(data), 'episodes': len(list(data.glob('task_episode_*.hdf5'))),
        'manifest': str(manifest) if manifest.exists() else None,
        'allowed_authorities': ['policy'],
        'source_commit': json.loads(source.read_text(encoding='utf-8')).get('commit') if source.exists() else None,
        'source': json.loads(source.read_text(encoding='utf-8')) if source.exists() else None,
        'host': socket.gethostname(), 'gpu': gpu, 'cuda_visible_devices': os.environ.get('CUDA_VISIBLE_DEVICES'),
        'torch': torch.__version__, 'argv': argv, 'resume': args.resume,
        'dataset_stats_from': STATS_FROM,
    }, indent=1) + '\n', encoding='utf-8')


if __name__ == '__main__':
    _sidecar(sys.argv[1:])
    train.main(sys.argv[1:])
