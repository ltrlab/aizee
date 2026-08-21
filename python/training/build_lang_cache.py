"""
build_lang_cache.py — precompute the Minerva task-string -> embedding cache.

Scans the episode HDF5 files for their `language_instruction` (or legacy
`task_tag`) attributes, optionally adds LLM paraphrases from a JSON file, encodes
every unique string ONCE with the frozen sentence encoder, and writes a cache
keyed by the normalized-string hash. Training and deployment then do a dict
lookup — the text tower never runs on the Jetson.

    python -m python.training.build_lang_cache \
        --data-dir episodes/ --out checkpoints/minerva/task_embeddings.npz \
        [--paraphrases paraphrases.json]

paraphrases.json format: {"pick up the red block": ["grab the red cube", ...], ...}
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import h5py

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from python.training.language import TextConditioner


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--lang-model", default="sentence-transformers/all-MiniLM-L6-v2")
    ap.add_argument("--paraphrases", default=None,
                    help="optional JSON {canonical: [variant, ...]}")
    args = ap.parse_args()

    strings: set[str] = set()
    for p in sorted(Path(args.data_dir).glob("episode_*.hdf5")):
        with h5py.File(p, "r") as f:
            s = f.attrs.get("language_instruction", f.attrs.get("task_tag", ""))
            if s:
                strings.add(str(s))
            seg_raw = f.attrs.get("segments")   # v7: per-phase action labels
            if seg_raw is not None:
                try:
                    if isinstance(seg_raw, bytes):
                        seg_raw = seg_raw.decode("utf-8")
                    for seg in json.loads(seg_raw):
                        if isinstance(seg, dict) and seg.get("label"):
                            strings.add(str(seg["label"]))
                except Exception:
                    pass
    if args.paraphrases:
        table = json.loads(Path(args.paraphrases).read_text())
        for k, variants in table.items():
            strings.add(k)
            strings.update(variants)

    print(f"{len(strings)} unique task strings to encode")
    tc = TextConditioner(model_name=args.lang_model)
    tc.build_cache(sorted(strings), args.out)
    print(f"wrote {Path(args.out).with_suffix('.npz')} (dim={tc.embed_dim})")


if __name__ == "__main__":
    main()
