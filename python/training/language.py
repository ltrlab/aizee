"""
language.py — cached text conditioning for Minerva's language-conditioned policy.

Design (from the language-conditioning research): the teleop dataset has a
FINITE set of task strings, so a frozen sentence encoder can be run entirely
OFFLINE and cached. At training time we look up a pooled embedding by a hash of
the normalized string; at deploy time the same lookup means the text tower
never runs on the Jetson (0 extra params, 0 ms on the control tick). Because
the encoder is frozen and cache-keyed, swapping encoders or adding paraphrases
never touches the episode HDF5 files.

Default encoder: sentence-transformers/all-MiniLM-L6-v2 (22M params, 384-d).
Alternatives (set model_name): all-mpnet-base-v2 (768-d, higher quality) or a
CLIP text tower. The policy projects whatever dim this yields to d_model.

Conditioning mechanism itself (token-prepend + AdaLN) lives in minerva_model.py;
this module only owns the encoder + cache + paraphrase table.

Offline usage:
    conditioner = TextConditioner()                       # lazy-loads the encoder
    conditioner.build_cache(all_task_strings, "task_embeddings.pt")

Train/deploy usage:
    conditioner = TextConditioner(cache_path="task_embeddings.pt")
    vec = conditioner.get("pick up the red block with the left arm")  # np.float32 [D]
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

_DEFAULT_MODEL = "sentence-transformers/all-MiniLM-L6-v2"
_DEFAULT_DIM = 384


def normalize_text(text: str) -> str:
    """Lowercase, strip, collapse internal whitespace — so trivially different
    spellings of the same instruction map to one cache entry."""
    return re.sub(r"\s+", " ", text.strip().lower())


def text_hash(text: str) -> str:
    """Stable cache key for a (normalized) task string."""
    return hashlib.sha1(normalize_text(text).encode("utf-8")).hexdigest()


class TextConditioner:
    """Frozen sentence encoder + on-disk embedding cache keyed by string hash.

    Args:
        model_name: sentence-transformers model id (or "hash" for the
            dependency-free deterministic fallback, useful in CI / offline dev).
        cache_path: optional .pt/.npz cache to load at init and update on save.
        embed_dim: override the embedding dim (auto-detected for real encoders).
        device: torch device for the encoder ("cpu" is fine — it's offline).
    """

    def __init__(
        self,
        model_name: str = _DEFAULT_MODEL,
        cache_path: Optional[str] = None,
        embed_dim: Optional[int] = None,
        device: str = "cpu",
    ):
        self.model_name = model_name
        self.device = device
        self._encoder = None                      # lazy
        self._embed_dim = embed_dim
        self._cache: Dict[str, np.ndarray] = {}
        # Normalize to the .npz that save_cache/load_cache actually use, so a
        # non-.npz cache_path (e.g. the documented '.pt') still loads.
        p = Path(cache_path) if cache_path else None
        if p is not None and p.suffix != ".npz":
            p = p.with_suffix(".npz")
        self.cache_path = p
        if self.cache_path is not None and self.cache_path.exists():
            self.load_cache(self.cache_path)

    # -- properties --------------------------------------------------------
    @property
    def embed_dim(self) -> int:
        if self._embed_dim is not None:
            return self._embed_dim
        if self.model_name == "hash":
            self._embed_dim = _DEFAULT_DIM
        else:
            self._ensure_encoder()               # sets _embed_dim
        return self._embed_dim

    # -- encoder -----------------------------------------------------------
    def _ensure_encoder(self):
        if self._encoder is not None or self.model_name == "hash":
            return
        try:
            from sentence_transformers import SentenceTransformer
        except ImportError as e:
            raise ImportError(
                "sentence-transformers is required to encode NEW task strings. "
                "Install it (`pip install sentence-transformers`) to build the "
                "cache, or construct TextConditioner(model_name='hash') for the "
                "deterministic dependency-free fallback (no semantic meaning)."
            ) from e
        self._encoder = SentenceTransformer(self.model_name, device=self.device)
        self._embed_dim = int(self._encoder.get_sentence_embedding_dimension())

    def _encode_hash_fallback(self, texts: Sequence[str]) -> np.ndarray:
        """Deterministic pseudo-embeddings from the string hash. Lets the whole
        pipeline run without sentence-transformers; paraphrases will NOT align in
        this mode, so use it only for shape/plumbing tests, never real training."""
        dim = self.embed_dim
        out = np.zeros((len(texts), dim), dtype=np.float32)
        for i, t in enumerate(texts):
            seed = int(text_hash(t)[:8], 16)
            rng = np.random.default_rng(seed)
            v = rng.standard_normal(dim).astype(np.float32)
            out[i] = v / (np.linalg.norm(v) + 1e-8)
        return out

    def encode(self, texts: Sequence[str]) -> np.ndarray:
        """Encode strings -> L2-normalized pooled embeddings [N, D] (float32)."""
        if self.model_name == "hash":
            return self._encode_hash_fallback(texts)
        self._ensure_encoder()
        emb = self._encoder.encode(
            list(texts), convert_to_numpy=True, normalize_embeddings=True,
            show_progress_bar=False,
        )
        return emb.astype(np.float32)

    # -- cache -------------------------------------------------------------
    def build_cache(
        self, strings: Sequence[str], out_path: Optional[str] = None,
    ) -> Dict[str, np.ndarray]:
        """Encode every unique string and (optionally) persist the cache."""
        uniq = sorted({normalize_text(s) for s in strings if s and s.strip()})
        if uniq:
            vecs = self.encode(uniq)
            for s, v in zip(uniq, vecs):
                self._cache[text_hash(s)] = v
        path = Path(out_path) if out_path else self.cache_path
        if path is not None:
            self.save_cache(path)
        return self._cache

    def save_cache(self, path: str | Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path.with_suffix(".npz"),
            keys=np.array(list(self._cache.keys())),
            vecs=np.stack(list(self._cache.values()))
            if self._cache else np.zeros((0, self.embed_dim), np.float32),
            model=np.array([self.model_name]),
        )

    def load_cache(self, path: str | Path):
        path = Path(path)
        npz = path if path.suffix == ".npz" else path.with_suffix(".npz")
        data = np.load(npz, allow_pickle=False)
        keys, vecs = data["keys"], data["vecs"]
        self._cache = {str(k): v.astype(np.float32) for k, v in zip(keys, vecs)}
        if vecs.shape[0]:
            self._embed_dim = int(vecs.shape[1])

    # -- lookup ------------------------------------------------------------
    def get(self, text: str) -> np.ndarray:
        """Pooled embedding for a task string. Cache hit = 0 ms; a miss lazily
        encodes once (off the control-tick critical path) and inserts."""
        key = text_hash(text)
        v = self._cache.get(key)
        if v is None:
            # Encode the NORMALIZED string so a miss matches what build_cache stored.
            v = self.encode([normalize_text(text)])[0]
            self._cache[key] = v
        return v

    def get_batch(self, texts: Sequence[str]) -> np.ndarray:
        return np.stack([self.get(t) for t in texts]).astype(np.float32)


class ParaphraseTable:
    """Optional task -> [paraphrases] map for instruction augmentation.

    At each training step, sample one paraphrase of the episode's canonical task
    string and condition on its cached embedding — teaches the policy to condition
    on *meaning*, not one exact sentence (the single biggest robustness win for
    one-model-many-tasks, per the DIAL line of work). Precompute paraphrases with
    an LLM offline; store as {canonical_string: [variant, ...]}.
    """

    def __init__(self, table: Optional[Dict[str, List[str]]] = None):
        self.table = {normalize_text(k): v for k, v in (table or {}).items()}

    def sample(self, canonical: str, rng: np.random.Generator) -> str:
        variants = self.table.get(normalize_text(canonical))
        if not variants:
            return canonical
        # Include the canonical string itself so it is also seen during training.
        return str(rng.choice(variants + [canonical]))


__all__ = ["TextConditioner", "ParaphraseTable", "normalize_text", "text_hash"]
