"""Main-loop profiler (from collect_demo.py)."""
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Main-loop profiler (per-section timing → log file, dump every 10 s)
# ---------------------------------------------------------------------------

class _LoopProfiler:
    def __init__(self, log_path: Optional[Path] = None) -> None:
        self._log = None
        if log_path is not None:
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self._log = open(log_path, "a", buffering=1, encoding="utf-8")
                self._log.write(f"\n=== profiler started {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            except Exception:
                self._log = None
        self._sec: dict[str, list[float]] = {}
        self._gauge: dict[str, list[float]] = {}
        self._work: list[float] = []
        self._period: list[float] = []
        self._t_sec: Optional[float] = None
        self._t_iter: Optional[float] = None
        self._t_prev: Optional[float] = None
        self._next_dump = time.perf_counter() + 10.0

    def begin(self) -> None:
        now = time.perf_counter()
        if self._t_prev is not None:
            self._period.append((now - self._t_prev) * 1000.0)
        self._t_prev = now
        self._t_iter = now
        self._t_sec = now

    def tick(self, name: str) -> None:
        if self._t_sec is None:
            return
        now = time.perf_counter()
        self._sec.setdefault(name, []).append((now - self._t_sec) * 1000.0)
        self._t_sec = now

    def gauge(self, name: str, value: float) -> None:
        self._gauge.setdefault(name, []).append(float(value))

    def end(self) -> None:
        if self._t_iter is None:
            return
        now = time.perf_counter()
        self._work.append((now - self._t_iter) * 1000.0)
        if now > self._next_dump:
            self.dump()
            self._next_dump = now + 10.0

    @staticmethod
    def _stats(arr: list[float]) -> Optional[tuple[float, float, float, float, int]]:
        if not arr:
            return None
        s = sorted(arr)
        n = len(s)
        return (sum(s) / n, s[n // 2], s[min(n - 1, int(n * 0.99))], s[-1], n)

    def dump(self) -> None:
        if self._log is None:
            self._sec.clear(); self._work.clear(); self._period.clear()
            return
        lines = [f"--- {time.strftime('%H:%M:%S')} ---"]
        for label, arr in (("period", self._period), ("work", self._work)):
            st = self._stats(arr)
            if st:
                lines.append(f"{label:8s} mean={st[0]:6.2f} p50={st[1]:6.2f} "
                             f"p99={st[2]:6.2f} max={st[3]:7.2f}  n={st[4]}")
        rows = []
        for name, arr in self._sec.items():
            st = self._stats(arr)
            if st:
                rows.append((st[3], st[2], st[0], name))
        rows.sort(reverse=True)
        for mx, p99, mean, name in rows:
            lines.append(f"  {name:14s} mean={mean:6.2f} p99={p99:6.2f} max={mx:7.2f}")
        for name, arr in self._gauge.items():
            st = self._stats(arr)
            if st:
                lines.append(f"  [g] {name:10s} mean={st[0]:6.2f} p50={st[1]:6.2f} "
                             f"p99={st[2]:6.2f} max={st[3]:7.2f}")
        self._log.write("\n".join(lines) + "\n")
        self._sec.clear(); self._gauge.clear()
        self._work.clear(); self._period.clear()
