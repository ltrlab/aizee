"""settings.py — persistent per-user settings for the Minerva collector.

These are per-machine UI/user preferences (selected microphone, voice model,
record length) — distinct from config/minerva.yaml, which holds the committed
hardware/model config. Stored as JSON at ~/.aizee/minerva_collector.json so
they survive across runs and never land in the repo.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_DEFAULT_PATH = Path.home() / ".aizee" / "minerva_collector.json"

DEFAULTS: Dict[str, Any] = {
    "mic_device": None,        # sounddevice input-device index; None = system default
    "voice_model": "base.en",  # faster-whisper / whisper model name
    "voice_seconds": 5.0,      # record window per voice capture
    "leader_swap": False,      # True = LEFT leader drives RIGHT arm and vice versa
}


class CollectorSettings:
    """Load/save a small dict of settings, backed by a JSON file."""

    def __init__(self, path: Optional[str] = None):
        self.path = Path(path) if path else _DEFAULT_PATH
        self.data: Dict[str, Any] = dict(DEFAULTS)
        self.load()

    def load(self) -> "CollectorSettings":
        try:
            if self.path.exists():
                loaded = json.loads(self.path.read_text())
                for k in DEFAULTS:
                    if k in loaded:
                        self.data[k] = loaded[k]
        except Exception:
            pass
        return self

    def save(self) -> bool:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.data, indent=2))
            return True
        except Exception:
            return False

    def get(self, key: str) -> Any:
        return self.data.get(key, DEFAULTS.get(key))

    def set(self, key: str, value: Any) -> None:
        self.data[key] = value

    def update(self, values: Dict[str, Any]) -> None:
        self.data.update(values)


def list_input_devices() -> List[Tuple[int, str]]:
    """[(index, name)] of audio INPUT devices; [] if sounddevice is unavailable."""
    try:
        import sounddevice as sd
        return [(i, str(d.get("name", f"device {i}")))
                for i, d in enumerate(sd.query_devices())
                if int(d.get("max_input_channels", 0)) > 0]
    except Exception:
        return []


__all__ = ["CollectorSettings", "list_input_devices", "DEFAULTS"]
