"""
test_minerva_settings.py — per-user collector settings store + device listing.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

_PY = Path(__file__).resolve().parents[3] / "python"
sys.path.insert(0, str(_PY))
sys.path.insert(0, str(_PY / "scripts"))

from collect_minerva_app.settings import CollectorSettings, DEFAULTS, list_input_devices


def run():
    tmp = Path(tempfile.mkdtemp(prefix="minerva_set_")) / "settings.json"
    s = CollectorSettings(tmp)
    assert s.get("voice_model") == DEFAULTS["voice_model"]
    assert s.get("mic_device") is None
    s.set("mic_device", 3)
    s.update({"voice_seconds": 4.0, "voice_model": "tiny.en"})
    assert s.save() and tmp.exists()

    s2 = CollectorSettings(tmp)   # round-trips from disk
    assert s2.get("mic_device") == 3
    assert s2.get("voice_seconds") == 4.0
    assert s2.get("voice_model") == "tiny.en"
    assert s2.get("nonexistent") is None
    print("  OK: settings load/save round-trip")

    devs = list_input_devices()
    assert isinstance(devs, list)
    assert all(isinstance(i, int) and isinstance(n, str) for i, n in devs)
    print(f"  OK: {len(devs)} audio input device(s) enumerated")
    print("SETTINGS TEST PASS")


def test_minerva_settings():
    run()


if __name__ == "__main__":
    run()
