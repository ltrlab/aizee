"""config.py — load config/minerva.yaml for the Minerva collector.

Thin wrapper: resolves the config path (CWD-relative, then repo-relative),
and exposes the endpoints / camera set / camera resolutions / control rates
the collector needs. Endpoint overrides come from CLI flags in
collect_minerva.py.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple

import yaml

# aizee/config/minerva.yaml relative to this file
# (collect_minerva_app/config.py -> parents[3] == aizee/)
_REPO_CONFIG = Path(__file__).resolve().parents[3] / "config" / "minerva.yaml"


def load_config(path: Optional[str] = None) -> dict:
    p = Path(path) if path else Path("config/minerva.yaml")
    if not p.exists() and _REPO_CONFIG.exists():
        p = _REPO_CONFIG
    if not p.exists():
        raise FileNotFoundError(f"minerva config not found: {path or p}")
    return yaml.safe_load(p.read_text()) or {}


def camera_endpoints(cfg: dict) -> Dict[str, str]:
    return dict(cfg["endpoints"]["cameras"])


def camera_sizes(cfg: dict) -> Dict[str, Tuple[int, int]]:
    """{name: (width, height)} the policy/recorder consume."""
    return {c: (int(v["width"]), int(v["height"])) for c, v in cfg["cameras"].items()}


def control(cfg: dict) -> dict:
    return dict(cfg.get("control", {}))


def safety(cfg: dict) -> dict:
    return dict(cfg.get("safety", {}))


# ---------------------------------------------------------------------------
# Jetson host resolution (same search collect_demo uses):
#   configured IP (LAN/POE) -> 10.42.0.1 (USB-C direct) -> 192.168.50.1 (WiFi AP)
# ---------------------------------------------------------------------------

def resolve_jetson_host(primary: str, probe_port: int = 5555, verbose: bool = True) -> str:
    """Return the first reachable Jetson address, probing the configured host, then
    the USB-C direct link, then the WiFi AP — the order collect_demo uses. Reuses
    collect_demo's rover-host probe (TCP on the ZMQ port + ssh:22, so it works even
    before motor_control is up). Falls back to `primary` if nothing answers."""
    try:
        from collect_demo_app.config import _resolve_rover_host
    except Exception:
        return primary
    host, tried = _resolve_rover_host(primary, str(probe_port))
    if verbose:
        arrow = " -> ".join(tried)
        if host != primary:
            print(f"[net] {primary} unreachable; Jetson at {host} (tried {arrow})", flush=True)
        else:
            print(f"[net] Jetson at {host} (priority: {arrow})", flush=True)
    return host


__all__ = ["load_config", "camera_endpoints", "camera_sizes", "control", "safety",
           "resolve_jetson_host"]
