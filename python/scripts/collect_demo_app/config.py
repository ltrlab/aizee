"""AIZEE teleop config loading, with shared endpoint discovery compatibility."""
from __future__ import annotations

from pathlib import Path

import yaml

from robot_platform.transport.discovery import (
    DEFAULT_FALLBACK_HOSTS as _ROVER_FALLBACK_HOSTS,
    host_reachable as _host_reachable,
    resolve_host as _resolve_rover_host,
    split_tcp_endpoint as _split_tcp_endpoint,
)


def _load_teleop_yaml() -> dict:
    here = Path(__file__).parent
    for candidate in [
        here / ".." / ".." / ".." / "config" / "teleop.yaml",
        Path("config") / "teleop.yaml",
    ]:
        p = candidate.resolve()
        if p.exists():
            return yaml.safe_load(p.read_text()) or {}
    return {}


def _load_endpoints() -> dict:
    return _load_teleop_yaml().get("endpoints", {})
