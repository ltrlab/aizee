"""Teleop-config / endpoint loading and rover host resolution (from collect_demo.py)."""
from __future__ import annotations

import socket
from pathlib import Path
from typing import Optional

import yaml

# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Rover host resolution
#
# The rover (Jetson) is reachable on different networks depending on how the
# operator is connected.  Probe them in priority order and point every rover
# ZMQ endpoint at the first one that answers:
#   1. the configured IP (config/teleop.yaml, default 192.168.0.27 — POE/LAN)
#   2. 10.42.0.1    — direct USB-C ethernet adapter (NetworkManager shared link)
#   3. 192.168.50.1 — the rover's own WiFi access point ("aizee" AP mode)
# ---------------------------------------------------------------------------

# Fallback rover IPs appended after the configured address, in priority order.
_ROVER_FALLBACK_HOSTS = ["10.42.0.1", "192.168.50.1"]


def _split_tcp_endpoint(ep: Optional[str]) -> tuple[Optional[str], Optional[str]]:
    """'tcp://host:port' -> ('host', 'port').  Returns (None, None) for an
    empty / non-tcp:// endpoint, and (host, None) when no port is present."""
    if not ep or "://" not in ep:
        return None, None
    rest = ep.split("://", 1)[1]
    host, sep, port = rest.rpartition(":")
    if not sep:
        return (rest or None), None
    return (host or None), (port or None)


def _host_reachable(host: str, ports, timeout: float = 0.6) -> bool:
    """True if a TCP connection to `host` succeeds on any of `ports`."""
    for port in ports:
        try:
            with socket.create_connection((host, int(port)), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def _resolve_rover_host(primary_host: str, cmd_port: Optional[str],
                        timeout: float = 0.6) -> tuple[str, list]:
    """Probe configured IP -> USB-C ethernet -> WiFi AP and return the first
    reachable host plus the ordered candidate list that was tried.  Falls back
    to `primary_host` if nothing answers.

    Probes both the ZMQ command port (what we actually use) and ssh (22, which
    is always up) so host selection works even before rover services start."""
    candidates: list = []
    for h in [primary_host, *_ROVER_FALLBACK_HOSTS]:
        if h and h not in candidates:
            candidates.append(h)
    ports = [p for p in (cmd_port, "22") if p]
    for h in candidates:
        if _host_reachable(h, ports, timeout):
            return h, candidates
    return primary_host, candidates
