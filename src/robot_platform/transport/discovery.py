"""TCP endpoint parsing and host discovery, independent of collector apps.

The historical fallback addresses remain available for compatibility. New
applications can pass their deployment's fallback hosts explicitly.
"""

from __future__ import annotations

import socket
from collections.abc import Iterable


DEFAULT_FALLBACK_HOSTS = ("10.42.0.1", "192.168.50.1")


def split_tcp_endpoint(endpoint: str | None) -> tuple[str | None, str | None]:
    """Split a TCP endpoint, preserving brackets around an IPv6 host.

    Empty and non-TCP endpoints return ``(None, None)``. A host without a
    port returns ``(host, None)``.
    """
    if not endpoint or not endpoint.startswith("tcp://"):
        return None, None
    rest = endpoint[len("tcp://"):]
    if rest.startswith("[") and rest.endswith("]"):
        return rest, None
    host, separator, port = rest.rpartition(":")
    if not separator:
        return (rest or None), None
    return (host or None), (port or None)


def host_reachable(host: str, ports: Iterable[int | str], timeout: float = 0.6) -> bool:
    """Return whether TCP connects to any supplied port (no protocol writes)."""
    for port in ports:
        try:
            with socket.create_connection((host.strip("[]"), int(port)), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def resolve_host(
    primary_host: str,
    cmd_port: int | str | None,
    timeout: float = 0.6,
    *,
    fallback_hosts: Iterable[str] = DEFAULT_FALLBACK_HOSTS,
) -> tuple[str, list[str]]:
    """Probe the primary and ordered fallback hosts on command and SSH ports.

    Return the selected host and the full deduplicated candidate list. If no
    candidate answers, retain the primary address, as the collectors did.
    Pass ``fallback_hosts=()`` to disable deployment-specific fallbacks.
    """
    candidates = list(dict.fromkeys(h for h in (primary_host, *fallback_hosts) if h))
    ports = [port for port in (cmd_port, "22") if port]
    for host in candidates:
        if host_reachable(host, ports, timeout):
            return host, candidates
    return primary_host, candidates


__all__ = ["DEFAULT_FALLBACK_HOSTS", "split_tcp_endpoint", "host_reachable", "resolve_host"]
