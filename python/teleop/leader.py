"""leader.py — Unified discovery for AIZEE leader-arm controllers.

The codebase supports two leader arms:

  * SO-101 (Feetech STS3215 over WaveShare USB-serial bus adapter) — the
    original arm, served by `so101_leader.So101Leader`.
  * OpenRB-150 + Dynamixel XL330 — newer arm built around a Robotis
    OpenRB-150 board, served by `openrb_leader.OpenRBLeader`.

Both classes expose the same duck-typed controller interface
(`connect / poll / close / clamped_joints / zero_offsets / directions /
JOINTS / AIZEE_JOINTS`), so call sites only need to know the *kind* at
discovery time to pick the right class and calibration file.

Typical usage:

    from leader import find_any_leader, get_leader_class, default_calib_path

    port, kind = find_any_leader(exclude=[estop_port], verbose=True)
    if port is not None:
        Cls   = get_leader_class(kind)
        calib = default_calib_path(kind)
        leader = Cls(port, calib=calib)
        leader.connect()
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

from so101_leader  import So101Leader,  find_so101_port,  _probe_so101
from so101_leader  import CALIB_PATH as _SO101_CALIB
from openrb_leader import OpenRBLeader, find_openrb_port, _probe_openrb
from openrb_leader import CALIB_PATH as _OPENRB_CALIB

# Public set of supported leader kinds.  "auto" is for arg-parsing convenience
# at the script layer; the discovery helpers themselves return concrete kinds.
LEADER_KINDS = ("so101", "openrb")


def get_leader_class(kind: str):
    """Return the leader class for a given kind string."""
    if kind == "so101":
        return So101Leader
    if kind == "openrb":
        return OpenRBLeader
    raise ValueError(f"Unknown leader kind: {kind!r}")


def default_calib_path(kind: str) -> Path:
    """Return the default calibration JSON path for a given kind."""
    if kind == "so101":
        return _SO101_CALIB
    if kind == "openrb":
        return _OPENRB_CALIB
    raise ValueError(f"Unknown leader kind: {kind!r}")


def probe_port(device: str, kind: str) -> Tuple[bool, str]:
    """Probe *device* as a leader of the given *kind*."""
    if kind == "so101":
        return _probe_so101(device)
    if kind == "openrb":
        return _probe_openrb(device)
    raise ValueError(f"Unknown leader kind: {kind!r}")


def find_any_leader(
    exclude: Optional[list[str]] = None,
    verbose: bool = False,
    prefer:  Optional[str] = None,
) -> Tuple[Optional[str], Optional[str]]:
    """Search the system for any supported leader arm.

    Tries SO-101 first by default (matches existing user setups), then
    OpenRB-150.  Pass `prefer="openrb"` to swap the search order.

    Returns (port, kind) where kind is one of LEADER_KINDS, or (None, None).
    """
    order = list(LEADER_KINDS)
    if prefer in order:
        order.remove(prefer)
        order.insert(0, prefer)

    for kind in order:
        if verbose:
            print(f"[leader] searching for {kind}...")
        if kind == "so101":
            port = find_so101_port(exclude=exclude, verbose=verbose)
        elif kind == "openrb":
            port = find_openrb_port(exclude=exclude, verbose=verbose)
        else:
            continue
        if port is not None:
            if verbose:
                print(f"[leader] {kind} found on {port}")
            return port, kind

    return None, None


def identify_port(device: str) -> Optional[str]:
    """Identify *device* as a known leader arm; return kind or None."""
    for kind in LEADER_KINDS:
        ok, _ = probe_port(device, kind)
        if ok:
            return kind
    return None
