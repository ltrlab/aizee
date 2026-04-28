#!/usr/bin/env python3
"""leader_monitor.py — live position + configured-limits monitor for any AIZEE leader.

Auto-detects which leader arm (SO-101 or OpenRB-150) is plugged in and
shows, for each of the 7 joints:

  * Current raw encoder ticks (0..4095, mod the encoder range)
  * Current radians (continuously unwrapped from calibration)
  * Calibrated raw range  [min_raw, max_raw]
  * Mapped AIZEE rad range [rad_min, rad_max]
  * Visual bar of where the joint is within its calibrated range
  * `OUT` flag if the joint is outside its calibrated band

Usage:
    python python/scripts/leader_monitor.py                 # auto-detect
    python python/scripts/leader_monitor.py --port COM4
    python python/scripts/leader_monitor.py --leader openrb
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from leader import (
    find_any_leader, get_leader_class, default_calib_path, LEADER_KINDS,
)


# ---------------------------------------------------------------------------
# ANSI helpers
# ---------------------------------------------------------------------------
def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_GRN = "\033[1;32m"
_YEL = "\033[1;33m"
_RED = "\033[1;31m"
_DIM = "\033[2m"
_RST = "\033[0m"


# Total visible width (inside the box border)
_W = 110


def _bar(value: float, lo: float, hi: float, width: int = 18) -> str:
    """Bar showing where *value* sits in [lo, hi].  Marks out-of-range with edge chars."""
    if hi <= lo:
        return "[" + "-" * width + "]"
    frac = (value - lo) / (hi - lo)
    if frac < 0:
        return "<" + "-" * width + "]"
    if frac > 1:
        return "[" + "-" * width + ">"
    pos = max(0, min(width - 1, int(frac * width)))
    cells = ["-"] * width
    cells[pos] = "#"
    return "[" + "".join(cells) + "]"


# ---------------------------------------------------------------------------
# Render
# ---------------------------------------------------------------------------
def _fmt_joint_row(
    joint:    str,
    aizee:    str,
    sid:      Optional[int],
    raw:      Optional[int],
    rad:      Optional[float],
    cal:      dict,
    obs:      Optional[tuple[int, int]] = None,
) -> str:
    """One per-joint line.  cal is the per-joint dict from calibration JSON.
    *obs* is the observed (min, max) tick range for this session, if any."""
    mn        = cal.get("min_raw", 0)
    mx        = cal.get("max_raw", 4095)
    r_min     = cal.get("rad_min", -3.14)
    r_max     = cal.get("rad_max",  3.14)
    direction = cal.get("direction", 1)
    sid_str   = f"{sid:>2}" if sid is not None else " ?"

    if obs is None:
        obs_str = "[ ----.. ----]"
    else:
        o_mn, o_mx = obs
        obs_str = f"[{o_mn:>4d}..{o_mx:>4d}]"

    if raw is None or rad is None:
        return (f"  {joint:<14} {sid_str}  {'---':>5}  "
                f"[{mn:>4d}..{mx:>4d}]  {obs_str}  {'---':>7}  "
                f"[{r_min:>+5.2f}..{r_max:>+5.2f}]  {'-' * 12}")

    # OUT flag if the unwrapped tick falls outside the calibrated band, taking
    # into account the three range types (normal / wrap / inverted).
    out = False
    if mn <= mx:
        out = (raw < mn) or (raw > mx)
    elif (mn - mx) > 2048:                           # genuine wrap
        out = (raw < mn) and (raw > mx)
    else:                                            # non-wrap inverted
        out = (raw > mn) or (raw < mx)
    flag    = f"{_RED}OUT{_RST}" if out else "   "

    # Bar: fraction of (raw - mn)/(mx - mn) for normal, (mn - raw)/(mn - mx) inverted.
    if mn <= mx:
        bar = _bar(raw, mn, mx, width=12)
    elif (mn - mx) > 2048:
        # Map [mn, 4095] U [0, mx] to a single 0..1 range for display.
        span = (4096 - mn) + mx
        if raw >= mn:
            frac_pos = raw - mn
        else:
            frac_pos = (4096 - mn) + raw
        bar = _bar(frac_pos, 0, span, width=12)
    else:
        bar = _bar(raw, mx, mn, width=12)   # reversed

    color = _DIM if direction == 0 else ""
    return (f"  {color}{joint:<14}{_RST} {sid_str}  {raw:>5d}  "
            f"[{mn:>4d}..{mx:>4d}]  {obs_str}  {rad:>+7.3f}  "
            f"[{r_min:>+5.2f}..{r_max:>+5.2f}]  {bar}  {flag}")


def _render(
    leader,
    kind:     str,
    port:     str,
    raw_pos:  Optional[dict[str, int]],
    unw_pos:  Optional[dict[str, int]],
    calib:    dict,
    observed: Optional[dict[str, tuple[int, int]]] = None,
    status:   str = "",
) -> list[str]:
    title = f"  {kind.upper()} Leader Monitor                  port: {port}"
    if not raw_pos:
        title += f"   {_YEL}(no read){_RST}"
    lines = [
        "=" * _W,
        title,
        "=" * _W,
        f"  {'joint':<14} {'ID':>2}  {'ticks':>5}  {'cal_range':>12}  "
        f"{'obs_range':>14}  {'rad':>7}  {'rad_range':>14}  bar          ",
        "  " + "-" * (_W - 2),
    ]
    j_calib = (calib or {}).get("joints", {}) if calib else {}
    observed = observed or {}
    for i, joint in enumerate(leader.JOINTS):
        cal   = j_calib.get(joint, {})
        sid   = cal.get("id", i + 1)
        raw   = raw_pos[joint] if raw_pos else None
        obs   = observed.get(joint)
        # Use unwrapped for radians (continuous across boundary), and
        # calibration math from the leader for the actual displayed rad.
        rad: Optional[float] = None
        if raw_pos is not None and unw_pos is not None and cal:
            mn    = cal.get("min_raw",  0)
            mx    = cal.get("max_raw",  4095)
            r_min = cal.get("rad_min", -3.14)
            r_max = cal.get("rad_max",  3.14)
            u     = unw_pos[joint]
            if mn <= mx:
                span     = mx - mn
                raw_frac = (u - mn) / span if span else 0.5
            elif (mn - mx) > 2048:
                span     = (mn + (4096 - mn) + mx) - mn
                raw_frac = (u - mn) / span if span else 0.5
            else:
                span     = mn - mx
                raw_frac = (mn - u) / span if span else 0.5
            frac = max(0.0, min(1.0, raw_frac))
            rad  = r_min + frac * (r_max - r_min)
        lines.append(_fmt_joint_row(joint, leader.AIZEE_JOINTS[i], sid,
                                    raw, rad, cal, obs))

    calib_path = default_calib_path(kind)
    calib_note = f"calib: {calib_path}" if calib else f"{_YEL}no calibration loaded ({calib_path}){_RST}"
    keys = "Q = quit    R = save observed range as cal limits    X = clear observed"
    if kind == "openrb":
        keys += "    C = center all"
    lines += [
        "  " + "-" * (_W - 2),
        f"  {calib_note}",
        f"  {keys}",
    ]
    if status:
        lines.append(f"  {status}")
    lines.append("=" * _W)
    return lines


def _draw(lines: list[str], n_prev: int = 0, first: bool = False) -> int:
    if not first and n_prev:
        sys.stdout.write(f"\033[{n_prev}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()
    return len(lines)


# ---------------------------------------------------------------------------
# Keyboard
# ---------------------------------------------------------------------------
def _setup_kb():
    """Cross-platform non-blocking single-key reader. Returns getch() callable."""
    if sys.platform == "win32":
        import msvcrt
        def getch():
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    return ch.decode("utf-8", errors="ignore").upper()
                except Exception:
                    return ""
            return ""
        return getch
    # POSIX
    import termios, tty, select
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    def getch():
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).upper()
        return ""
    return getch


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Live monitor for AIZEE leader arms (SO-101 or OpenRB-150)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",   default=None, help="Serial port (auto-detected if omitted)")
    ap.add_argument("--leader", default="auto", choices=("auto", *LEADER_KINDS),
                    help="Which leader arm to use (default: auto)")
    ap.add_argument("--baud",   type=int, default=1_000_000)
    ap.add_argument("--rate",   type=float, default=20.0, help="Update rate (Hz)")
    args = ap.parse_args()

    _ansi_on()

    # Discovery
    port = args.port
    kind = args.leader if args.leader != "auto" else None
    if port is None:
        if args.leader == "auto":
            print("Searching for any leader arm...")
            port, kind = find_any_leader(verbose=True)
        else:
            print(f"Searching for {args.leader}...")
            port, kind = find_any_leader(verbose=True, prefer=args.leader)
            if kind != args.leader:
                port = None
        if port is None:
            print("No leader arm detected.", file=sys.stderr)
            sys.exit(1)
    elif kind is None:
        # Port given but kind=auto — probe to figure out which.
        from leader import identify_port
        kind = identify_port(port)
        if kind is None:
            print(f"Could not identify leader on {port}.", file=sys.stderr)
            sys.exit(1)

    Cls   = get_leader_class(kind)
    calib = default_calib_path(kind)
    print(f"Using {kind} on {port}, calibration: {calib}")
    leader = Cls(port, args.baud, calib=calib)
    if not leader.connect():
        print(f"Failed to connect on {port}", file=sys.stderr)
        sys.exit(1)

    getch  = _setup_kb()
    period = 1.0 / max(1.0, args.rate)
    status_line = ""
    # Per-joint observed (min, max) ticks across this session.  Updated
    # every poll; can be saved to the calibration JSON via 'R' or wiped
    # via 'X' so the user can re-sweep without restarting the script.
    observed: dict[str, tuple[int, int]] = {}

    def _update_observed(raw: Optional[dict[str, int]]) -> None:
        if not raw:
            return
        for j, v in raw.items():
            if j not in observed:
                observed[j] = (v, v)
            else:
                mn, mx = observed[j]
                observed[j] = (min(mn, v), max(mx, v))

    # First draw (no data yet).
    lines = _render(leader, kind, port, None, None, leader._calib, observed)
    n = _draw(lines, first=True)

    try:
        while True:
            ch = getch()
            if ch in ("Q", "\x03"):    # Q or Ctrl-C
                break
            if ch == "C" and kind == "openrb":
                # Sequential centering — one servo at a time so USB inrush
                # stays well under the OpenRB-150's 500 mA limit.  Each
                # call blocks ~1-4s (firmware times out at 4s).
                from openrb_leader import (
                    CENTER_OK, CENTER_STATUS_NAMES,
                )
                results: list[tuple[str, int, int, int]] = []
                for i, joint in enumerate(leader.JOINTS):
                    sid = i + 1
                    status_line = f"{_YEL}centering {joint} (ID={sid})...{_RST}"
                    unw   = leader.read_unwrapped()
                    raw   = {j: v % 4096 for j, v in unw.items()} if unw else None
                    _update_observed(raw)
                    lines = _render(leader, kind, port, raw, unw,
                                    leader._calib, observed, status=status_line)
                    n     = _draw(lines, n_prev=n)
                    st, found_id, pos = leader.center_one(sid)
                    results.append((joint, sid, st, pos))
                ok_count  = sum(1 for _, _, s, _ in results if s == CENTER_OK)
                fail_bits = [
                    f"{j}({CENTER_STATUS_NAMES.get(s, hex(s))}@{p})"
                    for j, _, s, p in results if s != CENTER_OK
                ]
                if not fail_bits:
                    status_line = f"{_GRN}centered all 7 servos{_RST}"
                else:
                    status_line = (f"{_RED}centered {ok_count}/7  failed: "
                                   f"{'; '.join(fail_bits)}{_RST}")
            elif ch == "R":
                # Save observed (min, max) per joint into calibration JSON.
                if not observed:
                    status_line = (f"{_YEL}no positions observed yet — "
                                   f"sweep the arm before pressing R{_RST}")
                else:
                    try:
                        leader.save_limits(observed)
                        status_line = (
                            f"{_GRN}saved observed range to "
                            f"{default_calib_path(kind)} ({len(observed)} joints){_RST}"
                        )
                    except Exception as exc:
                        status_line = f"{_RED}save failed: {exc}{_RST}"
            elif ch == "X":
                # Clear observed extremes so the user can re-sweep cleanly.
                observed.clear()
                status_line = f"{_YEL}observed range cleared — sweep again{_RST}"
            unw   = leader.read_unwrapped()
            raw   = {j: v % 4096 for j, v in unw.items()} if unw else None
            _update_observed(raw)
            lines = _render(leader, kind, port, raw, unw,
                            leader._calib, observed, status=status_line)
            n     = _draw(lines, n_prev=n)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        leader.close()
        # Restore terminal state on POSIX.
        if sys.platform != "win32":
            try:
                import termios
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                                  termios.tcgetattr(sys.stdin.fileno()))
            except Exception:
                pass
        print()


if __name__ == "__main__":
    main()
