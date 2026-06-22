"""serial_safe.py — open a serial port without ever blocking forever.

pyserial's ``Serial(..., timeout=...)`` only bounds *read* calls.  The
constructor itself (open + termios / CommState reconfigure) can block
indefinitely on some devices:

  * Bluetooth SPP / RFCOMM ports — a paired BT speaker shows up as a
    "serial over Bluetooth link" COM/tty; opening it with no responsive peer
    hangs (tens of seconds on Windows, potentially forever on Linux RFCOMM).
  * USB modems / TTYs that block on carrier-detect (DCD).
  * Half-dead USB-serial adapters caught mid-reset.

Leader-arm discovery probes *every* enumerated serial port, so a single such
device would otherwise freeze the whole application before its GUI can launch.
This helper runs the blocking open in a throwaway daemon thread and abandons it
if it overruns, so one unresponsive port can never gate startup.

This is deliberately dependency-free (stdlib + pyserial) so both leader driver
modules can share it without import-order coupling.
"""

from __future__ import annotations

import threading

try:
    import serial
except ImportError:                       # pragma: no cover - probed by callers
    serial = None  # type: ignore

# Default ceiling for a single open().  Real leader adapters (CH340 / OpenRB
# USB-CDC) open in well under a second; 2s is generous headroom.
DEFAULT_OPEN_TIMEOUT = 2.0


def open_serial(device: str, baud: int, *, read_timeout: float,
                open_timeout: float = DEFAULT_OPEN_TIMEOUT,
                write_timeout: float = 1.0):
    """Open ``device`` but block no longer than ``open_timeout`` seconds.

    Returns an open ``serial.Serial``.  Raises:
      * ``TimeoutError``      — the open did not complete in time (unresponsive
                                device, e.g. a Bluetooth serial port);
      * ``serial.SerialException`` / ``OSError`` — the underlying open failed;
      * ``RuntimeError``      — pyserial is not installed.

    A ``write_timeout`` is always set so a subsequent ``write()`` on a port that
    opened but then went unresponsive can't block either.
    """
    if serial is None:
        raise RuntimeError("pyserial not installed")

    box: dict = {}
    abandoned = threading.Event()

    def _worker() -> None:
        try:
            s = serial.Serial(device, baud, timeout=read_timeout,
                              write_timeout=write_timeout)
        except BaseException as exc:       # noqa: BLE001 — relayed to caller
            box["err"] = exc
            return
        if abandoned.is_set():
            # Caller already gave up waiting; don't leak the handle.
            try:
                s.close()
            except Exception:
                pass
        else:
            box["ser"] = s

    t = threading.Thread(target=_worker, daemon=True,
                        name=f"serial-open:{device}")
    t.start()
    t.join(open_timeout)

    if t.is_alive():
        # The open is still stuck in a blocking syscall.  Abandon the daemon
        # thread (it will close the port if/when the syscall ever returns) and
        # let the caller move on — launch is never gated on one bad device.
        abandoned.set()
        raise TimeoutError(
            f"opening {device} timed out after {open_timeout:.1f}s "
            "(unresponsive device — e.g. a Bluetooth serial port?)")
    if "err" in box:
        raise box["err"]
    return box["ser"]
