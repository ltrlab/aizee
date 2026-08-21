"""
external_encoder.py — pluggable OUTPUT-shaft ground-truth encoder.

The ROBSTRIDE internal encoder (MECHPOS) sits BEFORE the gearbox, so it cannot
see gearbox backlash or the true output angle. Phase 0 therefore needs an
independent absolute encoder on the output shaft. This abstraction lets the
harness run and log ``NaN`` before the rig exists (NullEncoder), then swap to a
real serial encoder (e.g. an AS5048A read by an RP2040/Arduino) with no code
change.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod


class ExternalEncoder(ABC):
    @abstractmethod
    def read_angle_rad(self) -> float:
        """Latest output-shaft angle in radians (NaN if unavailable)."""

    def close(self) -> None:  # optional
        pass


class NullEncoder(ExternalEncoder):
    """Used before the Stage-0 rig is built — always returns NaN so the pipeline
    runs end-to-end and you can validate the CAN + logging path first."""

    def read_angle_rad(self) -> float:
        return math.nan


class SerialEncoder(ExternalEncoder):
    """Reads one float per line ('<angle_rad>\\n') from a serial MCU.

    Recommended firmware contract: sample the AS5048A (14-bit, ~0.022°) on a
    HARDWARE timer at >= 1 kHz and stream the newest reading; do NOT gate on the
    host USB timing. ``offset_rad`` zeroes it at the machined home pin;
    ``invert`` flips sign if the sensor turns opposite the joint.
    """

    def __init__(self, port: str, baud: int = 921600, offset_rad: float = 0.0,
                 invert: bool = False, timeout: float = 0.01) -> None:
        import serial  # lazy import; pyserial only needed on the bench machine
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self.offset_rad = offset_rad
        self.sign = -1.0 if invert else 1.0
        self._last = math.nan

    def read_angle_rad(self) -> float:
        # Drain to the newest line so we log the freshest sample, not a backlog.
        line = None
        while self._ser.in_waiting:
            line = self._ser.readline()
        if line is None:
            line = self._ser.readline()
        text = line.decode(errors="ignore").strip() if line else ""
        if text:
            try:
                self._last = self.sign * (float(text) - self.offset_rad)
            except ValueError:
                pass
        return self._last

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass
