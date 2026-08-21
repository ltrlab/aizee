"""
actuator_char — Phase-0 actuator characterization harness for Minerva's
ROBSTRIDE joints (RS00/RS02/RS03/RS04) over Jetson CAN.

Modules:
  robstride_mit    faithful Python port of the ROBSTRIDE MIT-mode CAN codec
                   (mirrors rust/motor_control/src/robstride.rs — the deployed,
                   RS03-EN-spec driver; NOT sine_wave_test.py's AK packing).
  external_encoder pluggable output-shaft ground-truth encoder (Null / Serial).
  analysis         capability-sheet metrics (pure numpy, unit-tested off-robot).
  harness          python-can I/O + safe sweep routines + CSV logging.

The CLI entry point is ``python/scripts/characterize_actuator.py``.
"""
from __future__ import annotations

from . import analysis, external_encoder, robstride_mit  # noqa: F401

__all__ = ["robstride_mit", "external_encoder", "analysis"]
