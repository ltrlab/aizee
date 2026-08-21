"""
harness.py — CAN I/O + safe sweep routines + CSV logging for ROBSTRIDE
actuator characterization. Runs headless on the Jetson (python-can / socketcan).

``import can`` is deferred to :class:`MotorBus` so the pure codec/analysis
modules stay importable (and unit-testable) on a machine with no CAN stack.

Safety model: enable -> soft-ramp into the profile -> always disable on exit
(finally + SIGINT/SIGTERM). Position sweeps use low kp/kd by default; torque
sweeps set kp=kd=0. Per-model clamping is handled inside the codec.
"""
from __future__ import annotations

import csv
import math
import signal
import time
from dataclasses import dataclass
from typing import Callable, Optional

from . import robstride_mit as rs
from .external_encoder import ExternalEncoder, NullEncoder

DEFAULT_LOOP_HZ = 200          # command + log rate (feedback arrives ~50 Hz; we oversample)
CmdFn = Callable[[float], "tuple[float, float, float, float, float]"]  # t -> (pos,vel,kp,kd,tau)

LOG_FIELDS = [
    "t", "cmd_pos", "cmd_vel", "cmd_kp", "cmd_kd", "cmd_tau",
    "fb_pos", "fb_vel", "fb_tau", "fb_temp", "fb_mode", "fb_err", "ext_angle",
]


@dataclass
class Joint:
    name: str
    model: str          # "RS00" | "RS02" | "RS03" | "RS04"
    can_id: int
    bus: str = "can1"


class MotorBus:
    """socketcan wrapper speaking the ROBSTRIDE native frame (via robstride_mit)."""

    def __init__(self, joint: Joint):
        import can  # deferred: only needed on the robot
        self.joint = joint
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=joint.bus, bitrate=1_000_000)

    def _send(self, arb: int, data: bytes) -> None:
        self.bus.send(self._can.Message(arbitration_id=arb, is_extended_id=True, data=data))

    def enable(self) -> None:  self._send(*rs.build_enable(self.joint.can_id))
    def disable(self) -> None: self._send(*rs.build_disable(self.joint.can_id))
    def zero(self) -> None:    self._send(*rs.build_zero_pos(self.joint.can_id))
    def save(self) -> None:    self._send(*rs.build_save_config(self.joint.can_id))

    def control(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send(*rs.build_control(self.joint.model, self.joint.can_id, pos, vel, kp, kd, tau))

    def read_feedback(self, timeout: float = 0.001) -> Optional[rs.Feedback]:
        msg = self.bus.recv(timeout=timeout)
        if msg is None or not rs.is_feedback_frame(msg.arbitration_id):
            return None
        try:
            return rs.decode_feedback(msg.arbitration_id, bytes(msg.data), self.joint.model)
        except Exception:
            return None

    def shutdown(self) -> None:
        try:
            self.disable()
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Command profiles: each returns a CmdFn t -> (pos, vel, kp, kd, tau)
# --------------------------------------------------------------------------- #
def step_profile(target: float, kp: float, kd: float, t_step: float = 0.5) -> CmdFn:
    return lambda t: (target if t >= t_step else 0.0, 0.0, kp, kd, 0.0)


def chirp_profile(amp: float, f0: float, f1: float, dur: float, kp: float, kd: float) -> CmdFn:
    """Logarithmic (exponential) sine chirp f0->f1 over ``dur`` seconds."""
    k = (f1 / f0) ** (1.0 / dur) if f1 > f0 else 1.0
    lnk = math.log(k) if k != 1.0 else 1.0

    def fn(t):
        phase = 2 * math.pi * f0 * ((k ** t - 1.0) / lnk) if k != 1.0 else 2 * math.pi * f0 * t
        return (amp * math.sin(phase), 0.0, kp, kd, 0.0)

    return fn


def triangle_profile(amp: float, period: float, kp: float, kd: float) -> CmdFn:
    """Slow triangle across a reversal — for backlash. Keep ``period`` large."""
    def fn(t):
        frac = (t % period) / period
        tri = 4.0 * abs(frac - 0.5) - 1.0           # -1 .. 1
        return (amp * tri, 0.0, kp, kd, 0.0)
    return fn


def torque_ramp_profile(tau_max: float, dur: float) -> CmdFn:
    """kp=kd=0, ramp commanded torque 0->tau_max — for stiction/breakaway + Kt."""
    return lambda t: (0.0, 0.0, 0.0, 0.0, tau_max * min(1.0, t / dur))


# --------------------------------------------------------------------------- #
# Runner
# --------------------------------------------------------------------------- #
def run_profile(bus: MotorBus, cmd_fn: CmdFn, duration: float,
                ext: Optional[ExternalEncoder] = None, loop_hz: int = DEFAULT_LOOP_HZ,
                settle_s: float = 1.0, csv_path: Optional[str] = None,
                clock: Callable[[], float] = time.perf_counter,
                sleep: Callable[[float], None] = time.sleep) -> list[dict]:
    """Enable, hold at the profile's t=0 command for ``settle_s`` (soft start),
    run ``cmd_fn`` for ``duration`` at ``loop_hz`` while logging, then disable.
    Always disables on exception / SIGINT. ``clock``/``sleep`` are injectable so
    the loop can be dry-run in tests without real time."""
    ext = ext or NullEncoder()
    rows: list[dict] = []
    dt = 1.0 / loop_hz
    stop = {"v": False}

    prev = signal.getsignal(signal.SIGINT)

    def _sig(_signum, _frame):
        stop["v"] = True

    try:
        signal.signal(signal.SIGINT, _sig)
    except (ValueError, TypeError):
        prev = None  # not in main thread (e.g. tests) — skip handler

    try:
        bus.enable()
        # Soft start: hold the profile's initial command so we don't jerk on enable.
        p0, v0, kp0, kd0, tau0 = cmd_fn(0.0)
        t0 = clock()
        while clock() - t0 < settle_s and not stop["v"]:
            bus.control(p0, v0, kp0, kd0, tau0)
            bus.read_feedback()
            sleep(dt)

        fb: Optional[rs.Feedback] = None
        t0 = clock()
        i = 0
        while not stop["v"]:
            t = clock() - t0
            if t >= duration:
                break
            pos, vel, kp, kd, tau = cmd_fn(t)
            bus.control(pos, vel, kp, kd, tau)
            got = bus.read_feedback()
            if got is not None:
                fb = got
            ext_angle = ext.read_angle_rad()
            rows.append(dict(
                t=t, cmd_pos=pos, cmd_vel=vel, cmd_kp=kp, cmd_kd=kd, cmd_tau=tau,
                fb_pos=(fb.position if fb else math.nan),
                fb_vel=(fb.velocity if fb else math.nan),
                fb_tau=(fb.torque if fb else math.nan),
                fb_temp=(fb.temperature if fb else math.nan),
                fb_mode=(int(fb.mode) if fb else -1),
                fb_err=(fb.error_bits if fb else -1),
                ext_angle=ext_angle,
            ))
            i += 1
            nap = (t0 + i * dt) - clock()
            if nap > 0:
                sleep(nap)
    finally:
        bus.shutdown()
        if prev is not None:
            try:
                signal.signal(signal.SIGINT, prev)
            except (ValueError, TypeError):
                pass

    if csv_path:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
            w.writeheader()
            w.writerows(rows)
    return rows
