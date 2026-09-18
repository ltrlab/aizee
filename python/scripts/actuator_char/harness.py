"""
harness.py — CAN I/O + safe sweep routines + CSV logging for ROBSTRIDE
actuator characterization. Runs headless on the Jetson (python-can / socketcan).

``import can`` is deferred to :class:`MotorBus` so the pure codec/analysis
modules stay importable (and unit-testable) on a machine with no CAN stack.

Safety model: enable -> soft-ramp into the profile -> always disable on exit
(finally + SIGINT; abrupt termination cannot guarantee delivery). A verified
device envelope bounds commands, slew rate, and feedback before each write.
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

DEFAULT_LOOP_HZ = 200          # command rate; feedback sample rate is measured separately
CmdFn = Callable[[float], "tuple[float, float, float, float, float]"]  # t -> (pos,vel,kp,kd,tau)

LOG_FIELDS = [
    "t", "cmd_pos", "cmd_vel", "cmd_kp", "cmd_kd", "cmd_tau",
    "fb_pos", "fb_vel", "fb_tau", "fb_temp", "fb_mode", "fb_err", "ext_angle",
    "fb_sequence", "fb_received_at", "fb_t", "fb_age_s", "fb_new", "ext_received_at", "ext_sequence",
]


@dataclass
class Joint:
    name: str
    model: str          # "RS00" | "RS02" | "RS03" | "RS04"
    can_id: int
    bus: str = "can1"


class MotorBus:
    """socketcan wrapper speaking the ROBSTRIDE native frame (via robstride_mit)."""

    def __init__(self, joint: Joint, *, profile=None, trace=None):
        from robot_platform.characterization.audit import validate_bench_profile
        self.limits = validate_bench_profile(profile or {}, joint)
        self.trace = trace
        self.sequence = 0
        import can  # deferred: only needed on the robot
        self.joint = joint
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=joint.bus, bitrate=1_000_000)

    def _send(self, arb: int, data: bytes) -> None:
        if self.trace:
            self.trace.record("can_tx_submit", {"bus": self.joint.bus, "arbitration_id": arb, "data_hex": data.hex()})
        self.bus.send(self._can.Message(arbitration_id=arb, is_extended_id=True, data=data))

    def enable(self) -> None:  self._send(*rs.build_enable(self.joint.can_id))
    def disable(self) -> None: self._send(*rs.build_disable(self.joint.can_id))
    def zero(self) -> None:    self._send(*rs.build_zero_pos(self.joint.can_id))
    def save(self) -> None:    self._send(*rs.build_save_config(self.joint.can_id))

    def control(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send(*rs.build_control(self.joint.model, self.joint.can_id, pos, vel, kp, kd, tau))

    def read_feedback(self, timeout: float = 0.001) -> Optional[rs.Feedback]:
        latest = None
        deadline = time.perf_counter() + timeout
        for _ in range(256):
            msg = self.bus.recv(timeout=max(0.0, deadline - time.perf_counter()) if latest is None else 0.0)
            if msg is None:
                break
            received_at = time.perf_counter()
            if self.trace:
                self.trace.record("can_rx", {"bus": self.joint.bus, "arbitration_id": msg.arbitration_id,
                                  "data_hex": bytes(msg.data).hex(), "kernel_timestamp": msg.timestamp}, time_s=received_at)
            if (not msg.is_extended_id or not rs.is_feedback_frame(msg.arbitration_id)
                    or ((msg.arbitration_id >> 8) & 0xFF) != self.joint.can_id):
                continue
            try:
                latest = rs.decode_feedback(msg.arbitration_id, bytes(msg.data), self.joint.model)
            except ValueError:
                continue
            self.sequence += 1
            latest.received_at, latest.sequence = received_at, self.sequence
            latest.kernel_timestamp, latest.raw_hex = msg.timestamp, bytes(msg.data).hex()
        return latest

    def shutdown(self) -> None:
        try:
            self.disable()
        except Exception as exc:
            print(f"[bench] disable submission failed: {exc}")
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
    """Slow reversal trajectory; backlash inference needs independent sensors."""
    def fn(t):
        frac = (t % period) / period
        tri = 4.0 * abs(frac - 0.5) - 1.0           # -1 .. 1
        return (amp * tri, 0.0, kp, kd, 0.0)
    return fn


def torque_ramp_profile(tau_max: float, dur: float) -> CmdFn:
    """kp=kd=0, ramp drive torque demand; this does not measure torque constant."""
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
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(loop_hz) or loop_hz <= 0:
        raise ValueError("positive finite duration and loop_hz required")
    limits = getattr(bus, "limits", None)
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
        origin = 0.0
        if limits:
            initial = bus.read_feedback(timeout=limits["feedback_timeout_s"])
            if initial is None or initial.error_bits:
                raise RuntimeError("fresh fault-free feedback required before a bench trajectory")
            origin = initial.position
            if not limits["min_position"] <= origin <= limits["max_position"]:
                raise RuntimeError("initial pose is outside the verified bench envelope")
        last_pos = origin

        def send(values):
            nonlocal last_pos
            pos, vel, kp, kd, tau = values
            pos += origin
            if limits:
                if (not all(math.isfinite(v) for v in (pos, vel, kp, kd, tau))
                        or not limits["min_position"] <= pos <= limits["max_position"]
                        or abs(vel) > limits["max_velocity"] or abs(tau) > limits["max_feedforward"]
                        or not 0 <= kp <= limits["max_kp"] or not 0 <= kd <= limits["max_kd"]):
                    raise ValueError("trajectory exceeds verified bench envelope")
                delta = limits["max_velocity"] * dt
                pos = max(last_pos - delta, min(last_pos + delta, pos))
            bus.control(pos, vel, kp, kd, tau)
            last_pos = pos
            return pos, vel, kp, kd, tau

        def check_feedback(feedback):
            if limits and (feedback is None or feedback.error_bits or
                           feedback.received_at is None or
                           not all(math.isfinite(v) for v in (feedback.position, feedback.velocity, feedback.torque, feedback.received_at)) or
                           clock() - feedback.received_at > limits["feedback_timeout_s"] or
                           not limits["min_position"] <= feedback.position <= limits["max_position"] or
                           abs(feedback.torque) > limits["max_reported_torque"] or
                           abs(feedback.velocity) > limits["max_velocity"]):
                raise RuntimeError("bench feedback lost, faulted, or outside verified envelope")
        # Soft start: hold the profile's initial command so we don't jerk on enable.
        fb = initial if limits else None
        t0 = clock()
        while clock() - t0 < settle_s and not stop["v"]:
            check_feedback(fb)
            send(cmd_fn(0.0))
            fb = bus.read_feedback() or fb
            check_feedback(fb)
            sleep(dt)

        fb_time = fb.received_at if fb and fb.received_at is not None else None
        fb_sequence = fb.sequence if fb else 0
        t0 = clock()
        i = 0
        while not stop["v"]:
            t = clock() - t0
            if t >= duration:
                break
            check_feedback(fb)
            pos, vel, kp, kd, tau = send(cmd_fn(t))
            got = bus.read_feedback()
            if got is not None:
                fb = got
                fb_time = got.received_at if got.received_at is not None else clock()
                fb_sequence = got.sequence or fb_sequence + 1
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
                fb_sequence=fb_sequence, fb_received_at=fb_time,
                fb_t=fb_time - t0 if fb_time is not None else math.nan,
                fb_age_s=clock() - fb_time if fb_time is not None else math.nan,
                fb_new=got is not None,
                ext_received_at=getattr(ext, "received_at", None), ext_sequence=getattr(ext, "sequence", None),
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
