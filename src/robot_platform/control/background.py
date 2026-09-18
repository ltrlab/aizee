"""Worker-owned ZMQ sockets with bounded commands and freshness-gated keepalive."""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
import math
import logging
import threading
import time
from typing import Any

from .session import RobotSession, ZmqRobotClient, route_joint_target

_LOG = logging.getLogger(__name__)


class ThreadedRobotClient:
    """Shared background transport for collector applications.

    Construction is offline; ``start`` creates all sockets on one worker. Motion
    submissions coalesce per endpoint, service commands have a bounded FIFO, and
    services run before pending motion. ``set_target`` opts into 100 Hz keepalive;
    ``submit`` sends once. Every actual arm send checks fresh position telemetry.
    Stale held targets are forgotten, so recovery requires a new submission.

    Public snapshot/read/submit methods serialize access to ``session``. Callers
    must not directly mutate the session after the worker has started.
    """

    def __init__(self, session: RobotSession, *, context: Any = None, reemit_hz: float = 100,
                 receive_only: bool = False, max_pending_controls: int = 256,
                 service_timeout_s: float = 1.0) -> None:
        if isinstance(reemit_hz, bool) or not math.isfinite(reemit_hz) or reemit_hz <= 0:
            raise ValueError("reemit_hz must be positive and finite")
        if not isinstance(max_pending_controls, int) or max_pending_controls < 1:
            raise ValueError("max_pending_controls must be a positive integer")
        if not math.isfinite(service_timeout_s) or service_timeout_s <= 0:
            raise ValueError("service_timeout_s must be positive and finite")
        self.session = session
        self._client = ZmqRobotClient(session, context=context, receive_only=receive_only)
        self._period = 1.0 / reemit_hz
        self._max_controls = max_pending_controls
        self._condition = threading.Condition(threading.RLock())
        self._controls: deque[tuple[str, dict, float]] = deque()
        self._service_timeout_s = service_timeout_s
        self._delivery_error: Exception | None = None
        self._pending_motion: dict[str, dict] = {}
        self._pending_target: tuple[dict[str, dict], bool] | None = None
        self._held: dict[str, tuple[dict, bool]] = {}
        self._motion_deadlines: dict[str, float] = {}
        self._target_deadline = 0.0
        self._blocked: set[str] = set()
        self._estopped: set[str] = set()
        self._closing = False
        self._close_deadline: float | None = None
        self._thread: threading.Thread | None = None
        self._started = threading.Event()
        self._startup_error: Exception | None = None
        self._error: Exception | None = None

    def start(self) -> ThreadedRobotClient:
        with self._condition:
            if self._closing:
                raise RuntimeError("client is closed")
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, daemon=True, name="RobotSessionIO")
                self._thread.start()
        if not self._started.wait(timeout=5.0):
            raise RuntimeError("robot transport worker did not start")
        if self._startup_error is not None:
            raise RuntimeError("robot transport worker could not connect") from self._startup_error
        return self

    @property
    def error(self) -> Exception | None:
        with self._condition:
            return self._error

    def _check_writable(self) -> None:
        if self._closing:
            raise RuntimeError("client is closed")
        if self._client.receive_only:
            raise RuntimeError("receive-only clients cannot send commands")

    def _drop_arm_motion(self, arm: str, *, drive: bool = False) -> None:
        self._held.pop(arm, None)
        if self._pending_target is not None:
            self._pending_target[0].pop(arm, None)
        command = self._pending_motion.get(arm)
        if command is None:
            return
        if not drive and command["type"] == "bundle" and "drive" in command:
            self._pending_motion[arm] = {"type": "bundle", "drive": command["drive"]}
        elif drive or command["type"] in ("arm_joints", "bundle"):
            self._pending_motion.pop(arm, None)

    @staticmethod
    def _drive_stop(command: dict) -> dict | None:
        drive = command if command["type"] == "drive" else command.get("drive") if command["type"] == "bundle" else None
        if drive is not None and drive.get("linear") == 0.0 and drive.get("angular") == 0.0:
            return {"type": "drive", **drive}
        return None

    @classmethod
    def _is_control(cls, command: dict) -> bool:
        return command["type"] not in ("arm_joints", "bundle", "drive") or (
            cls._drive_stop(command) is not None and "arm_joints" not in command)

    def submit(self, commands: Mapping[str, Mapping[str, Any]]) -> None:
        """Queue validated wire commands; no socket is touched by the caller."""
        with self._condition:
            self._check_writable()
            messages = self.session.validate_commands(commands)
            services = sum(self._is_control(msg) for msg in messages.values())
            if len(self._controls) + services > self._max_controls:
                raise BufferError("robot service command queue is full")
            for arm, command in messages.items():
                kind = command["type"]
                if not self._is_control(command):
                    # Gate when sending, after earlier enable/disable messages.
                    self._pending_motion[arm] = command
                    self._motion_deadlines[arm] = time.monotonic() + self.session.target_lease_s
                    if kind == "arm_joints" or kind == "bundle" and "arm_joints" in command:
                        self._held.pop(arm, None)
                        if self._pending_target is not None:
                            self._pending_target[0].pop(arm, None)
                    continue
                touches_arm = bool(set(command.get("motor_ids", ())).intersection(self.session.arms[arm].joints))
                if kind == "disable" and touches_arm or kind == "emergency_stop":
                    self._blocked.add(arm)
                    self._drop_arm_motion(arm, drive=kind == "emergency_stop")
                if kind == "emergency_stop":
                    self._estopped.add(arm)
                elif kind == "enable" and touches_arm:
                    self._blocked.add(arm)  # release only after the enable frame is accepted
                self._controls.append((arm, command, time.monotonic() + self._service_timeout_s))
            self._condition.notify_all()

    def set_target(self, positions, *, allow_partial: bool = True, require_torque: bool = False, **kwargs) -> None:
        """Lease a target; producers must renew within session.target_lease_s.

        A telemetry update or worker repeat never renews the producer's lease.
        Expiry forgets motion; the controller's command-timeout policy still applies.
        """
        with self._condition:
            self._check_writable()
            commands = route_joint_target(self.session.spec, self.session.arms, positions,
                                          allow_partial=allow_partial, **kwargs)
            for arm in self.session.arms:
                self._drop_arm_motion(arm)
            self._pending_target = (commands, require_torque)
            self._target_deadline = time.monotonic() + self.session.target_lease_s
            self._condition.notify_all()

    def clear_target(self, *, drop_drive: bool = False) -> None:
        """Forget queued/held arm motion; optionally drop pending nonzero drive too.

        Explicit drive stops and service commands remain queued for delivery.
        """
        with self._condition:
            self._pending_target = None
            self._held.clear()
            for arm in self.session.arms:
                self._drop_arm_motion(arm, drive=drop_drive)
            self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            return self.session.observation()

    def joint_field(self, field: str):
        with self._condition:
            return self.session.joint_field(field)

    def arm_ready(self, field: str = "position"):
        with self._condition:
            return self.session.arm_ready(field)

    def telemetry(self, arm: str, *, fresh_only: bool = True):
        with self._condition:
            return self.session.telemetry(arm, fresh_only=fresh_only)

    def telemetry_sample(self, arm: str):
        with self._condition:
            return self.session.telemetry_sample(arm)

    def _allowed(self, arm: str, command: dict, *, require_torque: bool = False) -> dict | None:
        if arm in self._estopped:
            return None
        telemetry = self.session.telemetry(arm)
        if telemetry is not None and telemetry.get("emergency_stop") is True:
            self._blocked.add(arm)
            return None
        commands = self.session.prepare_commands({arm: command}, require_torque=require_torque)
        result = commands.get(arm)
        if result is not None and arm in self._blocked:
            if result["type"] == "arm_joints":
                return None
            if result["type"] == "bundle":
                result.pop("arm_joints", None)
                if "drive" not in result:
                    return None
        return result

    def _send(self, arm: str, command: dict) -> bool:
        try:
            self._client.send_commands({arm: command})
            return True
        except Exception as exc:
            self._error = exc  # retain visibility; failure on one arm cannot starve another
            return False

    def _drain_controls(self) -> set[str]:
        """Retry backpressure without replaying accepted commands or starving peers."""
        blocked = set()
        retained = deque()
        for arm, command, deadline in self._controls:
            if self._close_deadline is not None:
                deadline = min(deadline, self._close_deadline)
            touches_arm = bool(set(command.get("motor_ids", ())).intersection(self.session.arms[arm].joints))
            if command["type"] == "disable" and touches_arm or command["type"] == "emergency_stop":
                self._blocked.add(arm)
            if command["type"] == "emergency_stop":
                self._estopped.add(arm)
            if arm not in blocked and self._send(arm, command):
                if command["type"] == "enable" and touches_arm:
                    self._blocked.discard(arm)
                elif command["type"] == "clear_emergency_stop":
                    self._estopped.discard(arm)
                continue
            blocked.add(arm)
            if time.monotonic() >= deadline:
                failure = RuntimeError(f"Could not send {command['type']} to {arm} before its delivery deadline")
                self._delivery_error = self._error = failure
                _LOG.error("%s", failure)
            else:
                retained.append((arm, command, deadline))
        self._controls = retained
        return blocked

    def _step(self) -> None:
        """One worker cycle; called under the condition lock (also used by offline tests)."""
        try:
            self._client.poll()
        except Exception as exc:
            self._error = exc
        pending_controls = self._drain_controls()
        if self._closing:
            self._held.clear()
            self._pending_target = None
            self._pending_motion.clear()
            return
        if time.monotonic() >= self._target_deadline:
            self._pending_target = None
            self._held.clear()
        if self._pending_target is not None:
            commands, require_torque = self._pending_target
            self._held = {arm: (cmd, require_torque) for arm, cmd in commands.items()}
            self._pending_target = None
        pending, self._pending_motion = self._pending_motion, {}
        for arm, command in pending.items():
            if time.monotonic() >= self._motion_deadlines.get(arm, 0.0):
                stop = self._drive_stop(command)
                if stop is not None:
                    self._send(arm, stop)
                continue
            if arm in pending_controls:
                self._pending_motion[arm] = command
                continue
            allowed = self._allowed(arm, command)
            if allowed is not None:
                self._send(arm, allowed)
        for arm, (command, require_torque) in list(self._held.items()):
            allowed = self._allowed(arm, command, require_torque=require_torque)
            if allowed is None:
                self._held.pop(arm, None)
            elif arm not in pending_controls:
                self._send(arm, allowed)

    def _run(self) -> None:
        try:
            self._client.connect()
        except Exception as exc:
            self._startup_error = self._error = exc
            self._started.set()
            return
        self._started.set()
        try:
            while True:
                started = time.monotonic()
                with self._condition:
                    self._step()
                    if self._closing and not self._controls:
                        break
                    self._condition.wait(timeout=max(0.0, self._period - (time.monotonic() - started)))
        except Exception as exc:
            with self._condition:
                self._error = exc
                self._closing = True
        finally:
            self._client.close()

    def close(self, *, timeout: float = 2.0) -> None:
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("close timeout must be positive and finite")
        with self._condition:
            if not self._closing:
                # Keep a last explicit drive stop from a combined motion frame,
                # while never sending its held arm/nonzero drive during shutdown.
                for arm, command in self._pending_motion.items():
                    stop = self._drive_stop(command)
                    if stop is not None:
                        self._controls.append((arm, stop, time.monotonic() + self._service_timeout_s))
                # Shutdown must not newly enable motors or perform calibration.
                self._controls = deque((arm, command, deadline) for arm, command, deadline in self._controls
                                       if command["type"] in ("disable", "emergency_stop")
                                       or self._drive_stop(command) is not None)
            self._closing = True
            self._close_deadline = time.monotonic() + timeout * 0.9
            self._condition.notify_all()
            worker = self._thread
        if worker is not None:
            worker.join(timeout=timeout)
            if worker.is_alive():
                raise RuntimeError("robot transport worker did not stop")
        elif self._controls:
            raise RuntimeError("Cannot deliver queued controls: robot transport was never started")
        if self._delivery_error is not None:
            raise RuntimeError("Robot service command delivery failed") from self._delivery_error

    def __enter__(self) -> ThreadedRobotClient:
        return self.start()

    def __exit__(self, *_: Any) -> None:
        self.close()
