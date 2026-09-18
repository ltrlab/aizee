"""Shared, explicit session for single- and dual-arm robots.

The routing and observation APIs use only the standard library. A session never
opens a socket. ``ZmqRobotClient.connect()`` opts into networking; callers own the
poll/send cadence. There is no enable, background resend, position hold, clipping,
or limit enforcement here. Applications retain responsibility for those policies.

Joint vectors follow the robot definition; each endpoint's ``joints`` lists the
Rust controller's positional wire order. Telemetry uses Rust's ``motors`` map.
Receive times use a local monotonic clock, independent of robot clock offsets.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
import math
import time
from types import MappingProxyType
from typing import Any
from .freshness import SourceFreshness


class StaleTelemetryError(ValueError):
    """An active arm lacks fresh, complete position and torque telemetry."""


class PartialSendError(RuntimeError):
    """A transport send failed; earlier arms may already have received commands."""

    def __init__(self, sent_arms: Sequence[str], failed_arm: str) -> None:
        self.sent_arms = tuple(sent_arms)
        self.failed_arm = failed_arm
        super().__init__(f"Send failed for {failed_arm!r}; already sent: {self.sent_arms!r}")


@dataclass(frozen=True)
class ArmEndpoint:
    command: str
    telemetry: str
    joints: tuple[str, ...]


@dataclass(frozen=True)
class ArmFreshness:
    received_at: float | None
    age_s: float
    fresh: bool
    complete: bool
    source_known: bool = False


@dataclass(frozen=True)
class Observation:
    """Immutable canonical vectors; missing/stale/inactive readings are NaN.

    ``valid_mask`` requires both measured position and torque. Field-specific
    masks let a display use a valid position even if torque is malformed.
    ``ready`` concerns active joints only; inactive placeholders are never valid.
    """

    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    torques: tuple[float, ...]
    active_mask: tuple[bool, ...]
    position_valid_mask: tuple[bool, ...]
    torque_valid_mask: tuple[bool, ...]
    valid_mask: tuple[bool, ...]
    arm_freshness: Mapping[str, ArmFreshness]

    @property
    def ready(self) -> bool:
        return all(valid for active, valid in zip(self.active_mask, self.valid_mask) if active)

    @property
    def position_ready(self) -> bool:
        return all(valid for active, valid in zip(self.active_mask, self.position_valid_mask) if active)


def _number(value: Any) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise ValueError("expected a finite numeric value")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("expected a finite numeric value") from exc
    if not math.isfinite(result):
        raise ValueError("expected a finite numeric value")
    return result


def _layout(spec: Any, arms: Mapping[str, Any]) -> tuple[tuple[str, ...], tuple[bool, ...], dict[str, ArmEndpoint]]:
    names = tuple(joint.name for joint in spec.joints)
    active = tuple(joint.active for joint in spec.joints)
    if not names or len(set(names)) != len(names):
        raise ValueError("Robot joint names must be nonempty and unique")
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("Robot joint names must be nonempty strings")
    if any(not isinstance(value, bool) for value in active):
        raise ValueError("Each joint must declare a boolean active flag")
    if not isinstance(arms, Mapping) or not arms:
        raise ValueError("At least one arm endpoint is required")
    endpoints: dict[str, ArmEndpoint] = {}
    assigned: set[str] = set()
    active_names = {name for name, enabled in zip(names, active) if enabled}
    for arm, values in arms.items():
        if not isinstance(arm, str) or not arm:
            raise ValueError("Arm names must be nonempty strings")
        if isinstance(values, ArmEndpoint):
            endpoint = ArmEndpoint(values.command, values.telemetry, tuple(values.joints))
        else:
            if not isinstance(values, Mapping):
                raise ValueError(f"Invalid endpoint for arm {arm!r}")
            raw_joints = values.get("joints")
            if not isinstance(raw_joints, (list, tuple)):
                raise ValueError(f"Arm {arm!r} must list its joints in wire order")
            endpoint = ArmEndpoint(values.get("command"), values.get("telemetry"), tuple(raw_joints))
        if any(not isinstance(url, str) or not url for url in (endpoint.command, endpoint.telemetry)):
            raise ValueError(f"Arm {arm!r} requires command and telemetry endpoints")
        if not endpoint.joints or any(not isinstance(name, str) for name in endpoint.joints):
            raise ValueError(f"Arm {arm!r} requires named joints")
        if len(set(endpoint.joints)) != len(endpoint.joints):
            raise ValueError(f"Duplicate joint in arm {arm!r}")
        if set(endpoint.joints) - active_names:
            raise ValueError(f"Arm {arm!r} contains unknown or inactive joints")
        if assigned.intersection(endpoint.joints):
            raise ValueError("A joint cannot belong to multiple arm endpoints")
        assigned.update(endpoint.joints)
        endpoints[arm] = endpoint
    if assigned != active_names:
        raise ValueError(f"Active joints without an endpoint: {sorted(active_names - assigned)}")
    for kind in ("command", "telemetry"):
        addresses = [getattr(endpoint, kind) for endpoint in endpoints.values()]
        if len(set(addresses)) != len(addresses):
            raise ValueError(f"Each arm requires a distinct {kind} endpoint")
    return names, active, endpoints


def _vector(label: str, values: Sequence[float], names: tuple[str, ...], active: tuple[bool, ...], *, allow_partial: bool = False) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} must be a joint vector")
    try:
        raw = tuple(values)
        result = tuple(float(value) if allow_partial and not isinstance(value, (bool, str, bytes)) else _number(value) for value in raw)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain finite numbers") from exc
    if len(result) != len(names):
        raise ValueError(f"{label} requires {len(names)} values, received {len(result)}")
    for name, enabled, value in zip(names, active, result):
        if not enabled and value != 0.0:
            raise ValueError(f"Cannot command inactive joint {name!r} in {label}")
    if label in ("kp", "kd") and any(value < 0.0 for value in result):
        raise ValueError(f"{label} cannot contain negative gains")
    return result


def route_joint_target(
    spec: Any,
    arms: Mapping[str, Any],
    positions: Sequence[float],
    *,
    velocities: Sequence[float] | None = None,
    kp: Sequence[float] | None = None,
    kd: Sequence[float] | None = None,
    torques: Sequence[float] | None = None,
    allow_partial: bool = False,
) -> dict[str, dict[str, Any]]:
    """Build independent Rust ``arm_joints`` payloads, without I/O or mutation.

    Every supplied vector uses canonical robot order. Inactive entries must be
    zero; they are never transmitted. Omitted velocities become zero; omitted
    gains/torques remain absent so the existing Rust defaults apply.
    """
    names, active, endpoints = _layout(spec, arms)
    vectors = {"positions": _vector("positions", positions, names, active, allow_partial=allow_partial)}
    if velocities is None:
        velocities = (0.0,) * len(names)
    for label, values in (("velocities", velocities), ("kp", kp), ("kd", kd), ("torques", torques)):
        if values is not None:
            vectors[label] = _vector(label, values, names, active, allow_partial=allow_partial)
    indices = {name: index for index, name in enumerate(names)}
    return {
        arm: {
            "type": "arm_joints",
            **{label: [values[indices[name]] for name in endpoint.joints] for label, values in vectors.items()},
        }
        for arm, endpoint in endpoints.items()
        if all(math.isfinite(values[indices[name]]) for values in vectors.values() for name in endpoint.joints)
    }


class RobotSession:
    """Pure in-memory routing and observation state for any robot definition."""

    def __init__(self, spec: Any, settings: Mapping[str, Any], *, max_age_s: float | None = None) -> None:
        arms = settings.get("endpoints", {}).get("arms", {})
        self.joint_names, self.active_mask, endpoints = _layout(spec, arms)
        self.spec = spec
        self.arms = MappingProxyType(endpoints)
        if max_age_s is None:
            max_age_s = settings.get("control", {}).get("stale_after_s", 0.5)
        self.max_age_s = _number(max_age_s)
        if self.max_age_s <= 0:
            raise ValueError("max_age_s must be greater than zero")
        self._samples: dict[str, tuple[float, tuple[float, ...], tuple[float, ...]]] = {}
        self._messages: dict[str, dict[str, Any]] = {}
        control = settings.get("control", {})
        self.require_source_freshness = control.get("require_source_freshness", False)
        if not isinstance(self.require_source_freshness, bool):
            raise ValueError("require_source_freshness must be boolean")
        self.target_lease_s = _number(control.get("target_lease_s", 0.5))
        if not 0 < self.target_lease_s <= 60:
            raise ValueError("target_lease_s must be positive and at most 60")
        self._source_trackers = {arm: SourceFreshness() for arm in self.arms}
        self._source_origins = {}

    def ingest_telemetry(self, arm: str, message: Mapping[str, Any], *, received_at: float | None = None) -> None:
        """Snapshot a Rust motors map; missing or invalid fields become NaN.

        Malformed latest telemetry replaces previous readings with invalid values
        rather than leaving an apparently healthy old arm in the cache.
        ``received_at`` is a monotonic timestamp, never a remote wall clock.
        """
        endpoint = self.arms[arm]
        received_at = _number(time.monotonic() if received_at is None else received_at)
        if arm in self._samples and received_at < self._samples[arm][0]:
            raise ValueError(f"Out-of-order receive time for arm {arm!r}")
        motors = message.get("motors", {}) if isinstance(message, Mapping) else {}
        if not isinstance(motors, Mapping):
            motors = {}

        def field_values(field: str) -> tuple[float, ...]:
            values = []
            for name in endpoint.joints:
                motor = motors.get(name)
                try:
                    value = _number(motor.get(field)) if isinstance(motor, Mapping) else math.nan
                except ValueError:
                    value = math.nan
                values.append(value)
            return tuple(values)

        self._samples[arm] = (received_at, field_values("position"), field_values("torque"))
        self._messages[arm] = deepcopy(dict(message)) if isinstance(message, Mapping) else {}
        self._source_origins[arm] = self._source_trackers[arm].ingest(
            self._messages[arm], endpoint.joints, received_at, required=self.require_source_freshness)

    def telemetry_sample(self, arm: str) -> tuple[dict[str, Any] | None, float | None]:
        """Defensive raw frame copy and its original local monotonic receive time."""
        if arm not in self.arms:
            raise KeyError(arm)
        sample = self._samples.get(arm)
        return deepcopy(self._messages.get(arm)), sample[0] if sample else None

    def telemetry(self, arm: str, *, now: float | None = None, fresh_only: bool = True) -> dict[str, Any] | None:
        if fresh_only and not self.observation(now=now).arm_freshness[arm].fresh:
            return None
        return self.telemetry_sample(arm)[0]

    def joint_field(self, field: str, *, now: float | None = None) -> tuple[float, ...]:
        """Any numeric motor field in canonical order, with NaNs for unavailable slots."""
        observation = self.observation(now=now)
        if field == "position":
            return observation.positions
        if field == "torque":
            return observation.torques
        indices = {name: index for index, name in enumerate(self.joint_names)}
        values = [math.nan] * len(self.joint_names)
        for arm, endpoint in self.arms.items():
            if not observation.arm_freshness[arm].fresh:
                continue
            motors = self._messages.get(arm, {}).get("motors", {})
            if not isinstance(motors, Mapping):
                continue
            for name in endpoint.joints:
                motor = motors.get(name)
                if isinstance(motor, Mapping):
                    try:
                        values[indices[name]] = _number(motor.get(field))
                    except ValueError:
                        pass
        return tuple(values)

    def arm_ready(self, field: str = "position", *, now: float | None = None) -> dict[str, bool]:
        values = self.joint_field(field, now=now)
        indices = {name: index for index, name in enumerate(self.joint_names)}
        return {arm: all(math.isfinite(values[indices[name]]) for name in ep.joints) for arm, ep in self.arms.items()}

    def validate_commands(self, commands: Mapping[str, Mapping[str, Any]]) -> dict[str, dict[str, Any]]:
        """Copy/validate existing Rust wire commands without sending or changing state.

        Raw arm vectors already follow the named endpoint's wire order. Service
        motor IDs may include non-arm motors (Aizee's wheels); Rust retains their
        ownership/state checks. This method never invents a motion command.
        """
        if not isinstance(commands, Mapping):
            raise ValueError("commands must map arm names to wire messages")
        result = {}
        motor_commands = {"enable", "disable", "zero_position", "mech_zero", "clear_fault", "trigger_fault"}

        def arm_payload(payload, size):
            if isinstance(payload, Mapping) and payload.get("type") == "arm_joints":
                payload = {key: value for key, value in payload.items() if key != "type"}
            if not isinstance(payload, Mapping) or set(payload) - {"positions", "velocities", "kp", "kd", "torques"}:
                raise ValueError("invalid arm_joints payload")
            output = {}
            for field in ("positions", "velocities", "kp", "kd", "torques"):
                if field not in payload:
                    if field in ("positions", "velocities"):
                        raise ValueError(f"arm_joints requires {field}")
                    continue
                raw = payload[field]
                if raw is None and field in ("kp", "kd", "torques"):
                    continue
                if not isinstance(raw, (list, tuple)):
                    raise ValueError(f"{field} must be a vector")
                values = [_number(value) for value in raw]
                if len(values) != size and not (field in ("kp", "kd", "torques") and not values):
                    raise ValueError(f"{field} requires {size} values")
                if field in ("kp", "kd") and any(value < 0 for value in values):
                    raise ValueError(f"{field} cannot contain negative gains")
                output[field] = values
            return output

        def drive_payload(payload):
            if isinstance(payload, Mapping) and payload.get("type") == "drive":
                payload = {key: value for key, value in payload.items() if key != "type"}
            if not isinstance(payload, Mapping) or set(payload) - {"linear", "angular", "kp", "kd"}:
                raise ValueError("invalid drive payload")
            if "linear" not in payload or "angular" not in payload:
                raise ValueError("drive requires linear and angular values")
            output = {key: _number(value) for key, value in payload.items() if value is not None or key in ("linear", "angular")}
            if any(output.get(key, 0.0) < 0 for key in ("kp", "kd")):
                raise ValueError("drive gains cannot be negative")
            return output

        for arm, message in commands.items():
            if arm not in self.arms:
                raise ValueError(f"Unknown arm endpoint {arm!r}")
            if not isinstance(message, Mapping):
                raise ValueError("wire command must be an object")
            payload = dict(message)
            kind = payload.pop("type", None)
            if kind == "arm_joints":
                payload = arm_payload(payload, len(self.arms[arm].joints))
            elif kind == "drive":
                payload = drive_payload(payload)
            elif kind == "bundle":
                if set(payload) - {"arm_joints", "drive"}:
                    raise ValueError("invalid bundle fields")
                payload = {key: (arm_payload(value, len(self.arms[arm].joints)) if key == "arm_joints" else drive_payload(value))
                           for key, value in payload.items() if value is not None}
            elif kind in motor_commands:
                allowed = {"motor_ids", "save"} if kind == "mech_zero" else {"motor_ids"}
                ids = payload.get("motor_ids")
                if set(payload) - allowed or not isinstance(ids, (list, tuple)) or any(not isinstance(v, str) or not v for v in ids):
                    raise ValueError(f"invalid {kind} motor_ids")
                if "save" in payload and not isinstance(payload["save"], bool):
                    raise ValueError("mech_zero save must be boolean")
                payload = {**payload, "motor_ids": list(ids)}
            elif kind in ("emergency_stop", "clear_emergency_stop"):
                if payload:
                    raise ValueError(f"{kind} takes no payload")
            else:
                raise ValueError(f"Unsupported command type {kind!r}")
            result[arm] = {"type": kind, **payload}
        return result

    def prepare_commands(self, commands: Mapping[str, Mapping[str, Any]], *, now: float | None = None,
                         require_torque: bool = False) -> dict[str, dict[str, Any]]:
        """Validate wire commands and remove stale arm motion; keep drive stops/services."""
        result = self.validate_commands(commands)
        ready = self.arm_ready(now=now)
        if require_torque:
            torque_ready = self.arm_ready("torque", now=now)
            ready = {arm: value and torque_ready[arm] for arm, value in ready.items()}
        for arm, command in list(result.items()):
            if ready[arm]:
                continue
            if command["type"] == "arm_joints":
                del result[arm]
            elif command["type"] == "bundle":
                command.pop("arm_joints", None)
                if "drive" not in command:
                    del result[arm]
        return result

    def observation(self, *, now: float | None = None) -> Observation:
        now = _number(time.monotonic() if now is None else now)
        positions, torques = [math.nan] * len(self.joint_names), [math.nan] * len(self.joint_names)
        indices = {name: index for index, name in enumerate(self.joint_names)}
        freshness: dict[str, ArmFreshness] = {}
        for arm, endpoint in self.arms.items():
            sample = self._samples.get(arm)
            received_at = sample[0] if sample else None
            age = now - received_at if received_at is not None else math.inf
            origins, source_known = self._source_origins.get(arm, ((), False))
            if origins:
                age = max(age, *(now - origin for origin in origins))
            fresh = 0.0 <= age <= self.max_age_s
            complete = False
            if sample is not None:
                complete = all(math.isfinite(value) for vector in sample[1:] for value in vector)
                if fresh:
                    for name, position, torque in zip(endpoint.joints, sample[1], sample[2]):
                        positions[indices[name]], torques[indices[name]] = position, torque
            freshness[arm] = ArmFreshness(received_at, age, fresh, complete, source_known)
        position_valid = tuple(math.isfinite(value) for value in positions)
        torque_valid = tuple(math.isfinite(value) for value in torques)
        return Observation(
            self.joint_names, tuple(positions), tuple(torques), self.active_mask,
            position_valid, torque_valid, tuple(p and t for p, t in zip(position_valid, torque_valid)),
            MappingProxyType(freshness),
        )

    def route_target(self, positions: Sequence[float], *, now: float | None = None,
                     allow_partial: bool = False, require_torque: bool = True, **kwargs: Any) -> dict[str, dict[str, Any]]:
        """Route fresh arms; opt-in partial control drops unavailable/NaN arms.

        Position readiness suffices for targeting with ``require_torque=False``;
        recording and torque-dependent policies retain the stricter default.
        """
        commands = route_joint_target(self.spec, self.arms, positions, allow_partial=allow_partial, **kwargs)
        ready = self.arm_ready(now=now)
        if require_torque:
            torque_ready = self.arm_ready("torque", now=now)
            ready = {arm: valid and torque_ready[arm] for arm, valid in ready.items()}
        if not allow_partial and not all(ready.values()):
            unavailable = [arm for arm, valid in ready.items() if not valid]
            raise StaleTelemetryError(f"Fresh, complete telemetry required for arms: {unavailable}")
        return {arm: command for arm, command in commands.items() if ready[arm]}


class ZmqRobotClient:
    """Opt-in, single-threaded PUSH/SUB transport around a ``RobotSession``.

    Construction is offline. ``connect`` only creates sockets; it never enables
    motors. ``send_target`` sends once after fresh telemetry validation. PUSH
    delivery is unacknowledged and two-arm sends cannot be atomic; failures name
    already-sent arms and are never retried here. Closing sends no command.
    """

    def __init__(self, session: RobotSession, *, context: Any = None, receive_only: bool = False) -> None:
        self.session = session
        self._context = context
        self._owns_context = context is None
        self._push: dict[str, Any] = {}
        self._sub: dict[str, Any] = {}
        self._zmq: Any = None
        self.receive_only = receive_only

    def connect(self) -> None:
        if self._sub:
            return
        import zmq

        self._zmq = zmq
        if self._context is None:
            self._context = zmq.Context()
        try:
            for arm, endpoint in self.session.arms.items():
                if not self.receive_only:
                    push = self._context.socket(zmq.PUSH)
                    self._push[arm] = push
                    push.setsockopt(zmq.LINGER, 0)
                    push.setsockopt(zmq.IMMEDIATE, 1)
                    push.setsockopt(zmq.SNDHWM, 1)
                    push.connect(endpoint.command)
                sub = self._context.socket(zmq.SUB)
                self._sub[arm] = sub
                sub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.CONFLATE, 1)
                sub.setsockopt(zmq.SUBSCRIBE, b"")
                sub.connect(endpoint.telemetry)
        except Exception:
            self.close()
            raise

    def poll(self, *, max_messages_per_arm: int = 32) -> Observation:
        """Drain up to the given count per arm without blocking; malformed frames fail visibly."""
        if not self._sub:
            raise RuntimeError("Call connect() before poll()")
        if not isinstance(max_messages_per_arm, int) or max_messages_per_arm < 1:
            raise ValueError("max_messages_per_arm must be a positive integer")
        from robot_platform.transport.wire import unpack_msg

        first_error = None
        for arm, socket in self._sub.items():
            for _ in range(max_messages_per_arm):
                try:
                    frame = socket.recv(flags=self._zmq.NOBLOCK)
                except self._zmq.Again:
                    break
                except Exception as exc:
                    self.session.ingest_telemetry(arm, {})
                    first_error = first_error or exc
                    break
                received_at = time.monotonic()
                try:
                    message = unpack_msg(frame)
                except Exception as exc:
                    self.session.ingest_telemetry(arm, {}, received_at=received_at)
                    first_error = first_error or exc
                    break  # other arms must still be drained this cycle
                self.session.ingest_telemetry(arm, message, received_at=received_at)
        if first_error is not None:
            raise first_error
        return self.session.observation()

    def send_target(self, positions: Sequence[float], **kwargs: Any) -> None:
        if self.receive_only:
            raise RuntimeError("receive-only clients cannot send commands")
        if not self._push:
            raise RuntimeError("Call connect() before send_target()")
        # Real sends always use the current clock; deterministic times belong in
        # the pure session tests, not on a transport path that can move hardware.
        if "now" in kwargs:
            raise TypeError("send_target does not accept a clock override")
        commands = self.session.route_target(positions, **kwargs)
        self._send_prepared(commands)

    def send_commands(self, commands: Mapping[str, Mapping[str, Any]]) -> None:
        """Send validated one-shot commands, gating arm motion at the current clock."""
        if self.receive_only:
            raise RuntimeError("receive-only clients cannot send commands")
        if not self._push:
            raise RuntimeError("Call connect() before send_commands()")
        commands = self.session.prepare_commands(commands)
        self._send_prepared(commands)

    def _send_prepared(self, commands: Mapping[str, Mapping[str, Any]]) -> None:
        from robot_platform.transport.wire import pack_msg

        frames = {arm: pack_msg(command) for arm, command in commands.items()}
        sent = []
        for arm, frame in frames.items():
            try:
                self._push[arm].send(frame, flags=self._zmq.NOBLOCK)
            except Exception as exc:
                raise PartialSendError(sent, arm) from exc
            sent.append(arm)

    def close(self) -> None:
        for socket in (*self._push.values(), *self._sub.values()):
            socket.close(linger=0)
        self._push.clear()
        self._sub.clear()
        if self._owns_context and self._context is not None:
            self._context.term()
            self._context = None

    def __enter__(self) -> ZmqRobotClient:
        self.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
