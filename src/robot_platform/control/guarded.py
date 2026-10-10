"""Explicit client for guarded protocol v1; transport and scheduling are injected.

This does not change legacy ZMQ publishers or enable a physical deployment.
The initial transport is the native, deterministic controller test harness.
Receiver timestamps are opaque: clients echo permits and never compare them
to their own clock. A renewal alone never refreshes a producer's old target.
"""
from __future__ import annotations

from copy import deepcopy
from typing import Callable, Mapping
import uuid


class GuardRejected(RuntimeError):
    def __init__(self, reason: str, status: dict):
        self.reason, self.status = reason, status
        super().__init__(f"Guarded controller rejected request: {reason}")


class GuardedControllerClient:
    """One explicit owner; no automatic retry, renewal, reset, enable or send."""

    def __init__(self, exchange: Callable[[dict], dict], *, group: str,
                 identity: Mapping[str, str], joint_names: tuple[str, ...],
                 session_id: str | None = None):
        self.exchange = exchange
        self.group = group
        self.identity = dict(identity)
        self.joint_names = tuple(joint_names)
        self.session_id = session_id or str(uuid.uuid4())
        self.status: dict | None = None
        self._epoch: str | None = None
        self._lease: int | None = None
        self._sequence = 0
        self.exchanges = 0

    def _call(self, request: dict) -> dict:
        self.exchanges += 1
        reply = self.exchange(deepcopy(request))
        status = reply["status"]
        self.status = deepcopy(status)
        if (status["protocol_version"] != 1 or status["group"] != self.group
                or status["identity"] != self.identity
                or tuple(status["joint_names"]) != self.joint_names):
            self._lease = None
            raise GuardRejected("receiver_identity_mismatch", status)
        if self._epoch is not None and status["epoch"] != self._epoch:
            self._lease = None
            raise GuardRejected("receiver_restarted", status)
        lease = status.get("lease")
        if lease is None or lease["session"] != self.session_id:
            self._lease = None
        if not reply["accepted"]:
            raise GuardRejected(reply["reason"], status)
        if lease is not None and lease["session"] == self.session_id and self._lease is not None:
            # An accepted arm or move whose collision verdict binds a fresh offer renews the lease with it,
            # as Renew does: the lease id follows.
            self._lease = lease["id"]
        return reply

    def _authority(self) -> dict:
        if self._lease is None or self._epoch is None:
            raise RuntimeError("Acquire ownership before sending guarded commands")
        self._sequence += 1
        return {"epoch": self._epoch, "session": self.session_id,
                "lease": self._lease, "sequence": self._sequence}

    @property
    def owned(self) -> bool:
        """Whether this client holds the receiver's lease."""
        return self._lease is not None

    def poll(self) -> dict:
        return self._call({"type": "status"})["status"]

    def acquire(self) -> dict:
        if self._lease is not None:
            raise RuntimeError("Already owns a lease")
        # A caller may deliberately acquire again after a restart/fault. This
        # operation never resets the state machine or arms the hardware.
        self._epoch = None
        offered = self._call({"type": "offer"})
        self._epoch = offered["status"]["epoch"]
        reply = self._call({"type": "acquire", "epoch": self._epoch,
                            "offer": offered["offer"]["id"], "session": self.session_id,
                            "group": self.group, "identity": self.identity})
        self._lease = reply["status"]["lease"]["id"]
        self._sequence = 0
        return reply["status"]

    def renew(self) -> dict:
        offered = self._call({"type": "offer"})
        reply = self._call({"type": "renew", "authority": self._authority(),
                            "offer": offered["offer"]["id"]})
        self._lease = reply["status"]["lease"]["id"]
        return reply["status"]

    def renew_with(self, offered: dict) -> dict:
        """Renew with an offer already taken this cycle (its collision offer), when no arm or move will
        bind it: one request instead of two."""
        reply = self._call({"type": "renew", "authority": self._authority(), "offer": offered["offer"]["id"]})
        self._lease = reply["status"]["lease"]["id"]
        return reply["status"]

    def collision_offer(self) -> dict:
        """Request a one-use receiver clock anchor before collision computation."""
        return self.freshness_offer()

    def freshness_offer(self) -> dict:
        """Request a one-use receiver clock anchor for ownership/command freshness."""
        return self._call({"type": "offer"})

    def arm(self, *, collision=None) -> dict:
        request = {"type": "arm", "authority": self._authority()}
        if collision is not None:
            request["collision"] = collision
        return self._call(request)["status"]

    def idle(self) -> dict:
        """Explicit zero-gain backdrive; never writes a mechanical zero."""
        return self._call({"type": "idle", "authority": self._authority()})["status"]

    def disable(self) -> dict:
        """Explicit removal of torque, distinct from a controlled fault stop."""
        return self._call({"type": "disable", "authority": self._authority()})["status"]

    def move(self, joints: list[dict], *, collision=None, ramp_efforts=None) -> dict:
        request = {"type": "move", "authority": self._authority(),
                   "identity": self.identity, "joints": joints}
        if collision is not None:
            request["collision"] = collision
        if ramp_efforts is not None:
            request["ramp_efforts"] = [float(effort) for effort in ramp_efforts]
        return self._call(request)["status"]

    def stop(self) -> dict:
        return self._call({"type": "stop", "authority": self._authority()})["status"]

    def direct_update(self, joints, offered, *, valid_for_ms, ramp_efforts=None, update_hold_gains=False):
        """Consume the previous reply's freshness token and return feedback plus the next one.

        No retry or automatic re-offer: an expired command must not gain a new lifetime.
        """
        request = {'type': 'direct_update', 'authority': self._authority(),
                           'identity': self.identity, 'offer': offered['offer']['id'], 'joints': joints,
                           'valid_for_ms': valid_for_ms, 'ramp_efforts': ramp_efforts or []}
        if update_hold_gains:
            request['update_hold_gains'] = True
        return self._call(request)

    def release(self) -> dict:
        return self._call({"type": "release", "authority": self._authority()})["status"]

    def reset(self) -> dict:
        return self._call({"type": "reset", "authority": self._authority()})["status"]

    def emergency_stop(self) -> dict:
        return self._call({"type": "emergency_stop"})["status"]


class GuardedBatchError(RuntimeError):
    def __init__(self, accepted_groups, failed_group, stop_failures):
        self.accepted_groups = tuple(accepted_groups)
        self.failed_group = failed_group
        self.stop_failures = dict(stop_failures)
        super().__init__(f"Guarded batch failed at {failed_group}; accepted={self.accepted_groups}; "
                         f"stop failures={tuple(self.stop_failures)}")


class GuardedSessionClient:
    """Route a RobotSession's joint commands to acknowledged guarded receivers.

    This first adapter requires an explicit simulation-only session. On a partial
    send/rejection it tries to stop EVERY receiver, including the one whose reply
    was lost. A disconnected receiver still relies on its independent lease
    timer. Separate endpoints are not an atomic distributed transaction.
    """

    def __init__(self, session, controllers: Mapping[str, GuardedControllerClient]):
        if not session.simulation_only:
            raise ValueError("Guarded session integration currently requires simulation_only")
        if set(controllers) != set(session.arms):
            raise ValueError("Every session arm requires a guarded controller")
        for arm, endpoint in session.arms.items():
            if controllers[arm].joint_names != endpoint.joints:
                raise ValueError(f"Joint order mismatch for {arm}")
        self.session, self.controllers = session, dict(controllers)

    def emergency_stop(self) -> dict[str, str]:
        failures = {}
        for arm, controller in self.controllers.items():
            try:
                controller.emergency_stop()
            except Exception as exc:
                failures[arm] = str(exc)
        return failures

    def _batch(self, operation: str, payloads=None, before_each=None):
        accepted, results = [], {}
        for arm, controller in self.controllers.items():
            try:
                if before_each is not None:
                    before_each()
                fn = getattr(controller, operation)
                results[arm] = fn(payloads[arm]) if payloads is not None else fn()
                accepted.append(arm)
            except Exception as exc:
                if operation == "acquire":
                    # A busy group belongs to someone else; roll back only our
                    # successful claims without interrupting that other owner.
                    failures = {}
                    for claimed in accepted:
                        try:
                            self.controllers[claimed].release()
                        except Exception as rollback:
                            failures[claimed] = str(rollback)
                else:
                    failures = self.emergency_stop()
                raise GuardedBatchError(accepted, arm, failures) from exc
        return results

    def acquire(self):
        return self._batch("acquire")

    def arm(self):
        return self._batch("arm")

    def renew(self):
        return self._batch("renew")

    def stop(self):
        return self._batch("stop")

    def send_commands(self, commands, *, now=None):
        validated = self.session.prepare_commands(commands, now=now)
        if set(validated) != set(self.controllers):
            raise ValueError("Guarded batches require fresh commands for every active arm")
        payloads = {}
        for arm, command in validated.items():
            if command["type"] != "arm_joints":
                raise ValueError("Guarded session adapter currently accepts joint motion only")
            count = len(self.controllers[arm].joint_names)
            # Gains must be deliberate; zero defaults could remove support.
            if any(len(command.get(field, [])) != count for field in ("positions", "velocities", "kp", "kd", "torques")):
                raise ValueError("Guarded commands require complete position, velocity, gain and effort vectors")
            payloads[arm] = [dict(zip(("position", "velocity", "kp", "kd", "effort"), values))
                             for values in zip(*(command[field] for field in ("positions", "velocities", "kp", "kd", "torques")))]
        gate = self.session.command_gate
        if gate is not None:
            gated = gate.filter_commands(self.session, validated)
            if gated != validated or not gate.dispatch_valid(self.session, validated):
                self.emergency_stop()
                raise ValueError("Collision gate rejected guarded batch")
        def before_each():
            if gate is not None and not gate.dispatch_valid(self.session, validated):
                raise ValueError("Collision verdict expired during guarded batch")
        return self._batch("move", payloads, before_each)
