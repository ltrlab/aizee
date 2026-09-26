"""Single-owner, whole-robot leader/collision/receiver transaction.

No device opening, automatic rearming, retries or background lease renewal.
The caller supplies fresh samples every cycle and must deliberately start a new
session after a stop. The Linux service still refuses physical network motion.
"""
from copy import deepcopy
import math
import time


class GuardedTeleopSupervisor:
    def __init__(self, controller, compiler, mapper, snapshot, *,
                 period_s=.04, cycle_budget_s=.035, clock=time.perf_counter):
        self.controller, self.compiler, self.mapper = controller, compiler, mapper
        self.snapshot, self.clock = snapshot, clock
        self.period_s, self.cycle_budget_s = period_s, cycle_budget_s
        if (not math.isfinite(period_s) or not math.isfinite(cycle_budget_s)
                or not 0 < cycle_budget_s <= period_s <= mapper.max_step_s
                or compiler.compute_budget_s > cycle_budget_s
                or compiler.config["collision"]["max_verdict_ms"] / 1000 <= period_s
                or controller.joint_names != compiler.names or mapper.names != compiler.names
                or controller.identity != compiler.config["identity"]
                or controller.group != compiler.config["group"]
                or mapper.limits != compiler.config["joints"]):
            raise ValueError("Matching whole-robot configuration and valid cycle budgets required")
        self.state, self.reason = "idle", None
        self.owned = False
        self.last_cycle = None
        self.stop_error = None

    def _certify(self, joints):
        started = self.clock()
        offered = self.controller.collision_offer()
        # The transport's single response includes status AND its feedback.
        # Reject any getter that supplies a cached/different receiver snapshot.
        sample = deepcopy(self.snapshot())
        if sample["reply"] != offered or sample["status"] != offered["status"]:
            raise ValueError("Feedback must come from the same receiver offer response")
        envelope = self.compiler.compile(offered, sample["feedback"], joints,
                                         started_at=started)
        return envelope, sample["feedback"]

    def _before_dispatch(self, samples, started):
        self.mapper.validate_fresh(samples, now=self.clock())
        if self.clock() - started >= self.cycle_budget_s:
            raise TimeoutError("Teleoperation cycle deadline exceeded")

    def stop(self, reason="operator_stop"):
        self.mapper.stop(reason)
        self.state, self.reason = "stopped", reason
        # A failed acquisition must never interrupt a different owner.
        if self.owned:
            self.owned = False
            try:
                self.controller.stop()
            except Exception as exc:
                # No retry/rearm. The receiver's local verdict/command timers
                # remain responsible for stopping if this acknowledgement fails.
                self.stop_error = str(exc)

    def begin(self, samples, *, engaged):
        """Deliberate engagement captures current robot and leader pose, then holds."""
        if self.state != "idle":
            raise RuntimeError("A stopped session requires explicit receiver recovery and a new supervisor")
        started = self.clock()
        try:
            if engaged is not True:
                raise ValueError("Explicit operator engagement required")
            self.mapper.validate_fresh(samples, now=started)
            status = self.controller.acquire()
            self.owned = True
            if status["state"] != "disarmed":
                raise ValueError("Receiver must be explicitly reset and disarmed before registration")
            envelope, feedback = self._certify(None)
            self.mapper.register(samples, [f["position"] for f in feedback], now=self.clock())
            self._before_dispatch(samples, started)
            status = self.controller.arm(collision=envelope)
            self._before_dispatch(samples, started)
            self.state = "tracking"
            self.last_cycle = dict(kind="registration", accepted=True, started_at=started,
                                   completed_at=self.clock(), status=status,
                                   collision=deepcopy(self.compiler.last_verdict))
            return self.last_cycle
        except Exception as exc:
            self.stop(str(exc))
            raise

    def step(self, samples, *, engaged):
        """One fresh, complete command batch; an error latches the session stopped."""
        if self.state != "tracking":
            raise RuntimeError("Explicit teleoperation registration required")
        started = self.clock()
        record = dict(kind="motion", accepted=False, started_at=started)
        self.last_cycle = record
        try:
            positions = self.mapper.propose(samples, now=started, engaged=engaged)
            joints = [dict(position=q, velocity=0., effort=0., kp=limit["hold_kp"], kd=limit["hold_kd"])
                      for q, limit in zip(positions, self.compiler.config["joints"])]
            record["proposed_positions"] = positions
            # Renewal never refreshes the old command or collision deadline.
            self.controller.renew()
            envelope, feedback = self._certify(joints)
            record.update(collision=deepcopy(self.compiler.last_verdict),
                          envelope=envelope, feedback=feedback)
            self._before_dispatch(samples, started)
            record["accepted"] = None  # acceptance is unknown until acknowledgement
            status = self.controller.move(joints, collision=envelope)
            record.update(accepted=True, status=status)
            # A late acknowledgement may follow acceptance. Record that fact,
            # stop immediately, and never disguise it as an unsent command.
            self._before_dispatch(samples, started)
            return record
        except Exception as exc:
            record["failure"] = str(exc)
            self.stop(str(exc))
            raise
        finally:
            record["completed_at"] = self.clock()
            record["stop_error"] = self.stop_error
