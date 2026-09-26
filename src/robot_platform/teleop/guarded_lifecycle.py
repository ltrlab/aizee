"""Run the existing GUI lifecycle through one whole-robot collision receiver.

No USB ownership, sockets, background sends or automatic recovery. The caller
supplies an already mapped absolute leader target and advances one 25 Hz cycle.
The same offer/feedback transaction certifies Hold, alignment and zero return.
"""
from copy import deepcopy
import math
import time


class GuardedLifecycleSession:
    def __init__(self, controller, compiler, lifecycle, snapshot, *,
                 period_s=.04, cycle_budget_s=.035, clock=time.perf_counter):
        self.controller, self.compiler, self.lifecycle = controller, compiler, lifecycle
        self.snapshot, self.clock = snapshot, clock
        self.period_s, self.cycle_budget_s = period_s, cycle_budget_s
        if (not all(math.isfinite(v) for v in (period_s, cycle_budget_s))
                or not 0 < cycle_budget_s <= period_s <= lifecycle.max_step_s
                or compiler.compute_budget_s > cycle_budget_s
                or compiler.config["collision"]["max_verdict_ms"] / 1000 <= period_s
                or controller.joint_names != compiler.names or lifecycle.names != compiler.names
                or controller.identity != compiler.config["identity"]
                or controller.group != compiler.config["group"]):
            raise ValueError("Matching complete robot configuration and valid timing required")
        for row, limit in zip(lifecycle.joints, compiler.config["joints"]):
            if any(row[key] != limit[key] for key in
                   ("name", "lower", "upper", "hold_kp", "hold_kd", "max_velocity")):
                raise ValueError("Lifecycle bounds and gains must match receiver configuration")
        self.owned = False
        self.stop_error = None
        self.last_cycle = None
        self.last_feedback = None

    def _deadline(self, started, target):
        now = self.clock()
        if now - started >= self.cycle_budget_s:
            raise TimeoutError("GUI teleoperation cycle exceeded its deadline")
        # A cycle without a mapped source only holds its last bounded target.
        if target is not None and self.lifecycle.state in ("ENGAGING", "TELEOP"):
            if not target or any(not 0 <= now - row["sampled_at"] < self.lifecycle.target_max_age_s
                                 for row in target):
                raise ValueError("Leader target expired before dispatch")

    def fault(self, reason):
        self.lifecycle.fault(str(reason))
        if self.owned:
            try:
                self.controller.stop()
            except Exception as exc:
                self.stop_error = str(exc)
        # Never retry/rearm here. Local receiver expiry remains authoritative.

    def disable(self):
        proposal = self.lifecycle.action("disable", [], now=self.clock())
        if self.owned:
            self.controller.disable()
            self.owned = False
        return proposal

    def cycle(self, *, action=None, target=None, started_at=None):
        started = self.clock() if started_at is None else started_at
        record = dict(action=action, accepted=False, started_at=started)
        self.last_cycle = record
        try:
            if action == "disable":
                record["proposal"] = self.disable()
                record["accepted"] = True
                return record
            if self.lifecycle.state == "FAULT":
                raise RuntimeError("Explicit receiver recovery required after a fault")
            if not self.owned and action == "idle":
                status = self.controller.acquire()
                self.owned = True
                if status["state"] != "disarmed":
                    raise ValueError("Receiver must be disarmed before Idle")
            elif self.owned:
                self.controller.renew()
            offer_started = self.clock()
            offered = self.controller.collision_offer()
            sample = deepcopy(self.snapshot())
            if sample["reply"] != offered or sample["status"] != offered["status"]:
                raise ValueError("Feedback must be from the same receiver offer")
            now_ms = offered["status"]["now_ms"]
            feedback = sample["feedback"]
            if len(feedback) != len(self.compiler.names):
                raise ValueError("Complete robot feedback required")
            rows = []
            for name, item in zip(self.compiler.names, feedback):
                age = (now_ms - item["sampled_at_ms"]) / 1000
                if item["fault"] or not math.isfinite(age) or age < 0:
                    raise ValueError("Faulted or invalid receiver feedback")
                rows.append(dict(name=name, position=item["position"], velocity=item["velocity"],
                                 sequence=item["sequence"], sampled_at=offer_started - age))
            self.last_feedback = deepcopy(rows)
            if action is not None:
                proposal = self.lifecycle.action(action, rows, target=target, now=self.clock())
            else:
                proposal = self.lifecycle.step(rows, target=target, now=self.clock())
            record.update(proposal=deepcopy(proposal), feedback=feedback)
            operation = proposal["operation"]
            if operation == "none":
                return record
            if not self.owned:
                raise ValueError("Idle must acquire robot ownership before applying commands")
            envelope = None
            if operation in ("enable", "move"):
                envelope = self.compiler.compile(offered, feedback,
                    None if operation == "enable" else proposal["joints"], started_at=offer_started)
                record.update(collision=deepcopy(self.compiler.last_verdict), envelope=envelope)
            self._deadline(started, target)
            record["accepted"] = None
            if operation == "idle":
                status = self.controller.idle()
            elif operation == "enable":
                status = self.controller.arm(collision=envelope)
            elif operation == "move":
                status = self.controller.move(proposal["joints"], collision=envelope)
            elif operation == "disable":
                status = self.controller.disable()
                self.owned = False
            else:
                raise ValueError(f"Unknown lifecycle operation {operation}")
            record.update(accepted=True, status=status)
            self._deadline(started, target)
            return record
        except Exception as exc:
            record["failure"] = str(exc)
            self.fault(exc)
            raise
        finally:
            record.update(completed_at=self.clock(), stop_error=self.stop_error)


def minerva_action(key, state):
    """Keep the established Minerva GUI/terminal keys, including T to Hold."""
    if key == "T":
        return "disengage" if state in ("ENGAGING", "TELEOP") else "engage"
    return {"I": "idle", "E": "enable", "H": "disable", "X": "shutdown"}.get(key)
