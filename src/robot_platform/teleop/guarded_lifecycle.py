"""Run the existing GUI lifecycle through one whole-robot collision receiver.

No USB ownership, sockets, background sends or automatic recovery. The caller
supplies an already mapped absolute leader target and advances one cycle.
The same offer/feedback transaction certifies Hold, alignment and zero return.
A move whose collision box is not clear holds the last target instead.
"""
from copy import deepcopy
import math
import time

from robot_platform.control.collision_envelope import CollisionBlocked

# A move whose collision box is not clear is tried again at these fractions of its step and speed, the last
# holding the target last sent: near the other arm only a slower move certifies.
REDUCED_STEPS = (.5, .2, 0.)
# A retry starts only with time left for its whole computation and this much for the move's dispatch: past its
# budget a cycle faults the session (Virtual · enforced on the Jetson, 2026-10-09: four blocked tries took 37 ms).
# With time for one more, it is the hold: a certified hold renews the receiver's verdict, without which it stops
# the arms once the last one lapses.
DISPATCH_S = .005


class GuardedLifecycleSession:
    def __init__(self, controller, compiler, lifecycle, snapshot, *,
                 period_s=.04, cycle_budget_s=.035, clock=time.perf_counter):
        self.controller, self.compiler, self.lifecycle = controller, compiler, lifecycle
        self.snapshot, self.clock = snapshot, clock
        self.period_s, self.cycle_budget_s = period_s, cycle_budget_s
        # The budget may pass the period: a slow cycle starts the next late; one past its budget faults.
        if (not all(math.isfinite(v) for v in (period_s, cycle_budget_s))
                or not 0 < period_s <= lifecycle.max_step_s or not 0 < cycle_budget_s <= lifecycle.max_step_s
                or compiler.compute_budget_s > cycle_budget_s
                or compiler.config["collision"]["max_verdict_ms"] / 1000 <= period_s
                or controller.joint_names != compiler.names or lifecycle.names != compiler.names
                or controller.identity != compiler.config["identity"]
                or controller.group != compiler.config["group"]):
            raise ValueError("Matching complete robot configuration and valid timing required")
        for row, limit in zip(lifecycle.joints, compiler.config["joints"]):
            # Commands keep headroom under the receiver's speed cap (control.approach), never above it.
            if (any(row[key] != limit[key] for key in ("name", "lower", "upper", "hold_kp", "hold_kd"))
                    or row["max_velocity"] > limit["max_velocity"]):
                raise ValueError("Lifecycle bounds and gains must match receiver configuration")
        self.owned = False
        self.stop_error = None
        self.last_cycle = None
        self.last_feedback = None
        self.blocked = None  # Why the last cycle's move was held, while it is.

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

    def disable_now(self):
        """The operator's disable, whatever the state: this adapter's disable while it owns the receiver, or
        else an emergency stop (always accepted), which removes torque from an arm the receiver holds."""
        try:
            self.lifecycle.action("disable", [], now=self.clock())
        except ValueError:
            pass
        try:
            if self.controller.owned:  # The client's lease, which a receiver stop revokes.
                self.controller.disable()
            else:
                self.controller.emergency_stop()
        finally:
            self.owned = False

    def _rows(self, offered, feedback, offer_started):
        """The lifecycle's feedback rows from a receiver offer's feedback, stamped on this clock."""
        now_ms = offered["status"]["now_ms"]
        if len(feedback) != len(self.compiler.names):
            raise ValueError("Complete robot feedback required")
        rows = []
        for name, item in zip(self.compiler.names, feedback):
            age = (now_ms - item["sampled_at_ms"]) / 1000
            if item["fault"] or not math.isfinite(age) or age < 0:
                raise ValueError("Faulted or invalid receiver feedback")
            rows.append(dict(name=name, position=item["position"], velocity=item["velocity"],
                             sequence=item["sequence"], sampled_at=offer_started - age))
        return rows

    def recover(self):
        """After a fault, take back the arm the receiver braked to a hold (a trip, or the stop this adapter
        asked for): acquire again, re-arm it where the receiver holds it, with its feed-forward, and start the
        lifecycle's shutdown from there, which the next cycles step down to the reference and then disable.
        False while the receiver is still braking."""
        status = self.controller.poll()
        if status["state"] == "stopping":
            return False
        if status["state"] != "stopped":
            raise RuntimeError(f"The receiver is {status['state']} ({status['reason']}): the arm's torque is off")
        if not self.controller.owned:  # Kept from a re-arm the receiver refused (a drive not answering yet).
            self.controller.acquire()
        self.owned = True
        offer_started = self.clock()
        offered = self.controller.collision_offer()
        sample = deepcopy(self.snapshot())
        if sample["reply"] != offered or sample["status"] != offered["status"]:
            raise ValueError("Feedback must be from the same receiver offer")
        held = offered["status"]["joints"]  # The stop's hold: the receiver re-arms there.
        envelope = self.compiler.compile(offered, sample["feedback"], held, started_at=offer_started)
        status = self.controller.arm(collision=envelope)
        rows = self._rows(offered, sample["feedback"], offer_started)
        self.lifecycle.recover(rows, [joint["position"] for joint in status["joints"]], now=self.clock())
        self.last_feedback = deepcopy(rows)
        return True

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
            # One offer a cycle: an enable or move binds it in its collision verdict, which renews the lease;
            # any other cycle renews with it. Two receiver requests a cycle, not four, each waiting for a tick.
            offer_started = self.clock()
            offered = self.controller.collision_offer()
            sample = deepcopy(self.snapshot())
            if sample["reply"] != offered or sample["status"] != offered["status"]:
                raise ValueError("Feedback must be from the same receiver offer")
            feedback = sample["feedback"]
            rows = self._rows(offered, feedback, offer_started)
            self.last_feedback = deepcopy(rows)
            if action is not None:
                proposal = self.lifecycle.action(action, rows, target=target, now=self.clock())
            else:
                proposal = self.lifecycle.step(rows, target=target, now=self.clock())
            record.update(proposal=deepcopy(proposal), feedback=feedback)
            operation = proposal["operation"]
            lease = offered["status"].get("lease") or {}
            if (operation in ("none", "idle") and self.owned
                    and offered["offer"]["expires_at_ms"] > lease.get("expires_at_ms", -1)):
                # Not when the lease already lasts as long (acquired this cycle, from an offer as fresh).
                self.controller.renew_with(offered)
            if operation == "none":
                return record
            if not self.owned:
                raise ValueError("Idle must acquire robot ownership before applying commands")
            envelope = None
            if operation in ("enable", "move"):
                try:
                    envelope = self.compiler.compile(offered, feedback,
                        None if operation == "enable" else proposal["joints"], started_at=offer_started)
                    self.blocked = None
                except CollisionBlocked as exc:
                    if operation != "move":
                        raise
                    # Not certifiable: the same move slower, a shorter step (near the other arm a box at speed
                    # is wide), else hold the target last sent. The leaders may move on, apart.
                    self.blocked = str(exc)
                    record.update(blocked=self.blocked, blocked_collision=deepcopy(exc.verdict),
                                  blocked_proposal=deepcopy(proposal))
                    last, fractions = exc, list(REDUCED_STEPS)
                    while fractions:
                        left = self.cycle_budget_s - DISPATCH_S - (self.clock() - started)
                        budget = self.compiler.compute_budget_s
                        if left < budget:
                            # No time for another try: hold the target last sent, unsent.
                            try:
                                proposal = self.lifecycle.reduce(0.)
                            except ValueError:
                                raise exc from None
                            record.update(out_of_time=fractions[0], proposal=deepcopy(proposal))
                            break
                        fraction = fractions.pop(0) if left >= 2 * budget else 0.
                        if fraction == 0.:
                            fractions = []
                        try:
                            proposal = self.lifecycle.reduce(fraction)
                        except ValueError:
                            raise exc from None
                        record["proposal"] = deepcopy(proposal)
                        try:
                            envelope = self.compiler.compile(offered, feedback, proposal["joints"],
                                                             started_at=offer_started)
                            record["reduced"] = fraction
                            break
                        except CollisionBlocked as again:
                            last = again
                            record.setdefault("reduced_blocked", []).append(
                                (fraction, again.verdict.get("reason"), again.verdict.get("clearance_lower_bound_m")))
                    if envelope is None:
                        # Nor the hold: send nothing. The receiver keeps driving its last command within the
                        # last verdict's box, and stops the arms itself when that lapses.
                        record.update(skipped=str(last), collision=deepcopy(self.compiler.last_verdict))
                        lease = offered["status"].get("lease") or {}
                        if offered["offer"]["expires_at_ms"] > lease.get("expires_at_ms", -1):
                            self.controller.renew_with(offered)
                        self._deadline(started, target)
                        record["accepted"] = True
                        return record
                record.update(collision=deepcopy(self.compiler.last_verdict), envelope=envelope)
            self._deadline(started, target)
            record["accepted"] = None
            if operation == "idle":
                status = self.controller.idle()
            elif operation == "enable":
                status = self.controller.arm(collision=envelope)
            elif operation == "move":
                status = self.controller.move(proposal["joints"], collision=envelope,
                                              ramp_efforts=proposal.get("ramp_efforts"))
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
