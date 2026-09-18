from pathlib import Path
import math
from types import SimpleNamespace

import pytest

from robot_platform.characterization.audit import audit_repository, validate_bench_profile


ROOT = Path(__file__).resolve().parents[2]


def test_repository_audit_does_not_claim_hardware_validation():
    report = audit_repository(ROOT)
    assert report["simulation_ready"] and not report["hardware_ready"]
    ids = {issue["id"] for issue in report["issues"]}
    assert {"RS00_SCALE", "RS03_SCALE", "DIRECTIONAL_BOUNDS", "TORQUE_REFERENCE"} <= ids
    assert report["manuals"]["RS00"]["torque"] == 14
    assert report["controllers"]["right"]["endpoints"]["command_sub"].endswith(":5575")


def test_missing_verification_blocks_before_opening_a_device(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "python/scripts"))
    from actuator_char.harness import Joint, MotorBus
    with pytest.raises(ValueError, match="schema_version"):
        MotorBus(Joint("test", "RS00", 10))


def test_feedback_filters_other_ids_and_preserves_six_fault_bits(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "python/scripts"))
    from actuator_char.harness import Joint, MotorBus
    from actuator_char import robstride_mit as rs
    frames = iter([
        SimpleNamespace(arbitration_id=(2 << 24) | (7 << 8), data=bytes(8), is_extended_id=True, timestamp=100),
        SimpleNamespace(arbitration_id=(2 << 24) | (0x20 << 16) | (10 << 8), data=bytes(8), is_extended_id=True, timestamp=101),
        None,
    ])
    bus = MotorBus.__new__(MotorBus)
    bus.joint, bus.trace, bus.sequence = Joint("test", "RS00", 10), None, 0
    bus.bus = SimpleNamespace(recv=lambda **_: next(frames))
    feedback = bus.read_feedback()
    assert feedback.motor_id == 10
    assert feedback.error_bits == 0x20
    assert feedback.sequence == 1 and feedback.received_at is not None
    assert feedback.kernel_timestamp == 101


def test_repeated_cache_is_not_counted_as_new_measurement(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "python/scripts"))
    from actuator_char.harness import run_profile, step_profile
    from actuator_char.robstride_mit import Feedback, MotorMode
    clock = SimpleNamespace(now=0.0)
    class Bus:
        count = 0
        def enable(self): pass
        def control(self, *_): pass
        def read_feedback(self):
            self.count += 1
            return Feedback(1, MotorMode.RUN, 0, 0, 0, 25, 0) if self.count == 1 else None
        def shutdown(self): pass
    def sleep(dt): clock.now += dt
    rows = run_profile(Bus(), step_profile(0.1, 1, 0.1), 0.05, settle_s=0,
                       clock=lambda: clock.now, sleep=sleep)
    assert sum(r["fb_new"] for r in rows) == 1
    assert len({r["fb_sequence"] for r in rows}) == 1
    assert rows[-1]["fb_age_s"] > rows[0]["fb_age_s"]


def test_unsettled_response_never_reports_finite_settle_time(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "python/scripts"))
    from actuator_char.analysis import step_metrics
    assert math.isinf(step_metrics([0, 1, 2], [0, 0.2, 0.4], 0, 1)["settle_s"])
