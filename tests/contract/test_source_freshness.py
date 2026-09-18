from copy import deepcopy
import math
import pytest

from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.control import RobotSession


def make_session(required=False):
    spec = load_robot("minerva")
    return RobotSession(spec, load_settings(spec, overrides={"control": {"require_source_freshness": required}}))


def frame(session, seq=1, age=0.0, producer="a"):
    return {"telemetry_schema_version": 2, "producer_id": producer,
            "motors": {name: {"position": 0.1, "torque": 0.2,
                                "feedback": {"sequence": seq, "age_s": age, "valid": True, "provenance": "can"}}
                       for name in session.arms["left"].joints}}


def test_republication_of_cached_sample_cannot_renew_age():
    session = make_session()
    session.ingest_telemetry("left", frame(session), received_at=10)
    session.ingest_telemetry("left", frame(session), received_at=10.6)
    assert not session.arm_ready(now=10.6)["left"]
    session.ingest_telemetry("left", frame(session, seq=2), received_at=10.7)
    assert session.arm_ready(now=10.7)["left"]


@pytest.mark.parametrize("change", [
    {"age_s": 2}, {"age_s": -1}, {"age_s": float("nan")},
    {"sequence": 0}, {"sequence": True}, {"valid": False}, {"provenance": "synthetic"},
])
def test_one_invalid_source_prevents_arm_motion(change):
    session = make_session()
    message = frame(session)
    next(iter(message["motors"].values()))["feedback"].update(change)
    session.ingest_telemetry("left", message, received_at=10)
    assert not session.arm_ready(now=10)["left"]


def test_old_sequence_epoch_and_schema_downgrade_fail_closed():
    session = make_session()
    for packet, ready in [(frame(session, 3), True), (frame(session, 2), False),
                          (frame(session, 4), True), (frame(session, 1, producer="b"), True),
                          (frame(session, 5), False)]:
        session.ingest_telemetry("left", packet, received_at=10)
        assert session.arm_ready(now=10)["left"] is ready
    message = frame(session)
    message.pop("telemetry_schema_version")
    message.pop("producer_id")
    session.ingest_telemetry("left", message, received_at=10)
    assert not session.arm_ready(now=10)["left"]


def test_legacy_freshness_explicitly_unknown_and_strict_mode_rejects():
    for required in (False, True):
        session = make_session(required)
        message = frame(session)
        message = {"motors": message["motors"]}
        session.ingest_telemetry("left", message, received_at=10)
        assert session.arm_ready(now=10)["left"] is (not required)
        assert not session.observation(now=10).arm_freshness["left"].source_known
