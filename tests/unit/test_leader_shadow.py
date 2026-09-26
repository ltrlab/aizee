"""Shadow input isolation, route completeness and full-cycle timing checks."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.teleop.leader_bridge import sample_reply
from robot_platform.teleop.openrb import LeaderSample


@pytest.fixture
def bench(monkeypatch):
    scripts = Path(__file__).resolve().parents[2] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("leader_shadow_under_test", scripts / "benchmark_leader_teleop.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def routes():
    return dict(schema_version=1, simulation_only=True, runtime_approved=False,
                routes=[dict(actuator="a", leader="left", channel=0, scale=1.),
                        dict(actuator="b", leader="right", channel=6, scale=-.01)])


@pytest.mark.parametrize("url", ["http://example.com:8767", "http://192.168.8.180:8767/",
    "http://localhost:8767", "http://127.0.0.1:5555", "https://127.0.0.1:8767",
    "http://user@127.0.0.1:8767", "http://127.0.0.1:8767/api/sample",
    "http://127.0.0.1:8767/?target=remote", "http://127.0.0.1:8767/#remote"])
def test_only_viewer_loopback_endpoint_is_accepted(bench, url):
    with pytest.raises(ValueError):
        bench.validate_viewer_url(url)
    assert bench.validate_viewer_url("http://127.0.0.1:8767/") == ("127.0.0.1", 8767)


@pytest.mark.parametrize("fault", ["approved", "missing_marker", "missing_joint", "wrong_order",
    "extra_joint", "unknown_source", "negative_channel", "eighth_channel", "bool_channel",
    "zero_scale", "infinite_scale", "bool_scale", "extra_motor_field", "missing_source"])
def test_routes_cannot_be_partial_physical_or_invalid(bench, fault):
    profile = routes()
    if fault == "approved": profile["runtime_approved"] = True
    elif fault == "missing_marker": del profile["simulation_only"]
    elif fault == "missing_joint": profile["routes"].pop()
    elif fault == "wrong_order": profile["routes"].reverse()
    elif fault == "extra_joint": profile["routes"].append(dict(profile["routes"][0]))
    elif fault == "unknown_source": profile["routes"][0]["leader"] = "other"
    elif fault == "negative_channel": profile["routes"][0]["channel"] = -1
    elif fault == "eighth_channel": profile["routes"][0]["channel"] = 7
    elif fault == "bool_channel": profile["routes"][0]["channel"] = True
    elif fault == "zero_scale": profile["routes"][0]["scale"] = 0
    elif fault == "infinite_scale": profile["routes"][0]["scale"] = float("inf")
    elif fault == "bool_scale": profile["routes"][0]["scale"] = True
    elif fault == "extra_motor_field": profile["routes"][0]["can_id"] = 4
    elif fault == "missing_source": profile["routes"][0]["leader"] = "right"
    with pytest.raises(ValueError):
        bench.validate_routes(profile, ("a", "b"), ("left", "right"))


def test_routes_keep_signed_prismatic_gain_and_are_copied(bench):
    profile = routes()
    actual = bench.validate_routes(profile, ("a", "b"), ("left", "right"))
    assert actual[1]["scale"] == -.01
    actual[1]["scale"] = 1
    assert profile["routes"][1]["scale"] == -.01


def sample(now=10., sequence=1):
    return LeaderSample((0.,) * 7, (0,) * 7, sequence, now - .002, now,
                        (0, 0, 1, 1), 100, 102)


def test_http_uses_existing_challenge_freshness_and_latches_replay_without_reconnect(bench, monkeypatch):
    clock, requests, connections = [10.], [], []
    profile = dict(id="fixture", leaders={"left": dict(usb_serial="L")}, action_token="secret")
    inputs = SimpleNamespace(pins={"left": "L"}, poll=lambda: {"left": sample()})

    class Connection:
        closed = False

        def __init__(self, host, port, timeout):
            assert (host, port, timeout) == ("127.0.0.1", 8767, .025)
            connections.append(self)

        def request(self, method, path, body, headers):
            requests.append((method, path, body, headers))

        def getresponse(self):
            method, path, body, headers = requests[-1]
            if method == "GET":
                assert path == "/api/profile"
                reply = profile
            else:
                assert path == "/api/sample" and headers["X-Leader-Token"] == "secret"
                reply = sample_reply(json.loads(body), inputs, "boot", clock=lambda: 10.003)
                clock[0] += .005
            data = json.dumps(reply).encode()
            return SimpleNamespace(status=200, read=lambda maximum: data[:maximum])

        def close(self):
            self.closed = True

    monkeypatch.setattr(bench.http.client, "HTTPConnection", Connection)
    source = bench.ViewerLeaderInput("http://127.0.0.1:8767/", clock=lambda: clock[0])
    row = source.poll()["left"]
    assert row.received_at - row.sampled_at == pytest.approx(.010)
    assert "secret" not in json.dumps(source.profile)
    with pytest.raises(ValueError, match="repeated"):
        source.poll()
    count = len(requests)
    with pytest.raises(RuntimeError, match="faulted"):
        source.poll()
    assert len(requests) == count and len(connections) == 1
    source.close()
    assert connections[0].closed


@pytest.mark.parametrize("status,size", [(302, 2), (200, 16385), (409, 2)])
def test_http_never_follows_redirects_and_rejects_bad_responses(bench, status, size):
    requests = []
    source = object.__new__(bench.ViewerLeaderInput)
    source.token = "secret"
    source.connection = SimpleNamespace(
        request=lambda *a, **kw: requests.append((a, kw)),
        getresponse=lambda: SimpleNamespace(status=status, read=lambda maximum: b" " * min(size, maximum)))
    with pytest.raises(ValueError, match="rejected the request|oversized response"):
        source._request("GET", "/api/profile")
    assert len(requests) == 1


def test_transient_stale_reply_is_a_skipped_sample_that_keeps_the_connection(bench):
    from robot_platform.teleop.openrb import LeaderSampleSkipped
    source = object.__new__(bench.ViewerLeaderInput)
    source.token = "secret"
    closed = []
    body = b'{"error": "Cached leader source is stale", "transient": true}'
    source.connection = SimpleNamespace(
        request=lambda *a, **kw: None, close=lambda: closed.append(True),
        getresponse=lambda: SimpleNamespace(status=409, read=lambda maximum: body))
    with pytest.raises(LeaderSampleSkipped, match="stale"):
        source._exchange({"type": "sample", "nonce": "0" * 32})
    assert not closed
    source.connection.getresponse = lambda: SimpleNamespace(
        status=409, read=lambda maximum: b'{"error": "Leader pair age or skew exceeded"}')
    with pytest.raises(ValueError, match="skew exceeded") as error:
        source._exchange({"type": "sample", "nonce": "0" * 32})
    assert not isinstance(error.value, LeaderSampleSkipped) and closed


def test_full_cycle_budget_includes_input_before_dispatch_and_late_ack(bench):
    clock = [10.030]
    supervisor = object.__new__(bench.ShadowSupervisor)
    supervisor.clock = lambda: clock[0]
    supervisor.mapper = SimpleNamespace(validate_fresh=lambda *a, **kw: None)
    supervisor.cycle_budget_s, supervisor.period_s = .035, .04
    supervisor.input_started_at = supervisor.scheduled_at = 10.
    supervisor._before_dispatch({}, 10.028)
    clock[0] = 10.036
    with pytest.raises(TimeoutError, match="including leader input"):
        supervisor._before_dispatch({}, 10.028)
    supervisor.input_started_at = 10.030
    clock[0] = 10.041
    with pytest.raises(TimeoutError, match="scheduled"):
        supervisor._before_dispatch({}, 10.038)


def test_failed_input_does_not_reuse_previous_accepted_command_or_collision(bench):
    clock = [10.]
    previous = dict(accepted=True, collision={"clear": True})
    supervisor = SimpleNamespace(last_cycle=previous, compiler=SimpleNamespace(last_verdict={"clear": True}),
                                 stop_error=None, stop=lambda reason: None)

    def poll():
        clock[0] += .026
        raise TimeoutError("input timeout")

    record = bench.run_cycle(SimpleNamespace(poll=poll), supervisor, scheduled_at=10., clock=lambda: clock[0])
    assert record["accepted"] is False and "collision" not in record
    assert record["failure"] == "input timeout" and record["input_ms"] == pytest.approx(26.)


def test_late_accepted_command_stays_reported_as_accepted_and_stops(bench):
    clock, stops = [10.], []
    supervisor = SimpleNamespace(last_cycle=None, compiler=SimpleNamespace(last_verdict=None),
                                 stop_error=None, stop=lambda reason: stops.append(reason))

    def step(samples, engaged):
        assert engaged is True
        clock[0] += .038
        supervisor.last_cycle = dict(accepted=True, started_at=10., completed_at=clock[0])
        supervisor.compiler.last_verdict = dict(clear=True)
        raise TimeoutError("late acknowledgement")

    supervisor.step = step
    record = bench.run_cycle(SimpleNamespace(poll=lambda: {"left": sample()}), supervisor,
                              scheduled_at=10., clock=lambda: clock[0])
    assert record["accepted"] is True and record["failure"] == "late acknowledgement" and stops
    assert record["collision"]["clear"] is True
    summary = bench.summarize([record], None, record["failure"], None)
    assert summary["accepted_cycles"] == 1 and summary["cycle_budget_misses"] == 1
    assert summary["maximum_source_age_at_completion_ms"] == pytest.approx(40.)
    assert not summary["silent_producer_stop_verified"]


@pytest.mark.parametrize("reply", ["accepted_late", "lost", "rejected"])
def test_registration_records_arm_reply_before_post_ack_deadline_stop(bench, reply):
    clock, calls = [10.], []

    def exchange(request):
        calls.append(request["type"])
        if request["type"] == "arm":
            if reply == "lost":
                raise TimeoutError("lost acknowledgement")
            clock[0] = 10.036
            return dict(accepted=reply != "rejected", status={"state": "armed" if reply != "rejected" else "disarmed"})
        return dict(accepted=True, status={"state": "stopped"})

    recorded = bench.ArmAcknowledgement(exchange)

    def arm(**kw):
        result = recorded({"type": "arm"})
        if not result["accepted"]:
            raise RuntimeError("arm rejected")
        return result["status"]

    supervisor = object.__new__(bench.ShadowSupervisor)
    supervisor.clock = lambda: clock[0]
    supervisor.cycle_budget_s, supervisor.period_s = .035, .04
    supervisor.state, supervisor.reason, supervisor.owned = "idle", None, False
    supervisor.last_cycle, supervisor.stop_error = None, None
    supervisor.controller = SimpleNamespace(exchange=recorded, acquire=lambda: {"state": "disarmed"},
                                            arm=arm, stop=lambda: recorded({"type": "stop"}))
    supervisor.mapper = SimpleNamespace(validate_fresh=lambda *a, **kw: None,
                                         register=lambda *a, **kw: None, stop=lambda reason: None)
    supervisor.compiler = SimpleNamespace(last_verdict=None)
    supervisor._certify = lambda joints: ({}, [])
    record = bench.run_cycle(SimpleNamespace(poll=lambda: {"left": sample()}), supervisor,
                              registration=True, scheduled_at=10., clock=lambda: clock[0])
    expected = {"accepted_late": True, "lost": None, "rejected": False}[reply]
    assert record["accepted"] is expected
    assert record["arm_acknowledgement"]["accepted"] is expected
    assert record["failure"] and supervisor.state == "stopped" and calls == ["arm", "stop"]


def test_collision_rejection_in_registration_is_counted_and_stops(bench):
    stops = []
    supervisor = SimpleNamespace(last_cycle=None, compiler=SimpleNamespace(last_verdict=None),
                                 stop_error=None, stop=lambda reason: stops.append(reason))

    def begin(samples, engaged):
        supervisor.compiler.last_verdict = dict(clear=False, reason="insufficient_clearance")
        raise ValueError("Collision envelope unavailable")

    supervisor.begin = begin
    record = bench.run_cycle(SimpleNamespace(poll=lambda: {"left": sample()}), supervisor,
                              registration=True, scheduled_at=10., clock=lambda: 10.)
    summary = bench.summarize([], record, record["failure"], None)
    assert record["accepted"] is False and stops and summary["collision_rejections"] == 1
