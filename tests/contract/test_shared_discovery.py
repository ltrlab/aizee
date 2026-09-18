"""Discovery behavior with mocked TCP probes; never contacts a robot."""

from unittest.mock import MagicMock

import pytest

from robot_platform.transport import discovery


@pytest.mark.parametrize(
    ("endpoint", "expected"),
    [
        (None, (None, None)),
        ("", (None, None)),
        ("ipc:///tmp/robot", (None, None)),
        ("tcp://robot:5555", ("robot", "5555")),
        ("tcp://robot", ("robot", None)),
        ("tcp://[::1]:5555", ("[::1]", "5555")),
        ("tcp://[::1]", ("[::1]", None)),
    ],
)
def test_split_tcp_endpoint(endpoint, expected):
    assert discovery.split_tcp_endpoint(endpoint) == expected


def test_discovery_preserves_candidate_priority_and_ssh_fallback(monkeypatch):
    calls = []

    def reachable(host, ports, timeout):
        calls.append((host, ports, timeout))
        return host == "usb"

    monkeypatch.setattr(discovery, "host_reachable", reachable)
    result = discovery.resolve_host(
        "lan", 5555, timeout=0.05, fallback_hosts=("lan", "usb", "wifi", "usb")
    )
    assert result == ("usb", ["lan", "usb", "wifi"])
    assert calls == [("lan", [5555, "22"], 0.05), ("usb", [5555, "22"], 0.05)]


def test_unreachable_hosts_retain_primary_and_custom_settings_disable_fallbacks(monkeypatch):
    probe = MagicMock(return_value=False)
    monkeypatch.setattr(discovery, "host_reachable", probe)
    assert discovery.resolve_host("offline", None, fallback_hosts=()) == ("offline", ["offline"])
    probe.assert_called_once_with("offline", ["22"], 0.6)


def test_host_probe_closes_connection_and_tries_next_port(monkeypatch):
    connection = MagicMock()
    connect = MagicMock(side_effect=[OSError("connection refused"), connection])
    monkeypatch.setattr(discovery.socket, "create_connection", connect)
    assert discovery.host_reachable("[::1]", ["5555", "22"], timeout=0.05)
    assert [call.args for call in connect.call_args_list] == [(("::1", 5555),), (("::1", 22),)]
    connection.__enter__.assert_called_once()
    connection.__exit__.assert_called_once()


def test_legacy_discovery_imports_reexport_shared_functions():
    from python.scripts.collect_demo_app import config

    assert config._resolve_rover_host is discovery.resolve_host
    assert config._split_tcp_endpoint is discovery.split_tcp_endpoint
    assert config._host_reachable is discovery.host_reachable


def test_minerva_calls_shared_discovery(monkeypatch):
    from python.scripts.collect_minerva_app import config

    resolver = MagicMock(return_value=("selected", ["primary", "selected"]))
    monkeypatch.setattr(config, "resolve_host", resolver)
    assert config.resolve_jetson_host("primary", probe_port=5556, verbose=False) == "selected"
    resolver.assert_called_once_with("primary", "5556")
