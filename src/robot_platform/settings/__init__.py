"""Compose robot defaults, ordered settings files and explicit overrides.

Reading/resolving settings never probes the network or imports an application.
"""
from __future__ import annotations

from copy import deepcopy
from importlib.resources import files
import json
from pathlib import Path
from urllib.parse import urlsplit

from robot_platform.contracts import ConfigurationError, RobotSpec


def merge_settings(base: dict, overlay: dict) -> dict:
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_settings(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _keys(data: object, allowed: set[str], label: str) -> None:
    if not isinstance(data, dict):
        raise ConfigurationError(f"{label} must be an object")
    extra = set(data) - allowed
    if extra:
        raise ConfigurationError(f"unknown {label} fields: {sorted(extra)}")


def validate_settings(spec: RobotSpec, data: dict) -> None:
    _keys(data, {"schema_version", "robot", "host", "endpoints", "control", "applications", "simulation"}, "settings")
    if "simulation" in data:
        sim = data["simulation"]
        _keys(sim, {"run_id", "evidence"}, "simulation")
        if not isinstance(sim.get("run_id"), str) or not sim["run_id"] or sim.get("evidence") != "synthetic":
            raise ConfigurationError("simulation requires a run_id and synthetic evidence label")
    if data.get("schema_version") != 1 or data.get("robot") != spec.name:
        raise ConfigurationError(f"settings require schema_version=1 and robot={spec.name!r}")
    host = data.get("host")
    if not isinstance(host, str) or not host or any(c in host for c in "/{} \t\r\n"):
        raise ConfigurationError("host must be a hostname or IP address")
    if "simulation" in data and host != "127.0.0.1":
        raise ConfigurationError("simulation settings require host=127.0.0.1")
    endpoints = data.get("endpoints")
    _keys(endpoints, {"arms", "cameras", "services"}, "endpoints")
    arms = endpoints.get("arms")
    if not isinstance(arms, dict) or not arms:
        raise ConfigurationError("endpoints.arms must contain at least one control group")
    routed = []
    motor_urls = []
    for group, endpoint in arms.items():
        _keys(endpoint, {"command", "telemetry", "joints"}, f"arm {group}")
        if group not in spec.groups or endpoint.get("joints") != list(spec.groups[group]):
            raise ConfigurationError(f"arm {group} joints must match the robot group's ordered joints")
        routed.extend(endpoint["joints"])
        motor_urls.extend([endpoint.get("command"), endpoint.get("telemetry")])
    if len(set(routed)) != len(routed) or set(routed) != set(spec.active_joint_names):
        raise ConfigurationError("arm routes must cover every active joint exactly once and no inactive joints")
    cameras = endpoints.get("cameras", {})
    services = endpoints.get("services", {})
    if not isinstance(cameras, dict) or not isinstance(services, dict):
        raise ConfigurationError("camera and service endpoints must be objects")
    known = {c.name for c in spec.cameras}
    required = {c.name for c in spec.cameras if c.required}
    if set(cameras) - known or required - set(cameras):
        raise ConfigurationError("camera endpoints must use known roles and include all required cameras")
    urls = motor_urls + list(cameras.values()) + list(services.values())
    for url in urls:
        try:
            parsed = urlsplit(url) if isinstance(url, str) else None
            if parsed is None or parsed.scheme != "tcp" or not parsed.hostname or parsed.port is None or parsed.path or parsed.query or parsed.fragment or parsed.username or not 1 <= parsed.port <= 65535:
                raise ValueError("expected tcp://host:port")
            if "simulation" in data and parsed.hostname != "127.0.0.1":
                raise ValueError("simulation endpoints must use 127.0.0.1")
        except ValueError as exc:
            raise ConfigurationError(f"invalid endpoint {url!r}: {exc}") from exc
    if len(set(urls)) != len(urls):
        raise ConfigurationError("endpoint addresses must be distinct within a robot profile")
    control = data.get("control", {})
    _keys(control, {"stale_after_s", "target_lease_s", "require_source_freshness"}, "control")
    lease = control.get("target_lease_s", 0.5)
    if isinstance(lease, bool) or not isinstance(lease, (float, int)) or not 0 < lease <= 60:
        raise ConfigurationError("target_lease_s must be positive and at most 60")
    if not isinstance(control.get("require_source_freshness", False), bool):
        raise ConfigurationError("require_source_freshness must be boolean")
    stale = control.get("stale_after_s", 0.5)
    if isinstance(stale, bool) or not isinstance(stale, (float, int)) or not 0 < stale <= 60:
        raise ConfigurationError("stale_after_s must be positive and at most 60")
    apps = data.get("applications", {})
    _keys(apps, {"collect", "train"}, "applications")
    for name, app in apps.items():
        _keys(app, {"adapter", "gui", "data_dir", "output_dir", "options"}, f"application {name}")
        if not isinstance(app.get("adapter"), str):
            raise ConfigurationError(f"application {name} needs an adapter")
        if "gui" in app and not isinstance(app["gui"], bool):
            raise ConfigurationError("gui must be boolean")
        for key in ("data_dir", "output_dir"):
            if key in app and (not isinstance(app[key], str) or not app[key]):
                raise ConfigurationError(f"{key} must be a nonempty path string")
        options = app.get("options", [])
        if not isinstance(options, list) or any(not isinstance(o, str) for o in options):
            raise ConfigurationError("application options must be a list of argument strings")


def load_settings(spec: RobotSpec, paths=(), overrides: dict | None = None) -> dict:
    resource = files(__package__).joinpath("defaults", spec.name + ".json")
    data = json.loads(resource.read_text(encoding="utf-8")) if resource.is_file() else {
        "schema_version": 1, "robot": spec.name, "host": "localhost", "endpoints": {"arms": {}},
    }
    for value in paths:
        path = Path(value).expanduser()
        try:
            overlay = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ConfigurationError(f"cannot load settings {path}: {exc}") from exc
        if not isinstance(overlay, dict):
            raise ConfigurationError(f"settings {path} must be an object")
        data = merge_settings(data, overlay)
    data = merge_settings(data, overrides or {})
    host = data.get("host", "")
    if not isinstance(host, str):
        raise ConfigurationError("host must be a string")
    address = f"[{host}]" if ":" in host and not host.startswith("[") else host

    def substitute(value):
        if isinstance(value, dict):
            return {key: substitute(v) for key, v in value.items()}
        if isinstance(value, list):
            return [substitute(v) for v in value]
        return value.replace("{host}", address) if isinstance(value, str) else value

    data["endpoints"] = substitute(data.get("endpoints", {}))
    # JSON null explicitly disables an optional camera. Required and unknown
    # roles remain subject to validation instead of silently disappearing.
    cameras = data["endpoints"].get("cameras", {}) if isinstance(data["endpoints"], dict) else {}
    optional_cameras = {c.name for c in spec.cameras if not c.required}
    if isinstance(cameras, dict):
        for name in optional_cameras:
            if name in cameras and cameras[name] is None:
                del cameras[name]
    validate_settings(spec, data)
    return data
