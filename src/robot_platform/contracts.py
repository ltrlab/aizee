"""Validated robot vocabulary, independent of applications and hardware libraries."""
from __future__ import annotations

from dataclasses import dataclass
import re


class ConfigurationError(ValueError):
    """A robot definition or settings combination is invalid."""


def identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[a-z][a-z0-9_-]*", value):
        raise ConfigurationError(f"{field} must be a lowercase identifier")
    return value


@dataclass(frozen=True)
class JointSpec:
    name: str
    unit: str
    active: bool = True


@dataclass(frozen=True)
class CameraSpec:
    name: str
    required: bool = True


@dataclass(frozen=True)
class RobotSpec:
    name: str
    joints: tuple[JointSpec, ...]
    groups: dict[str, tuple[str, ...]]
    cameras: tuple[CameraSpec, ...]
    capabilities: tuple[str, ...]
    schema_version: int = 1

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.name for j in self.joints)

    @property
    def active_joint_names(self) -> tuple[str, ...]:
        return tuple(j.name for j in self.joints if j.active)

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version, "name": self.name,
            "joints": [{"name": j.name, "unit": j.unit, "active": j.active} for j in self.joints],
            "groups": {name: list(joints) for name, joints in self.groups.items()},
            "cameras": [{"name": c.name, "required": c.required} for c in self.cameras],
            "capabilities": list(self.capabilities),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "RobotSpec":
        if not isinstance(data, dict):
            raise ConfigurationError("robot definition must be an object")
        allowed = {"schema_version", "name", "joints", "groups", "cameras", "capabilities"}
        if set(data) - allowed:
            raise ConfigurationError(f"unknown robot fields: {sorted(set(data) - allowed)}")
        if data.get("schema_version") != 1:
            raise ConfigurationError("robot schema_version must be 1")
        name = identifier(data.get("name"), "robot name")
        if not isinstance(data.get("joints"), list) or not data["joints"]:
            raise ConfigurationError("robot joints must be a nonempty list")
        joints = []
        for j in data["joints"]:
            if not isinstance(j, dict) or set(j) - {"name", "unit", "active"}:
                raise ConfigurationError("invalid joint definition")
            unit, active = j.get("unit"), j.get("active", True)
            if unit not in ("rad", "m", "normalized") or not isinstance(active, bool):
                raise ConfigurationError("joint requires unit rad/m/normalized and boolean active")
            joints.append(JointSpec(identifier(j.get("name"), "joint name"), unit, active))
        names = [j.name for j in joints]
        if len(set(names)) != len(names):
            raise ConfigurationError("joint names must be unique")
        if not isinstance(data.get("groups"), dict):
            raise ConfigurationError("robot groups must be an object")
        groups = {}
        for key, members in data["groups"].items():
            identifier(key, "group name")
            if not isinstance(members, list) or not members or any(not isinstance(m, str) for m in members):
                raise ConfigurationError(f"group {key} must contain joint names")
            if len(set(members)) != len(members) or not set(members) <= set(names):
                raise ConfigurationError(f"group {key} has duplicate or unknown joints")
            groups[key] = tuple(members)
        cameras = []
        if not isinstance(data.get("cameras", []), list):
            raise ConfigurationError("cameras must be a list")
        for camera in data.get("cameras", []):
            if not isinstance(camera, dict) or set(camera) - {"name", "required"}:
                raise ConfigurationError("invalid camera definition")
            required = camera.get("required", True)
            if not isinstance(required, bool):
                raise ConfigurationError("camera required must be boolean")
            cameras.append(CameraSpec(identifier(camera.get("name"), "camera name"), required))
        if len({c.name for c in cameras}) != len(cameras):
            raise ConfigurationError("camera names must be unique")
        caps = data.get("capabilities", [])
        if not isinstance(caps, list):
            raise ConfigurationError("capabilities must be a list")
        capabilities = tuple(identifier(c, "capability") for c in caps)
        if len(set(capabilities)) != len(capabilities):
            raise ConfigurationError("capabilities must be unique")
        return cls(name, tuple(joints), groups, tuple(cameras), capabilities)
