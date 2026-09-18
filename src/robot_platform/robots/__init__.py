"""Robot definitions are resources, loaded without connecting to a robot."""
from __future__ import annotations

from importlib.resources import files
import json
from pathlib import Path

from robot_platform.contracts import ConfigurationError, RobotSpec


def available_robots() -> tuple[str, ...]:
    return tuple(sorted(p.name[:-5] for p in files(__package__).joinpath("definitions").iterdir()
                        if p.name.endswith(".json")))


def load_robot(name_or_path: str | Path) -> RobotSpec:
    value = str(name_or_path)
    if value in available_robots():
        source = files(__package__).joinpath("definitions", value + ".json")
    else:
        source = Path(value).expanduser()
        if not source.is_file():
            raise ConfigurationError(f"unknown robot {value!r}; choose {', '.join(available_robots())} or a definition JSON file")
    try:
        return RobotSpec.from_dict(json.loads(source.read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError) as exc:
        raise ConfigurationError(f"cannot load robot {value!r}: {exc}") from exc
