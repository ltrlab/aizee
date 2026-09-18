"""Launch existing applications through validated robot/settings contracts.

The compatibility applications still use source-checkout assets and calibration.
This boundary is explicit: shared APIs and profiles work from wheels, while these
adapters require an editable checkout or --repo / ROBOT_PLATFORM_REPO.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys

from robot_platform.contracts import ConfigurationError, RobotSpec
from robot_platform.robots import load_robot
from robot_platform.settings import validate_settings


@dataclass(frozen=True)
class Application:
    name: str
    command: str
    contract: str
    script: str


APPLICATIONS = {
    a.name: a for a in (
        Application("aizee.collect", "collect", "aizee", "python/scripts/collect_demo.py"),
        Application("minerva.collect", "collect", "minerva", "python/scripts/collect_minerva.py"),
        Application("aizee.train", "train", "aizee", "python/training/train.py"),
        Application("minerva.train", "train", "minerva", "python/training/train_minerva.py"),
    )
}


def repository_root(explicit: str | Path | None = None) -> Path:
    override = explicit or os.environ.get("ROBOT_PLATFORM_REPO")
    candidates = [Path(override).expanduser()] if override else list(Path(__file__).resolve().parents)
    for candidate in candidates:
        if (candidate / "pyproject.toml").is_file() and (candidate / "python/scripts/collect_demo.py").is_file():
            return candidate.resolve()
    raise ConfigurationError("legacy application adapters require the source checkout; install with pip install -e . or pass --repo / ROBOT_PLATFORM_REPO")


def _compatible(spec: RobotSpec, app: Application) -> None:
    expected = load_robot(app.contract)
    if (spec.joints != expected.joints or spec.groups != expected.groups
            or spec.cameras != expected.cameras or set(spec.capabilities) != set(expected.capabilities)):
        raise ConfigurationError(f"{app.name} requires the {app.contract} joint/camera/capability contract; robot {spec.name!r} is incompatible")


@dataclass(frozen=True)
class LaunchPlan:
    application: str
    robot: str
    argv: tuple[str, ...]
    cwd: str
    settings_json: str

    @property
    def settings(self) -> dict:
        return json.loads(self.settings_json)

    def to_dict(self) -> dict:
        return {"application": self.application, "robot": self.robot, "argv": list(self.argv),
                "cwd": self.cwd, "settings": self.settings,
                "note": "Source-checkout compatibility application. Plan creation does not import the app or contact hardware."}

    def execute(self) -> int:
        env = os.environ.copy()
        env["ROBOT_PLATFORM_SETTINGS_JSON"] = self.settings_json
        # Legacy help/status text contains Unicode and is often piped on Windows.
        env.setdefault("PYTHONIOENCODING", "utf-8")
        return subprocess.call(list(self.argv), cwd=self.cwd, env=env)


def build_launch_plan(spec: RobotSpec, settings: dict, command: str, *,
                      repo: str | Path | None = None, adapter: str | None = None,
                      extra_args=(), caller_cwd: str | Path | None = None) -> LaunchPlan:
    validate_settings(spec, settings)
    options = settings.get("applications", {}).get(command)
    if options is None:
        raise ConfigurationError(f"no {command!r} application configured for {spec.name}")
    selected = adapter or options["adapter"]
    app = APPLICATIONS.get(selected)
    if app is None or app.command != command:
        raise ConfigurationError(f"unknown or incompatible {command} adapter {selected!r}")
    _compatible(spec, app)
    root = repository_root(repo)
    script = root / app.script
    if not script.is_file():
        raise ConfigurationError(f"application entry point is missing: {script}")
    caller = Path(caller_cwd or Path.cwd()).resolve()

    def path_option(key: str) -> str:
        value = options.get(key)
        if not value:
            raise ConfigurationError(f"application {command} requires {key}")
        path = Path(value).expanduser()
        return str(path.resolve() if path.is_absolute() else (caller / path).resolve())

    endpoints = settings["endpoints"]
    argv = [sys.executable, str(script)]
    managed = {"--output-dir", "--data-dir", "--config", "--host", "--cmd", "--telem",
               "--gripper-cam", "--gripper-cam-ctrl", "--scene-cam", "--ups", "--gui", "--no-gui",
               "--left-cmd", "--left-telem", "--right-cmd", "--right-telem"}
    if command == "train":
        argv += ["--data-dir", path_option("data_dir"), "--output-dir", path_option("output_dir")]
    else:
        argv += ["--output-dir", path_option("output_dir")]
        if app.contract == "aizee":
            arm = endpoints["arms"]["main"]
            argv += ["--cmd", arm["command"], "--telem", arm["telemetry"],
                     "--gripper-cam", endpoints["cameras"]["gripper"],
                     "--scene-cam", endpoints["cameras"].get("scene", "")]
            services = endpoints.get("services", {})
            for role, flag in (("ups", "--ups"), ("gripper_camera_control", "--gripper-cam-ctrl")):
                if role not in services:
                    raise ConfigurationError(f"aizee.collect requires the {role} service endpoint")
                argv += [flag, services[role]]
            if options.get("gui", True):
                argv.append("--gui")
        else:
            argv += ["--host", settings["host"], "--config", str(root / "config/minerva.yaml"),
                     "--gui" if options.get("gui", True) else "--no-gui"]
            for side in ("left", "right"):
                arm = endpoints["arms"][side]
                argv += [f"--{side}-cmd", arm["command"], f"--{side}-telem", arm["telemetry"]]
    forwarded = list(options.get("options", [])) + list(extra_args)
    for value in forwarded:
        option = value.split("=", 1)[0]
        if option.startswith("--") and any(flag.startswith(option) for flag in managed):
            # Existing argparse entry points accept abbreviated long options.
            # A prefix must not bypass endpoint/path ownership by the profile.
            raise ConfigurationError(f"{option} is managed by settings; change the settings field or use --set")
    argv.extend(forwarded)
    return LaunchPlan(app.name, spec.name, tuple(argv), str(root), json.dumps(settings, allow_nan=False))
