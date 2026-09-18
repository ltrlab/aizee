"""One CLI for robot selection, settings inspection and application launch."""
from __future__ import annotations

import argparse
import json
import sys

from robot_platform.apps.launcher import APPLICATIONS, build_launch_plan
from robot_platform.contracts import ConfigurationError
from robot_platform.robots import available_robots, load_robot
from robot_platform.settings import load_settings, merge_settings


def _overrides(values: list[str], host: str | None) -> dict:
    result = {}
    for expression in values:
        key, sep, raw = expression.partition("=")
        parts = key.split(".")
        if not sep or not all(parts):
            raise ConfigurationError("--set requires dotted.key=value")
        try:
            value = json.loads(raw)
        except ValueError:
            value = raw
        nested = value
        for part in reversed(parts):
            nested = {part: nested}
        result = merge_settings(result, nested)
    if host is not None:
        result["host"] = host
    return result


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="robot-platform", description="Shared Aizee / Minerva robot platform")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="list built-in robots and application adapters")
    sub.add_parser("sim", help="loopback simulation and hardware audit (sim --help)")
    for command in ("describe", "config", "run"):
        s = sub.add_parser(command, help={"describe": "show a robot definition", "config": "print validated resolved settings", "run": "launch an application or inspect its plan"}[command])
        s.add_argument("--robot", required=True, help="built-in robot name or definition JSON path")
        if command != "describe":
            s.add_argument("--settings", action="append", default=[], help="JSON overlay; repeat in increasing precedence")
            s.add_argument("--host", help="override endpoint host placeholders")
            s.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="final dotted settings override; values may be JSON")
        if command == "run":
            s.add_argument("application", choices=["collect", "train"])
            s.add_argument("--adapter", choices=sorted(APPLICATIONS), help="override the configured compatible application adapter")
            s.add_argument("--repo", help="legacy source checkout (defaults to editable install location)")
            s.add_argument("--plan", action="store_true", help="print the launch plan without opening devices or running the app")
    return p


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "sim":
        from robot_platform.simulation.cli import main as sim_main
        return sim_main(argv[1:])
    forwarded = []
    if "--" in argv:
        split = argv.index("--")
        argv, forwarded = argv[:split], argv[split + 1:]
    p = parser()
    args = p.parse_args(argv)
    try:
        if forwarded and args.command != "run":
            raise ConfigurationError("application arguments after -- require the run command")
        if args.command == "list":
            result = {"robots": list(available_robots()), "applications": [
                {"adapter": a.name, "command": a.command, "robot_contract": a.contract}
                for a in APPLICATIONS.values()]}
        else:
            spec = load_robot(args.robot)
            if args.command == "describe":
                result = spec.to_dict()
            else:
                settings = load_settings(spec, args.settings, _overrides(args.set, args.host))
                if args.command == "config":
                    result = settings
                else:
                    plan = build_launch_plan(spec, settings, args.application, repo=args.repo,
                                             adapter=args.adapter, extra_args=forwarded)
                    if not args.plan:
                        return plan.execute()
                    result = plan.to_dict()
        print(json.dumps(result, indent=2))
        return 0
    except (ConfigurationError, OSError) as exc:
        p.error(str(exc))
    return 2
