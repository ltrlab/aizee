"""Commands for loopback simulation and read-only hardware audits."""
import argparse
import json
from pathlib import Path

from .server import ProtocolServer, SCENARIOS


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-platform sim")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="start synthetic loopback motors and cameras")
    run.add_argument("--robot", required=True, choices=("aizee", "minerva"))
    run.add_argument("--output", required=True, help="new run directory; must not exist")
    run.add_argument("--duration", type=float, default=0, help="seconds; zero runs until Ctrl-C")
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    run.add_argument("--no-cameras", action="store_true")
    audit = sub.add_parser("audit", help="read-only inventory; never opens devices")
    audit.add_argument("--repo", type=Path, default=Path.cwd())
    audit.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "audit":
            from robot_platform.characterization.audit import audit_repository
            report = audit_repository(args.repo)
            encoded = json.dumps(report, indent=2)
            if args.output:
                with args.output.open("x", encoding="utf-8") as file:
                    file.write(encoded)
            print(encoded)
            return 0 if report["hardware_ready"] else 2
        with ProtocolServer(args.robot, args.output, seed=args.seed, scenario=args.scenario, cameras=not args.no_cameras) as server:
            print(f"SIMULATION ONLY: {args.robot}; dynamics are synthetic", flush=True)
            print(f"Settings: {server.output / 'settings.json'}", flush=True)
            print(f'robot-platform run collect --robot {args.robot} --settings "{server.output / "settings.json"}"', flush=True)
            server.run(args.duration)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, OSError, RuntimeError) as exc:
        parser.error(str(exc))
