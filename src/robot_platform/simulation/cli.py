"""Commands for loopback simulation and offline hardware/model audits."""
import argparse
import json
from pathlib import Path

from .server import ProtocolServer, SCENARIOS


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-platform sim")
    sub = parser.add_subparsers(dest="command", required=True)
    proxies = sub.add_parser("build-collision-model", help="generate broad-phase bounds; CAD confirms all final contacts")
    proxies.add_argument("--model", type=Path, required=True)
    proxies.add_argument("--output", type=Path, required=True, help="new runtime collision JSON; no source CAD changes")
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
    model = sub.add_parser("audit-model", help="audit a local URDF; never fetches CAD or opens devices")
    model_source = model.add_mutually_exclusive_group(required=True)
    model_source.add_argument("--urdf", type=Path)
    model_source.add_argument("--model", type=Path, help="shared mechanical model profile")
    model.add_argument("--package-root", type=Path, help="local root for package:// mesh paths")
    model.add_argument("--expectations", type=Path, help="JSON revolute/prismatic name lists, or source manifest with expected_joints")
    model.add_argument("--output", type=Path, help="new report file; must not exist")
    probe = sub.add_parser("validate-limit-probe", help="replay synthetic contact signals; no devices or motor commands")
    probe.add_argument("--output", type=Path, help="new report file; must not exist")
    collision = sub.add_parser("check-collision", help="offline whole-robot state/path check; no runtime interlock")
    collision_source = collision.add_mutually_exclusive_group(required=True)
    collision_source.add_argument("--urdf", type=Path)
    collision_source.add_argument("--model", type=Path, help="shared mechanical model profile")
    collision.add_argument("--package-root", type=Path)
    collision.add_argument("--scene", required=True, type=Path, help="JSON positions, optional end_positions/max_joint_step, allowed_pairs and environment boxes")
    collision.add_argument("--output", type=Path, help="new report file; must not exist")
    sweep = sub.add_parser("sweep-limits", help="offline per-branch collision limit candidates for visual review")
    sweep.add_argument("--model", type=Path, required=True)
    sweep.add_argument("--request", type=Path, required=True, help="explicit reference, branches, resolution and interface hypotheses")
    sweep.add_argument("--output", type=Path, required=True, help="new candidate report; never motor settings")
    args = parser.parse_args(argv)
    try:
        profile = None
        if args.command == "build-collision-model":
            from robot_platform.models import ModelProfile
            from .runtime_model import build_runtime_model
            report = build_runtime_model(ModelProfile.load(args.model), args.output)
            print(json.dumps({k: report[k] for k in ("model_id", "source_triangles", "proxy_triangles", "geometry_role", "geometry_reviewed")}))
            return 0
        if args.command == "sweep-limits":
            import sys
            from robot_platform.models import ModelProfile
            from .limit_sweep import sweep_model
            if args.output.exists():
                raise ValueError("Refusing to overwrite a sweep report")
            profile = ModelProfile.load(args.model)
            request = json.loads(args.request.read_bytes())
            def progress(name, result):
                print(f"{name}: {result['status']} {result['bounds']} {result['unit']}", file=sys.stderr, flush=True)
            report = sweep_model(profile, request, progress=progress)
            with args.output.open("x", encoding="utf-8") as file:
                file.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
            print(json.dumps({"report": str(args.output), "samples": report["samples_checked"],
                              "status": report["status"]}))
            return 0 if all(r["status"] == "candidate" for r in report["results"].values()) else 2
        if args.command in {"audit-model", "check-collision"}:
            if args.model:
                from robot_platform.models import ModelProfile
                if args.package_root:
                    raise ValueError("model profiles already declare their package roots")
                profile = ModelProfile.load(args.model)
                args.urdf, args.package_root = profile.urdf, profile.package_root
            elif args.package_root is None:
                args.package_root = args.urdf.parent
        if args.command == "check-collision":
            from dataclasses import asdict
            import hashlib
            from .collision import BoxObstacle, CollisionScene
            request = json.loads(args.scene.read_text(encoding="utf-8"))
            allowed_fields = {"positions", "end_positions", "max_joint_step", "allowed_pairs", "environment", "model", "schema_version"}
            if not isinstance(request, dict) or "positions" not in request or set(request) - allowed_fields:
                raise ValueError("scene requires positions and only documented optional fields")
            if ("end_positions" in request) != ("max_joint_step" in request):
                raise ValueError("path checks require both end_positions and max_joint_step")
            if "model" in request or "schema_version" in request:
                if request.get("schema_version") != 1 or not isinstance(request.get("model"), dict):
                    raise ValueError("invalid saved pose schema")
                if request["model"].get("urdf_sha256") != hashlib.sha256(args.urdf.read_bytes()).hexdigest():
                    raise ValueError("saved pose belongs to a different URDF revision")
            try:
                obstacles = tuple(BoxObstacle(**item) for item in request.get("environment", []))
                scene = CollisionScene.from_urdf(args.urdf, args.package_root,
                    allowed_pairs=request.get("allowed_pairs", []), environment=obstacles,
                    mesh_resolver=profile.resolve_mesh if profile else None)
            except TypeError as exc:
                raise ValueError("invalid scene pairs or environment boxes") from exc
            report = {"evidence": "offline_collision_geometry", "runtime_interlock_active": False,
                      "physical_model_validated": False, "continuous_collision_proof": False,
                      "urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
                      "scene_request_sha256": hashlib.sha256(args.scene.read_bytes()).hexdigest(),
                      "joint_units": scene.joint_units}
            if "end_positions" in request:
                result = scene.check_path(request["positions"], request["end_positions"],
                                          max_joint_step=request["max_joint_step"])
                report.update(asdict(result))
                report["checked_samples_clear"] = result.collision_free
            else:
                pairs = scene.check_state(request["positions"])
                report.update(samples_checked=1, colliding_pairs=pairs, checked_samples_clear=not pairs)
            encoded = json.dumps(report, indent=2, allow_nan=False)
            if args.output:
                with args.output.open("x", encoding="utf-8") as file:
                    file.write(encoded + "\n")
            print(encoded)
            return 0 if report["checked_samples_clear"] else 2
        if args.command == "validate-limit-probe":
            from robot_platform.characterization.limit_probe import synthetic_validation
            report = synthetic_validation()
            encoded = json.dumps(report, indent=2, allow_nan=False)
            if args.output:
                with args.output.open("x", encoding="utf-8") as file:
                    file.write(encoded + "\n")
            print(encoded)
            return 0 if report["passed"] else 2
        if args.command == "audit-model":
            from .model_audit import audit_urdf
            expected = {}
            if args.expectations:
                expected = json.loads(args.expectations.read_text(encoding="utf-8"))
                if not isinstance(expected, dict):
                    raise ValueError("expectations must be a JSON object")
                expected = expected.get("expected_joints", expected)
                if not isinstance(expected, dict) or not expected or set(expected) - {"revolute", "prismatic"}:
                    raise ValueError("expectations must contain revolute/prismatic name lists")
                if any(not isinstance(names, list) or any(not isinstance(name, str) or not name for name in names)
                       for names in expected.values()):
                    raise ValueError("expected joint names must be lists of nonempty strings")
            report = audit_urdf(args.urdf, args.package_root,
                                expected_revolute_joints=tuple(expected.get("revolute", ())),
                                expected_prismatic_joints=tuple(expected.get("prismatic", ())),
                                mesh_resolver=profile.resolve_mesh if profile else None)
            encoded = json.dumps(report, indent=2, allow_nan=False)
            if args.output:
                with args.output.open("x", encoding="utf-8") as file:
                    file.write(encoded + "\n")
            print(encoded)
            return 0 if report["static_checks_passed"] else 2
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
        return 0 if args.command == "run" else 130
    except (ValueError, OSError, RuntimeError) as exc:
        parser.error(str(exc))
