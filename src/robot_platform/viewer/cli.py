"""One viewer command for catalog models, external profiles or raw URDFs."""
import argparse
import os
from pathlib import Path

from robot_platform.models import ModelProfile, discover_models


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-platform view")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--robot", help="select an id discovered in the model catalog")
    source.add_argument("--model", type=Path, help="external model.json profile")
    source.add_argument("--urdf", type=Path, help="standalone URDF; groups inferred")
    parser.add_argument("--models-dir", type=Path, help="directory of */model.json profiles")
    parser.add_argument("--package-root", type=Path, help="asset root for --urdf")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.port <= 65535:
            raise ValueError("port must be in 1..65535")
        if args.package_root and not args.urdf:
            raise ValueError("--package-root requires --urdf; profiles declare their own paths")
        if args.model:
            profiles = [ModelProfile.load(args.model)]
        elif args.urdf:
            profiles = [ModelProfile.from_urdf(args.urdf, args.package_root)]
        else:
            directory = args.models_dir
            if directory is None:
                candidates = [Path(os.environ["ROBOT_PLATFORM_REPO"])] if os.environ.get("ROBOT_PLATFORM_REPO") else [Path.cwd(), *Path(__file__).resolve().parents]
                directory = next((p / "urdf" for p in candidates if (p / "urdf").is_dir()), None)
            if directory is None:
                raise ValueError("Provide --models-dir, --model or --urdf outside a source checkout")
            profiles = discover_models(directory)
        from aiohttp import web
        from .server import create_app
        app = create_app(profiles, args.robot)
        print(f"Model viewer: http://127.0.0.1:{args.port} (rendered joints only)", flush=True)
        web.run_app(app, host="127.0.0.1", port=args.port, print=None)
        return 0
    except (ValueError, OSError, ImportError) as exc:
        parser.error(str(exc))
