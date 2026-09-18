#!/usr/bin/env python3
"""Build a static repository inventory without importing or running robot code.

Run from any directory: python <repo>/scripts/repo_inventory.py
Inputs: git-tracked paths and their current working-tree contents. Untracked and
ignored data are excluded. Destination areas are proposals, not move commands.
"""
from __future__ import annotations

import ast
from collections import Counter
import json
from pathlib import Path
import subprocess
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "architecture"

# Order matters: specific exceptions precede directory-wide classifications.
AREAS = {
    "vendor": ("Vendored OpenRB board support", "third_party/openrb150_bsp/"),
    "robot_assets": ("Aizee robot model and meshes", "robots/aizee/assets/"),
    "rust_control": ("Shared Rust motor backend; morphology assumptions remain", "crates/motor_control/"),
    "rust_comms": ("Shared Rust wire protocol", "crates/comms/"),
    "rust_lidar": ("Reusable LiDAR driver; optional rover service", "crates/lidar_control/"),
    "rust_workspace": ("Rust workspace metadata", "Cargo.toml at repository root"),
    "platform": ("Implemented shared Python package", "src/robot_platform/ (current)"),
    "settings": ("Shared application and bench settings examples", "settings/examples/ (current)"),
    "wire": ("Shared Python wire protocol", "src/robot_platform/transport/"),
    "robot_definition": ("Robot-specific constants mixed with reusable helpers", "src/robot_platform/robots/ + shared helpers (split)"),
    "gravity": ("Robot-specific gravity models and reusable fitting math", "src/robot_platform/control/ + robots/ (split)"),
    "kinematics": ("Kinematic algorithms with Aizee model assumptions", "src/robot_platform/kinematics/ + robots/aizee/ (split)"),
    "aizee_app": ("Aizee collector and supporting application code", "src/robot_platform/apps/collect/aizee/ (extract shared services)"),
    "minerva_app": ("Minerva collector and supporting application code", "src/robot_platform/apps/collect/minerva/ (extract shared services)"),
    "leaders": ("Leader drivers mixed with Aizee joint mapping", "src/robot_platform/devices/leaders/ + robots/aizee/ (split)"),
    "webxr": ("WebXR transport/UI with Aizee IK integration", "src/robot_platform/apps/webxr/ + devices/leaders/ (split)"),
    "learning": ("Learning algorithms, datasets, training and analysis CLIs", "src/robot_platform/learning/ + data/ + apps/train/ (split)"),
    "policy": ("Robot-specific inference applications", "src/robot_platform/apps/policy/"),
    "services": ("Device service candidates with deployment defaults", "src/robot_platform/devices/ + apps/services/ (split)"),
    "operations": ("Heartbeat/setup operations with machine and robot assumptions", "src/robot_platform/apps/operations/ + deploy/ (split)"),
    "bench": ("Actuator characterization tooling", "src/robot_platform/apps/bench/ + devices/ (split)"),
    "python_tools": ("Calibration, teleop, replay and diagnostic entry points", "src/robot_platform/apps/ (classify by function before moving)"),
    "tests": ("Offline unit, contract, GUI and learning checks", "tests/unit/ + contract/ + ml/; remaining python/tests/ checks"),
    "deployment": ("Deployment and host administration", "deploy/"),
    "calibration": ("Active calibration, alignment and gravity inputs", "robots/<robot>/calibration/examples/; active values in local state"),
    "profiles": ("Robot, camera and teleop configuration", "robots/<robot>/profiles/ + deploy/machines/ (split)"),
    "firmware": ("Embedded device projects", "firmware/ (retain device ownership)"),
    "docs": ("Documentation and actuator reference material", "docs/ (organize platform, robots, operations, research)"),
    "assets": ("Repository illustrations and research output images", "docs/assets/; future generated plots outside source"),
    "repo": ("Repository metadata and dependency declarations", "repository root; role-specific packaging"),
}


def area(p: str) -> str:
    if p.startswith("src/robot_platform/"):
        return "platform"
    if p.startswith("settings/"):
        return "settings"
    if p in ("Cargo.toml", "Cargo.lock", ".cargo/config.toml"):
        return "rust_workspace"
    if "/openrb150_bsp/" in p:
        return "vendor"
    if p.startswith("urdf/"):
        return "robot_assets"
    if p.startswith("rust/"):
        for name, key in (("motor_control", "rust_control"), ("comms", "rust_comms"),
                          ("lidar_control", "rust_lidar")):
            if p.startswith(f"rust/{name}/"):
                return key
        return "rust_workspace"
    if p.startswith(("tests/", "python/tests/")) or Path(p).name.startswith("test_"):
        return "tests"
    if p == "python/common/wire.py":
        return "wire"
    if p.startswith("python/common/"):
        return "robot_definition"
    if p.startswith("python/control/"):
        return "gravity"
    if p.startswith("python/ik/"):
        return "kinematics"
    if p.startswith("python/scripts/collect_demo"):
        return "aizee_app"
    if p.startswith("python/scripts/collect_minerva"):
        return "minerva_app"
    if p.startswith("python/teleop/") and Path(p).name != "teleop.py":
        return "leaders"
    if p.startswith("python/web/"):
        return "webxr"
    if p.startswith("python/training/"):
        return "learning"
    if p.endswith(("/act_policy_node.py", "/minerva_policy_node.py")):
        return "policy"
    if p.endswith("/find_motors.py"):
        return "python_tools"
    if p.startswith("python/nodes/"):
        return "services"
    if p.startswith("python/tools/"):
        return "operations"
    if p.startswith("python/scripts/actuator_char/") or p.endswith("/characterize_actuator.py"):
        return "bench"
    if p.startswith("python/"):
        return "python_tools"
    if p.startswith(("scripts/", "config/systemd/", "config/udev/")):
        return "deployment"
    if p.startswith("config/"):
        return "calibration" if p.endswith(".json") else "profiles"
    if p.startswith("firmware/"):
        return "firmware"
    if p.startswith("docs/") or p == "README.md":
        return "docs"
    if p.startswith("viz/") or p == "image.png":
        return "assets"
    return "repo"


def main() -> None:
    def git(*args: str) -> str:
        return subprocess.check_output(["git", "-C", str(ROOT), *args]).decode("utf-8")

    paths = sorted(p for p in git("ls-files", "-z").split("\0") if p and (ROOT / p).is_file())
    path_set = set(paths)
    entries, edges, parse_errors = [], [], []
    roots = ("", "src/", "python/", "python/scripts/", "python/teleop/")

    def resolve(module: str) -> list[str]:
        stem = module.replace(".", "/")
        return sorted({candidate for root in roots
                       for candidate in (root + stem + ".py", root + stem + "/__init__.py")
                       if candidate in path_set})

    for p in paths:
        f = ROOT / p
        entry = {"path": p, "area": area(p), "present": f.is_file()}
        if f.is_file() and p not in ("docs/architecture/repository_inventory.json", "docs/architecture/REPOSITORY_INVENTORY.md"):
            entry["working_tree_bytes"] = f.stat().st_size
        if f.is_file() and f.suffix in (".py", ".rs", ".cpp", ".h", ".sh", ".js") and area(p) != "vendor":
            source = f.read_text(encoding="utf-8-sig")
            entry["lines"] = len(source.splitlines())
            if f.suffix == ".py":
                entry["sys_path_mutation_lines"] = [i for i, line in enumerate(source.splitlines(), 1)
                                                     if "sys.path.insert(" in line or "sys.path.append(" in line]
                try:
                    tree = ast.parse(source, filename=p)
                except SyntaxError as exc:
                    parse_errors.append({"path": p, "error": str(exc)})
                else:
                    for node in ast.walk(tree):
                        modules = []
                        if isinstance(node, ast.Import):
                            modules = [alias.name for alias in node.names]
                        elif isinstance(node, ast.ImportFrom):
                            prefix = node.module or ""
                            if node.level:
                                parent = p.rsplit("/", 1)[0].split("/")
                                base = parent[:len(parent) - node.level + 1]
                                prefix = ".".join(base + ([prefix] if prefix else []))
                            modules = [prefix] + [prefix + "." + alias.name for alias in node.names]
                        targets = {target for module in modules for target in resolve(module)}
                        for target in sorted(targets):
                            edges.append({"source": p, "line": node.lineno, "target": target})
        entries.append(entry)

    edges.sort(key=lambda e: (e["source"], e["line"], e["target"]))
    snapshot = {"head": git("rev-parse", "HEAD").strip(), "branch": git("branch", "--show-current").strip(),
                "scope": "Present Git-tracked paths, working-tree contents; excludes untracked/ignored files. Generated inventory sizes omitted to avoid self-reference.",
                "import_method": "AST candidate imports resolved against known repository roots; includes conditional/test imports; not a runtime call graph. Dynamic imports, shell/config references and cross-language edges are not resolved.",
                "areas": {k: {"description": v[0], "proposed_area": v[1]} for k, v in AREAS.items()},
                "files": entries, "python_import_edges": edges, "parse_errors": parse_errors}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "repository_inventory.json").write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    counts = Counter(e["area"] for e in entries)
    python_entries = [e for e in entries if e["path"].endswith(".py")]
    path_hacks = [e for e in python_entries if e.get("sys_path_mutation_lines")]
    md = ["# Repository file inventory", "", "Generated by `scripts/repo_inventory.py`. Do not edit this file by hand.", "",
          f"Baseline: `{snapshot['head'][:7]}` on `{snapshot['branch']}`. {snapshot['scope']}", "",
          "[Architecture map](REPOSITORY_MAP.md) · [Monorepo plan](MONOREPO_PLAN.md) · [Machine-readable inventory](repository_inventory.json)", "",
          f"- **{len(entries)} tracked files**; **{len(python_entries)} Python files**.",
          f"- **{len(path_hacks)} Python files** contain literal `sys.path.insert/append` calls.",
          f"- **{len(edges)} candidate local Python import edges** (source line → target file).",
          f"- **{len(parse_errors)} Python parse errors** during static inspection.", "",
          "Import counts include conditional and test imports. They do not prove runtime reachability or dead code. "
          "Line counts include comments and blank lines. Byte sizes reflect the local checkout, including hydrated LFS files.", "",
          "## Ownership and destination areas", "", "Destinations are recommendations; mixed modules need extraction before relocation.", "",
          "| Area | Files | Proposed area |", "|---|---:|---|"]
    for key in AREAS:
        if counts[key]:
            desc, target = AREAS[key]
            md.append(f"| {desc} | {counts[key]} | `{target}` |")
    md += ["", "## Largest authored code files", "", "| File | Lines |", "|---|---:|"]
    for e in sorted((e for e in entries if "lines" in e), key=lambda e: (-e["lines"], e["path"]))[:15]:
        md.append(f"| [{e['path']}](../../{quote(e['path'])}) | {e['lines']} |")
    md += ["", "## File catalogue", "", "Vendor and mesh files are summarized here and enumerated individually in the JSON inventory."]
    for key, (desc, target) in AREAS.items():
        members = [e for e in entries if e["area"] == key]
        if not members:
            continue
        md += ["", f"### {desc}", "", f"Proposed area: `{target}`", ""]
        if key in ("vendor", "robot_assets"):
            md += [f"{len(members)} tracked files. See the JSON inventory for all paths."]
        else:
            for e in members:
                suffix = f" — {e['lines']} lines" if "lines" in e else ""
                md.append(f"- [{e['path']}](../../{quote(e['path'])}){suffix}")
    (OUT / "REPOSITORY_INVENTORY.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(json.dumps({"tracked_files": len(entries), "python_files": len(python_entries),
                      "path_hack_files": len(path_hacks), "local_import_edges": len(edges),
                      "parse_errors": parse_errors, "counts": counts}, indent=2))


if __name__ == "__main__":
    main()
