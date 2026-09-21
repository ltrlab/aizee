"""Local, robot-independent URDF metadata and asset resolution.

Mechanical coordinates are separate from actuator/recording RobotSpec slots.
No model/profile operation opens a device or changes a calibration file.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path, PureWindowsPath
import re
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET


def numbers(value, count, label, positive=False):
    try:
        values = [float(x) for x in (value.split() if isinstance(value, str) else value)]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: expected {count} finite numbers") from exc
    if len(values) != count or any(not math.isfinite(x) or (positive and x <= 0) for x in values):
        raise ValueError(f"{label}: expected {count} finite {'positive ' if positive else ''}numbers")
    return values


def origin(element):
    nodes = element.findall("origin")
    if len(nodes) > 1:
        raise ValueError("Multiple origins on one element")
    node = nodes[0] if nodes else ET.Element("origin")
    return {key: numbers(node.get(key, "0 0 0"), 3, key) for key in ("xyz", "rpy")}


@dataclass
class ModelProfile:
    id: str
    title: str
    urdf: Path
    package_root: Path
    packages: dict = field(default_factory=dict)
    groups: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    provisional_limits: list = field(default_factory=list)
    profile_sha256: str | None = None
    limit_review: Path | None = None
    front_direction: list = field(default_factory=lambda: [1., 0., 0.])
    collision_model: Path | None = None
    collision_branches: dict = field(default_factory=dict)
    keepout_ignored_links: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        raw = path.read_bytes()
        data = json.loads(raw)
        keys = {"schema_version", "id", "title", "urdf", "package_root", "packages", "groups", "notes", "provisional_limits", "limit_review", "front_direction", "collision_model", "collision_branches", "keepout_ignored_links"}
        if not isinstance(data, dict) or set(data) - keys or data.get("schema_version") != 1:
            raise ValueError(f"Invalid model profile: {path}")
        if not isinstance(data.get("id"), str) or not re.fullmatch(r"[a-z][a-z0-9_-]*", data["id"]):
            raise ValueError("Model id must be a lowercase identifier")
        if not isinstance(data.get("title"), str) or not data["title"]:
            raise ValueError("Model title must be nonempty")
        def local(value):
            if not isinstance(value, str) or not value or "://" in value:
                raise ValueError("Profile paths must name local files/directories")
            return (path.parent / value).resolve()
        urdf = local(data.get("urdf"))
        packages = data.get("packages", {})
        groups = data.get("groups", {})
        if not isinstance(packages, dict) or any(not isinstance(k, str) or not k for k in packages):
            raise ValueError("packages must map package names to directories")
        if not isinstance(groups, dict) or any(not k or not isinstance(v, list) or not v or any(not isinstance(n, str) for n in v) for k, v in groups.items()):
            raise ValueError("groups must map labels to joint-name lists")
        for key in ("notes", "provisional_limits"):
            if not isinstance(data.get(key, []), list) or any(not isinstance(s, str) for s in data.get(key, [])):
                raise ValueError(f"{key} must be a string list")
        front = numbers(data.get("front_direction", [1, 0, 0]), 3, "front_direction")
        length = math.hypot(*front[:2])
        if not math.isfinite(length) or length == 0 or front[2] != 0:
            raise ValueError("front_direction must be a nonzero horizontal direction (Z up)")
        front = [front[0]/length, front[1]/length, 0.]
        return cls(data["id"], data["title"], urdf,
                   local(data["package_root"]) if "package_root" in data else urdf.parent,
                   {k: local(v) for k, v in packages.items()}, groups,
                   data.get("notes", []), data.get("provisional_limits", []), hashlib.sha256(raw).hexdigest(),
                   local(data["limit_review"]) if "limit_review" in data else None, front_direction=front,
                   collision_model=local(data["collision_model"]) if "collision_model" in data else None,
                   collision_branches=data.get("collision_branches", {}),
                   keepout_ignored_links=data.get("keepout_ignored_links", {}))

    @classmethod
    def from_urdf(cls, path, package_root=None):
        path = Path(path).resolve()
        return cls("custom-" + hashlib.sha256(str(path).encode()).hexdigest()[:10], path.stem,
                   path, Path(package_root).resolve() if package_root else path.parent)

    def resolve_mesh(self, uri):
        """Resolve only selected local geometry, including legacy Windows URIs."""
        uri = unquote(uri).replace("\\", "/")
        parsed = urlsplit(uri)
        if parsed.query or parsed.fragment or not uri or "\x00" in uri:
            raise ValueError(f"Invalid mesh URI: {uri!r}")
        if parsed.scheme == "package":
            package = parsed.netloc
            packages = dict(self.packages)
            package_name = ET.parse(self.urdf).getroot().get("name")
            if (self.package_root / "package.xml").is_file():
                package_name = ET.parse(self.package_root / "package.xml").getroot().findtext("name")
            packages.setdefault(package_name, self.package_root)
            if package not in packages:
                raise ValueError(f"Unknown mesh package {package!r}; add it to the model profile")
            base, candidate = packages[package], packages[package] / parsed.path.lstrip("/")
        elif parsed.scheme or parsed.netloc or PureWindowsPath(uri).is_absolute() or Path(uri).is_absolute():
            raise ValueError(f"Only relative or package:// local meshes are supported: {uri!r}")
        else:
            base, candidate = self.package_root, self.urdf.parent / uri
        candidate = candidate.resolve()
        if not candidate.is_relative_to(base.resolve()):
            raise ValueError("Mesh escapes its configured asset directory")
        if candidate.suffix.lower() not in {".stl", ".obj", ".ply", ".gltf", ".glb"}:
            raise ValueError(f"Unsupported mesh format: {candidate.suffix}")
        with candidate.open("rb") as stream:
            header = stream.read(64)
        if not header or header.startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError(f"Empty or unhydrated mesh: {candidate.name}; run git lfs pull")
        return candidate


def discover_models(directory):
    profiles = [ModelProfile.load(p) for p in sorted(Path(directory).glob("*/model.json"))]
    if len({p.id for p in profiles}) != len(profiles):
        raise ValueError("Duplicate model ids in catalog")
    return profiles


class UrdfModel:
    def __init__(self, profile, *, apply_review=True):
        self.profile = profile
        raw = profile.urdf.read_bytes()
        root = ET.fromstring(raw)
        if root.tag != "robot":
            raise ValueError("Expected a URDF robot root")
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.links = [link.get("name") for link in root.findall("link")]
        if not self.links or any(not n for n in self.links) or len(set(self.links)) != len(self.links):
            raise ValueError("URDF links must have unique nonempty names")
        ignored = profile.keepout_ignored_links
        if (not isinstance(ignored, dict) or any(not isinstance(name, str) or name not in self.links
                or not isinstance(reason, str) or not reason.strip() for name, reason in ignored.items())):
            raise ValueError("keepout_ignored_links must map existing link names to nonempty reasons")
        self.joints = []
        seen, children = set(), set()
        for node in root.findall("joint"):
            name, kind = node.get("name"), node.get("type")
            if not name or name in seen or kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise ValueError(f"Duplicate/unnamed joint or unsupported joint type: {name} / {kind}")
            seen.add(name)
            parents, kids = node.findall("parent"), node.findall("child")
            if len(parents) != 1 or len(kids) != 1:
                raise ValueError(f"Joint {name} needs one parent and child")
            parent, child = parents[0].get("link"), kids[0].get("link")
            if parent not in self.links or child not in self.links or child in children or child == parent:
                raise ValueError(f"Invalid graph at {name}")
            children.add(child)
            if any(len(node.findall(tag)) > 1 for tag in ("axis", "limit", "mimic")):
                raise ValueError(f"Duplicate axis, limit or mimic on {name}")
            axis = node.find("axis")
            axis = numbers(axis.get("xyz", "1 0 0") if axis is not None else "1 0 0", 3, "joint axis")
            if kind != "fixed" and abs(sum(v*v for v in axis)-1) > 1e-5:
                raise ValueError(f"Joint {name} requires a unit axis")
            limit = node.find("limit")
            bounds = None
            if kind in {"prismatic", "revolute"}:
                if limit is None:
                    raise ValueError(f"Joint {name} needs finite URDF limits")
                bounds = numbers([limit.get("lower"), limit.get("upper")], 2, "joint limits")
                if bounds[0] > bounds[1]:
                    raise ValueError(f"Reversed joint limits: {name}")
            mimic = node.find("mimic")
            if mimic is not None:
                if kind == "fixed":
                    raise ValueError("Fixed joints cannot mimic")
                multiplier, offset = numbers([mimic.get("multiplier", "1"), mimic.get("offset", "0")], 2, "mimic")
                mimic = {"joint": mimic.get("joint"), "multiplier": multiplier, "offset": offset}
            self.joints.append(dict(name=name, type=kind, parent=parent, child=child, origin=origin(node),
                                    axis=axis, bounds=bounds, mimic=mimic, unit="m" if kind == "prismatic" else "rad"))
        roots = set(self.links) - children
        if len(roots) != 1:
            raise ValueError("Model must have one root link")
        self.root = roots.pop()
        connected, pending = {self.root}, list(self.joints)
        while pending:
            ready = [j for j in pending if j["parent"] in connected and j["child"] not in connected]
            if not ready:
                raise ValueError("Disconnected or cyclic URDF graph")
            for joint in ready:
                connected.add(joint["child"])
                pending.remove(joint)
        if connected != set(self.links):
            raise ValueError("Disconnected links")
        self.by_name = {j["name"]: j for j in self.joints}
        self.independent = [j["name"] for j in self.joints if j["type"] != "fixed" and j["mimic"] is None]
        self.mimic_order = []
        resolved, pending = set(self.independent), [j for j in self.joints if j["mimic"] is not None]
        while pending:
            ready = [j for j in pending if j["mimic"]["joint"] in resolved]
            if not ready:
                raise ValueError("Missing or cyclic mimic source")
            for joint in ready:
                if joint["unit"] != self.by_name[joint["mimic"]["joint"]]["unit"]:
                    raise ValueError("Mimic source and follower units differ")
                self.mimic_order.append(joint["name"])
                resolved.add(joint["name"])
                pending.remove(joint)
        # Map follower bounds back to each independent coordinate, including
        # chained negative/zero multipliers; never offer impossible jaw poses.
        expressions = {n: (n, 1., 0.) for n in self.independent}
        intervals = {n: list(self.by_name[n]["bounds"] or [-math.inf, math.inf]) for n in self.independent}
        for n in self.mimic_order:
            mimic = self.by_name[n]["mimic"]
            source, a, b = expressions[mimic["joint"]]
            a, b = a*mimic["multiplier"], b*mimic["multiplier"]+mimic["offset"]
            expressions[n] = (source, a, b)
            bounds = self.by_name[n]["bounds"]
            if bounds is not None:
                if a == 0:
                    if not bounds[0] <= b <= bounds[1]:
                        raise ValueError("Constant mimic lies outside its bounds")
                else:
                    low, high = sorted((v-b)/a for v in bounds)
                    intervals[source] = [max(intervals[source][0], low), min(intervals[source][1], high)]
        for n, (low, high) in intervals.items():
            if low > high:
                raise ValueError("Mimic bounds have no feasible interval")
            if not math.isfinite(low):
                low = min(-math.pi, high-2*math.pi) if math.isfinite(high) else -math.pi
            if not math.isfinite(high):
                high = max(math.pi, low+2*math.pi)
            self.by_name[n]["control_bounds"] = [low, high]
            self.by_name[n]["default"] = max(low, min(high, 0.))
        grouped = [n for group in profile.groups.values() for n in group]
        if len(grouped) != len(set(grouped)) or set(grouped) - set(self.independent):
            raise ValueError("Profile groups must contain unique independent mechanical joints")
        if set(profile.provisional_limits) - set(self.independent):
            raise ValueError("Unknown provisional-limit joint")
        self.groups = dict(profile.groups)
        ungrouped = [n for n in self.independent if n not in grouped]
        if ungrouped:
            self.groups["Other joints" if self.groups else "Joints"] = ungrouped
        self.assets, self.geometry = [], []
        materials = {m.get("name"): m for m in root.findall("material")}
        for link in root.findall("link"):
            for kind in ("visual", "collision"):
                for ordinal, node in enumerate(link.findall(kind)):
                    geometry = node.find("geometry")
                    if geometry is None or len(geometry) != 1:
                        raise ValueError("Each visual/collision needs one geometry")
                    shape = geometry[0]
                    desc = dict(link=link.get("name"), role=kind, origin=origin(node), shape=shape.tag)
                    if kind == "collision":
                        desc["collision_id"] = f"{link.get('name')}/{ordinal}"
                    material = node.find("material")
                    if material is not None and material.find("color") is None:
                        material = materials.get(material.get("name"), material)
                    color = material.find("color") if material is not None else None
                    desc["color"] = numbers(color.get("rgba") if color is not None else ".65 .7 .75 1", 4, "rgba")
                    if any(v < 0 or v > 1 for v in desc["color"]):
                        raise ValueError("RGBA must be in [0,1]")
                    if shape.tag == "mesh":
                        mesh = profile.resolve_mesh(shape.get("filename", ""))
                        if mesh not in self.assets:
                            self.assets.append(mesh)
                        desc.update(asset=self.assets.index(mesh), scale=numbers(shape.get("scale", "1 1 1"), 3, "mesh scale", True))
                    elif shape.tag == "box":
                        desc["size"] = numbers(shape.get("size"), 3, "box size", True)
                    elif shape.tag in ("sphere", "cylinder"):
                        desc["radius"] = numbers([shape.get("radius")], 1, "radius", True)[0]
                        if shape.tag == "cylinder":
                            desc["length"] = numbers([shape.get("length")], 1, "length", True)[0]
                    else:
                        raise ValueError(f"Unsupported geometry: {shape.tag}")
                    self.geometry.append(desc)
        self.limit_review = None
        if apply_review and profile.limit_review:
            review = json.loads(profile.limit_review.read_bytes())
            if (review.get("schema_version") != 1 or review.get("kind") != "collision_limit_review"
                    or review.get("status") != "awaiting_visual_review"
                    or review.get("urdf_sha256") != self.sha256 or review.get("model_id") != profile.id):
                raise ValueError("Limit review does not match this model/URDF revision")
            assets = review.get("assets", [])
            if len(assets) != len(self.assets) or any(item.get("sha256") != hashlib.sha256(p.read_bytes()).hexdigest()
                                                    for item, p in zip(assets, self.assets)):
                raise ValueError("Limit review mesh revision mismatch; rerun the sweep")
            reference = review["reference_positions"]
            self.positions(reference)
            results = review["results"]
            if not results or set(results) - set(self.independent):
                raise ValueError("Limit review contains unknown independent joints")
            for name, result in results.items():
                joint = self.by_name[name]
                lo, hi = numbers(result["bounds"], 2, "review bounds")
                source_lo, source_hi = joint["control_bounds"]
                if (result["unit"] != joint["unit"] or result["status"] not in {"candidate", "blocked_at_reference", "unresolved_contact"}
                        or not source_lo <= lo <= reference[name] <= hi <= source_hi
                        or result["status"] != "candidate" and lo != hi):
                    raise ValueError(f"Invalid candidate viewing range: {name}")
                joint["control_bounds"] = [lo, hi]
                joint["limit_review"] = {k: v for k, v in result.items() if k != "samples"}
            for name in self.independent:
                self.by_name[name]["default"] = reference[name]
            self.limit_review = {k: v for k, v in review.items() if k not in {"results", "assets"}}
            self.limit_review["results"] = {n: self.by_name[n]["limit_review"] for n in results}

    def positions(self, q):
        if not isinstance(q, dict) or set(q) != set(self.independent):
            raise ValueError("Provide every independent joint exactly once")
        if any(type(v) not in (int, float) for v in q.values()):
            raise ValueError("Joint positions must be numbers")
        result = {n: numbers([v], 1, n)[0] for n, v in q.items()}
        for n in self.mimic_order:
            m = self.by_name[n]["mimic"]
            result[n] = result[m["joint"]]*m["multiplier"] + m["offset"]
        for n, value in result.items():
            bounds = self.by_name[n]["bounds"]
            if not math.isfinite(value) or bounds is not None and not bounds[0]-1e-12 <= value <= bounds[1]+1e-12:
                raise ValueError(f"Position outside URDF range: {n}")
        return result

    def metadata(self):
        return dict(id=self.profile.id, title=self.profile.title, urdf_sha256=self.sha256,
                    profile_sha256=self.profile.profile_sha256, root=self.root, links=self.links,
                    joints=self.joints, independent=self.independent, mimic_order=self.mimic_order,
                    groups=self.groups, geometry=self.geometry, notes=self.profile.notes,
                    provisional_limits=self.profile.provisional_limits, limit_review=self.limit_review,
                    front_direction=self.profile.front_direction,
                    runtime_collision_available=self.profile.collision_model is not None,
                    keepout_ignored_links=dict(self.profile.keepout_ignored_links))
