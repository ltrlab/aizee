"""Offline single-joint collision envelopes for explicit reference postures.

These are review candidates, not physical endpoints or a collision-free box in
joint space. Each branch is checked against itself and the shared body, with
other declared branches excluded. All exclusions remain explicit evidence.
"""
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import time

from robot_platform.models import UrdfModel
from .collision import CollisionScene


def descendants(model, joint_name):
    links = {model.by_name[joint_name]["child"]}
    for _ in model.joints:
        new = {j["child"] for j in model.joints if j["parent"] in links}
        if new <= links:
            break
        links.update(new)
    return links


def motion_groups(model, joint_name):
    """Links with identical affected-joint ancestry share one rigid motion."""
    driven = {joint_name}
    for name in model.mimic_order:
        if model.by_name[name]["mimic"]["joint"] in driven:
            driven.add(name)
    branches = {n: descendants(model, n) for n in driven}
    return {link: frozenset(n for n, links in branches.items() if link in links) for link in model.links}


def scan_interval(check, *, anchor, bounds, step, tolerance, margin):
    """Scan outward from a clear anchor; never jump over a sampled collision."""
    if not all(math.isfinite(v) for v in [anchor, *bounds, step, tolerance, margin]):
        raise ValueError("Sweep parameters must be finite")
    if not bounds[0] <= anchor <= bounds[1] or not 0 < tolerance <= step or margin < 0:
        raise ValueError("Invalid sweep bounds, resolution or margin")
    if (bounds[1]-bounds[0])/step > 10000:
        raise ValueError("Sweep exceeds 10,000 coarse samples per joint")
    initial = check(anchor)
    if initial:
        return {"status": "blocked_at_reference", "bounds": [anchor, anchor],
                "reference_contacts": initial, "lower": None, "upper": None}
    ends = []
    for direction, stop in [(-1, bounds[0]), (1, bounds[1])]:
        clear = anchor
        endpoint = {"status": "search_bound", "clear": stop, "contact": None, "pairs": []}
        count = math.ceil(abs(stop-anchor)/step)
        for index in range(1, count+1):
            value = stop if index == count else anchor + direction*step*index
            pairs = check(value)
            if pairs:
                contact = value
                while abs(contact-clear) > tolerance:
                    mid = (contact+clear)/2
                    found = check(mid)
                    if found:
                        contact, pairs = mid, found
                    else:
                        clear = mid
                endpoint = {"status": "contact", "clear": clear, "contact": contact, "pairs": pairs}
                break
            clear = value
        usable = endpoint["clear"]
        if endpoint["status"] == "contact":
            usable = max(anchor, usable-margin) if direction > 0 else min(anchor, usable+margin)
        # Verify the actual displayed endpoint after applying the inward margin.
        endpoint_pairs = check(usable)
        if endpoint_pairs:
            return {"status": "unresolved_contact", "bounds": [anchor, anchor],
                    "reference_contacts": [], "unresolved_contacts": endpoint_pairs,
                    "reason": "Inward margin landed on an unsampled contact; finer sampling or interface review required.",
                    "lower": None, "upper": None}
        endpoint["candidate"] = usable
        ends.append(endpoint)
    return {"status": "candidate", "bounds": [e["candidate"] for e in ends],
            "lower": ends[0], "upper": ends[1]}


def sweep_model(profile, request, *, progress=None):
    started = time.perf_counter()
    model = UrdfModel(profile, apply_review=False)
    if request.get("schema_version") != 1 or request.get("urdf_sha256") != model.sha256:
        raise ValueError("Sweep request must name this exact URDF revision")
    reference = request["reference_positions"]
    model.positions(reference)
    groups = request["groups"]
    if not isinstance(groups, list) or not groups:
        raise ValueError("Declare independently swept branches")
    branches, swept = {}, set()
    for group in groups:
        name, root, names = group["name"], group["root_joint"], group["joints"]
        if name in branches or root not in model.by_name or not names:
            raise ValueError("Invalid or duplicate sweep group")
        links = descendants(model, root)
        if any(links & other for other in branches.values()):
            raise ValueError("Sweep branches must not overlap")
        for n in names:
            if n not in model.independent or n in swept or model.by_name[n]["child"] not in links:
                raise ValueError("Sweep joints must belong to one declared branch")
            swept.add(n)
        branches[name] = links
    scene = CollisionScene.from_urdf(profile.urdf, profile.package_root, mesh_resolver=profile.resolve_mesh)
    shapes = {s["id"]: s for s in scene.shape_metadata}
    excluded = set()
    for item in request.get("assembly_interfaces", []):
        pair = item["shapes"]
        if (len(pair) != 2 or len(set(pair)) != 2 or any(s not in shapes for s in pair)
                or not item.get("reason") or item.get("review_status") != "provisional"):
            raise ValueError("Assembly interfaces require valid shape pairs and a provisional reason")
        a, b = (shapes[s]["link"] for s in pair)
        if not any({j["parent"], j["child"]} == {a, b} for j in model.joints):
            raise ValueError("Provisional assembly interfaces must join adjacent links")
        excluded.add(tuple(sorted(pair)))
    raw_reference = scene.check_shape_pairs(reference)
    units = request["resolution"]
    results, samples = {}, 0
    for group in groups:
        own = branches[group["name"]]
        other = set().union(*(v for k, v in branches.items() if k != group["name"]))
        included = [s for s in scene.shape_metadata if s["link"] not in other]
        scope_pairs = [(a["id"], b["id"]) for a, b in itertools.combinations(included, 2)
                       if a["link"] != b["link"] and (a["link"] in own or b["link"] in own)
                       and tuple(sorted((a["id"], b["id"]))) not in excluded]
        for name in group["joints"]:
            joint = model.by_name[name]
            if joint["bounds"] is None:
                raise ValueError("Continuous joints need a bounded exploration model")
            resolution = units[joint["unit"]]
            motions = motion_groups(model, name)
            moving_pairs = [pair for pair in scope_pairs
                            if motions[shapes[pair[0]]["link"]] != motions[shapes[pair[1]]["link"]]]
            scope_set = {tuple(sorted(pair)) for pair in scope_pairs}
            initial_pairs = tuple(pair for pair in raw_reference if tuple(sorted(pair)) in scope_set)
            # The full branch reference is checked first. During a single-joint
            # sweep, pairs sharing one rigid motion keep their relative pose;
            # rechecking them in world coordinates adds roundoff, not evidence.
            checks = []
            def check(value):
                pose = {**reference, name: value}
                pairs = initial_pairs if value == reference[name] else scene.check_shape_pairs(pose, moving_pairs)
                checks.append({"value": value, "pairs": pairs})
                return pairs
            result = scan_interval(check, anchor=reference[name], bounds=joint["control_bounds"],
                                   step=resolution["step"], tolerance=resolution["tolerance"],
                                   margin=resolution["margin"])
            result.update(group=group["name"], unit=joint["unit"], source_bounds=joint["control_bounds"],
                          scope_links=sorted(set(model.links)-other), moving_shape_pairs=len(moving_pairs), samples=checks)
            samples += len(checks)
            results[name] = result
            if progress:
                progress(name, result)
    assets = [{"index": i, "name": p.name,
               "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for i, p in enumerate(model.assets)]
    return {"schema_version": 1, "kind": "collision_limit_review", "status": "awaiting_visual_review",
            "created_utc": datetime.now(timezone.utc).isoformat(), "model_id": profile.id,
            "request_sha256": hashlib.sha256(json.dumps(request, sort_keys=True, allow_nan=False).encode()).hexdigest(),
            "source_version": request.get("source_version"),
            "urdf_sha256": model.sha256, "assets": assets,
            "method": "Single-joint outward sweeps from the recorded reference; own branch plus shared body; other branches excluded.",
            "limitations": ["Provisional assembly-interface exclusions require visual review.",
                            "Other joints stay at the reference. Combining these intervals is not guaranteed collision-free.",
                            "Discrete triangle-surface samples can miss between-sample collisions and solid containment.",
                            "Search bounds without contact are not discovered mechanical endpoints."],
            "runtime_interlock_active": False, "physical_validation": False,
            "reference_positions": reference, "resolution": units, "groups": groups,
            "assembly_interfaces": request.get("assembly_interfaces", []),
            "raw_reference_contacts": raw_reference, "shape_metadata": scene.shape_metadata,
            "results": results, "samples_checked": samples, "elapsed_seconds": time.perf_counter()-started}
