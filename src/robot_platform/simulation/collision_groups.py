"""Robot-independent collision scopes; grouping never excludes a physical pair."""
from .collision import CollisionError


SCOPES = {"inter_branch": 1, "within_branch": 2, "body": 4, "environment": 8, "all": 15}


def scope_mask(scope):
    if not isinstance(scope, str) or scope not in SCOPES:
        raise CollisionError(f"Unknown collision scope: {scope!r}")
    return SCOPES[scope]


class CollisionGroups:
    def __init__(self, model):
        configured = model.profile.collision_branches
        if not isinstance(configured, dict) or any(
                not isinstance(label, str) or not label or not isinstance(root, str) or not root
                for label, root in configured.items()):
            raise CollisionError("collision_branches must map labels to root joint names")
        joints = {j["name"]: j for j in model.joints}
        if any(root not in joints for root in configured.values()):
            raise CollisionError("Unknown collision branch root joint")
        if len(set(configured.values())) != len(configured):
            raise CollisionError("Duplicate collision branch root joint")
        roots = {root: "branch:" + label for label, root in configured.items()}
        children = {}
        for joint in model.joints:
            children.setdefault(joint["parent"], []).append(joint)
        self.links = {}
        pending = [(model.root, "body")]
        while pending:
            link, group = pending.pop()
            self.links[link] = group
            for joint in children.get(link, ()):
                child_group = group
                if joint["name"] in roots:
                    if group != "body":
                        raise CollisionError("Collision branch roots must not overlap or nest")
                    child_group = roots[joint["name"]]
                elif not configured and group == "body" and joint["type"] != "fixed":
                    child_group = "branch:" + joint["name"]
                pending.append((joint["child"], child_group))

    def group(self, shape):
        return "environment" if shape.get("obstacle") else self.links[shape["link"]]

    def kind(self, a, b):
        ga, gb = self.group(a), self.group(b)
        if "environment" in (ga, gb):
            return "environment"
        if "body" in (ga, gb):
            return "body"
        return "within_branch" if ga == gb else "inter_branch"

    def pairs(self, shapes, pairs, scope):
        scope_mask(scope)
        return tuple((i, j) for i, j in pairs if scope == "all" or self.kind(shapes[i], shapes[j]) == scope)
