"""The head's shoulder clearance. Turned to a side and tilted all the way down, the head reaches its shoulder, so how far
down it may tilt depends on how far it is turned: coupled limits, as NAO's head pitch range narrows with its yaw. The
per-joint limits, the servos' own among them, are a rectangle that holds those corners.

The operator records corners in Helm (Limits → Head, helm.head_setup): with the servos limp, the head turned to a side
and tilted down until it is just clear of the shoulder. They are saved in the head's limits file, so they belong to
its reference (`clearance`):
- `corners`: [{pan, tilt}], radians of each servo from its reference reading (0 is straight ahead and level);
- `down`: the sign of tilt that is down, the side every corner lies on;
- `margin_rad`: how much less far down than a corner the head may tilt.

The floor, how far down the head may tilt at a pan, is the whole tilt range straight ahead, runs straight to each
corner less the margin, and never eases off further out: past its last corner, no further down than there. So the
region allowed only shrinks outward and downward, and a move back toward straight ahead and level on both joints
(Return) stays in it, however each joint's speed shares the move.
"""

import math

MARGIN_RAD = 0.1  # About 6 degrees above each corner: the servo's tracking error and the head's give.
MIN_CORNER_RAD = 0.1  # A corner is turned and tilted at least this far: less says nothing about a shoulder.
PAN, TILT = 'neck_z', 'neck_y'  # The CAD joints head_pan and head_tilt drive.


def section(corners, *, margin_rad=MARGIN_RAD):
    """The limits file's clearance from the corners recorded: at least one each side, all tilted down the same way."""
    corners = [{'pan': float(c['pan']), 'tilt': float(c['tilt'])} for c in corners]
    if not all(math.isfinite(c['pan']) and math.isfinite(c['tilt']) for c in corners):
        raise ValueError('A corner is a pan and a tilt reading')
    if any(abs(c['pan']) < MIN_CORNER_RAD for c in corners):
        raise ValueError('A corner is recorded with the head turned to a side')
    if {math.copysign(1.0, c['pan']) for c in corners} != {-1.0, 1.0}:
        raise ValueError('Record a corner on each side: the head reaches both shoulders')
    downs = {math.copysign(1.0, c['tilt']) for c in corners}
    if len(downs) != 1 or any(abs(c['tilt']) < MIN_CORNER_RAD for c in corners):
        raise ValueError('Each corner is recorded with the head tilted down, the same way every time')
    if not (math.isfinite(margin_rad) and margin_rad >= 0):
        raise ValueError('The margin is a non-negative angle')
    return {'corners': corners, 'down': int(downs.pop()), 'margin_rad': float(margin_rad)}


def unbounded():
    """No floor: the same steps for a head with no clearance saved (a virtual session's)."""
    return Clearance({'corners': [], 'down': -1, 'margin_rad': 0.0}, math.inf)


def from_limits(limits, reference):
    """The clearance saved in a head limits file, for its reference; None when none is saved."""
    saved = (limits or {}).get('clearance')
    if saved is None:
        return None
    if limits.get('reference_id') != reference.get('id'):
        raise ValueError("The head's shoulder clearance belongs to another reference: record its corners again")
    tilt = next(m for m in reference['motors'] if m['joint'] == TILT)
    joint = limits['joints'][tilt['name']]
    low, high = joint['min_rad'] - tilt['raw_reference_rad'], joint['max_rad'] - tilt['raw_reference_rad']
    return Clearance(saved, -low if saved['down'] < 0 else high)


class Clearance:
    """The floor of a saved clearance section. `reach`: how far down the head may tilt straight ahead (its saved tilt
    limit on the down side), radians from the reference."""

    def __init__(self, saved, reach):
        self.down, self.margin, self.reach = int(saved['down']), float(saved['margin_rad']), float(reach)
        if self.down not in (-1, 1) or not self.reach > 0:
            raise ValueError('A clearance needs its down side and the tilt range below level')
        self.sides = {}
        for side in (-1, 1):
            knots, depth = [(0.0, self.reach)], self.reach
            for corner in sorted((c for c in saved['corners'] if c['pan'] * side > 0), key=lambda c: abs(c['pan'])):
                # Never easing off outward, and level always allowed (a pan alone never reaches the shoulder).
                depth = min(depth, max(0.0, self.down * corner['tilt'] - self.margin))
                knots.append((abs(corner['pan']), depth))
            self.sides[side] = knots

    def depth(self, pan):
        """How far down (radians, positive) the head may tilt at `pan`."""
        knots, x = self.sides[1 if pan >= 0 else -1], abs(pan)
        for (x0, y0), (x1, y1) in zip(knots, knots[1:]):
            if x <= x1:
                return y1 if x1 <= x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        return knots[-1][1]

    def clear(self, pan, tilt):
        return self.down * tilt <= self.depth(pan) + 1e-9

    def clamp(self, pan, tilt):
        """The pose, its tilt raised to the floor where it is below it (its pan kept)."""
        depth = self.depth(pan)
        return (pan, tilt) if self.down * tilt <= depth else (pan, self.down * depth)

    def step(self, start, goal, limit):
        """The next pose from `start` toward `goal` (both (pan, tilt)), each joint moving at most `limit` = (pan, tilt)
        radians: straight toward the goal as far as both joints allow, its tilt raised onto the floor where the line
        dips under it (the floor rises toward the shoulder), shorter where that rise is more than the tilt's limit. A
        start below the floor first rises straight up (its pan kept), the way out of the corner."""
        pan, tilt = start
        if not self.clear(pan, tilt):
            return pan, tilt - self.down * min(limit[1], self.down * tilt - self.depth(pan))
        goal = self.clamp(*goal)
        moves = [goal[0] - pan, goal[1] - tilt]
        scale = min([1.0] + [cap / abs(move) for move, cap in zip(moves, limit) if abs(move) > cap])
        for _ in range(30):
            pose = self.clamp(pan + moves[0] * scale, tilt + moves[1] * scale)
            if abs(pose[1] - tilt) <= limit[1] * (1 + 1e-9):
                return pose
            scale /= 2
        return pan, tilt
