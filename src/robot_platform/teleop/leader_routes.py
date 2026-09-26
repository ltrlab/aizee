"""Leader-to-robot routes: each leader channel's full range spans its robot joint's.

Every route scales straight and covers the robot's whole usable range. A
gripper maps end to end, since its released trigger and closed jaws already sit
at ends. An arm route is anchored at the operator's leader zero, which maps onto
the robot's reference pose, with one scale for both directions: the larger of
(robot travel / leader travel) on the two sides of the zero. So each robot end
is reached within the leader's range; on the side where the leader has travel to
spare, the robot stops at its limit and the leader goes on past it.
For an arm joint the route sign says whether the leader angle and the robot's
CAD joint angle increase together; for a gripper, +1 means the leader angle
increasing opens it (gripper slide axes can be mirrored in CAD). The operator
verifies each sign. Leader readings are unwrapped around each range's center,
so an encoder crossing its 0/2pi seam, or restarting its turn count after a
power cycle, cannot jump.

Routes are expressed for AbsoluteLeaderMapper, in receiver (motor) units.
"""
from dataclasses import replace

from .leader_ranges import wrap


def unwrap_samples(samples, centers):
    """Express each ranged channel within +/-pi of its range center; others stay raw."""
    result = {}
    for name, sample in samples.items():
        positions = tuple(q if center is None else center + wrap(q - center)
                          for q, center in zip(sample.positions_rad, centers[name]))
        result[name] = replace(sample, positions_rad=positions)
    return result


def build_routes(route_settings, ranges, joint_map, usable, opens=None, anchors=None):
    """AbsoluteLeaderMapper routes in receiver order.

    Each route also reports its 'scale', 'reach' (share of the robot range covered
    within the leader's range, 1 unless a leader side is under 0.2 rad) and
    'overtravel' (leader travel past the point where the robot stops at a limit).

    route_settings: {motor: {leader, channel (1-based), sign}}.
    ranges: {leader: [(center_rad, span_rad) or None per channel]} (saved leader ranges).
    joint_map: {cad_joint: {actuator, scale, offset}} from the receiver config.
    usable: ordered {motor: (low, high)} receiver target intervals.
    opens: {gripper motor: +1/-1}, the motor direction that opens it.
    anchors: {arm motor: (leader zero reading, robot reference position)}.
    """
    scale = {row['actuator']: row['scale'] for row in joint_map.values()}
    opens, anchors = opens or {}, anchors or {}
    routes = []
    for motor, (low, high) in usable.items():
        entry = route_settings.get(motor)
        if entry is None:
            raise ValueError(f'No leader route for {motor}')
        leader, channel = entry['leader'], entry['channel'] - 1
        if leader not in ranges or not 0 <= channel < len(ranges[leader]):
            raise ValueError(f'{motor}: unknown leader or channel')
        if entry.get('sign') not in (-1, 1):
            raise ValueError(f'{motor}: route sign must be +1 or -1')
        if ranges[leader][channel] is None:
            raise ValueError(f'{motor}: no saved range for the {leader} leader channel {channel + 1}; '
                             'sweep it in the Leaders tab')
        center, span = ranges[leader][channel]
        if motor in opens:
            direction = entry['sign'] * opens[motor]
        else:  # The sign relates leader and CAD angles; the CAD-to-motor scale sets the motor direction.
            direction = entry['sign'] * (1 if scale[motor] > 0 else -1)
        route = dict(actuator=motor, leader=leader, channel=channel, raw_min=center - span / 2,
                     raw_max=center + span / 2, follower_min=low, follower_max=high, direction=direction)
        route.update(scale=(high - low) / span, reach=1., overtravel=0.)
        if motor in anchors and motor not in opens:
            reading, reference = anchors[motor]
            zero = min(max(center + wrap(reading - center), route['raw_min']), route['raw_max'])
            home = min(max(reference, low), high)
            # Robot travel toward the end each leader side reaches, and the leader travel on that side.
            near, far = (home - low, high - home) if direction > 0 else (high - home, home - low)
            below, above = zero - route['raw_min'], route['raw_max'] - zero
            gain = max((robot / leader for robot, leader in ((near, below), (far, above)) if leader > .2),
                       default=route['scale'])
            route.update(raw_zero=zero, follower_zero=home, gain=gain, scale=gain)
            ends = [min(high, max(low, home + direction * gain * (x - zero))) for x in (route['raw_min'], route['raw_max'])]
            route['reach'] = abs(ends[1] - ends[0]) / (high - low)
            route['overtravel'] = max(0., below - near / gain, above - far / gain)
        routes.append(route)
    return routes
