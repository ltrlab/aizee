"""Gravity feed-forward for each arm's pitch chain, from its joints' measured gravity slices.

Minerva's J2, J3 and J4 turn about parallel horizontal axes (J1 about a vertical one;
the wrist joints carry little). The torque a chain joint holds against gravity is a sum
of sines of the absolute angles of the links beyond it, in motor coordinates:

    tau_i = s_i * sum_{k >= i} a_k * sin(phi_k),    phi_k = sum_{j <= k} s_j * q_j

where s_j (+1 or -1) says whether joint j's motor turns the same way about the common
axis as the chain's first joint (from the model's joint frames and the verified
motor-to-CAD directions), q = 0 is the reference pose (the arm hanging straight down,
every phi 0) and a_k is link k's gravity moment, with everything beyond it. A joint's
servo sweep measures its own slice at the reference pose, tau_i = A_i sin(q_i) (the other
joints at 0; torque_map's gravity_fit), so a_k = A_k - A_{k+1}. The chain's joints with
fits, from the first, get feed-forward; the last fitted one carries the links beyond it
(exact while they are at the reference, close otherwise).
"""
import math

HORIZONTAL = .02  # |axis . vertical| below this: a joint gravity turns (sin of 1.1 degrees).
PARALLEL = .999  # |axis . axis| above this: parallel axes.


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def world_axes(model):
    """{joint: its axis in the model's root frame at the zero pose} (z is up)."""
    by_child = {joint['child']: joint for joint in model.joints}
    axes = {}
    for joint in model.joints:
        chain, current = [joint], joint
        while current['parent'] in by_child:
            current = by_child[current['parent']]
            chain.append(current)
        rotation = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        for step in reversed(chain):
            rotation = _multiply(rotation, _rotation(step['origin']['rpy']))
        axes[joint['name']] = [sum(rotation[i][k] * joint['axis'][k] for k in range(3)) for i in range(3)]
    return axes


def pitch_chains(model, directions, motors):
    """Per arm (the motors of one bus, in order): [(motor, s)] for its pitch chain, the first run of
    consecutive joints turning about horizontal axes parallel to the first one's. s is +1 where the
    motor turns the same way as the chain's first joint's about their common axis."""
    axes = world_axes(model)
    cad = {row['motor']: (joint, row['sign']) for joint, row in directions.items()}
    arms = {}
    for motor in motors:
        arms.setdefault(motor.get('bus'), []).append(motor['name'])
    chains = {}
    for bus, names in arms.items():
        chain, first = [], None
        for name in names:
            if name not in cad:
                if chain:
                    break
                continue
            joint, sign = cad[name]
            axis = axes[joint]
            if first is None:
                if abs(axis[2]) < HORIZONTAL:
                    first = (axis, sign)
                    chain.append((name, 1))
                continue
            dot = sum(x * y for x, y in zip(axis, first[0]))
            if abs(dot) < PARALLEL:
                break
            chain.append((name, int(math.copysign(1, dot)) * sign * first[1]))
        if chain:
            chains[bus] = chain
    return chains


def gravity_terms(chain, fits):
    """{motor: [(coefficient, {motor: multiplier})]} for a pitch chain [(motor, s)] and slice fits
    {motor: {'A': ...}}: tau_i = sum coefficient * sin(sum multiplier * q). Only the chain's prefix
    with fits."""
    fitted = []
    for motor, sign in chain:
        fit = fits.get(motor)
        if not fit or not isinstance(fit.get('A'), (int, float)) or not math.isfinite(fit['A']):
            break
        fitted.append((motor, sign, float(fit['A'])))
    moments = [a - (fitted[k + 1][2] if k + 1 < len(fitted) else 0.) for k, (_, _, a) in enumerate(fitted)]
    angles, angle = [], {}
    for motor, sign, _ in fitted:
        angle = dict(angle, **{motor: sign})
        angles.append(angle)
    return {motor: [(sign * moments[k], angles[k]) for k in range(i, len(fitted))]
            for i, (motor, sign, _) in enumerate(fitted)}


def gravity_effort(terms, positions):
    """The holding torque for one joint's `terms` at `positions` ({motor: rad})."""
    return sum(coefficient * math.sin(sum(m * positions[motor] for motor, m in angle.items()))
               for coefficient, angle in terms)


def gravity_model(model, directions, motors, fits):
    """{motor: terms} for every arm's pitch chain with fits (see the module docstring)."""
    result = {}
    for chain in pitch_chains(model, directions, motors).values():
        result.update(gravity_terms(chain, fits or {}))
    return result
