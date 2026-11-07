"""Synthetic stills of the table board, for rehearsing the wrist calibration offline: a known camera, mount and board
placement, arm poses sampled until the camera sees the board, and the tags' corners projected with pixel noise.

The defaults are the simulation's guesses (simulation/task_scene.py's wrist camera, the sim workcell's rectangle
placement), not measurements: they only need to be plausible.
"""

import math

import numpy as np

from .geometry import apply, invert, project, transform

# The sim's wrist camera on the gripper body (task_scene.py): MuJoCo pos (0, .035, .035) and xyaxes (1 0 0, 0 .606
# .795), turned into OpenCV's camera axes (x right, y down, z forward) as the columns of the rotation.
_C, _S = np.array([0.606, 0.795]) / math.hypot(0.606, 0.795)
SIM_MOUNT = transform(np.array([[1.0, 0.0, 0.0], [0.0, -_C, _S], [0.0, -_S, -_C]]), (0.0, 0.035, 0.035))
# The sim workcell's placeholder: the rectangle's origin at base (0.325, -0.1584) on the table top (z 0.512), its x
# axis along base -x.
SIM_PLACEMENT = transform(np.diag([-1.0, -1.0, 1.0]), (0.325, -0.1584, 0.512))
# A wide M12 lens on a 640 x 480 image, with barrel distortion: fx, fy, cx, cy, k1, k2, p1, p2, k3.
SIM_CAMERA = np.array([330.0, 331.0, 322.0, 236.0, -0.32, 0.11, 0.0008, -0.0005, 0.0])


def visible(camera, view, corners, size, margin=8.0, facing=0.15):
    """The tag's pixels when all four corners are in the image (inside `margin`), in front of the camera and the
    tag faces it; else None."""
    points = apply(view, corners)
    if (points[:, 2] < 0.05).any():
        return None
    normal = view[:3, :3] @ np.array([0.0, 0.0, 1.0])
    centre = points.mean(axis=0)
    if normal @ centre / np.linalg.norm(centre) > -facing:
        return None
    pixels = project(camera, points)
    if ((pixels < margin) | (pixels > np.array(size) - 1 - margin)).any():
        return None
    return pixels


def sample_stills(
    board,
    chain,
    camera_name,
    rng,
    count,
    *,
    camera=SIM_CAMERA,
    mount=SIM_MOUNT,
    placement=SIM_PLACEMENT,
    offsets=None,
    size=(640, 480),
    noise_px=0.15,
    ranges=None,
    min_tags=1,
    joint_errors=None,
    tries=200000,
):
    """`count` stills of camera_name, each from a random joint state (uniform in `ranges`, {joint: (low, high)},
    default +-pi) whose camera sees at least min_tags whole tags. `joint_errors` ({joint: radians}) is where the arm
    really is beyond what its joints report: the stills report the joints without it."""
    from .wrist_calibration import Still

    ranges = ranges or {name: (-math.pi, math.pi) for name in chain.joints}
    corners = board.layout(offsets)
    stills = []
    for _ in range(tries):
        q = {name: float(rng.uniform(*ranges[name])) for name in chain.joints}
        view = invert(chain.pose(q, joint_errors) @ mount) @ placement  # T_camera_board
        seen = {}
        for tag, quad in corners.items():
            pixels = visible(camera, view, quad, size)
            if pixels is not None:
                seen[tag] = pixels + rng.normal(0.0, noise_px, pixels.shape)
        if len(seen) >= min_tags:
            stills.append(Still(camera_name, q, seen, name=f'{camera_name}-{len(stills):02d}'))
            if len(stills) == count:
                return stills
    raise RuntimeError(f'Only {len(stills)} of {count} views found in {tries} tries')
