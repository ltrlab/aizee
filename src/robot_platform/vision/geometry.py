"""Rigid transforms and the pinhole camera with Brown-Conrady distortion, in OpenCV's conventions.

Pixels are (u, v) with (0, 0) at the centre of the top-left pixel, u to the right and v down. The camera frame has x
to the right, y down and z forward along the optical axis. A camera is nine numbers, (fx, fy, cx, cy, k1, k2, p1, p2,
k3): OpenCV's camera matrix and its five distortion coefficients in OpenCV's order, so a result plugs into
cv2.undistort or cv2.projectPoints unchanged.

A transform T_a_b is a 4x4 matrix taking points in frame b to frame a.
"""
import math

import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(rotvec):
    """The rotation matrix of a rotation vector (axis times angle, radians); smooth through zero."""
    v = np.asarray(rotvec, float)
    theta2 = float(v @ v)
    k = skew(v)
    if theta2 < 1e-8:
        a, b = 1 - theta2 / 6, .5 - theta2 / 24
    else:
        theta = math.sqrt(theta2)
        a, b = math.sin(theta) / theta, (1 - math.cos(theta)) / theta2
    return np.eye(3) + a * k + b * (k @ k)


def rotations(rotvecs):
    """(N, 3, 3) rotation matrices of (N, 3) rotation vectors."""
    v = np.asarray(rotvecs, float).reshape(-1, 3)
    theta2 = (v * v).sum(axis=1)
    small = theta2 < 1e-8
    theta = np.sqrt(np.where(small, 1., theta2))
    a = np.where(small, 1 - theta2 / 6, np.sin(theta) / theta)
    b = np.where(small, .5 - theta2 / 24, (1 - np.cos(theta)) / np.where(small, 1., theta2))
    k = np.zeros((len(v), 3, 3))
    k[:, 0, 1], k[:, 0, 2], k[:, 1, 0] = -v[:, 2], v[:, 1], v[:, 2]
    k[:, 1, 2], k[:, 2, 0], k[:, 2, 1] = -v[:, 0], -v[:, 1], v[:, 0]
    return np.eye(3) + a[:, None, None] * k + b[:, None, None] * (k @ k)


def invert_each(t):
    """Inverses of (N, 4, 4) rigid transforms."""
    result = np.zeros_like(t)
    rt = np.transpose(t[:, :3, :3], (0, 2, 1))
    result[:, :3, :3] = rt
    result[:, :3, 3] = -np.einsum('nij,nj->ni', rt, t[:, :3, 3])
    result[:, 3, 3] = 1.
    return result


def rotvec(matrix):
    """The rotation vector of a rotation matrix, angle in [0, pi]."""
    r = np.asarray(matrix, float)
    cos = min(1., max(-1., (np.trace(r) - 1) / 2))
    theta = math.acos(cos)
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2
    if theta < 1e-6:
        return vee
    if math.pi - theta < 1e-2:
        # Near a half turn the antisymmetric part vanishes. The symmetric part is cos(theta) I + (1 - cos(theta))
        # a a^T, so a a^T follows exactly; its dominant column gives the axis, the antisymmetric part its sign.
        outer = ((r + r.T) / 2 - cos * np.eye(3)) / (1 - cos)
        k = int(np.argmax(np.diag(outer)))
        axis = outer[:, k] / math.sqrt(outer[k, k])
        if axis @ vee < 0:
            axis = -axis
        return axis * theta
    return vee * theta / math.sin(theta)


def rpy_matrix(roll, pitch, yaw):
    """URDF's roll-pitch-yaw: about the fixed x, then y, then z axes."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(r):
    """URDF roll, pitch and yaw of a rotation matrix (pitch in [-pi/2, pi/2])."""
    pitch = math.asin(max(-1., min(1., -r[2, 0])))
    if abs(r[2, 0]) > 1 - 1e-9:  # Gimbal lock: put the whole turn in yaw.
        return 0., pitch, math.atan2(-r[0, 1], r[1, 1])
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


def transform(rotvec_or_matrix, translation):
    t = np.eye(4)
    r = np.asarray(rotvec_or_matrix, float)
    t[:3, :3] = r if r.shape == (3, 3) else rotation(r)
    t[:3, 3] = translation
    return t


def invert(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return result


def apply(t, points):
    """Points (N, 3) in frame b to frame a, for t = T_a_b."""
    return np.asarray(points, float) @ t[:3, :3].T + t[:3, 3]


def pose_vector(t):
    """(rotation vector, translation) as six numbers."""
    return np.concatenate([rotvec(t[:3, :3]), t[:3, 3]])


def pose_matrix(v):
    return transform(v[:3], v[3:6])


def project(camera, points):
    """Pixels (N, 2) of camera-frame points (N, 3). Points at or behind the camera are pushed just in front of it, so
    a solver sees a large but finite residual instead of a sign flip."""
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = camera
    p = np.asarray(points, float)
    z = np.maximum(p[:, 2], 1e-3)
    x, y = p[:, 0] / z, p[:, 1] / z
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([fx * xd + cx, fy * yd + cy], axis=1)


def normalize(camera, pixels, iterations=20):
    """Undistorted normalized image coordinates (x/z, y/z) of pixels: the inverse of project, by fixed-point
    iteration (good for the moderate distortion of the wrist lenses)."""
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = camera
    uv = np.asarray(pixels, float)
    xd, yd = (uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy
    x, y = xd.copy(), yd.copy()
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = (xd - dx) / radial, (yd - dy) / radial
    return np.stack([x, y], axis=1)


def homography(source, target):
    """The plane-to-plane homography (3x3, H[2, 2] = 1) taking source (N, 2) to target (N, 2), N >= 4, by the
    normalized direct linear transform."""
    def conditioner(points):
        mean = points.mean(axis=0)
        scale = math.sqrt(2) / max(np.sqrt(((points - mean) ** 2).sum(axis=1)).mean(), 1e-12)
        return np.array([[scale, 0., -scale * mean[0]], [0., scale, -scale * mean[1]], [0., 0., 1.]])
    src, dst = np.asarray(source, float), np.asarray(target, float)
    if len(src) < 4 or len(src) != len(dst):
        raise ValueError('A homography needs at least four point pairs')
    ts, td = conditioner(src), conditioner(dst)
    s = np.c_[src, np.ones(len(src))] @ ts.T
    d = np.c_[dst, np.ones(len(dst))] @ td.T
    rows = []
    for (x, y, w), (u, v, q) in zip(s, d):
        rows.append([0, 0, 0, -q * x, -q * y, -q * w, v * x, v * y, v * w])
        rows.append([q * x, q * y, q * w, 0, 0, 0, -u * x, -u * y, -u * w])
    h = np.linalg.svd(np.array(rows))[2][-1].reshape(3, 3)
    h = np.linalg.inv(td) @ h @ ts
    return h / h[2, 2]


def plane_pose(h, points):
    """T_camera_plane from the homography taking plane (x, y) to normalized image coordinates, with the plane points
    it was fitted to (N, 2) in front of the camera. (The plane's origin may well be behind it.)"""
    h1, h2, h3 = h[:, 0], h[:, 1], h[:, 2]
    scale = 2 / (np.linalg.norm(h1) + np.linalg.norm(h2))
    depth = np.asarray(points, float) @ h[2, :2] + h[2, 2]
    if np.mean(depth) * scale < 0:
        scale = -scale
    r1, r2 = h1 * scale, h2 * scale
    u, _, vt = np.linalg.svd(np.c_[r1, r2, np.cross(r1, r2)])
    r = u @ vt
    if np.linalg.det(r) < 0:
        r = u @ np.diag([1., 1., -1.]) @ vt
    return transform(r, h3 * scale)


def chordal_mean(rotations):
    """The rotation nearest (in the Frobenius sense) to the mean of rotation matrices."""
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    r = u @ vt
    return r if np.linalg.det(r) > 0 else u @ np.diag([1., 1., -1.]) @ vt
