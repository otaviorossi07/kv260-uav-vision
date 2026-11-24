"""Small pose helpers: quaternions, rotation angles, pose interpolation, Umeyama alignment.

Conventions: quaternions are (w, x, y, z), Hamilton, as in EuRoC. A pose T_AB (4x4) maps points
from frame B to frame A, so the position of B in A is T_AB[:3, 3].
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """(w, x, y, z) -> 3x3 rotation matrix. q need not be normalized."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> (w, x, y, z) with w >= 0."""
    m = np.asarray(R, dtype=float)
    tr = np.trace(m)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def rotation_angle_deg(R: np.ndarray) -> float:
    """Angle of a rotation matrix, degrees."""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two 3-vectors, degrees."""
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def make_pose(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def invert_pose(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_pose(R.T, -R.T @ t)


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)


def interpolate_poses(
    t_query: np.ndarray, t: np.ndarray, p: np.ndarray, q: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate positions (linear) and quaternions (slerp) at t_query.

    t must be increasing. Returns (p_i (M,3), q_i (M,4), valid (M,) bool); queries outside
    [t[0], t[-1]] are invalid and their rows are NaN.
    """
    t_query = np.asarray(t_query)
    m = len(t_query)
    p_i = np.full((m, 3), np.nan)
    q_i = np.full((m, 4), np.nan)
    valid = (t_query >= t[0]) & (t_query <= t[-1])
    idx = np.clip(np.searchsorted(t, t_query, side="right") - 1, 0, len(t) - 2)
    for k in np.nonzero(valid)[0]:
        i = idx[k]
        span = float(t[i + 1] - t[i])
        u = 0.0 if span == 0 else float(t_query[k] - t[i]) / span
        p_i[k] = p[i] + u * (p[i + 1] - p[i])
        q_i[k] = slerp(q[i], q[i + 1], u)
    return p_i, q_i, valid


def exp_so3(w: np.ndarray) -> np.ndarray:
    """Rotation matrix of the rotation vector w (rad), Rodrigues formula."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(w, dtype=float) / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def integrate_gyro(t_ns: np.ndarray, w: np.ndarray, t0_ns: int, t1_ns: int) -> np.ndarray:
    """Rotation of the IMU frame at t1 relative to t0 (R_B0_B1) from body-frame rates (N, 3) rad/s.

    Piecewise-constant rate per sample interval, clipped to [t0, t1]. No bias correction.
    """
    R = np.eye(3)
    i = max(int(np.searchsorted(t_ns, t0_ns, side="right")) - 1, 0)
    while i < len(t_ns) - 1 and t_ns[i] < t1_ns:
        a = max(t_ns[i], t0_ns)
        b = min(t_ns[i + 1], t1_ns)
        if b > a:
            R = R @ exp_so3(0.5 * (w[i] + w[i + 1]) * (b - a) * 1e-9)
        i += 1
    return R


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False) -> Tuple[float, np.ndarray, np.ndarray]:
    """Least-squares s, R, t with dst ~= s * R @ src + t (Umeyama 1991). src, dst: (N, 3)."""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    if with_scale:
        var_s = (xs ** 2).sum() / len(src)
        s = float(np.trace(np.diag(D) @ S) / var_s)
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t
