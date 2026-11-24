"""Monocular keyframe VO: relative pose between keyframes from the essential matrix.

Monocular VO gives rotation and the *direction* of translation, not its length (scale). In the
full system the scale comes from fusion with the IMU (and/or height), not from this module.

Keyframe rule: features are tracked frame to frame; when the median pixel displacement since the
last keyframe exceeds min_parallax_px (or too few tracks survive), estimate the pose between the
keyframe and the current frame. If that succeeds, the current frame becomes the new keyframe.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from perception.vo.frontend import FeatureTracker, TrackerConfig


@dataclass
class VoConfig:
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    min_parallax_px: float = 12.0
    min_track_ratio: float = 0.4  # force a keyframe attempt when tracks drop below this fraction
    min_inliers: int = 30
    ransac_px: float = 1.0
    ransac_prob: float = 0.999
    # USAC_DEFAULT = RANSAC with local optimization (OpenCV >= 4.5). On synthetic points with 0.3 px
    # noise it gave ~10x lower rotation and direction error than plain RANSAC.
    ransac_method: str = "USAC_DEFAULT"
    # Sampson-error LM refinement on the inliers. USAC is much weaker in OpenCV 4.5 (board) than
    # in 5.x; the refinement makes accuracy independent of the OpenCV version.
    refine: bool = True


@dataclass(frozen=True)
class KeyframeStep:
    """Motion from keyframe A (t0) to keyframe B (t1), expressed as the pose of B in A."""

    t0_ns: int
    t1_ns: int
    R_ab: np.ndarray  # 3x3
    t_ab_dir: np.ndarray  # (3,), unit vector: direction of B's position in A's frame
    n_tracked: int
    n_inliers: int
    parallax_px: float


def estimate_relative_pose(
    p0_norm: np.ndarray,
    p1_norm: np.ndarray,
    thresh_norm: float,
    prob: float = 0.999,
    method: str = "USAC_DEFAULT",
    refine: bool = True,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Pose of camera 1 in camera 0 from normalized image points (N, 2).

    Returns (R_01, t_01_unit, inlier_mask) or None. OpenCV's recoverPose gives the point
    transform x1 = R x0 + t; the pose of camera 1 in camera 0 is its inverse.
    method: an OpenCV robust-estimator name; falls back to RANSAC if this OpenCV lacks it.
    """
    if len(p0_norm) < 8:
        return None
    p0 = np.ascontiguousarray(p0_norm, dtype=np.float64)
    p1 = np.ascontiguousarray(p1_norm, dtype=np.float64)
    flag = getattr(cv2, method, cv2.RANSAC)
    E, mask = cv2.findEssentialMat(p0, p1, focal=1.0, pp=(0.0, 0.0), method=flag,
                                   prob=prob, threshold=thresh_norm)
    if E is None or E.shape[1:] != (3,) or E.shape[0] % 3:
        return None
    # The 5-point solver can return several candidates stacked as (3k, 3): keep the one with the
    # most points in front of both cameras. Use the distanceThresh overload: the short form drops
    # points farther than 50 baselines (6 m at a 12 cm baseline), i.e. most of a room-sized scene.
    best = None
    for k in range(E.shape[0] // 3):
        # keywords matter: passed positionally, OpenCV's binding picks the short overload
        out = cv2.recoverPose(E[3 * k:3 * k + 3], p0, p1, cameraMatrix=np.eye(3),
                              distanceThresh=1e4, mask=mask.copy())
        if best is None or out[0] > best[0]:
            best = out
    n, R, t, mask = best[:4]
    if n < 8:
        return None
    inl = mask.ravel() > 0
    if refine:
        R, t = refine_pose_sampson(p0[inl], p1[inl], R, t.ravel())
    R_01 = R.T
    t_01 = (-R.T @ np.reshape(t, 3)).ravel()
    norm = np.linalg.norm(t_01)
    if norm == 0:
        return None
    return R_01, t_01 / norm, inl


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _sampson_residuals(p0: np.ndarray, p1: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Signed Sampson distances for x1 = R x0 + t (normalized coordinates, (N, 2) each)."""
    E = _skew(t) @ R
    x0 = np.column_stack([p0, np.ones(len(p0))])
    x1 = np.column_stack([p1, np.ones(len(p1))])
    Ex0 = x0 @ E.T
    Etx1 = x1 @ E
    e = np.sum(x1 * Ex0, axis=1)
    den = Ex0[:, 0] ** 2 + Ex0[:, 1] ** 2 + Etx1[:, 0] ** 2 + Etx1[:, 1] ** 2
    return e / np.sqrt(np.maximum(den, 1e-18))


def refine_pose_sampson(
    p0: np.ndarray, p1: np.ndarray, R: np.ndarray, t: np.ndarray, iters: int = 10
) -> Tuple[np.ndarray, np.ndarray]:
    """Levenberg-Marquardt on (rotation, translation direction) minimizing Sampson error.

    Same convention as cv2.recoverPose (x1 = R x0 + t); t is kept unit length. Pure numpy, so the
    result does not depend on which OpenCV robust estimator produced the starting point.
    """
    t = t / np.linalg.norm(t)
    lam = 1e-3

    def apply(R0, t0, B, d):
        Rd, _ = cv2.Rodrigues(d[:3].reshape(3, 1))
        tn = t0 + B @ d[3:]
        return R0 @ Rd, tn / np.linalg.norm(tn)

    r = _sampson_residuals(p0, p1, R, t)
    cost = float(r @ r)
    for _ in range(iters):
        # tangent basis of the unit sphere at t
        a = np.array([1.0, 0, 0]) if abs(t[0]) < 0.9 else np.array([0, 1.0, 0])
        b1 = np.cross(t, a)
        b1 /= np.linalg.norm(b1)
        B = np.column_stack([b1, np.cross(t, b1)])
        J = np.empty((len(r), 5))
        eps = 1e-7
        for k in range(5):
            d = np.zeros(5)
            d[k] = eps
            Rk, tk = apply(R, t, B, d)
            J[:, k] = (_sampson_residuals(p0, p1, Rk, tk) - r) / eps
        H = J.T @ J
        g = J.T @ r
        improved = False
        for _ in range(5):
            step = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), g)
            R_new, t_new = apply(R, t, B, step)
            r_new = _sampson_residuals(p0, p1, R_new, t_new)
            c_new = float(r_new @ r_new)
            if c_new < cost:
                R, t, r, cost = R_new, t_new, r_new, c_new
                lam = max(lam / 10, 1e-9)
                improved = True
                break
            lam *= 10
        if not improved or np.linalg.norm(step) < 1e-10:
            break
    return R, t


class MonoVO:
    def __init__(self, K: np.ndarray, dist: np.ndarray, config: Optional[VoConfig] = None) -> None:
        self.K = np.asarray(K, dtype=np.float64)
        self.dist = np.asarray(dist, dtype=np.float64)
        self.cfg = config or VoConfig()
        self.tracker = FeatureTracker(self.cfg.tracker)
        self.focal = float((self.K[0, 0] + self.K[1, 1]) / 2.0)
        self._kf_t: Optional[int] = None
        self._kf_pts: Dict[int, np.ndarray] = {}  # id -> pixel position in the keyframe
        self._kf_count = 0
        # per-stage wall time of the last process() call, ms (stages that did not run are absent);
        # used by tools/bench to decide which stages are worth moving to the FPGA
        self.last_stage_ms: Dict[str, float] = {}

    def _timed(self, stage: str, fn, *args):
        t0 = time.perf_counter()
        out = fn(*args)
        self.last_stage_ms[stage] = self.last_stage_ms.get(stage, 0.0) + (time.perf_counter() - t0) * 1e3
        return out

    def _undistort(self, pts: np.ndarray) -> np.ndarray:
        if len(pts) == 0:
            return np.empty((0, 2))
        return cv2.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), self.K, self.dist).reshape(-1, 2)

    def _set_keyframe(self, t_ns: int, img: np.ndarray) -> None:
        self._timed("replenish", self.tracker.replenish, img)
        self._kf_t = t_ns
        self._kf_pts = {int(i): p.copy() for i, p in zip(self.tracker.ids, self.tracker.pts)}
        self._kf_count = len(self._kf_pts)

    def process(self, t_ns: int, img: np.ndarray) -> Optional[KeyframeStep]:
        """Feed one grayscale frame. Returns a KeyframeStep when a new keyframe is accepted."""
        self.last_stage_ms = {}
        ids, pts = self._timed("track", self.tracker.track, img)
        if self._kf_t is None:
            self._set_keyframe(t_ns, img)
            return None
        common = [k for k, i in enumerate(ids) if int(i) in self._kf_pts]
        if len(common) < self.cfg.min_inliers:
            self._set_keyframe(t_ns, img)  # tracking lost: restart from this frame
            return None
        cur = pts[common]
        ref = np.array([self._kf_pts[int(ids[k])] for k in common], dtype=np.float32)
        p_ref = self._timed("undistort", self._undistort, ref)
        p_cur = self._timed("undistort", self._undistort, cur)
        parallax = float(np.median(np.linalg.norm(p_cur - p_ref, axis=1)) * self.focal)
        few_tracks = len(common) < self.cfg.min_track_ratio * max(self._kf_count, 1)
        if parallax < self.cfg.min_parallax_px and not few_tracks:
            return None
        est = self._timed("pose", estimate_relative_pose, p_ref, p_cur, self.cfg.ransac_px / self.focal,
                          self.cfg.ransac_prob, self.cfg.ransac_method, self.cfg.refine)
        if est is None or int(est[2].sum()) < self.cfg.min_inliers:
            if few_tracks:
                self._set_keyframe(t_ns, img)
            return None
        R_ab, t_dir, inl = est
        step = KeyframeStep(self._kf_t, t_ns, R_ab, t_dir, len(common), int(inl.sum()), parallax)
        self._set_keyframe(t_ns, img)
        return step
