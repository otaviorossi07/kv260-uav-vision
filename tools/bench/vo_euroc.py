"""Bench stage B2: monocular VO on a EuRoC MAV sequence, scored against ground truth.

    python tools/bench/vo_euroc.py data/euroc/V1_01_easy [--max-frames N] [--threads N]

Writes results/b2_vo_euroc/<sequence>_<platform>.json (metrics, config, environment) and
<sequence>_<platform>_traj.csv (keyframe positions, estimate vs ground truth) and
<sequence>_<platform>_steps.csv (per keyframe step: VO and ground-truth directions, errors).

Metrics:
- rotation error per keyframe step (deg): what the camera measures directly;
- translation *direction* error per step (deg), for steps that moved > 2 cm;
- "scale-oracle" trajectory: VO rotation + direction, step length taken from ground truth.
  Monocular VO has no metric scale; in the full system it comes from the IMU. This isolates
  how the camera part drifts. ATE RMSE (SE3-aligned) and end drift as % of path length;
- VO rotation vs the integrated gyro over the same interval (camera-IMU consistency, what VIO
  fusion relies on);
- tracking: keyframes, tracked features, RANSAC inliers, chain breaks (tracking resets);
- timing: per-frame VO time (image decode excluded), median / p95, fps.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from ingest.euroc import EurocSequence, load_gray
from perception.vo.geometry import (
    angle_between_deg,
    integrate_gyro,
    interpolate_poses,
    invert_pose,
    make_pose,
    quat_to_rot,
    rotation_angle_deg,
    umeyama,
)
from perception.vo.mono_vo import MonoVO, VoConfig


def gt_camera_poses(seq: EurocSequence, t_query: np.ndarray, T_BC: np.ndarray):
    t, p, q = seq.ground_truth()
    p_i, q_i, valid = interpolate_poses(t_query, t, p, q)
    poses = {}
    for k in np.nonzero(valid)[0]:
        poses[int(t_query[k])] = make_pose(quat_to_rot(q_i[k]), p_i[k]) @ T_BC
    return poses


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def by_length(err, length, edges=(0.02, 0.05, 0.10, 0.20, np.inf)):
    """Median / p95 direction error per ground-truth step-length bin (m)."""
    err, length = np.asarray(err), np.asarray(length)
    out = {}
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = err[(length > lo) & (length <= hi)]
        key = f"{lo:.2f}-{hi:.2f}m" if np.isfinite(hi) else f">{lo:.2f}m"
        out[key] = {"n": int(len(sel)), "median": pct(sel, 50), "p95": pct(sel, 95)}
    return out


def run(seq_dir: Path, out_dir: Path, max_frames: int, threads: int, tag: str,
        cfg: Optional[VoConfig] = None) -> dict:
    cv2.setNumThreads(threads)
    seq = EurocSequence(seq_dir)
    cam = seq.camera("cam0")
    frames = list(seq.frames("cam0"))
    if max_frames:
        frames = frames[:max_frames]
    cfg = cfg or VoConfig()
    vo = MonoVO(cam.K, cam.dist, cfg)

    t_frames = np.array([t for t, _ in frames], dtype=np.int64)
    gt = gt_camera_poses(seq, t_frames, cam.T_BC)

    steps, frame_ms = [], []
    for t_ns, path in frames:
        img = load_gray(path)
        t0 = time.perf_counter()
        step = vo.process(t_ns, img)
        frame_ms.append((time.perf_counter() - t0) * 1e3)
        if step is not None:
            steps.append(step)

    imu = seq.imu() if (seq.root / "imu0" / "data.csv").exists() else None
    R_BC = cam.T_BC[:3, :3]
    if imu is not None:
        tb, bw, _ = seq.ground_truth_biases()
        bias_at_imu = np.column_stack([np.interp(imu[0], tb, bw[:, k]) for k in range(3)])
        gyro_corr = imu[1] - bias_at_imu

    rot_err, dir_err, gyro_err, gyro_err_corr, rows = [], [], [], [], []
    dir_len, step_rows = [], []
    est_pos, gt_pos = [], []
    breaks, path_len = 0, 0.0
    T_est = None
    last_t1 = None
    for s in steps:
        if s.t0_ns not in gt or s.t1_ns not in gt:
            continue
        Ta, Tb = gt[s.t0_ns], gt[s.t1_ns]
        T_ab = invert_pose(Ta) @ Tb
        rot_err.append(rotation_angle_deg(T_ab[:3, :3].T @ s.R_ab))
        if imu is not None:
            R_gyro_cam = R_BC.T @ integrate_gyro(imu[0], imu[1], s.t0_ns, s.t1_ns) @ R_BC
            gyro_err.append(rotation_angle_deg(R_gyro_cam.T @ s.R_ab))
            R_gyro_cam = R_BC.T @ integrate_gyro(imu[0], gyro_corr, s.t0_ns, s.t1_ns) @ R_BC
            gyro_err_corr.append(rotation_angle_deg(R_gyro_cam.T @ s.R_ab))
        L = float(np.linalg.norm(T_ab[:3, 3]))
        if L > 0.02:
            dir_err.append(angle_between_deg(T_ab[:3, 3], s.t_ab_dir))
            dir_len.append(L)
        step_rows.append((s.t0_ns, s.t1_ns, L, *s.t_ab_dir, *(T_ab[:3, 3] / max(L, 1e-12)),
                          rot_err[-1], s.n_inliers))
        if T_est is None or s.t0_ns != last_t1:
            if T_est is not None:
                breaks += 1
            T_est = Ta.copy()  # (re)anchor the chain on ground truth
            est_pos.append(T_est[:3, 3].copy())
            gt_pos.append(Ta[:3, 3].copy())
            rows.append((s.t0_ns, *T_est[:3, 3], *Ta[:3, 3]))
        T_est = T_est @ make_pose(s.R_ab, s.t_ab_dir * L)
        path_len += L
        est_pos.append(T_est[:3, 3].copy())
        gt_pos.append(Tb[:3, 3].copy())
        rows.append((s.t1_ns, *T_est[:3, 3], *Tb[:3, 3]))
        last_t1 = s.t1_ns

    est_pos, gt_pos = np.array(est_pos), np.array(gt_pos)
    ate = end_drift = None
    if len(est_pos) >= 3:
        _, R, t = umeyama(est_pos, gt_pos, with_scale=False)
        aligned = (R @ est_pos.T).T + t
        ate = float(np.sqrt(np.mean(np.sum((aligned - gt_pos) ** 2, axis=1))))
        end_drift = float(np.linalg.norm(est_pos[-1] - gt_pos[-1]) / path_len * 100) if path_len else None

    frame_ms = np.array(frame_ms)
    total_s = frame_ms.sum() / 1e3
    duration_s = (t_frames[-1] - t_frames[0]) / 1e9 if len(t_frames) > 1 else 0.0
    result = {
        "stage": "B2",
        "sequence": seq.name,
        "camera": "cam0",
        "frames": len(frames),
        "duration_s": round(duration_s, 2),
        "keyframes": len(steps),
        "chain_breaks": breaks,
        "rotation_error_deg": {"median": pct(rot_err, 50), "p95": pct(rot_err, 95), "max": max(rot_err, default=None)},
        "direction_error_deg": {"median": pct(dir_err, 50), "p95": pct(dir_err, 95), "n": len(dir_err)},
        "direction_error_by_step_length": by_length(dir_err, dir_len),
        # VO rotation vs integrated gyro via the camera-IMU extrinsics: raw, and with the gyro bias
        # from the dataset's ground-truth estimate removed (what a VIO filter has to estimate)
        "vo_vs_gyro_rotation_deg": {"median": pct(gyro_err, 50), "p95": pct(gyro_err, 95), "n": len(gyro_err)},
        "vo_vs_gyro_bias_corrected_deg": {"median": pct(gyro_err_corr, 50), "p95": pct(gyro_err_corr, 95)},
        "scale_oracle_trajectory": {
            "path_length_m": round(path_len, 3),
            "ate_rmse_m": ate,
            "end_drift_pct_of_path": end_drift,
        },
        "tracking": {
            "tracked_median": pct([s.n_tracked for s in steps], 50),
            "inliers_median": pct([s.n_inliers for s in steps], 50),
            "parallax_px_median": pct([s.parallax_px for s in steps], 50),
        },
        "timing_ms_per_frame": {"median": pct(frame_ms, 50), "p95": pct(frame_ms, 95), "max": float(frame_ms.max())},
        "fps": round(len(frames) / total_s, 1) if total_s else None,
        "realtime_factor": round(duration_s / total_s, 2) if total_s else None,
        "config": {
            "min_parallax_px": cfg.min_parallax_px, "min_inliers": cfg.min_inliers,
            "ransac_px": cfg.ransac_px, "ransac_method": cfg.ransac_method, "max_features": cfg.tracker.max_features,
            "lk_window_px": cfg.tracker.lk_window_px, "lk_levels": cfg.tracker.lk_levels,
            "fb_max_err_px": cfg.tracker.fb_max_err_px,
        },
        "environment": {
            "platform": tag, "machine": platform.machine(), "processor": platform.processor(),
            "python": platform.python_version(), "opencv": cv2.__version__, "numpy": np.__version__,
            "cv_threads": threads, "cpu_count": os.cpu_count(),
            "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        },
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{seq.name}_{tag}"
    (out_dir / f"{stem}.json").write_text(json.dumps(result, indent=2))
    with open(out_dir / f"{stem}_traj.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ns", "est_x", "est_y", "est_z", "gt_x", "gt_y", "gt_z"])
        for r in rows:
            w.writerow([r[0]] + [f"{v:.4f}" for v in r[1:]])
    with open(out_dir / f"{stem}_steps.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t0_ns", "t1_ns", "gt_len_m", "vo_dir_x", "vo_dir_y", "vo_dir_z",
                    "gt_dir_x", "gt_dir_y", "gt_dir_z", "rot_err_deg", "inliers"])
        for r in step_rows:
            w.writerow([r[0], r[1]] + [f"{v:.5f}" for v in r[2:-1]] + [r[-1]])
    return result


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("sequence", type=Path, help="EuRoC sequence folder (contains mav0/)")
    p.add_argument("--out", type=Path, default=Path("results/b2_vo_euroc"))
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--threads", type=int, default=max(1, int((os.cpu_count() or 1) * 0.3)),
                   help="OpenCV threads (default ~30%% of CPU threads)")
    p.add_argument("--tag", default=platform.machine().lower(), help="platform tag in output names")
    p.add_argument("--min-parallax", type=float, default=VoConfig.min_parallax_px,
                   help="keyframe parallax threshold, px (default %(default)s)")
    a = p.parse_args()
    r = run(a.sequence, a.out, a.max_frames, a.threads, a.tag, VoConfig(min_parallax_px=a.min_parallax))
    print(json.dumps({k: r[k] for k in ("sequence", "frames", "keyframes", "chain_breaks",
                                        "rotation_error_deg", "direction_error_deg", "vo_vs_gyro_rotation_deg",
                                        "vo_vs_gyro_bias_corrected_deg", "scale_oracle_trajectory", "fps",
                                        "realtime_factor")}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
