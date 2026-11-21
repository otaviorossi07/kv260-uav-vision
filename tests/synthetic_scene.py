"""Synthetic camera sequences with known poses, for VO tests without a dataset.

Renders a random 3D point cloud as small blurred dots seen by a pinhole camera that moves along a
known path, and can write the result in EuRoC ASL layout (images, sensor.yaml, ground truth).
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

from perception.vo.geometry import make_pose, rot_to_quat

W, H = 640, 480
K = np.array([[400.0, 0, 320.0], [0, 400.0, 240.0], [0, 0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def random_points(n: int = 600, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-4, 5, n), rng.uniform(-2.5, 2.5, n), rng.uniform(3, 9, n)])


def trajectory(n_frames: int, step_m: float = 0.02, yaw_rad: float = 0.002) -> List[np.ndarray]:
    """Camera poses T_WC: sideways translation along +x with a slow yaw (camera frame: z forward)."""
    return [make_pose(rot_y(yaw_rad * k), np.array([step_m * k, 0.01 * np.sin(0.1 * k), 0.0]))
            for k in range(n_frames)]


def render(points_w: np.ndarray, T_WC: np.ndarray, seed: int = 0) -> np.ndarray:
    R, p = T_WC[:3, :3], T_WC[:3, 3]
    Xc = (R.T @ (points_w - p).T).T
    Xc = Xc[Xc[:, 2] > 0.5]
    uv = (K @ (Xc / Xc[:, 2:3]).T).T[:, :2]
    img = np.full((H, W), 40.0, np.float32)
    for (u, v), z in zip(uv, Xc[:, 2]):
        if 0 <= u < W and 0 <= v < H:
            cv2.circle(img, (int(round(u * 4)), int(round(v * 4))), 8, 220.0, -1, lineType=cv2.LINE_AA, shift=2)
    img = cv2.GaussianBlur(img, (0, 0), 1.2)
    rng = np.random.default_rng(seed)
    img += rng.normal(0, 2.0, img.shape).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8)


def write_fake_euroc(root: Path, n_frames: int = 60, dt_ns: int = 50_000_000) -> Tuple[Path, List[np.ndarray]]:
    """Write a EuRoC-layout sequence (camera = body, T_BS = I). Returns (sequence dir, poses)."""
    seq = root / "SYN_01"
    cam = seq / "mav0" / "cam0"
    (cam / "data").mkdir(parents=True)
    gt_dir = seq / "mav0" / "state_groundtruth_estimate0"
    gt_dir.mkdir(parents=True)
    pts = random_points()
    poses = trajectory(n_frames)
    t0 = 1_000_000_000_000
    with open(cam / "data.csv", "w") as f_img, open(gt_dir / "data.csv", "w") as f_gt:
        f_img.write("#timestamp [ns],filename\n")
        f_gt.write("#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,bw_x,bw_y,bw_z,ba_x,ba_y,ba_z\n")
        for k, T in enumerate(poses):
            t = t0 + k * dt_ns
            cv2.imwrite(str(cam / "data" / f"{t}.png"), render(pts, T, seed=k))
            f_img.write(f"{t},{t}.png\n")
            q = rot_to_quat(T[:3, :3])
            f_gt.write(",".join(str(v) for v in [t, *T[:3, 3], *q] + [0.0] * 9) + "\n")
    # IMU at 200 Hz; body = camera, so the gyro sees the trajectory's constant yaw rate about +y
    imu_dir = seq / "mav0" / "imu0"
    imu_dir.mkdir(parents=True)
    yaw_rate = 0.002 / (dt_ns * 1e-9)  # trajectory() default yaw_rad per frame
    with open(imu_dir / "data.csv", "w") as f_imu:
        f_imu.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
        for t in range(t0, t0 + n_frames * dt_ns, 5_000_000):
            f_imu.write(f"{t},0.0,{yaw_rate},0.0,0.0,-9.81,0.0\n")
    (cam / "sensor.yaml").write_text(
        "%YAML:1.0\n"
        "sensor_type: camera\n"
        "T_BS:\n  cols: 4\n  rows: 4\n  data: [1.0, 0.0, 0.0, 0.0,\n"
        "         0.0, 1.0, 0.0, 0.0,\n         0.0, 0.0, 1.0, 0.0,\n         0.0, 0.0, 0.0, 1.0]\n"
        "rate_hz: 20\n"
        "resolution: [640, 480]\n"
        "camera_model: pinhole\n"
        "intrinsics: [400.0, 400.0, 320.0, 240.0] #fu, fv, cu, cv\n"
        "distortion_model: radial-tangential\n"
        "distortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n"
    )
    return seq, poses
