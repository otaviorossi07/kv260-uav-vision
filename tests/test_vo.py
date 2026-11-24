"""Unit tests for the VO stack (bench stage B2) on synthetic data; no dataset needed."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from ingest.euroc import EurocSequence, load_gray  # noqa: E402
from perception.vo.frontend import FeatureTracker  # noqa: E402
from perception.vo.geometry import (  # noqa: E402
    angle_between_deg,
    exp_so3,
    integrate_gyro,
    interpolate_poses,
    invert_pose,
    make_pose,
    quat_to_rot,
    rot_to_quat,
    rotation_angle_deg,
    umeyama,
)
from perception.vo.mono_vo import MonoVO, estimate_relative_pose  # noqa: E402
from tests.synthetic_scene import K, random_points, render, rot_y, trajectory, write_fake_euroc  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def random_rotation(rng) -> np.ndarray:
    q = rng.normal(size=4)
    return quat_to_rot(q)


# ---------------------------------------------------------------- geometry
def test_quaternion_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        R = random_rotation(rng)
        assert np.allclose(quat_to_rot(rot_to_quat(R)), R, atol=1e-9)


def test_rotation_angle_and_vector_angle():
    assert rotation_angle_deg(rot_y(np.radians(30))) == pytest.approx(30.0)
    assert angle_between_deg(np.array([1.0, 0, 0]), np.array([0, 2.0, 0])) == pytest.approx(90.0)


def test_umeyama_recovers_similarity():
    rng = np.random.default_rng(2)
    src = rng.normal(size=(100, 3))
    R, t, s = random_rotation(rng), np.array([1.0, -2.0, 0.5]), 2.5
    dst = s * (R @ src.T).T + t
    s_e, R_e, t_e = umeyama(src, dst, with_scale=True)
    assert s_e == pytest.approx(s)
    assert np.allclose(R_e, R) and np.allclose(t_e, t)


def test_interpolate_poses_midpoint_and_range():
    t = np.array([0, 10, 20])
    p = np.array([[0.0, 0, 0], [1.0, 0, 0], [1.0, 1.0, 0]])
    q = np.array([rot_to_quat(np.eye(3)), rot_to_quat(rot_y(np.radians(20))), rot_to_quat(rot_y(np.radians(40)))])
    p_i, q_i, valid = interpolate_poses(np.array([5, 15, 25]), t, p, q)
    assert valid.tolist() == [True, True, False]
    assert np.allclose(p_i[0], [0.5, 0, 0]) and np.allclose(p_i[1], [1.0, 0.5, 0])
    assert rotation_angle_deg(quat_to_rot(q_i[0])) == pytest.approx(10.0, abs=1e-6)


def test_integrate_gyro_constant_rate():
    t = np.arange(0, 1_000_000_001, 5_000_000)  # 200 Hz for 1 s
    w = np.tile([0.0, 0.5, 0.0], (len(t), 1))  # 0.5 rad/s about y
    R = integrate_gyro(t, w, 100_000_000, 700_000_000)  # 0.6 s, between samples
    assert np.allclose(R, rot_y(0.3), atol=1e-9)
    assert np.allclose(exp_so3(np.array([0.0, 0.3, 0.0])), rot_y(0.3))


# ---------------------------------------------------------------- relative pose
def test_essential_matrix_pose_from_points():
    rng = np.random.default_rng(3)
    X0 = np.column_stack([rng.uniform(-3, 3, 200), rng.uniform(-2, 2, 200), rng.uniform(3, 10, 200)])
    R_01 = rot_y(np.radians(3.0))  # pose of camera 1 in camera 0
    t_01 = np.array([0.3, 0.05, 0.1])
    X1 = (R_01.T @ (X0 - t_01).T).T
    p0 = X0[:, :2] / X0[:, 2:3] + rng.normal(0, 0.3 / 400, (200, 2))  # 0.3 px noise at f=400
    p1 = X1[:, :2] / X1[:, 2:3] + rng.normal(0, 0.3 / 400, (200, 2))
    R_e, t_e, inl = estimate_relative_pose(p0, p1, thresh_norm=1.0 / 400)
    assert rotation_angle_deg(R_01.T @ R_e) < 0.2
    assert angle_between_deg(t_01, t_e) < 2.0
    assert inl.sum() > 180


# ---------------------------------------------------------------- tracker
def test_tracker_follows_known_shift():
    pts = random_points(seed=4)
    img0 = render(pts, np.eye(4), seed=0)
    M = np.float32([[1, 0, 3.0], [0, 1, -2.0]])
    img1 = cv2.warpAffine(img0, M, (img0.shape[1], img0.shape[0]), borderValue=40)
    tr = FeatureTracker()
    tr.track(img0)
    tr.replenish(img0)
    ids0, p0 = tr.ids.copy(), tr.pts.copy()
    ids1, p1 = tr.track(img1)
    assert len(ids1) > 0.8 * len(ids0)
    d = p1 - p0[np.searchsorted(ids0, ids1)]
    assert np.allclose(np.median(d, axis=0), [3.0, -2.0], atol=0.1)


# ---------------------------------------------------------------- full VO on rendered frames
def test_mono_vo_on_rendered_sequence():
    pts = random_points(seed=5)
    poses = trajectory(40)
    vo = MonoVO(K, np.zeros(4))
    steps = []
    for k, T in enumerate(poses):
        s = vo.process(k * 50_000_000, render(pts, T, seed=k))
        if s is not None:
            steps.append((k, s))
    assert len(steps) >= 3
    t_of = {k * 50_000_000: T for k, T in enumerate(poses)}
    for _, s in steps:
        T_ab = invert_pose(t_of[s.t0_ns]) @ t_of[s.t1_ns]
        assert rotation_angle_deg(T_ab[:3, :3].T @ s.R_ab) < 0.5
        assert angle_between_deg(T_ab[:3, 3], s.t_ab_dir) < 10.0


# ---------------------------------------------------------------- EuRoC reader + bench tool
def test_euroc_reader_and_bench_tool(tmp_path):
    seq_dir, poses = write_fake_euroc(tmp_path, n_frames=50)
    seq = EurocSequence(seq_dir)
    cam = seq.camera()
    assert np.allclose(cam.K, K) and np.allclose(cam.T_BC, np.eye(4))
    assert cam.resolution == (640, 480) and cam.rate_hz == 20
    frames = list(seq.frames())
    assert len(frames) == 50 and load_gray(frames[0][1]).shape == (480, 640)
    t, p, q = seq.ground_truth()
    assert np.allclose(p[-1], poses[-1][:3, 3])

    spec = importlib.util.spec_from_file_location("vo_euroc", REPO / "tools" / "bench" / "vo_euroc.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    r = mod.run(seq_dir, tmp_path / "out", max_frames=0, threads=1, tag="test")
    assert r["keyframes"] >= 3 and r["chain_breaks"] == 0
    assert r["rotation_error_deg"]["median"] < 0.5
    assert r["vo_vs_gyro_rotation_deg"]["n"] == r["keyframes"]
    assert r["vo_vs_gyro_rotation_deg"]["median"] < 0.5
    assert r["scale_oracle_trajectory"]["ate_rmse_m"] < 0.05
    assert (tmp_path / "out" / "SYN_01_test.json").exists()
    assert (tmp_path / "out" / "SYN_01_test_traj.csv").exists()


def test_euroc_yaml_parser_matches_real_layout(tmp_path):
    """sensor.yaml in EuRoC's cam0 layout (%YAML:1.0, multi-line T_BS block, trailing comments)."""
    d = tmp_path / "S" / "mav0" / "cam0"
    d.mkdir(parents=True)
    (d / "sensor.yaml").write_text(
        "%YAML:1.0\n# General sensor definitions.\nsensor_type: camera\n"
        "comment: VI-Sensor cam0 (MT9M034)\n\n# Sensor extrinsics wrt. the body-frame.\nT_BS:\n"
        "  cols: 4\n  rows: 4\n"
        "  data: [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,\n"
        "         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,\n"
        "        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,\n"
        "         0.0, 0.0, 0.0, 1.0]\n\n# Camera specific definitions.\nrate_hz: 20\n"
        "resolution: [752, 480]\ncamera_model: pinhole\n"
        "intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv\n"
        "distortion_model: radial-tangential\n"
        "distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]\n"
    )
    cam = EurocSequence(tmp_path / "S").camera()
    assert cam.K[0, 0] == pytest.approx(458.654) and cam.K[1, 2] == pytest.approx(248.375)
    assert cam.dist[3] == pytest.approx(1.76187114e-05)
    assert cam.T_BC[0, 3] == pytest.approx(-0.0216401454975) and cam.T_BC[3, 3] == 1.0
    assert cam.resolution == (752, 480)
