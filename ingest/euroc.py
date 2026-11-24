"""EuRoC MAV dataset reader (ASL folder format) - recorded frame source for bench stage B2.

Layout of one sequence (e.g. V1_01_easy/):
    mav0/cam0/data.csv, mav0/cam0/data/<t_ns>.png, mav0/cam0/sensor.yaml
    mav0/imu0/data.csv
    mav0/state_groundtruth_estimate0/data.csv

Ground truth is the pose of the body (IMU) frame B in the world W: T_WB. The camera pose is
T_WC = T_WB @ T_BC, where T_BC is 'T_BS' in the camera's sensor.yaml.
Dataset: Burri et al., "The EuRoC micro aerial vehicle datasets", IJRR 2016 (non-commercial use).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Tuple, Union

import numpy as np

PathLike = Union[str, Path]


@dataclass(frozen=True)
class CameraCalib:
    K: np.ndarray  # 3x3
    dist: np.ndarray  # radtan (k1, k2, p1, p2)
    resolution: Tuple[int, int]  # (width, height)
    T_BC: np.ndarray  # 4x4, camera -> body
    rate_hz: float


def _yaml_list(text: str, key: str) -> List[float]:
    """Numbers of 'key: [..]' or 'key:\\n  ...\\n  data: [..]' (EuRoC's %YAML:1.0, not PyYAML-safe)."""
    m = re.search(rf"^{key}:\s*(.*?)(?=^\S|\Z)", text, re.S | re.M)
    if not m:
        raise KeyError(key)
    block = m.group(1)
    br = re.search(r"\[(.*?)\]", block, re.S)
    body = br.group(1) if br else block
    return [float(v) for v in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", body)]


def _read_csv(path: Path) -> np.ndarray:
    return np.loadtxt(path, delimiter=",", comments="#", ndmin=2)


class EurocSequence:
    def __init__(self, root: PathLike) -> None:
        root = Path(root)
        self.root = root / "mav0" if (root / "mav0").is_dir() else root
        if not (self.root / "cam0").is_dir():
            raise FileNotFoundError(f"not a EuRoC ASL sequence: {root}")
        self.name = (root if root.name != "mav0" else root.parent).name

    def camera(self, cam: str = "cam0") -> CameraCalib:
        text = (self.root / cam / "sensor.yaml").read_text()
        fu, fv, cu, cv = _yaml_list(text, "intrinsics")
        K = np.array([[fu, 0, cu], [0, fv, cv], [0, 0, 1.0]])
        dist = np.array(_yaml_list(text, "distortion_coefficients"))
        w, h = (int(v) for v in _yaml_list(text, "resolution"))
        T = np.array(_yaml_list(text, "T_BS")[-16:]).reshape(4, 4)  # skip 'cols: 4 rows: 4'
        rate = _yaml_list(text, "rate_hz")[0]
        return CameraCalib(K, dist, (w, h), T, rate)

    def frames(self, cam: str = "cam0") -> Iterator[Tuple[int, Path]]:
        """(timestamp_ns, image path) in time order."""
        d = self.root / cam
        with open(d / "data.csv") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                ts, name = line.strip().split(",")[:2]
                yield int(ts), d / "data" / name.strip()

    def ground_truth(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(t_ns (N,), p_WB (N,3), q_WB (N,4) as w,x,y,z)."""
        a = _read_csv(self.root / "state_groundtruth_estimate0" / "data.csv")
        return a[:, 0].astype(np.int64), a[:, 1:4], a[:, 4:8]

    def ground_truth_biases(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(t_ns (N,), gyro bias rad/s (N,3), accel bias m/s^2 (N,3)) estimated with the ground truth."""
        a = _read_csv(self.root / "state_groundtruth_estimate0" / "data.csv")
        return a[:, 0].astype(np.int64), a[:, 11:14], a[:, 14:17]

    def imu(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(t_ns (N,), gyro rad/s (N,3), accel m/s^2 (N,3))."""
        a = _read_csv(self.root / "imu0" / "data.csv")
        return a[:, 0].astype(np.int64), a[:, 1:4], a[:, 4:7]


def load_gray(path: PathLike) -> np.ndarray:
    import cv2

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise IOError(f"cannot read image {path}")
    return img
