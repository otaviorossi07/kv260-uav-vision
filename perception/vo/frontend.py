"""Sparse feature tracking: Shi-Tomasi corners + pyramidal Lucas-Kanade with a forward-backward check.

Plain OpenCV on the CPU (A53 on the KV260). Moving LK to the PL is phase 7's "only if measured".
Uses only APIs present in OpenCV 4.5 (Ubuntu 22.04 apt package on the board).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class TrackerConfig:
    max_features: int = 300
    quality_level: float = 0.01
    min_distance_px: int = 15
    block_size: int = 7
    lk_window_px: int = 21
    lk_levels: int = 3
    fb_max_err_px: float = 1.0  # forward-backward consistency threshold


class FeatureTracker:
    """Tracks corners frame to frame. Each feature has a stable integer id."""

    def __init__(self, config: Optional[TrackerConfig] = None) -> None:
        self.cfg = config or TrackerConfig()
        self._lk = dict(
            winSize=(self.cfg.lk_window_px, self.cfg.lk_window_px),
            maxLevel=self.cfg.lk_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self.prev_img: Optional[np.ndarray] = None
        self.pts = np.empty((0, 2), np.float32)  # current positions, pixels (distorted)
        self.ids = np.empty((0,), np.int64)
        self._next_id = 0

    def track(self, img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Track existing features into img. Returns (ids, pts) of the survivors."""
        if self.prev_img is not None and len(self.pts):
            p0 = self.pts.reshape(-1, 1, 2)
            p1, st1, _ = cv2.calcOpticalFlowPyrLK(self.prev_img, img, p0, None, **self._lk)
            p0r, st2, _ = cv2.calcOpticalFlowPyrLK(img, self.prev_img, p1, None, **self._lk)
            fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
            h, w = img.shape[:2]
            p1 = p1.reshape(-1, 2)
            inside = (p1[:, 0] >= 0) & (p1[:, 0] < w) & (p1[:, 1] >= 0) & (p1[:, 1] < h)
            ok = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < self.cfg.fb_max_err_px) & inside
            self.pts, self.ids = p1[ok].astype(np.float32), self.ids[ok]
        self.prev_img = img
        return self.ids, self.pts

    def replenish(self, img: np.ndarray) -> None:
        """Detect new corners away from the current ones, up to max_features."""
        need = self.cfg.max_features - len(self.pts)
        if need <= 0:
            return
        mask = np.full(img.shape[:2], 255, np.uint8)
        for x, y in self.pts:
            cv2.circle(mask, (int(x), int(y)), self.cfg.min_distance_px, 0, -1)
        new = cv2.goodFeaturesToTrack(
            img, need, self.cfg.quality_level, self.cfg.min_distance_px,
            mask=mask, blockSize=self.cfg.block_size,
        )
        if new is None:
            return
        new = new.reshape(-1, 2).astype(np.float32)
        self.pts = np.vstack([self.pts, new])
        self.ids = np.concatenate([self.ids, np.arange(self._next_id, self._next_id + len(new))])
        self._next_id += len(new)
