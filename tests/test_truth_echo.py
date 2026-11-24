"""Unit tests for the pure helpers of tools/bench/truth_echo.py (no FC needed)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "bench"))
from truth_echo import DelayLine, ned_from_global, quat_mul, quat_to_rot, step_response  # noqa: E402

ORIGIN = (-353632621, 1491652374, 584.08)  # ArduPilot SITL default home (CMAC)


def test_ned_from_global():
    assert np.allclose(ned_from_global(*ORIGIN, origin=ORIGIN), 0)
    # 1e-5 deg of latitude north is ~1.113 m; altitude up is negative D
    ned = ned_from_global(ORIGIN[0] + 100, ORIGIN[1], ORIGIN[2] + 2.0, origin=ORIGIN)
    assert abs(ned[0] - 1.1132) < 1e-3 and abs(ned[1]) < 1e-9 and abs(ned[2] + 2.0) < 1e-9
    # east is scaled by cos(latitude)
    ned = ned_from_global(ORIGIN[0], ORIGIN[1] + 100, ORIGIN[2], origin=ORIGIN)
    assert abs(ned[1] - 1.1132 * math.cos(math.radians(-35.3632621))) < 1e-3


def test_quaternion_yaw():
    yaw = math.radians(30)
    qz = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    R = quat_to_rot(qz)
    assert np.allclose(R @ [1, 0, 0], [math.cos(yaw), math.sin(yaw), 0])
    # composing two yaw rotations adds the angles
    assert np.allclose(quat_to_rot(quat_mul(qz, qz)) @ [1, 0, 0], [math.cos(2 * yaw), math.sin(2 * yaw), 0])


def test_delay_line():
    d = DelayLine(0.05)
    d.push(1.00, "a")
    d.push(1.02, "b")
    assert d.pop_due(1.049) == []
    assert d.pop_due(1.05) == [(1.00, "a")]
    assert d.pop_due(2.0) == [(1.02, "b")]


def test_step_response_first_order():
    t = np.arange(0, 5, 0.01)
    tau = 1.0
    x = np.where(t >= 1.0, 0.3 * (1 - np.exp(-(t - 1.0) / tau)), 0.0)
    r = step_response(t, x, 1.0, 0.3, 0.0)
    assert abs(r["t63_ms"] - 1000) < 15
    assert abs(r["t90_ms"] - 1000 * math.log(10)) < 15
    # a downward step works too
    r = step_response(t, 0.3 - x, 1.0, -0.3, 0.3)
    assert abs(r["t63_ms"] - 1000) < 15
