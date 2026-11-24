"""Bench stage B4: truth echo. Feed the simulated truth back to the FC as vision odometry.

    python tools/bench/truth_echo.py COM6 --tag usb
    python tools/bench/truth_echo.py /dev/ttyUSB0,921600 --tag uart7

ArduPilot with the on-board simulation only (the Nucleo SimVIO build, or SITL). The tool reads
the simulated truth (SIM_STATE), adds known noise and a known delay, and sends it as ODOMETRY.
The error is therefore known exactly, which tests the transport, the time handling and EKF3's
use of external navigation on a real MCU.

What it does, in phases (seconds from start):
  0-8    echo running, EKF still on GPS (baseline)
  8      switch EKF3 source set 1 to external nav (EK3_SRC1_POSXY/VELXY/YAW = 6), set VISO_DELAY_MS
  8-28   settle
  28-48  steady state: EKF position vs truth, innovation test ratio
  48-63  step: echo shifted +step_m north; EKF reaction (onset, 63 %, 90 %)
  63-78  step removed
Original parameters are restored at the end (the Nucleo also forgets them at reboot).

Safety: sends only PARAM_SET (EKF source and vision delay), SET_MESSAGE_INTERVAL and ODOMETRY.
It never arms or changes mode, refuses to start if the FC is armed, and refuses an FC that does
not report SIM_STATE (a real vehicle), so fake vision is never sent to real hardware.

A stationary vehicle cannot show whether the delay is compensated correctly (that needs motion);
this run checks fusion, accuracy and the EKF's reaction to a vision change.

Writes results/b4_nucleo/truth_echo_<tag>.json and truth_echo_<tag>_log.csv.
"""

from __future__ import annotations

import argparse
import collections
import csv
import json
import math
import os
import platform
import sys
import time
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from fc_link.link import parse_connection  # noqa: E402

EARTH_RADIUS_M = 6378137.0
EXT_NAV = 6
PHASES = {"switch": 8.0, "steady": 28.0, "step_on": 48.0, "step_off": 63.0, "end": 78.0}


# ------------------------------------------------------------------ pure helpers (unit-tested)
def ned_from_global(lat_e7: int, lon_e7: int, alt_m: float,
                    origin: Tuple[int, int, float]) -> np.ndarray:
    """Flat-earth NED (m) of a point relative to an origin (lat_e7, lon_e7, alt_m)."""
    lat0, lon0, alt0 = origin
    d2r = math.pi / 180.0
    n = (lat_e7 - lat0) * 1e-7 * d2r * EARTH_RADIUS_M
    e = (lon_e7 - lon0) * 1e-7 * d2r * EARTH_RADIUS_M * math.cos(lat0 * 1e-7 * d2r)
    return np.array([n, e, -(alt_m - alt0)])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class DelayLine:
    """Holds items until `delay_s` after their timestamp (the injected vision delay)."""

    def __init__(self, delay_s: float) -> None:
        self.delay_s = delay_s
        self._q: Deque[Tuple[float, object]] = collections.deque()

    def push(self, t: float, item: object) -> None:
        self._q.append((t, item))

    def pop_due(self, now: float) -> List[Tuple[float, object]]:
        out = []
        while self._q and self._q[0][0] + self.delay_s <= now:
            out.append(self._q.popleft())
        return out


def step_response(t: np.ndarray, x: np.ndarray, t_step: float, step: float,
                  x_before: float) -> Dict[str, Optional[float]]:
    """Times (ms after t_step) at which x first passes 10 %, 63 % and 90 % of a step."""
    out: Dict[str, Optional[float]] = {}
    for name, frac in (("onset_10pct_ms", 0.1), ("t63_ms", 0.63), ("t90_ms", 0.9)):
        after = np.nonzero((t >= t_step) & ((x - x_before) / step >= frac))[0]
        out[name] = float((t[after[0]] - t_step) * 1e3) if len(after) else None
    return out


def pct(a, q) -> Optional[float]:
    return float(np.percentile(a, q)) if len(a) else None


# ------------------------------------------------------------------ FC side
class Fc:
    def __init__(self, connection: str) -> None:
        from pymavlink import mavutil
        self.mavutil = mavutil
        device, baud = parse_connection(connection)
        kw = {"source_system": 1, "source_component": 191}
        if baud is not None:
            kw["baud"] = baud
        self.m = mavutil.mavlink_connection(device, **kw)
        self.ts = self.tc = None

    def wait_heartbeat(self, timeout: float):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.autopilot != self.mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.ts, self.tc = hb.get_srcSystem(), hb.get_srcComponent()
                return hb
        return None

    def interval(self, msg_id: int, hz: float) -> None:
        self.m.mav.command_long_send(self.ts, self.tc, 511, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def request_message(self, msg_id: int) -> None:
        self.m.mav.command_long_send(self.ts, self.tc, 512, 0, msg_id, 0, 0, 0, 0, 0, 0)

    def param_set(self, name: str, value: float) -> None:
        self.m.mav.param_set_send(self.ts, self.tc, name.encode(), float(value),
                                  self.mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def param_request(self, name: str) -> None:
        self.m.mav.param_request_read_send(self.ts, self.tc, name.encode(), -1)

    def recv(self):
        return self.m.recv_match(blocking=False)


def run(args) -> dict:
    rng = np.random.default_rng(args.seed)
    fc = Fc(args.connection)
    hb = fc.wait_heartbeat(30)
    if hb is None:
        raise SystemExit("no autopilot heartbeat")
    if hb.autopilot != fc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise SystemExit("truth echo needs ArduPilot (SIM_STATE + VISO); got autopilot %d" % hb.autopilot)
    if hb.base_mode & fc.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("FC is armed: refusing to change EKF sources")

    for mid, hz in ((108, args.rate), (32, args.rate), (193, 10), (30, 10)):
        fc.interval(mid, hz)  # SIM_STATE, LOCAL_POSITION_NED, EKF_STATUS_REPORT, ATTITUDE
    fc.request_message(49)  # GPS_GLOBAL_ORIGIN

    names = ["EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_YAW", "VISO_DELAY_MS", "VISO_TYPE",
             "EK3_SRC2_POSXY", "EK3_SRC3_POSXY"]
    for n in names:
        fc.param_request(n)

    params: Dict[str, float] = {}
    origin: Optional[Tuple[int, int, float]] = None
    got_sim = False
    got_pos = False  # LOCAL_POSITION_NED: the EKF is navigating on GPS (baseline is meaningful)
    t_wait = time.monotonic()
    end = t_wait + 120  # after a reboot: GPS fix, origin, then EKF position (~30-60 s)
    next_req = 0.0
    while time.monotonic() < end and not (origin and got_sim and got_pos and len(params) == len(names)):
        if time.monotonic() >= next_req:
            if origin is None:
                fc.request_message(49)
            for n in names:
                if n not in params:
                    fc.param_request(n)
            next_req = time.monotonic() + 3
        msg = fc.m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "SIM_STATE":
            got_sim = True
        elif t == "LOCAL_POSITION_NED":
            got_pos = True
        elif t == "GPS_GLOBAL_ORIGIN":
            origin = (msg.latitude, msg.longitude, msg.altitude * 1e-3)
        elif t == "PARAM_VALUE" and msg.param_id in names:
            params[msg.param_id] = msg.param_value
        if not got_sim and time.monotonic() - t_wait > 15:
            break
    if not got_sim:
        raise SystemExit("no SIM_STATE: not a simulated FC, refusing to send fake vision")
    if origin is None:
        raise SystemExit("no GPS_GLOBAL_ORIGIN: the EKF has no origin yet (wait for GPS fix)")
    if not got_pos:
        raise SystemExit("no LOCAL_POSITION_NED within 120 s: the EKF has no GPS position")
    if len(params) < len(names):
        raise SystemExit("parameters not readable: %s" % sorted(set(names) - set(params)))
    if EXT_NAV not in (params["EK3_SRC2_POSXY"], params["EK3_SRC3_POSXY"]):
        # EKF3 allocates its external-nav buffers at boot only if a source set uses it
        raise SystemExit("no EKF3 source set used external nav at boot (EK3_SRC2/3_POSXY != 6): "
                         "the EKF would ignore vision, see docs/errors.md E23")
    if params.get("VISO_TYPE", 0) == 0:
        raise SystemExit("VISO_TYPE is 0: visual odometry disabled in this firmware/params")
    original = {k: params[k] for k in names[:4]}
    print("origin", origin, "original params", original, flush=True)

    delay = DelayLine(args.delay_ms / 1e3)
    t0 = time.monotonic()
    truth_log: List[Tuple[float, np.ndarray]] = []  # (t, ned) at SIM_STATE receipt
    ekf_log: List[Tuple[float, np.ndarray]] = []
    sent_log: List[Tuple[float, float, np.ndarray]] = []  # (t_send, t_truth, ned sent)
    ekf_status: List[Tuple[float, int, float]] = []
    texts: List[Tuple[float, str]] = []
    switched = False
    restored = False
    stage_ms: Dict[str, List[float]] = {"handle_rx": [], "send": []}

    def set_params(vals: Dict[str, float]) -> None:
        for k, v in vals.items():
            fc.param_set(k, v)

    try:
        while True:
            now = time.monotonic()
            el = now - t0
            if el >= PHASES["end"]:
                break
            if not switched and el >= PHASES["switch"]:
                set_params({"EK3_SRC1_POSXY": EXT_NAV, "EK3_SRC1_VELXY": EXT_NAV,
                            "EK3_SRC1_YAW": EXT_NAV, "VISO_DELAY_MS": args.viso_delay_ms})
                switched = True
            for t_truth, item in delay.pop_due(time.monotonic()):
                a = time.perf_counter()
                ned, q, vel_b = item
                usec = int(t_truth * 1e6)  # sample time on our clock; FC corrects the offset
                nan21 = [float("nan")] * 21
                fc.m.mav.odometry_send(
                    usec, 20, 12,  # MAV_FRAME_LOCAL_FRD, MAV_FRAME_BODY_FRD
                    float(ned[0]), float(ned[1]), float(ned[2]), [float(v) for v in q],
                    float(vel_b[0]), float(vel_b[1]), float(vel_b[2]),
                    float("nan"), float("nan"), float("nan"), nan21, nan21, 0, 0, 100)
                stage_ms["send"].append((time.perf_counter() - a) * 1e3)
                sent_log.append((time.monotonic(), t_truth, ned))
            msg = fc.recv()
            if msg is None:
                time.sleep(0.001)
                continue
            a = time.perf_counter()
            t = msg.get_type()
            if t == "SIM_STATE":
                ned = ned_from_global(msg.lat_int, msg.lon_int, msg.alt, origin)
                truth_log.append((now, ned))
                q = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
                dyaw = rng.normal(0, math.radians(args.noise_yaw_deg))
                q = quat_mul(np.array([math.cos(dyaw / 2), 0, 0, math.sin(dyaw / 2)]), q)
                vel_b = quat_to_rot(q).T @ np.array([msg.vn, msg.ve, msg.vd]) + rng.normal(0, args.noise_vel, 3)
                ned_meas = ned + rng.normal(0, args.noise_m, 3)
                if PHASES["step_on"] <= el < PHASES["step_off"]:
                    ned_meas = ned_meas + np.array([args.step_m, 0.0, 0.0])
                delay.push(now, (ned_meas, q, vel_b))
            elif t == "LOCAL_POSITION_NED":
                ekf_log.append((now, np.array([msg.x, msg.y, msg.z])))
            elif t == "EKF_STATUS_REPORT":
                ekf_status.append((now, msg.flags, msg.pos_horiz_variance))
            elif t == "STATUSTEXT":
                texts.append((el, msg.text))
                print("  FC: %5.1f s %s" % (el, msg.text), flush=True)
            elif t == "HEARTBEAT" and msg.get_srcSystem() == fc.ts and msg.get_srcComponent() == fc.tc:
                if msg.base_mode & fc.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise SystemExit("FC armed during the run: stopping")
            stage_ms["handle_rx"].append((time.perf_counter() - a) * 1e3)
    finally:
        set_params(original)
        restored = True

    # ---------------------------------------------------------------- metrics
    tt = np.array([x[0] - t0 for x in truth_log])
    truth = np.array([x[1] for x in truth_log])
    te = np.array([x[0] - t0 for x in ekf_log])
    ekf = np.array([x[1] for x in ekf_log])
    # truth at each EKF sample: last SIM_STATE received before it (vehicle is stationary)
    idx = np.clip(np.searchsorted(tt, te, side="right") - 1, 0, len(tt) - 1)
    err = ekf - truth[idx]
    herr = np.linalg.norm(err[:, :2], axis=1)

    def window(t, a, b):
        return (t >= a) & (t < b)

    base = window(te, 2.0, PHASES["switch"])
    steady = window(te, PHASES["steady"], PHASES["step_on"])
    ts_st = np.array([x[0] - t0 for x in ekf_status])
    ratio = np.array([x[2] for x in ekf_status])
    st_steady = window(ts_st, PHASES["steady"], PHASES["step_on"])

    x_before = float(np.median(err[window(te, PHASES["step_on"] - 5, PHASES["step_on"]), 0]))
    # step starts in the echo when the first shifted truth sample is *sent*
    sent_t = np.array([s[0] - t0 for s in sent_log])
    sent_truth_t = np.array([s[1] - t0 for s in sent_log])
    first_on = sent_t[np.nonzero(sent_truth_t >= PHASES["step_on"])[0][0]]
    first_off = sent_t[np.nonzero(sent_truth_t >= PHASES["step_off"])[0][0]]
    on = step_response(te, err[:, 0], first_on, args.step_m, x_before)
    x_top = float(np.median(err[window(te, PHASES["step_off"] - 3, PHASES["step_off"]), 0]))
    off = step_response(te, err[:, 0], first_off, -(x_top - x_before), x_top)
    added = (sent_t - sent_truth_t) * 1e3 - args.delay_ms

    ext_nav_text = [t for t in texts if "external nav" in t[1].lower() or "extnav" in t[1].lower()]
    result = {
        "tool": "tools/bench/truth_echo.py",
        "tag": args.tag,
        "date": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "connection": args.connection,
        "config": {"rate_hz": args.rate, "injected_delay_ms": args.delay_ms,
                   "VISO_DELAY_MS": args.viso_delay_ms, "noise_m": args.noise_m,
                   "noise_vel_mps": args.noise_vel, "noise_yaw_deg": args.noise_yaw_deg,
                   "step_m": args.step_m, "seed": args.seed, "phases_s": PHASES,
                   "original_params": original, "params_restored": restored},
        "environment": {"python": platform.python_version(), "platform": platform.platform()},
        "echo": {"sent": len(sent_log), "rate_hz": len(sent_log) / PHASES["end"],
                 "added_latency_ms_median": pct(added, 50), "added_latency_ms_p95": pct(added, 95)},
        "ekf_uses_ext_nav_text": [t[1] for t in ext_nav_text],
        "baseline_gps": {"h_err_m_median": pct(herr[base], 50), "h_err_m_p95": pct(herr[base], 95)},
        "steady_ext_nav": {
            "samples": int(steady.sum()),
            "h_err_m_median": pct(herr[steady], 50), "h_err_m_p95": pct(herr[steady], 95),
            "h_err_m_max": float(herr[steady].max()) if steady.any() else None,
            "z_err_m_median": pct(np.abs(err[steady, 2]), 50),
            "pos_test_ratio_median": pct(ratio[st_steady], 50),
            "pos_test_ratio_max": float(ratio[st_steady].max()) if st_steady.any() else None,
        },
        "step_on": dict(on, reached_m=x_top - x_before),
        "step_off": off,
        "timing_by_stage": {k: {"runs": len(v), "median_ms": pct(v, 50), "p95_ms": pct(v, 95)}
                            for k, v in stage_ms.items()},
        "fc_text": texts,
    }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / ("truth_echo_%s.json" % args.tag)).write_text(json.dumps(result, indent=2))
    with open(out / ("truth_echo_%s_log.csv" % args.tag), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ekf_n", "ekf_e", "ekf_d", "truth_n", "truth_e", "truth_d"])
        for k in range(len(te)):
            w.writerow(["%.4f" % te[k]] + ["%.4f" % v for v in ekf[k]] + ["%.4f" % v for v in truth[idx[k]]])
    return result


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("connection", help="COM6, /dev/ttyUSB0,921600, tcp:127.0.0.1:5760 ...")
    p.add_argument("--tag", default="run")
    p.add_argument("--rate", type=float, default=30.0, help="echo rate, Hz (SIM_STATE request)")
    p.add_argument("--delay-ms", type=float, default=50.0, help="injected vision delay")
    p.add_argument("--viso-delay-ms", type=float, default=None, help="VISO_DELAY_MS (default: = delay)")
    p.add_argument("--noise-m", type=float, default=0.02)
    p.add_argument("--noise-vel", type=float, default=0.02)
    p.add_argument("--noise-yaw-deg", type=float, default=0.5)
    p.add_argument("--step-m", type=float, default=0.3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default=os.path.join("results", "b4_nucleo"))
    args = p.parse_args(argv)
    if args.viso_delay_ms is None:
        args.viso_delay_ms = args.delay_ms
    r = run(args)
    print(json.dumps({k: r[k] for k in ("echo", "ekf_uses_ext_nav_text", "baseline_gps",
                                        "steady_ext_nav", "step_on", "step_off")}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
