"""Receive timing of the FC link: message rates, ATTITUDE spacing (FC clock vs arrival), TIMESYNC.

    python tools/bench/fc_rx_timing.py COM6 --tag nucleo_usb_windows
    python tools/bench/fc_rx_timing.py /dev/ttyUSB0,921600 --tag nucleo_uart7_kv260

Read-only: uses fc_link.FcLink (HEARTBEAT, stream requests, TIMESYNC). Shows whether messages
arrive evenly (FC sends every 20 ms at 50 Hz) or bunched by the receive path, which adds latency
(docs/errors.md E15). Writes results/b4_nucleo/fc_rx_timing_<tag>.json.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from fc_link.link import FcLink  # noqa: E402


def stats(a) -> dict:
    a = np.asarray(a, dtype=float)
    if not len(a):
        return {}
    return {"n": int(len(a)), "median": float(np.median(a)), "p95": float(np.percentile(a, 95)),
            "max": float(a.max())}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("connection")
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--tag", default="run")
    p.add_argument("--out", default=os.path.join("results", "b4_nucleo"))
    args = p.parse_args(argv)

    link = FcLink(args.connection)
    fw = link.connect(timeout_s=30)
    arrivals = []
    orig = link._handle

    def handle(msg):  # record ATTITUDE arrival (PC monotonic) and FC timestamp
        if msg.get_type() == "ATTITUDE" and link._from_target(msg):
            arrivals.append((time.monotonic(), msg.time_boot_ms))
        orig(msg)

    link._handle = handle
    link.start()
    time.sleep(2.0)  # let the requested rates take effect
    n0 = {k: link.rx_count(k) for k in ("ATTITUDE", "LOCAL_POSITION_NED", "HEARTBEAT")}
    k0 = len(arrivals)
    time.sleep(args.seconds)
    rates = {k: (link.rx_count(k) - v) / args.seconds for k, v in n0.items()}
    rtt = link.timesync.rtt_ns
    link.close()

    a = np.array(arrivals[k0:], dtype=float)
    rx_ms = np.diff(a[:, 0]) * 1e3
    fc_ms = np.diff(a[:, 1])
    result = {
        "tool": "tools/bench/fc_rx_timing.py", "tag": args.tag, "firmware": fw,
        "date": time.strftime("%Y-%m-%dT%H:%M:%S"), "connection": args.connection,
        "seconds": args.seconds,
        "environment": {"python": platform.python_version(), "platform": platform.platform()},
        "rate_hz": rates,
        "attitude_fc_spacing_ms": stats(fc_ms),
        "attitude_arrival_spacing_ms": stats(rx_ms),
        "attitude_arrivals_under_3ms_pct": float(100 * np.mean(rx_ms < 3)) if len(rx_ms) else None,
        "timesync_rtt_ms_min_of_window": rtt / 1e6 if rtt is not None else None,
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / ("fc_rx_timing_%s.json" % args.tag)).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
