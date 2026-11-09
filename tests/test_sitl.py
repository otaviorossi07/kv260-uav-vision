"""Phase 2 acceptance: fc_link against real SITL. Run with a SITL already up:

    pytest -m sitl -k ardupilot     # sim_vehicle.py -v ArduCopter --no-mavproxy
    pytest -m sitl -k px4           # make px4_sitl sihsim_quadx

Override connections with SITL_ARDUPILOT / SITL_PX4 environment variables.
"""

from __future__ import annotations

import os
import time

import pytest

from fc_link import ardupilot, px4
from fc_link.link import FcLink

pytestmark = pytest.mark.sitl


def wait_for(cond, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.1)
    return False


def run_acceptance(connection: str, expected_fw: str) -> None:
    link = FcLink(connection)
    try:
        assert link.connect(timeout_s=60.0) == expected_fw
        link.start()
        assert wait_for(lambda: link.attitude() is not None, 10.0), "no ATTITUDE"
        # count ATTITUDE updates over 2 s to check the requested rate is roughly honoured
        seen = set()
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            seen.add(link.attitude().time_boot_ms)
            time.sleep(0.005)
        assert len(seen) >= 20, f"ATTITUDE rate too low: {len(seen) / 2:.0f} Hz"
        assert wait_for(lambda: link.timesync.offset_ns is not None, 10.0), "no TIMESYNC reply"
        # EKF needs an origin before LOCAL_POSITION_NED appears
        assert wait_for(lambda: link.local_position() is not None, 120.0), "no LOCAL_POSITION_NED"
        assert link.fc_alive()
        print(
            f"\n{expected_fw}: ATTITUDE ~{len(seen) / 2:.0f} Hz, "
            f"timesync rtt {link.timesync.rtt_ns / 1e6:.2f} ms"
        )
    finally:
        link.close()


def test_ardupilot_sitl():
    run_acceptance(os.environ.get("SITL_ARDUPILOT", ardupilot.SITL_CONNECTION), ardupilot.NAME)


def test_px4_sitl():
    run_acceptance(os.environ.get("SITL_PX4", px4.SITL_CONNECTION), px4.NAME)
