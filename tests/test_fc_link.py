"""Unit tests for fc_link against a fake FC on localhost UDP."""

from __future__ import annotations

import math
import time

import pytest
from pymavlink import mavutil

from fc_link.link import FcLink, TimeSync, parse_connection
from tests.fake_fc import FakeFc, free_udp_port


def wait_for(cond, timeout_s: float = 5.0) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture(params=["px4", "ardupilot"])
def link_and_fc(request):
    autopilot = {
        "px4": mavutil.mavlink.MAV_AUTOPILOT_PX4,
        "ardupilot": mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    }[request.param]
    port = free_udp_port()
    link = FcLink(f"udpin:127.0.0.1:{port}")
    fc = FakeFc(port, autopilot=autopilot).start()
    assert link.connect(timeout_s=5.0) == request.param
    link.start()
    yield link, fc
    link.close()
    fc.stop()


def test_parse_connection():
    assert parse_connection("/dev/ttyPS1,921600") == ("/dev/ttyPS1", 921600)
    assert parse_connection("udpin:0.0.0.0:14540") == ("udpin:0.0.0.0:14540", None)
    assert parse_connection("tcp:127.0.0.1:5760") == ("tcp:127.0.0.1:5760", None)


def test_timesync_offset_and_rtt_filter():
    ts = TimeSync()
    # true offset 1000 ns; one noisy sample with large RTT, one clean
    assert ts.add_sample(ts1_ns=0, tc1_ns=5_000_000 + 1000 + 3_000_000, now_ns=10_000_000)
    assert ts.add_sample(ts1_ns=100_000_000, tc1_ns=100_000_500 + 1000, now_ns=100_001_000)
    assert ts.offset_ns == 1000
    assert ts.rtt_ns == 1000
    assert not ts.add_sample(ts1_ns=0, tc1_ns=0, now_ns=1_000_000_000)  # RTT too large
    assert not ts.add_sample(ts1_ns=10, tc1_ns=0, now_ns=5)  # negative RTT


def test_connect_timeout_without_fc():
    link = FcLink(f"udpin:127.0.0.1:{free_udp_port()}")
    with pytest.raises(TimeoutError):
        link.connect(timeout_s=1.0)


def test_reads_attitude_and_local_position(link_and_fc):
    link, _ = link_and_fc
    assert wait_for(lambda: link.attitude() is not None and link.local_position() is not None)
    att, pos = link.attitude(), link.local_position()
    assert math.isclose(att.roll, 0.1, abs_tol=1e-6)
    assert math.isclose(att.pitch, -0.2, abs_tol=1e-6)
    assert math.isclose(att.yaw, 1.5, abs_tol=1e-6)
    assert (pos.x, pos.y, pos.z) == pytest.approx((1.0, 2.0, -3.0))
    assert (pos.vx, pos.vy, pos.vz) == pytest.approx((0.1, 0.2, 0.3))
    assert link.fc_alive()


def test_sends_heartbeat_and_requests_streams(link_and_fc):
    link, fc = link_and_fc
    assert wait_for(lambda: fc.heartbeats_from_board >= 1)
    assert wait_for(lambda: len(fc.commands) >= 2)
    assert set(fc.commands) == {mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL}


def test_timesync_estimates_offset(link_and_fc):
    link, fc = link_and_fc
    assert wait_for(lambda: link.timesync.offset_ns is not None, timeout_s=3.0)
    error_ms = abs(link.timesync.offset_ns - fc.clock_offset_ns) / 1e6
    assert error_ms < 5.0
    fc_now = fc.now_ns()
    board = link.fc_time_to_board_ns(fc_now)
    assert abs(board - time.monotonic_ns()) < 10_000_000


def test_answers_fc_timesync_requests(link_and_fc):
    _, fc = link_and_fc
    assert wait_for(lambda: fc.timesync_replies_from_board >= 1, timeout_s=3.0)


def test_never_commands_flight(link_and_fc):
    """Phase 2 link is read-only: only SET_MESSAGE_INTERVAL, no arm/mode/setpoints."""
    _, fc = link_and_fc
    time.sleep(1.5)
    assert fc.forbidden == []
    assert all(c == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for c in fc.commands)
