"""PX4-specific parts of the FC link.

- SITL: the onboard/API MAVLink instance sends to UDP 14540 -> connect with 'udpin:0.0.0.0:14540'.
- Hardware: TELEM2 with MAV_1_CONFIG = TELEM 2, MAV_1_MODE = Onboard, SER_TEL2_BAUD = 921600.
  Onboard mode already streams ATTITUDE and LOCAL_POSITION_NED; we still request the rate.
"""

from __future__ import annotations

from pymavlink import mavutil

from fc_link._common import set_message_interval

NAME = "px4"
SITL_CONNECTION = "udpin:0.0.0.0:14540"


def request_streams(mav, target_system: int, target_component: int, hz: float) -> None:
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    ):
        set_message_interval(mav, target_system, target_component, msg_id, hz)
