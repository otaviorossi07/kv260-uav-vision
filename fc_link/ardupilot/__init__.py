"""ArduPilot-specific parts of the FC link.

- SITL without MAVProxy: 'sim_vehicle.py -v ArduCopter --no-mavproxy' listens on TCP 5760
  -> connect with 'tcp:127.0.0.1:5760'.
- Hardware: TELEM2 = SERIAL2 with SERIAL2_PROTOCOL = 2 (MAVLink2), SERIAL2_BAUD = 921.
  SET_MESSAGE_INTERVAL works on ArduPilot 4.0+; SR2_* params are the fallback.
- LOCAL_POSITION_NED only appears once the EKF has an origin (GPS lock in SITL: ~20-40 s).
"""

from __future__ import annotations

from pymavlink import mavutil

from fc_link._common import set_message_interval

NAME = "ardupilot"
SITL_CONNECTION = "tcp:127.0.0.1:5760"


def request_streams(mav, target_system: int, target_component: int, hz: float) -> None:
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    ):
        set_message_interval(mav, target_system, target_component, msg_id, hz)
