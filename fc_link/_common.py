"""Helpers shared by the firmware modules. Read-only requests; nothing here commands flight."""

from __future__ import annotations

from pymavlink import mavutil


def set_message_interval(mav, target_system: int, target_component: int, msg_id: int, hz: float) -> None:
    interval_us = int(1e6 / hz) if hz > 0 else -1
    mav.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0, 0, 0, 0, 0,
    )
