"""Minimal fake flight controller over UDP for unit tests (no SITL needed).

Uses a plain socket on an ephemeral port plus the MAVLink 2 encoder/parser, not mavutil 'udpout':
on Windows mavutil binds udpout to the destination port number, which loops the link's replies
back to the link itself when both ends run on one machine.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import List

from pymavlink.dialects.v20 import common as mavlink2


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _SocketWriter:
    def __init__(self, sock: socket.socket, addr) -> None:
        self.sock, self.addr = sock, addr

    def write(self, buf: bytes) -> None:
        try:
            self.sock.sendto(buf, self.addr)
        except OSError:
            pass  # peer closed (Windows reports ICMP port unreachable as an error)


FORBIDDEN = {
    "SET_MODE", "COMMAND_INT", "SET_POSITION_TARGET_LOCAL_NED", "SET_POSITION_TARGET_GLOBAL_INT",
    "SET_ATTITUDE_TARGET", "MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE",
}


class FakeFc:
    """Sends HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED; answers and sends TIMESYNC.

    Its clock is board monotonic + clock_offset_ns, so tests know the true offset.
    Records every COMMAND_LONG and any flight-commanding message it receives.
    """

    def __init__(
        self,
        port: int,
        autopilot: int = mavlink2.MAV_AUTOPILOT_PX4,
        clock_offset_ns: int = 5_000_000_000,
        rate_hz: float = 50.0,
    ) -> None:
        self.autopilot = autopilot
        self.clock_offset_ns = clock_offset_ns
        self.rate_hz = rate_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.005)
        self.mav = mavlink2.MAVLink(_SocketWriter(self.sock, ("127.0.0.1", port)), 1, 1)
        self.mav.robust_parsing = True
        self.commands: List[int] = []
        self.forbidden: List[str] = []
        self.timesync_replies_from_board = 0
        self.heartbeats_from_board = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def now_ns(self) -> int:
        return time.monotonic_ns() + self.clock_offset_ns

    def start(self) -> "FakeFc":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.sock.close()

    def _run(self) -> None:
        t0 = time.monotonic()
        next_hb = next_data = next_ts = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_hb:
                self.mav.heartbeat_send(
                    mavlink2.MAV_TYPE_QUADROTOR, self.autopilot, 0, 0, mavlink2.MAV_STATE_STANDBY
                )
                next_hb = now + 1.0
            if now >= next_data:
                boot_ms = int((now - t0) * 1000)
                self.mav.attitude_send(boot_ms, 0.1, -0.2, 1.5, 0.0, 0.0, 0.0)
                self.mav.local_position_ned_send(boot_ms, 1.0, 2.0, -3.0, 0.1, 0.2, 0.3)
                next_data = now + 1.0 / self.rate_hz
            if now >= next_ts:
                self.mav.timesync_send(0, self.now_ns())  # FC-side request
                next_ts = now + 1.0
            try:
                data = self.sock.recv(4096)
            except (socket.timeout, OSError):
                continue
            for msg in self.mav.parse_buffer(data) or []:
                self._handle(msg)

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "TIMESYNC":
            if msg.tc1 == 0:
                self.mav.timesync_send(self.now_ns(), msg.ts1)
            else:
                self.timesync_replies_from_board += 1
        elif mtype == "COMMAND_LONG":
            self.commands.append(msg.command)
        elif mtype == "HEARTBEAT":
            self.heartbeats_from_board += 1
        elif mtype in FORBIDDEN:
            self.forbidden.append(mtype)
