"""Firmware-agnostic MAVLink link to the flight controller (PX4 or ArduPilot).

What it does (phase 2):
- sends HEARTBEAT as an onboard computer (1 Hz);
- reads ATTITUDE and LOCAL_POSITION_NED and keeps the latest of each;
- runs TIMESYNC both ways: answers FC requests and estimates the FC-to-board clock offset.

What it never does: arm, take off, change flight mode, or send setpoints.
Code that can command flight needs an explicit --allow-flight flag (see CLAUDE.md).

All MAVLink I/O happens in one background thread; getters are thread-safe.
"""

from __future__ import annotations

import collections
import threading
import time
from dataclasses import dataclass
from typing import Callable, Deque, Optional, Tuple

from pymavlink import mavutil

from fc_link import ardupilot, px4

MAV_COMP_ID_ONBOARD_COMPUTER = 191


@dataclass(frozen=True)
class Attitude:
    time_boot_ms: int  # FC clock
    roll: float  # rad
    pitch: float
    yaw: float
    rollspeed: float  # rad/s
    pitchspeed: float
    yawspeed: float
    rx_monotonic_ns: int  # board clock at receipt


@dataclass(frozen=True)
class LocalPositionNed:
    time_boot_ms: int  # FC clock
    x: float  # m, NED
    y: float
    z: float
    vx: float  # m/s
    vy: float
    vz: float
    rx_monotonic_ns: int  # board clock at receipt


def parse_connection(connection: str) -> Tuple[str, Optional[int]]:
    """'/dev/ttyPS1,921600' -> ('/dev/ttyPS1', 921600); 'udpin:0.0.0.0:14540' -> (..., None)."""
    if "," in connection:
        device, baud = connection.rsplit(",", 1)
        return device, int(baud)
    return connection, None


class TimeSync:
    """FC-to-board clock offset from TIMESYNC round trips.

    offset_ns = fc_time_ns - board_monotonic_ns. Uses the sample with the smallest round-trip
    time in a sliding window, since that one has the least queueing delay.
    """

    def __init__(self, window: int = 20, max_rtt_ns: int = 50_000_000) -> None:
        self._samples: Deque[Tuple[int, int]] = collections.deque(maxlen=window)  # (rtt, offset)
        self._max_rtt_ns = max_rtt_ns

    def add_sample(self, ts1_ns: int, tc1_ns: int, now_ns: int) -> bool:
        rtt = now_ns - ts1_ns
        if rtt < 0 or rtt > self._max_rtt_ns:
            return False
        offset = tc1_ns - (ts1_ns + now_ns) // 2
        self._samples.append((rtt, offset))
        return True

    @property
    def offset_ns(self) -> Optional[int]:
        if not self._samples:
            return None
        return min(self._samples)[1]

    @property
    def rtt_ns(self) -> Optional[int]:
        if not self._samples:
            return None
        return min(self._samples)[0]


class FcLink:
    def __init__(
        self,
        connection: str,
        source_system: int = 1,
        source_component: int = MAV_COMP_ID_ONBOARD_COMPUTER,
        stream_hz: float = 50.0,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.connection = connection
        self.source_system = source_system
        self.source_component = source_component
        self.stream_hz = stream_hz
        self._clock = clock
        self._mav = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._attitude: Optional[Attitude] = None
        self._local_pos: Optional[LocalPositionNed] = None
        self._last_fc_heartbeat_ns: Optional[int] = None
        self.timesync = TimeSync()
        self.firmware: Optional[str] = None  # "px4" | "ardupilot"
        self.target_system: Optional[int] = None
        self.target_component: Optional[int] = None

    # ------------------------------------------------------------------ lifecycle
    def connect(self, timeout_s: float = 30.0) -> str:
        """Open the link, wait for the FC heartbeat, detect firmware, request streams.

        Returns the detected firmware name. Raises TimeoutError if no autopilot heartbeat.
        """
        device, baud = parse_connection(self.connection)
        kwargs = dict(
            source_system=self.source_system,
            source_component=self.source_component,
            autoreconnect=True,
        )
        if baud is not None:
            kwargs["baud"] = baud
        self._mav = mavutil.mavlink_connection(device, **kwargs)

        # udpin needs to receive before it can send, so listen first and send heartbeats while waiting.
        deadline = time.monotonic() + timeout_s
        next_hb = 0.0
        hb = None
        while time.monotonic() < deadline:
            if time.monotonic() >= next_hb:
                self._send_heartbeat()
                next_hb = time.monotonic() + 1.0
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
            if msg is not None and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                hb = msg
                break
        if hb is None:
            self._mav.close()
            self._mav = None
            raise TimeoutError(f"no autopilot HEARTBEAT on {self.connection} in {timeout_s} s")

        self.target_system = hb.get_srcSystem()
        self.target_component = hb.get_srcComponent()
        self._last_fc_heartbeat_ns = self._clock()
        if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self.firmware = px4.NAME
            px4.request_streams(self._mav, self.target_system, self.target_component, self.stream_hz)
        elif hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.firmware = ardupilot.NAME
            ardupilot.request_streams(
                self._mav, self.target_system, self.target_component, self.stream_hz
            )
        else:
            self._mav.close()
            self._mav = None
            raise RuntimeError(f"unsupported autopilot type {hb.autopilot}")
        return self.firmware

    def start(self) -> None:
        if self._mav is None:
            raise RuntimeError("call connect() first")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fc_link", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._mav is not None:
            self._mav.close()
            self._mav = None

    def __enter__(self) -> "FcLink":
        self.connect()
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ getters
    def attitude(self) -> Optional[Attitude]:
        with self._lock:
            return self._attitude

    def local_position(self) -> Optional[LocalPositionNed]:
        with self._lock:
            return self._local_pos

    def fc_alive(self, max_age_s: float = 3.0) -> bool:
        with self._lock:
            last = self._last_fc_heartbeat_ns
        return last is not None and (self._clock() - last) < max_age_s * 1e9

    def fc_time_to_board_ns(self, fc_time_ns: int) -> Optional[int]:
        """Convert an FC timestamp (ns since FC boot) to board monotonic ns. None until synced."""
        offset = self.timesync.offset_ns
        return None if offset is None else fc_time_ns - offset

    # ------------------------------------------------------------------ worker
    def _run(self) -> None:
        next_hb = 0.0
        next_ts = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_hb:
                self._send_heartbeat()
                next_hb = now + 1.0
            if now >= next_ts:
                # TIMESYNC request: tc1 = 0, ts1 = our time
                self._mav.mav.timesync_send(0, self._clock())
                next_ts = now + 1.0
            msg = self._mav.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self._handle(msg)

    def _send_heartbeat(self) -> None:
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _from_target(self, msg) -> bool:
        # Only the autopilot component we connected to (ignores gimbals, cameras, GCS, ourselves).
        return (
            msg.get_srcSystem() == self.target_system
            and msg.get_srcComponent() == self.target_component
        )

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA" or not self._from_target(msg):
            return
        now = self._clock()
        if mtype == "ATTITUDE":
            att = Attitude(
                msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw,
                msg.rollspeed, msg.pitchspeed, msg.yawspeed, now,
            )
            with self._lock:
                self._attitude = att
        elif mtype == "LOCAL_POSITION_NED":
            pos = LocalPositionNed(
                msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, now
            )
            with self._lock:
                self._local_pos = pos
        elif mtype == "HEARTBEAT":
            with self._lock:
                self._last_fc_heartbeat_ns = now
        elif mtype == "TIMESYNC":
            if msg.tc1 == 0:
                # FC asks us: answer with our time, echo its ts1
                self._mav.mav.timesync_send(now, msg.ts1)
            else:
                # answer to our request
                self.timesync.add_sample(msg.ts1, msg.tc1, now)
