"""Monitor the FC link: python -m fc_link [--connection CONN] [--seconds N]

Read-only: prints firmware, attitude, local position and TIMESYNC offset once per second.
Connection default comes from FC_CONNECTION in .env (or the environment).
"""

from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

from fc_link.link import FcLink


def load_dotenv(path: Path = Path(".env")) -> None:
    """Minimal .env reader (KEY=VALUE lines). Existing environment variables win."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if value.strip():
            os.environ.setdefault(key.strip(), value.strip())


def main() -> int:
    load_dotenv()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--connection", default=os.environ.get("FC_CONNECTION", "udpin:0.0.0.0:14540"))
    p.add_argument("--seconds", type=float, default=10.0)
    args = p.parse_args()

    link = FcLink(args.connection)
    fw = link.connect()
    link.start()
    print(f"connected: {fw} (sys {link.target_system}, comp {link.target_component})")
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(1.0)
            att, pos = link.attitude(), link.local_position()
            off, rtt = link.timesync.offset_ns, link.timesync.rtt_ns
            a = (
                f"rpy=({math.degrees(att.roll):6.1f},{math.degrees(att.pitch):6.1f},"
                f"{math.degrees(att.yaw):6.1f}) deg"
                if att else "attitude=none"
            )
            q = f"ned=({pos.x:7.2f},{pos.y:7.2f},{pos.z:7.2f}) m" if pos else "local_pos=none"
            t = f"offset={off / 1e6:.1f} ms rtt={rtt / 1e6:.2f} ms" if off is not None else "timesync=none"
            print(f"{a}  {q}  {t}  alive={link.fc_alive()}")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
