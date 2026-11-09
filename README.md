# KV260 UAV Vision

Embedded vision for UAVs on an **AMD Kria KV260** companion computer, talking MAVLink to a
**PX4 or ArduPilot** flight controller (Cube Orange). Development follows a bench-first path:
every feature is closed in the loop against SITL/HITL before it ever flies.

> **Status:** Phase 1 done · Phase 2 (flight-controller link) code and unit tests done, SITL
> acceptance pending. See [Roadmap](#roadmap).

---

## Contents

- [System overview](#system-overview)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Flight-controller link (`fc_link`)](#flight-controller-link-fc_link)
- [Testing](#testing)
- [Host tools container (Vitis AI 3.5)](#host-tools-container-vitis-ai-35)
- [Roadmap](#roadmap)
- [Pinned stack](#pinned-stack)
- [Safety](#safety)
- [Further reading](#further-reading)
- [License](#license)

## System overview

```
 Simulation PC (MATLAB/Simulink, UAV Toolbox, QGC / Mission Planner)
   │  synthetic camera frames (GigE, RTP/UDP or ROS 2)
   ▼
 KV260 companion computer ──── MAVLink (TELEM2, 921600) ────► Cube Orange (PX4 / ArduPilot)
   ingest → perception (ArUco, DPU detector) → fc_link            ▲
                                                                  │ HITL sensors / actuators
                                                     Simulink UAV dynamics, DroneCAN bench
```

| Element | Role |
|---|---|
| Cube Orange (PX4 or ArduPilot) | Flight controller, the item under test |
| KV260 (K26 SOM, B4096 DPU) | Companion computer: frame ingest, perception, MAVLink |
| Simulation PC | UAV dynamics, synthetic 3D camera, ground control |
| Bench | DroneCAN bus (GPS, airspeed, CAN ESCs), USB-CAN for fault injection, propulsion stand |

## Repository layout

```
kv260-uav-vision/
├── fc_link/          # MAVLink bridge: common API (link.py) + px4/ ardupilot/ specifics
├── ingest/           # frame sources: ethernet (sim), usb, mipi          (phase 3)
├── perception/
│   ├── aruco/        # marker detection + LANDING_TARGET                (phase 4)
│   └── dpu/          # xmodels, VART runner, postprocess                (phase 5)
├── models/           # float + quantized models, arch.json, compile scripts
├── sim/              # SITL setup (SITL.md), Simulink camera stream config
├── tools/bench/      # latency and fps measurement scripts
├── tests/            # pytest; SITL integration tests marked @sitl
├── results/          # benchmark outputs (committed as evidence)
├── Dockerfile        # Vitis AI 3.5 host tools
└── CLAUDE.md         # working rules and phase plan for Claude Code
```

## Quick start

Requires Python 3.8+ (3.8 is what the Vitis AI 3.5 container ships; the KV260 runs 3.10).

```bash
git clone https://github.com/otaviorossi07/kv260-uav-vision.git
cd kv260-uav-vision
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pytest                             # unit tests, no hardware or SITL needed
```

Configuration lives in `.env` (never committed). Copy the template and fill it in:

```bash
cp .env.example .env
```

| Variable | Example | Meaning |
|---|---|---|
| `FC_CONNECTION` | `/dev/ttyPS1,921600` | Board UART to TELEM2 |
| | `tcp:127.0.0.1:5760` | ArduPilot SITL (`--no-mavproxy`) |
| | `udpin:0.0.0.0:14540` | PX4 SITL onboard link |
| `KV260_HOST`, `KV260_USER` | | Board access for deploy scripts |
| `SIM_PC_HOST` | | Simulation PC streaming frames |

## Flight-controller link (`fc_link`)

`fc_link.link.FcLink` is a firmware-agnostic, **read-only** MAVLink link built on
[pymavlink](https://github.com/ArduPilot/pymavlink).

- Sends `HEARTBEAT` at 1 Hz as `MAV_TYPE_ONBOARD_CONTROLLER` (component 191).
- Detects PX4 or ArduPilot from the autopilot `HEARTBEAT`; firmware-specific defaults and
  stream requests live in `fc_link/px4/` and `fc_link/ardupilot/`.
- Requests `ATTITUDE` and `LOCAL_POSITION_NED` at 50 Hz (`MAV_CMD_SET_MESSAGE_INTERVAL`) and keeps
  the latest of each, stamped with the board's monotonic clock at receipt.
- `TIMESYNC` both ways: answers FC requests and estimates the FC-to-board clock offset from the
  minimum-RTT sample in a sliding window. `fc_time_to_board_ns()` converts FC timestamps, which
  phase 6 uses for latency measurement.
- Accepts messages only from the autopilot component it connected to.
- All I/O runs in one background thread; getters are thread-safe.

```python
from fc_link.link import FcLink

with FcLink("udpin:0.0.0.0:14540") as link:     # connect() + start()
    print(link.firmware)                         # "px4" or "ardupilot"
    att = link.attitude()                        # Attitude | None
    pos = link.local_position()                  # LocalPositionNed | None
    offset = link.timesync.offset_ns             # FC clock - board clock, ns
```

Command-line monitor:

```bash
python -m fc_link --connection tcp:127.0.0.1:5760 --seconds 20
```

### Flight-controller setup

| | PX4 | ArduPilot |
|---|---|---|
| Port | `MAV_1_CONFIG` = TELEM 2 | `SERIAL2_PROTOCOL` = 2 (MAVLink2) |
| Mode | `MAV_1_MODE` = Onboard | — |
| Baud | `SER_TEL2_BAUD` = 921600 | `SERIAL2_BAUD` = 921 |
| SITL connection | `udpin:0.0.0.0:14540` | `tcp:127.0.0.1:5760` |

## Testing

| Command | What it runs |
|---|---|
| `pytest` | Unit tests against a fake flight controller over localhost UDP (`tests/fake_fc.py`) |
| `pytest -m sitl -k ardupilot -s` | Phase 2 acceptance against a running ArduPilot SITL |
| `pytest -m sitl -k px4 -s` | Phase 2 acceptance against a running PX4 SITL |

SITL tests are excluded by default. Building and running both SITLs in WSL (Ubuntu 22.04) is
described step by step in [sim/SITL.md](sim/SITL.md).

CI (GitHub Actions) runs the unit tests on Python 3.8 and 3.10, the two versions the code must
support.

## Host tools container (Vitis AI 3.5)

Quantization and compilation for the DPU run in the official Vitis AI 3.5 CPU image
(`xilinx/vitis-ai-pytorch-cpu:ubuntu2004-3.5.0.306`, Python 3.8).

```bash
docker build -t kv260-uav-vision-host .
docker run --rm -it -v "$PWD:/workspace" kv260-uav-vision-host     # Windows cmd: "%cd%:/workspace"
# inside:
conda activate vitis-ai-pytorch
pytest -q
```

## Roadmap

A phase is done only when its acceptance check passes.

| # | Phase | Acceptance | Status |
|---|---|---|---|
| 1 | Scaffold and environment | `pytest` runs; container starts | ✅ Done |
| 2 | Flight-controller link | Integration test passes on ArduPilot SITL and PX4 SITL | 🟡 Code + unit tests done; SITL pending |
| 3 | Frame ingest | 640×480 at ≥ 30 fps for 5 min, frame drops logged | ⬜ |
| 4 | ArUco precision landing (`LANDING_TARGET`) | Unit tests on synthetic images; SITL landing within 0.2 m | ⬜ |
| 5 | DNN on the DPU (YOLOX-nano / Model Zoo YOLO) | fps and mAP on a held-out set in `results/` | ⬜ |
| 6 | Latency instrumentation | Median and p95 capture → FC receipt; `EKF2_EV_DELAY` / `VISO_DELAY_MS` notes | ⬜ |
| 7 | Visual odometry (`ODOMETRY`) | Sparse optical flow on A53; PL only if phase 6 shows a CPU bottleneck | ⬜ |

## Pinned stack

Vitis AI 3.5 · Kria 2023.2 platforms · B4096 DPU (`DPUCZDX8G`) · Ubuntu 22.04 on the KV260 ·
ROS 2 Humble. These are not upgraded without an explicit decision.

## Safety

- No code arms, takes off or changes flight mode on real hardware without an explicit
  `--allow-flight` flag. The current code sends no such commands at all, and a unit test
  enforces it.
- Default tests target SITL or a fake FC only.
- Propulsion-stand tests run props-off unless stated otherwise.
- No secrets or board addresses in the repository: they come from `.env`.

## Further reading

[docs/resources.md](docs/resources.md) lists the open-source projects, articles and papers this
work builds on (Kria platforms, Vitis AI, DPU + ROS 2 examples, PX4/ArduPilot precision landing,
FPGA visual odometry).

## License

[MIT](LICENSE)
