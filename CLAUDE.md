# KV260 UAV Vision

Embedded vision on an AMD Kria KV260 companion computer for a UAV. The loop is closed
in the SITL/HITL bench before any flight.

## System context

- **Flight controller (item under test):** Cube Orange running PX4 or ArduPilot.
- **Simulation PC:** MATLAB/Simulink UAV dynamics, UAV Toolbox (PX4 HITL or ArduPilot JSON),
  QGroundControl / Mission Planner.
- **Companion computer:** KV260, linked to the Cube Orange on TELEM2
  (MAVLink, or uXRCE-DDS for PX4 + ROS 2).
- **Camera input in HITL:** synthetic frames from the Simulink/Unreal 3D camera over Gigabit
  Ethernet; real MIPI or USB3 camera later.
- **Other bench hardware:** DroneCAN bus (GPS, airspeed, CAN ESCs), USB-CAN adapter for fault
  injection, propulsion test stand.

Work one phase at a time; a phase is done only when its acceptance check passes.

## Rules

- **Simple first.** Prefer OpenCV/Python and existing libraries over custom HLS or C++ until a
  measured bottleneck justifies it.
- **Evidence over assumption.** Every performance claim (fps, latency, accuracy) comes from a
  script in `tools/bench/` with its output saved under `results/`.
- **Pinned stack.** Vitis AI 3.5, Kria 2023.2, B4096 DPU (`DPUCZDX8G`), Ubuntu 22.04,
  ROS 2 Humble. Do not upgrade any of these without asking.
  Python code must run on Python 3.8 (Vitis AI 3.5 host container) and 3.10 (Ubuntu 22.04 on
  the board): no `match`, no `X | Y` type hints unless `from __future__ import annotations`.
- **Firmware-agnostic.** Support PX4 and ArduPilot behind one interface; firmware-specific code
  lives in `fc_link/px4/` and `fc_link/ardupilot/`.
- **Safety.** Never write code that arms, takes off or switches flight mode on real hardware
  without an explicit `--allow-flight` flag. Default tests target SITL only. Bench tests with
  the propulsion stand run props-off unless the user says otherwise.
- **No secrets or board IPs in the repo**; read them from `.env` (template: `.env.example`).

## Repo layout

```
kv260-uav-vision/
├── CLAUDE.md
├── fc_link/          # MAVLink / uXRCE-DDS bridge (common API + px4/ ardupilot/)
├── ingest/           # frame sources: ethernet (sim), usb, mipi
├── perception/
│   ├── aruco/        # phase 4
│   └── dpu/          # phase 5: xmodels, VART runner, postprocess
├── models/           # float + quantized models, arch.json, compile scripts
├── sim/              # Simulink camera stream config, SITL launch scripts
├── tools/bench/      # latency and fps measurement
├── tests/            # pytest; SITL integration tests marked @sitl
└── results/          # benchmark outputs (committed)
```

## Phases

1. **Scaffold and environment.** Create the layout, a `pyproject.toml`, and a Dockerfile for the
   Vitis AI 3.5 host tools. Acceptance: `pytest` runs (empty) and the container starts.
2. **Flight-controller link.** `fc_link` sends `HEARTBEAT`, reads `ATTITUDE` and
   `LOCAL_POSITION_NED`, handles `TIMESYNC`, using pymavlink or MAVSDK over UART (`/dev/ttyPS1`
   or USB serial, 921600). Acceptance: integration test passes against ArduPilot SITL and PX4 SITL.
3. **Frame ingest.** Receive frames from the simulation PC over Ethernet (GStreamer RTP/UDP or a
   ROS 2 image topic) with a capture timestamp per frame. Acceptance: 640×480 at ≥ 30 fps
   received for 5 min with frame drops logged.
4. **ArUco precision landing.** Detect the marker, estimate pose with the calibrated camera,
   publish `LANDING_TARGET` (`MAV_FRAME_LOCAL_NED` for PX4; angles + distance for ArduPilot
   `PLND_TYPE=1`). Acceptance: unit tests on synthetic images; SITL landing within 0.2 m of the
   target. Note: PX4 precision landing needs a valid global position.
5. **DNN on the DPU.** Quantize a small detector (YOLOX-nano or a Model Zoo YOLO), compile for the
   B4096 `arch.json`, run with VART on the board. Acceptance: fps and mAP on a held-out set saved
   in `results/`.
6. **Latency instrumentation.** Measure capture → detection → MAVLink send → FC receipt.
   Acceptance: median and p95 reported; value fed to `EKF2_EV_DELAY` / `VISO_DELAY_MS` notes.
7. **Visual odometry (later).** Start with sparse optical flow on the A53; move pyramidal LK to
   the PL with Vitis Vision only if phase 6 shows the CPU is the bottleneck. Send `ODOMETRY`.

## When unsure

Stop and ask before: changing the pinned stack, touching the bitstream/platform, adding a
dependency that needs building from source on the board, or any code path that can command
flight.

## Status

- Phase 1: DONE (2026-11-08). `pytest` passes locally (Python 3.14 venv) and in the container
  (`xilinx/vitis-ai-pytorch-cpu:ubuntu2004-3.5.0.306`, Python 3.8.6, env `vitis-ai-pytorch`).
  Run the container from cmd with `docker run --rm -it -v "%cd%:/workspace" kv260-uav-vision-host`.
- Phase 2: code + unit tests done (`fc_link/link.py`, pymavlink; fake FC in `tests/fake_fc.py`).
  Unit tests pass on 3.14 (Windows) and 3.8 (container). Open: SITL acceptance
  (`pytest -m sitl`, both firmwares) — setup in `sim/SITL.md`, run in WSL Ubuntu-22.04.
  Note: the image does not include pymavlink yet; rebuild it (`docker build`) to pick it up.
