# Bench test ladder

Everything that can be proven before flight, in order. Each stage adds one real element and keeps
the rest known, so a failure points at the new element. A stage is done when its check passes and
the output is saved under `results/` (same rule as the phases in `CLAUDE.md`).

Key constraint for VIO: the camera and the IMU must see the **same motion**. Synthetic frames of a
moving drone plus a real IMU sitting still on the desk give the EKF two stories that disagree. Each
stage below states where the motion comes from.

| Stage | Flight controller | Motion source | Camera | IMU | Phase |
|---|---|---|---|---|---|
| B0 | fake FC (`tests/fake_fc.py`) | none | none | none | 2 |
| B1 | ArduPilot SITL + PX4 SITL (WSL) | SITL physics | none | simulated | 2 |
| B2 | none | recorded dataset (EuRoC MAV) | recorded | recorded | 7 (algorithm) |
| B3 | ArduPilot SITL, Simulink JSON physics | Simulink | Unreal, over Ethernet | simulated, consistent | 3, 6, 7 |
| B4 | NUCLEO-H743ZI2, simulation on the board | on-board simulation | none (truth echo) | simulated | 2, 6 |
| B5 | NUCLEO-H743ZI2, real sensors | hand / gimbal rig | real USB camera | real (rigid with camera) | 7 |
| B6 | NUCLEO-H743ZI2 + DroneCAN | as B5 | as B5 | as B5 | bench |
| B7 | Cube Orange (item under test) | Simulink HITL, then rig | as B3 / B5 | as B3 / B5 | all |

## B0 - fake FC (done)
`pytest`: 20 tests on Python 3.8, 3.10, 3.14.

## B1 - SITL, both firmwares (Phase 2 acceptance)
Setup and commands in `sim/SITL.md`. Needs a WSL **Ubuntu-22.04** distro: the existing `Ubuntu`
distro is 26.04, which the ArduPilot and PX4 setup scripts don't support.
Check: `pytest -m sitl -k ardupilot` and `-k px4` pass.

## B2 - VO on recorded data (no hardware)
Run the visual-odometry front end (sparse optical flow, OpenCV) on EuRoC MAV sequences, which give
camera + IMU + ground truth. First on the PC for correctness, then on the KV260 A53 for timing.
Datasets are downloaded to `data/` (not committed).
Check: trajectory error (ATE RMSE, drift as % of path length) and fps per sequence, from a
`tools/bench/` script, in `results/`.

## B3 - closed-loop VIO in SITL
Simulink owns the truth: UAV dynamics feed ArduPilot SITL over the JSON interface, and the
Unreal camera renders from the same pose and streams to the KV260 (Phase 3 ingest). The KV260
runs VO and sends `VISION_POSITION_ESTIMATE` / `ODOMETRY`; EKF3 fuses it with the simulated IMU
(`VISO_TYPE=1`, `EK3_SRC1_POSXY=6`, `EK3_SRC1_VELXY=6`, `EK3_SRC1_POSZ=1`). GPS off
(`SIM_GPS_DISABLE=1`) so the position can only come from vision. The KV260 sets the EKF origin
with `SET_GPS_GLOBAL_ORIGIN`.
Check: EKF accepts vision for a scripted trajectory; position error vs Simulink truth (median,
p95, final drift) and the measured delay (Phase 6) behind `VISO_DELAY_MS`, in `results/`.
Arming in SITL only; any code that arms must refuse real hardware without `--allow-flight`.

## B4 - Nucleo, simulation on the board, real UART
ArduPilot `NucleoH743` target (simulation runs on the H7, no sensors needed). Its default build
turns off external navigation (`EK3_FEATURE_EXTERNAL_NAV 0`, `HAL_VISUALODOM_ENABLED 0` in
`hwdef/include/SimOnHW.inc`); our build turns both back on.
Link: KV260 UART to a Nucleo UART at 921600, or the ST-LINK USB serial (USART3) from the PC.
Vision input is a **truth echo**: the KV260 reads the simulated pose (`SIMSTATE`), adds known
noise and delay, and sends it back as vision. That tests transport, `TIMESYNC` over real serial,
and EKF delay handling on a real MCU, with the error known exactly.
Check: B1 tests pass with a serial connection; EKF fuses the echo; the recovered delay matches the
injected one.

## B5 - Nucleo with real sensors (physical VIO rig)
Camera, IMU and Nucleo bolted to one rigid plate, moved by hand or on a gimbal. A new
`NucleoH743ZI2` hwdef derived from ArduPilot's `NucleoH753ZI` (same Nucleo-144 board):
MCU `STM32H743xx`, IMU on SPI1 with chip select on PA4, barometer on I2C1 (PB8/PB9).
Check PA7 (SPI1 MOSI) against the Ethernet solder bridges in UM2407 before wiring.
IMU: try the MPU6050 first (I2C, limited rate); if ArduPilot can't run it well, use an
ICM-42688-P SPI breakout. A global-shutter USB camera is preferred for VO.
Camera-IMU extrinsics measured on the plate; camera intrinsics from the Phase 4 calibration.
Check: closed loop of known length (e.g. a 2 m square, back to the start mark):
return-to-start error and drift %, in `results/`.

## B6 - DroneCAN on the Nucleo
CAN transceiver on CAN1 (PD0/PD1), bench DroneCAN GPS / airspeed on the bus. USB-CAN adapter
injects faults (node loss, bus errors, bad data).
Check: each injected fault produces the expected EKF/failsafe reaction, logged.
The NUCLEO-F446RE cannot run ArduPilot or AP_Periph (no STM32F446 support); it can later be a
custom libcanard node (fake sensor, misbehaving node) with its own transceiver.

## B7 - Cube Orange
Repeat B1, B3 (as PX4 or ArduPilot HITL from Simulink) and B5 on the Cube Orange, the item under
test. Propulsion stand props-off unless stated otherwise.

## Shopping list
- Needed for B5: a global-shutter USB (UVC) camera; ICM-42688-P SPI breakout if the MPU6050 fails.
- Optional: barometer breakout (BMP388, DPS310 or MS5611) for B5; a GPS only for outdoor
  comparison of GPS against VIO.
