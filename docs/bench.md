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

## B2 - VO on recorded data (no hardware) - PC part done 2026-11-21, KV260 timing open
Results and findings: `results/b2_vo_euroc/SUMMARY.md`.
Run the visual-odometry front end (sparse optical flow, OpenCV) on EuRoC MAV sequences, which give
camera + IMU + ground truth. First on the PC for correctness, then on the KV260 A53 for timing.
Datasets are downloaded to `data/` (not committed).
Check: trajectory error (ATE RMSE, drift as % of path length) and fps per sequence, from a
`tools/bench/` script, in `results/`.

Run: `python tools/bench/vo_euroc.py data/euroc/V1_01_easy` -> `results/b2_vo_euroc/`.
Code: `ingest/euroc.py` (reader), `perception/vo/` (tracker, keyframe mono VO, geometry).
Dataset: ETH Research Collection, doi 10.3929/ethz-b-000690084 (per-room zips, non-commercial
use); the original robotics.ethz.ch links are dead.
Monocular VO has no metric scale, so the tool scores what the camera measures (rotation,
direction of travel, rotation vs integrated gyro) and a "scale-oracle" trajectory that takes
step lengths from ground truth. Metric scale comes from the IMU / height in B3 and later.
OpenCV notes (found while testing, both affect the board's OpenCV 4.5):
- `findEssentialMat(..., RANSAC)` is not refined; `USAC_DEFAULT` is ~10x better on OpenCV 5.x
  but much weaker on 4.5. A Sampson-error LM refinement on the inliers (`refine=True`) brings
  4.5 close to 5.x (synthetic, 0.3 px noise: rotation 0.13 deg / direction 2.3 deg median).
- `recoverPose` short form drops points beyond 50 baselines (6 m at 12 cm) in its cheirality
  check; use the `distanceThresh` overload, and pass it by keyword (positionally the Python
  binding silently picks the short form).

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

## B4 - Nucleo, simulation on the board, real UART - USB link + stationary truth echo pass 2026-11-23; UART7 and delay check open
Board definition, build script and flashing notes: `sim/nucleo/`. Results: `results/b4_nucleo/SUMMARY.md`.
Step 1 done (2026-11-23): flashed on the ZI2, Phase 2 acceptance passes over USB CN13 (3/3,
ATTITUDE 50.5 Hz, TIMESYNC rtt 3.5-4 ms after fixing E15),
`results/b4_nucleo/step1_usb_acceptance.txt`.
Step 2, stationary truth echo over USB, done (2026-11-23): `tools/bench/truth_echo.py`; EKF3 fuses
the echo (7 mm median error with 20 mm noise), step response t63 ~1.5 s. Needed a vision
source set at boot (E23).
Open: the same over UART7 from the KV260; the delay check, which needs the simulated vehicle to
move, i.e. arming it (asks first: `--allow-flight` path, even though the Nucleo drives no motors).
Needed: simulated GPS (`GPS_TYPE 100`) with a placeholder GPS port, and compiled-in defaults
because the board stores no parameters (details in `sim/nucleo/README.md`).
ArduPilot `NucleoH743` target (simulation runs on the H7, no sensors needed). Its default build
turns off external navigation (`EK3_FEATURE_EXTERNAL_NAV 0`, `HAL_VISUALODOM_ENABLED 0` in
`hwdef/include/SimOnHW.inc`); our build turns both back on.
Link: KV260 UART to Nucleo UART7 (SERIAL3) at 921600; USB CN13 (SERIAL0) from the PC.
Vision input is a **truth echo**: the KV260 reads the simulated pose (`SIM_STATE`), adds known
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
