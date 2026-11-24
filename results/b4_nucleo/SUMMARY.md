# B4 results: NUCLEO-H743ZI2, ArduPilot with on-board simulation

Firmware `NucleoH743-SimVIO`, Copter-4.5.7-3-gb72075518f. Link: USB CN13 (COM6) from the
Windows PC, Python 3.14. The KV260 / UART7 runs are still to do.

## Step 1: Phase 2 acceptance over USB (`step1_usb_acceptance.txt`)

| Date | ATTITUDE | TIMESYNC rtt | Note |
|---|---|---|---|
| 2026-11-23 17:59 | 32-40 Hz | 6.5-7.6 ms | before the E15 fix |
| 2026-11-23 22:22 | 50.5 Hz | 3.5-4.1 ms | after the E15 fix, 3/3 pass |

## Receive timing (`fc_rx_timing_nucleo_usb_windows.json`, `tools/bench/fc_rx_timing.py`)

20 s, after the E15 fix: ATTITUDE and LOCAL_POSITION_NED 50.5 Hz. FC spacing 20.0 ms median
(p95 21); arrival spacing on the PC 19.8 ms median (p95 21.5), no arrivals bunched (< 3 ms apart):
0 %. TIMESYNC rtt 2.0 ms.
Before the fix (same measurement, scratch script): arrival spacing 26.3 ms median, 25 % of
messages bunched, so a 50 Hz stream looked like 40 Hz to anything sampling the latest value.

## Truth echo (`truth_echo_*.json` / `_log.csv`, `tools/bench/truth_echo.py`)

The tool reads the simulated truth (SIM_STATE), adds noise (0.02 m position, 0.02 m/s velocity,
0.5 deg yaw) and a 50 ms delay, and sends it back as ODOMETRY at 30 Hz. EKF3 switches to external
nav 8 s into the run. The vehicle is stationary and disarmed (nothing armed, no mode change).

| Run | VISO_DELAY_MS | Baseline GPS h err (median) | Ext nav h err median / p95 / max | Test ratio max | Step on: onset / t63 / t90 | Step off: t63 |
|---|---|---|---|---|---|---|
| usb_d50_s1 | 50 | 5.1 mm | 7.8 / 14.7 / 16.1 mm | 0.021 | 375 / 1521 / 2929 ms | 1616 ms |
| usb_d50_s2 | 50 | 13.2 mm | 6.5 / 12.6 / 14.7 mm | 0.020 | 471 / 1553 / 2861 ms | 1682 ms |
| usb_d50_viso0 | 0 (wrong) | 10.5 mm | 6.6 / 11.8 / 13.0 mm | 0.022 | 506 / 1651 / 3088 ms | 1648 ms |

Echo: 30.6 Hz sent; the script adds 1.3-1.4 ms (median, p95 2.3 ms) on top of the injected delay.
Step: +0.30 m north added to the echo for 15 s; the EKF reaches 0.297-0.308 m.

Findings:
- EKF3 fuses the echo on the MCU: position error 7 mm median with 20 mm per-axis noise,
  innovation test ratio <= 0.022 (gate is 1). The first run after boot logs "EKF3 IMU0/IMU1 is
  using external nav data"; later switches in the same boot do not repeat the message.
- Fusion needs an external-nav source set at boot (E23); fixed with compiled-in `EK3_SRC2_*`.
- The EKF follows a vision step with t63 ~1.5-1.7 s: with `VISO_POS_M_NSE` 0.2 m and a
  stationary IMU it trusts the IMU more than a sudden vision jump. Input for the EKF analysis.
- A wrong `VISO_DELAY_MS` (0 vs 50) changes nothing measurable while stationary: delay errors only
  show when the vehicle moves (position error ~ speed x delay error). The delay check therefore
  needs motion, which on this board means arming the simulated vehicle; not done (needs the
  `--allow-flight` path, see `docs/bench.md` B4).
- Height error ~35-47 mm median: height comes from the simulated barometer (`EK3_SRC2_POSZ 1`).
