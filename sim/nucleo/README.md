# NUCLEO-H743ZI2 bench flight controller (stage B4)

ArduCopter with the simulation running on the H7 (no sensors needed) and external navigation /
visual odometry re-enabled, so EKF3 fuses vision messages from the KV260. See `docs/bench.md` B4.

## Build (WSL Ubuntu 22.04, ArduPilot Copter-4.5 in ~/ardupilot)

```
bash sim/nucleo/build_nucleo_sim.sh ~/ardupilot 8
```
Output: `~/ardupilot/build/NucleoH743-SimVIO/bin/arducopter_with_bl.hex` (bootloader + firmware).
Built 2026-11-21 from Copter-4.5.7-3-gb72075518f: 1.25 MB of 2 MB flash; symbol check shows
`AP_VisualOdom` (15 functions), `NavEKF3_core::writeExtNavData` and the on-board simulator
(`SITL::`). A copy for flashing from Windows is in `data/nucleo/` (not committed).

## Flash (Windows, STM32CubeProgrammer)

1. USB to the ST-LINK connector (CN1). Connect: ST-LINK, SWD, mode "Under reset".
2. Full chip erase, then program `arducopter_with_bl.hex` (addresses come from the hex),
   verify, disconnect, reset.
3. USB to the user connector (CN13): ArduPilot shows up as a COM port (MAVLink on SERIAL0).

To check tomorrow on the ZI2 (not verified yet):
- The base board assumes an 8 MHz HSE in bypass mode from the ST-LINK MCO, which is the ZI2
  default; if the board does not enumerate on CN13, check the clock solder bridges in UM2407.
- USB OTG on CN13 may need solder bridge settings per UM2407.

## Ports and parameters

- SERIAL0 = USB (CN13). SERIAL3 = UART7: PE8 TX, PF6 RX (3.3 V) on the ST morpho headers; this is
  the KV260 link (`/dev/ttyPS1,921600` on the board side).
- Parameters are **not saved** on this board (`HAL_USE_EMPTY_STORAGE` in the base definition):
  they reset to defaults at every boot, so the bench script sets them over MAVLink each time:
  `SERIAL3_PROTOCOL 2`, `SERIAL3_BAUD 921`, `VISO_TYPE 1`, `EK3_SRC1_POSXY 6`,
  `EK3_SRC1_VELXY 6`, `EK3_SRC1_YAW 6` (vision), `VISO_DELAY_MS` from phase 6.
  Setting parameters does not arm or change mode.
