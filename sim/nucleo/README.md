# NUCLEO-H743ZI2 bench flight controller (stage B4)

ArduCopter with the simulation running on the H7 (no sensors needed) and external navigation /
visual odometry re-enabled, so EKF3 fuses vision messages from the KV260. See `docs/bench.md` B4.

## Build (WSL Ubuntu 22.04, ArduPilot Copter-4.5 in ~/ardupilot)

```
bash sim/nucleo/build_nucleo_sim.sh ~/ardupilot 8
```
Run it in a login shell from Windows (`wsl -d Ubuntu-22.04 bash -lc "..."`): the ARM toolchain
is only on the login PATH (docs/errors.md E24).
Output: `~/ardupilot/build/NucleoH743-SimVIO/bin/arducopter_with_bl.hex` (bootloader + firmware).
Built 2026-11-23 from Copter-4.5.7-3-gb72075518f (hex sha256 4bce5051...bc5c, with the
`EK3_SRC2_*` defaults): 1.26 MB of 2 MB flash; symbol check shows
`AP_VisualOdom`, `NavEKF3_core::writeExtNavData`, the on-board simulator (`SITL::`) and the
simulated GPS (`AP_GPS_SITL`). A copy for flashing from Windows is in `data/nucleo/` (not committed).

## Flash (Windows, STM32CubeProgrammer CLI; ships with STM32CubeIDE)

```
STM32_Programmer_CLI -c port=SWD mode=UR -e all -d arducopter_with_bl.hex -v -rst
```
USB to the ST-LINK connector (CN1). Mode must be "under reset" (`mode=UR`): once ArduPilot runs,
a hot-plug or normal SWD connection fails with "Unable to get core ID"; under reset always works.
Back up the old flash first if it matters: `-c port=SWD mode=UR -u 0x08000000 0x200000 backup.bin`.

Then a second USB cable to the user connector (CN13): ArduPilot enumerates as USB 1209:5741 (a new
COM port, MAVLink on SERIAL0). Verified on the ZI2 2026-11-23 with the board's default solder
bridges (8 MHz HSE from the ST-LINK MCO, USB OTG on CN13): no changes needed.

## Ports

| ArduPilot | Peripheral | Pins / connector | Use |
|---|---|---|---|
| SERIAL0 | USB OTG FS | CN13 | MAVLink to the PC |
| SERIAL1 | USART3 | PD8 TX, PD9 RX (ST-LINK virtual COM) | placeholder GPS port, see below |
| SERIAL3 | UART7 | PE8 TX, PF6 RX (3.3 V, morpho headers) | MAVLink2 921600 to the KV260 (`/dev/ttyPS1`) |

## Parameters

Parameters are **not saved** on this board (`HAL_USE_EMPTY_STORAGE` in the base definition), and
some only take effect after a reboot (serial protocols, GPS and visual-odometry backend), so they
are compiled in from `NucleoH743-SimVIO/defaults.parm` (ArduPilot puts it in ROMFS and loads it
at boot):

- `GPS_TYPE 100`: simulated GPS. It is only created if some serial port has the GPS protocol,
  hence `SERIAL1_PROTOCOL 5` on USART3; the driver reads and writes nothing on that port.
  Without it the GPS has no fix, the EKF has no origin and `LOCAL_POSITION_NED` never appears.
- `SERIAL3_PROTOCOL 2`, `SERIAL3_BAUD 921`: KV260 link.
- `VISO_TYPE 1`: MAVLink visual odometry.
- `EK3_SRC2_POSXY/VELXY/YAW 6`, `EK3_SRC2_POSZ 1`: source set 2 = external nav. EKF3 creates its
  external-nav buffers at boot only if a source set uses external nav; without this, switching to
  vision at runtime fuses nothing (docs/errors.md E23). Set 1 stays GPS.

Runtime settings (take effect without reboot) are set by `tools/bench/truth_echo.py` over MAVLink
and restored at the end: `EK3_SRC1_POSXY 6`, `EK3_SRC1_VELXY 6`, `EK3_SRC1_YAW 6` (vision),
`VISO_DELAY_MS`. Setting parameters does not arm or change mode.

## Results

`results/b4_nucleo/SUMMARY.md`: Phase 2 acceptance over USB passes (3/3, ATTITUDE 50.5 Hz,
TIMESYNC rtt 3.5-4 ms); stationary truth echo: EKF3 fuses vision, 7 mm median error.
