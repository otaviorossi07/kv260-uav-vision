#!/bin/bash
# Build ArduCopter for the NUCLEO-H743ZI2 bench FC (stage B4). Run in WSL Ubuntu 22.04.
#   bash sim/nucleo/build_nucleo_sim.sh [ARDUPILOT_DIR] [JOBS]
# Output: <ARDUPILOT_DIR>/build/NucleoH743-SimVIO/bin/arducopter_with_bl.hex (flash at 0x08000000
# with STM32CubeProgrammer over the ST-LINK) and arducopter.apj.
set -euo pipefail
AP=${1:-$HOME/ardupilot}
JOBS=${2:-8}
HERE=$(cd "$(dirname "$0")" && pwd)
BOARD=NucleoH743-SimVIO

# board definition from this repo into the ArduPilot tree (untracked there)
rm -rf "$AP/libraries/AP_HAL_ChibiOS/hwdef/$BOARD"
cp -r "$HERE/$BOARD" "$AP/libraries/AP_HAL_ChibiOS/hwdef/$BOARD"
# same MCU and board as NucleoH743: reuse its bootloader so waf can produce *_with_bl.hex
cp "$AP/Tools/bootloaders/NucleoH743_bl.bin" "$AP/Tools/bootloaders/${BOARD}_bl.bin"
cp "$AP/Tools/bootloaders/NucleoH743_bl.hex" "$AP/Tools/bootloaders/${BOARD}_bl.hex"

cd "$AP"
nice -n 10 ./waf configure --board "$BOARD"
nice -n 10 ./waf copter -j"$JOBS"
ls -la "build/$BOARD/bin/"
# confirm the features are in: the strings appear only when the code is compiled in
for s in VISO_TYPE EK3_SRC1_POSXY; do
  printf '%-16s ' "$s"; grep -c "$s" "build/$BOARD/bin/arducopter.bin" || true
done
