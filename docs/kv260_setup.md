# KV260 preparation

Goal of the first session: the board boots Ubuntu 22.04, is reachable over SSH, runs this repo's
tests, runs the B2 VO benchmark (`--tag kv260`), and talks to the Nucleo over a UART.
Nothing here touches the bitstream or the DPU yet (Phase 5).

## Hardware checklist

| Item | Notes |
|---|---|
| KV260 with fan/heatsink fitted | runs hot without the fan |
| 12 V power supply for the KV260 | in the Basic Accessory Pack; otherwise the AMD-specified 12 V adapter |
| microSD, 32 GB or more | the 16 GB card in the accessory pack is tight for Ubuntu + data |
| micro-USB cable to J4 | serial console + JTAG; needed for the first boot |
| Ethernet cable | to your router or directly to the PC |
| 3.3 V USB-UART adapter + 3 jumper wires | KV260 USB <-> Nucleo UART7 (PE8/PF6) + GND. **3.3 V only** |

## PC software
- Image writer: balenaEtcher (or Rufus).
- Serial terminal: PuTTY or Tera Term (115200 8N1).

## Steps

1. **Boot firmware.** Ubuntu 22.04 for Kria needs recent boot firmware on the SOM's QSPI flash;
   boards that shipped long ago may have firmware too old to boot it. If the board does not boot
   the 22.04 image, update the boot firmware with AMD's Image Recovery tool (hold the FWUEN button
   while powering on, then open the tool's web page). Follow AMD's Kria boot-firmware wiki for the
   exact file matching the pinned stack (Kria 2023.2). Ask before flashing firmware.
2. **Image.** Download the official "Ubuntu 22.04 LTS for AMD Kria K26" image from ubuntu.com
   and write it to the microSD card with balenaEtcher.
3. **First boot.** Connect J4 and Ethernet, insert the card, power on. Several COM ports appear on
   the PC; the console is one of them at 115200. Log in as `ubuntu` / `ubuntu`; you will be asked
   to set a new password.
4. **Network + SSH.** `ip -4 addr` to find the address. Put it in `.env` on the PC
   (`KV260_HOST=...`), **not** in the repo. From the PC: `ssh ubuntu@<address>`; add an SSH key.
5. **Do not upgrade the release.** Decline any prompt to move to 24.04 (pinned stack). Before a
   full `apt upgrade`, record the versions (step 9) and ask: it can change the kernel and Xilinx
   packages that the DPU work depends on.
6. **Packages (apt, no source builds):**
   ```
   sudo apt update
   sudo apt install -y git python3-pip python3-venv python3-opencv python3-numpy
   ```
   Ubuntu 22.04 ships OpenCV 4.5.4; our code is tested on 4.5.5 (same API generation).
7. **Repo + Python env** (venv sees the apt OpenCV/numpy; no `[vision]` extra on the board):
   ```
   git clone https://github.com/otaviorossi07/kv260-uav-vision.git && cd kv260-uav-vision
   python3 -m venv --system-site-packages ~/venv && . ~/venv/bin/activate
   pip install -e ".[dev]"
   pytest -q
   ```
   If pip starts **compiling** anything (for example a pymavlink dependency without an ARM wheel),
   stop and report it: CLAUDE.md requires asking before source builds on the board.
8. **VO benchmark (B2 timing).** Copy one EuRoC sequence (only `mav0/cam0`, `mav0/imu0`,
   `mav0/state_groundtruth_estimate0` are used) from the PC with `scp -r`, then
   `python tools/bench/vo_euroc.py <seq> --tag kv260`. Output goes to `results/b2_vo_euroc/`,
   including `timing_by_stage` for the FPGA decision.
9. **Report back** (paste the output):
   ```
   lsb_release -d; uname -r; python3 --version
   python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
   cat /proc/cmdline; ls -l /dev/ttyPS* /dev/ttyUSB* 2>/dev/null
   sudo xmutil boardid -b som 2>/dev/null | head; free -h; df -h /
   ```
   `/proc/cmdline` shows which `ttyPS` is the console (error E22 in `docs/errors.md`).
10. **Nucleo link.** USB-UART adapter in a KV260 USB port: adapter TX -> PF6, adapter RX -> PE8,
    GND -> GND. Then `python -m fc_link --connection /dev/ttyUSB0,921600` and
    `SITL_ARDUPILOT=/dev/ttyUSB0,921600 pytest -m sitl -k ardupilot -s`, then the bench tools
    (results compare with the USB runs in `results/b4_nucleo/SUMMARY.md`):
    ```
    python tools/bench/fc_rx_timing.py /dev/ttyUSB0,921600 --tag nucleo_uart7_kv260
    python tools/bench/truth_echo.py /dev/ttyUSB0,921600 --tag uart7_d50_s1
    ```
    The truth echo changes EKF source parameters on the Nucleo (restored at the end); it never
    arms, and refuses to run on an FC without the simulation.

## Later (Phase 5, not now)
Vitis AI 3.5 runtime (VART), the DPU overlay and `xlnx-config`: check which packages match
Ubuntu 22.04 + Kria 2023.2 + Vitis AI 3.5 before installing anything; ask first.
