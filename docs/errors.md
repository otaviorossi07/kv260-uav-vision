# Error log

Every error found in this project, including environment and tooling problems, so nobody has to
rediscover them. Add an entry when an error is found, and update it when it is fixed.
Status: **open**, **workaround** (works, cause not removed), **fixed**.

Entry format: symptom (what you see), cause, fix, where it is documented, status.

## Environment (PC, WSL, tools)

### E1. WSL distro was Ubuntu 26.04 - 2026-11-21 - fixed
- Symptom: ArduPilot and PX4 setup scripts refuse to run.
- Cause: they support Ubuntu up to 22.04/24.04; the existing `Ubuntu` distro was 26.04.
- Fix: installed `Ubuntu-22.04` (the pinned version), removed 26.04. `sim/SITL.md` step 0.

### E2. PX4 setup: `Invalid requirement: 'matplotlib>=3.0.*'` - 2026-11-21 - workaround
- Symptom: `Tools/setup/ubuntu.sh` stops at "Installing PX4 Python3 dependencies".
- Cause: newer pip (pulled in by the ArduPilot prerequisites) rejects `>=X.*` in PX4 v1.15.4's
  `requirements.txt`.
- Fix: install from a copy with the `.*` removed. `sim/SITL.md` section 2.

### E3. PX4 SITL build fails intermittently: `manifest 'build.ninja' still dirty after 100 tries` - 2026-11-21, investigated 2026-11-23 - open (cause known)
- Symptom: about 1 in 3 fresh `make px4_sitl_default` builds fail in the nested
  Micro-XRCE-DDS-Client build; running `make` again succeeds.
- Cause: the Windows clock is 17.7 s fast and never synchronized ("Local CMOS Clock"). Hyper-V
  pushes Windows time into WSL, `systemd-timesyncd` pulls it back to internet time, so the WSL
  clock jumps -17.7 s / +17.7 s about every 5 s (measured). A backward jump leaves
  `CMakeSystem.cmake` dated in the future and ninja regenerates `build.ninja` forever.
  Not the parallel build setting (2/6 failures with and without `CMAKE_BUILD_PARALLEL_LEVEL`).
- Fix: synchronize the Windows clock (user action). Workaround: run `make` again.
  `sim/SITL.md` section 2.
- Lesson: the first build log lost the real error because it was written through an asynchronous
  `tee` that had not flushed when the shell exited; write build logs with plain `> file 2>&1`.

### E4. `python` on Windows hangs - 2026-11-23 - workaround
- Symptom: a command calling `python` or `python3` in Git Bash never returns.
- Cause: on this PC they resolve to the Microsoft Store placeholder, which waits for input.
- Fix: call the venv interpreter by full path.

### E5. Inline `wsl -- bash -c '...'` commands break - 2026-11-21 - workaround
- Symptom: quoting errors, `$` variables expanded by the Windows-side shell.
- Fix: write a script file, strip CR (`tr -d '\r'`), run it in WSL.

### E6. Windows clock skew affects wall-clock comparisons - 2026-11-23 - open
- Same root cause as E3. Our measurements use monotonic clocks (`fc_link` TIMESYNC,
  `perf_counter` in the VO bench) and are not affected; anything comparing wall-clock time
  between Windows and WSL (or the KV260 later) is. Fix with E3.

### E7. CI warning: Node.js 20 deprecated for `actions/checkout@v4` - 2026-11-21 - open
- Harmless today; update the action version when GitHub stops forcing Node 24.

### E8. Docker image has no pymavlink (or pyserial) - 2026-11-21 - open
- The image was built before these dependencies were added. Fix: `docker build` again.

## Flight-controller link and SITL

### E9. PX4 SITL: "no ATTITUDE" in the acceptance test - 2026-11-21 - fixed
- Cause: PX4 streams ATTITUDE only after EKF2 has aligned, about 40-50 s after SIH start;
  ArduPilot sends it at once.
- Fix: test waits up to 90 s (`tests/test_sitl.py`).

### E10. pyserial missing: `ModuleNotFoundError: No module named 'serial'` - 2026-11-23 - fixed
- Cause: pymavlink does not depend on pyserial, but every serial link (USB, UART) needs it.
- Fix: `pyserial>=3.4` added to the project dependencies.

## Nucleo bench flight controller (B4)

### E11. No ArduPilot COM port after flashing - 2026-11-22 - fixed (not a fault)
- Cause: only the ST-LINK USB (CN1) was connected; ArduPilot's MAVLink is on the user USB (CN13).
- Fix: second cable on CN13 (enumerates as USB 1209:5741). `sim/nucleo/README.md`.

### E12. SWD "Unable to get core ID" while ArduPilot runs - 2026-11-22 - workaround
- Symptom: hot-plug or normal debugger connection fails; flashing still works.
- Fix: connect "under reset" (`mode=UR`). `sim/nucleo/README.md`.

### E13. No LOCAL_POSITION_NED on the Nucleo - 2026-11-23 - fixed
- Cause: the simulated GPS was off (`GPS_TYPE 1`, no GPS hardware), so no fix and no EKF origin.
  The simulated GPS (`GPS_TYPE 100`) is only created if some serial port has the GPS protocol,
  and the only free UART (UART7) is the KV260 link.
- Fix: USART3 added as SERIAL1 with the GPS protocol as a placeholder; `GPS_TYPE 100`.

### E14. Nucleo parameters lost at reboot; reboot-only parameters cannot be set - 2026-11-23 - fixed
- Cause: the base board stores no parameters (`HAL_USE_EMPTY_STORAGE`); serial protocol, GPS type
  and `VISO_TYPE` take effect only after a reboot, which wipes them.
- Fix: compiled-in `sim/nucleo/NucleoH743-SimVIO/defaults.parm`.

### E15. Nucleo ATTITUDE 32-40 Hz instead of the requested 50 Hz - 2026-11-23 - open
- Passes the test (>= 10 Hz) but below SITL's 50 Hz. Not investigated yet (CPU load of the
  on-board simulation or USB scheduling). Check before Phase 6 latency work.

### E16. Build script path wrong when run from a copy - 2026-11-21 - fixed
- `build_nucleo_sim.sh` locates the board files relative to itself; run it in place.

## Visual odometry (B2)

### E17. EuRoC download links dead; host returns 429 - 2026-11-21 - workaround
- Original robotics.ethz.ch links are gone; the ETH Research Collection rate-limits.
- Fix: dataset from doi 10.3929/ethz-b-000690084, one resumable download.

### E18. `findEssentialMat(..., RANSAC)` imprecise - 2026-11-21 - fixed
- Cause: plain RANSAC is not refined; `USAC_DEFAULT` is much better on OpenCV 5.x, weaker on 4.5.
- Fix: USAC plus a Sampson-error LM refinement (`refine=True`). `docs/bench.md` B2.

### E19. `recoverPose` drops distant points - 2026-11-21 - fixed
- Cause: the short form rejects points beyond 50 baselines; passing `distanceThresh`
  positionally silently selects the short form in the Python binding.
- Fix: pass `distanceThresh` by keyword. `docs/bench.md` B2.

### E20. VO-vs-gyro rotation mismatch - 2026-11-21 - fixed
- Cause: EuRoC gyro bias. Fix: bias-corrected metric added.

### E21. 5.6 deg travel-direction bias on V1_01 - 2026-11-21 - open
- Not a fixed calibration offset (other sequences don't share it), not a time offset
  (+-40 ms scanned). First hypothesis (calibration offset) was wrong.
  `results/b2_vo_euroc/SUMMARY.md`.

## KV260

### E22. `/dev/ttyPS1` may be the KV260 serial console - 2026-11-23 - open (to verify)
- CLAUDE.md plans the flight-controller link on `/dev/ttyPS1`; on Kria Ubuntu that is likely the
  console. Check on the board; alternative a USB-UART adapter (`/dev/ttyUSB0`).
