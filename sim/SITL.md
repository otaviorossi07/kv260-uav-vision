# SITL in WSL (phase 2 acceptance)

Both SITLs and the `pytest -m sitl` run happen inside one WSL distro, so `127.0.0.1` works
without port forwarding. Use **Ubuntu 22.04**: the ArduPilot and PX4 setup scripts support
22.04/24.04, not 26.04, and 22.04 matches the KV260.

## 0. Ubuntu 22.04 distro (once, from Windows cmd)

```
wsl --install -d Ubuntu-22.04
wsl -d Ubuntu-22.04
```
Create the Linux user when asked. All commands below run inside that shell.

## 1. ArduPilot SITL (once: ~20-30 min)

```
sudo apt update && sudo apt install -y git
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git checkout Copter-4.5
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

Run (terminal 1). Listens on TCP 5760, no MAVProxy:
```
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -w
```
`-w` wipes parameters to defaults; drop it on later runs.

## 2. PX4 SITL (once: ~30-40 min)

```
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
git checkout v1.15.4
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
```
If it stops at "Installing PX4 Python3 dependencies" with `Invalid requirement: 'matplotlib>=3.0.*'`
(newer pip, pulled in by the ArduPilot prereqs, rejects that syntax), install from a fixed copy;
nothing after that step is needed for SIH:
```
sed 's/>=\([0-9.]*\)\.\*/>=\1/' Tools/setup/requirements.txt > /tmp/px4-req.txt
python3 -m pip install --user -r /tmp/px4-req.txt
```
Close the shell and open it again (`exit`, then `wsl -d Ubuntu-22.04`).

Run (terminal 1). SIH simulator, no Gazebo needed; onboard MAVLink goes to UDP 14540:
```
cd ~/PX4-Autopilot
make px4_sitl sihsim_quadx
```
If that target fails to build, fall back to Gazebo headless: `HEADLESS=1 make px4_sitl gz_x500`.

**Intermittent build failure: `ninja: error: manifest 'build.ninja' still dirty after 100 tries`**
(inside `uxrce_dds_client/.../libmicroxrceddsclient_project-build`, about 1 in 3 fresh builds on
this PC, investigated 2026-11-23). Cause: the WSL clock steps back and forth by the Windows clock
error. Hyper-V pushes the Windows time into WSL every few seconds, `systemd-timesyncd` pulls it
back to internet time; with the Windows clock 17.7 s fast (never synchronized, source "Local CMOS
Clock") the WSL clock jumped -17.7 s / +17.7 s about every 5 s (measured: realtime minus monotonic
clock). A backward step during the nested Micro-XRCE-DDS-Client configure leaves
`CMakeFiles/3.22.1/CMakeSystem.cmake` dated in the future, so every regenerated `build.ninja`
looks older than it and ninja gives up. The `CMAKE_BUILD_PARALLEL_LEVEL` setting makes no
difference (2/6 failures with and without). Fixes:
- Real fix: synchronize the Windows clock (Settings > Time & language > Date & time > "Set time
  automatically" + "Sync now"; check with `w32tm /stripchart /computer:time.windows.com /samples:3`).
- Workaround: run `make px4_sitl_default` again; the outer build reconfigures the nested project
  and completes (verified).
Our measurements use monotonic clocks (`fc_link` TIMESYNC, `perf_counter` in the VO bench), so
results are not affected; anything that compares wall-clock times across Windows and WSL is.

## 3. Python env for the tests (once)

```
sudo apt install -y python3-venv
python3 -m venv ~/kv260-venv
source ~/kv260-venv/bin/activate
cd /mnt/c/Users/<you>/Documents/kv260-uav-vision    # adjust to your repo path
pip install -e ".[dev]"
```

## 4. Run the acceptance tests (terminal 2, with one SITL running in terminal 1)

```
source ~/kv260-venv/bin/activate
cd /mnt/c/Users/<you>/Documents/kv260-uav-vision    # adjust to your repo path
pytest -m sitl -k ardupilot -s     # with ArduPilot SITL up
pytest -m sitl -k px4 -s           # with PX4 SITL up
```
Each test waits up to 60 s for the heartbeat, up to 90 s for the first `ATTITUDE` and up to
120 s for `LOCAL_POSITION_NED` (the EKF needs a GPS origin first). Run one SITL at a time.
PX4 sends `ATTITUDE` only after EKF2 aligns, measured at ~40-50 s after SIH starts; ArduPilot
sends it at once. The PX4 test therefore takes ~75 s.

Quick manual check instead of pytest:
```
python -m fc_link --connection tcp:127.0.0.1:5760 --seconds 20     # ArduPilot
python -m fc_link --connection udpin:0.0.0.0:14540 --seconds 20    # PX4
```
