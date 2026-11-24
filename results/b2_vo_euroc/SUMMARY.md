# B2 - monocular VO on EuRoC Vicon room 1 (2026-11-21)

Tool: `tools/bench/vo_euroc.py` (default config: 300 Shi-Tomasi features, LK 21 px / 3 levels,
forward-backward check 1 px, keyframe at 12 px median parallax, `USAC_DEFAULT` + Sampson LM
refinement). Camera cam0, 752x480 at 20 Hz. Host: x86-64 PC in WSL Ubuntu 22.04, one process,
OpenCV threads 8. Files: `<seq>_<tag>.json` (all metrics + config + environment),
`_traj.csv` (keyframe positions), `_steps.csv` (per-step directions and errors).

## Results, Python 3.10 + OpenCV 4.5.5 (closest to the board's 4.5.4) - tag `x86_64-cv45`

| Sequence | Frames | Keyframes | Chain breaks | Rotation err med / p95 (deg) | VO vs gyro, bias-corrected med (deg) | Direction err med / p95 (deg) | Scale-oracle ATE (m) | End drift (% path) | fps (PC) |
|---|---|---|---|---|---|---|---|---|---|
| V1_01_easy | 2912 | 1217 | 5 | 0.071 / 0.40 | 0.046 | 5.56 / 29.9 | 0.313 | 0.67 | 228 |
| V1_02_medium | 1710 | 1212 | 13 | 0.064 / 0.56 | 0.062 | 2.89 / 33.8 | 0.279 | 0.49 | 150 |
| V1_03_difficult | 2149 | 1452 | 102 | 0.095 / 0.86 | 0.095 | 4.35 / 50.7 | 0.194 | 0.04 | 139 |

Same runs with OpenCV 5.1 (tag `x86_64-cv51`) are equal or slightly worse (V1_01 ATE 0.389 m,
V1_02 0.396 m, V1_03 0.238 m), so the board's OpenCV 4.5 is not a handicap with the refinement.

"Scale-oracle": VO rotation and direction, step length from ground truth. Monocular VO has no
metric scale; these numbers isolate the camera part. They are not a VIO result.
"Chain breaks": tracking resets (the chain is re-anchored on ground truth at each one).

## Findings

1. **Rotation is good**: 0.06-0.10 deg median per keyframe step (~0.1-0.15 s).
2. **VO agrees with the gyro once its bias is removed**: 0.05-0.10 deg median, vs 0.46 deg on
   V1_01 with the raw gyro. EuRoC's gyro bias is up to ~0.08 rad/s, so the VIO filter must
   estimate gyro bias. The camera-IMU extrinsics (`T_BS`) are consistent with the VO rotation.
3. **Keyframe parallax 12 px is kept** (sweep 12/20/30/45 px on V1_01, `sweep/`): larger values
   do not reduce direction error (~5.6 deg throughout) and make rotation worse. The sweep JSONs
   predate the bias-corrected and per-step outputs.
4. **Direction error has a systematic part on V1_01** (5.6 deg median, constant across step
   lengths 2 cm to > 20 cm). A single 6.0 deg rotation fitted on V1_01 removes most of it
   (median 1.66 deg; half/half cross-check 1.74 deg), but it is **not** a fixed property:
   V1_02 / V1_03 fit only ~2 deg, about 4 deg away from V1_01's, and V1_01's rotation makes
   V1_02 worse. A ground-truth time offset is also ruled out (scan -40..+40 ms: minimum at
   0 +/- 5 ms on all three). Cause open; candidate: rotation-translation ambiguity of
   monocular VO under this sequence's motion. VIO fusion with the IMU is the usual remedy.
5. **V1_03_difficult loses tracking 102 times** (fast motion, motion blur). For the drone this
   argues for a global-shutter camera and short exposure, and for VIO to bridge the gaps.
6. **Speed**: 139-228 fps on the PC, single process. The KV260 A53 number needs the board
   (same tool with `--tag kv260`); phase 7 moves LK to the PL only if that measurement says so.
