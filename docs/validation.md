# Validation

What is under test is the integration — D435i → `SensorData` → RTAB-Map — and how the system
behaves in a real room. RTAB-Map's own algorithms are taken as given, as is everything else the
library provides (see the [README](../README.md)).

Validation has two halves: runs made with the camera in hand, and analysis of what a run leaves
behind. The analysis is automated by `tools/validate_run.py`. Everything else is judged by eye,
from the HUD, the console and `rtabmap-databaseViewer`.

## The analysis script

```bash
./tools/validate_run.py runs/t6_1.db                                   # database only
./tools/validate_run.py --traj runs/t6_1_trajectory.txt runs/t6_1.db
./tools/validate_run.py runs/*.db                                      # compare several runs
```

The database is opened read-only and nothing is recomputed: every number is already stored in it.
Optimized poses come from the exported trajectory when `--traj` is given, otherwise from the last
optimization RTAB-Map saved. The exit code is 1 when any check fails.

| Check | A failure means |
|---|---|
| `export` | the trajectory is missing nodes, or timestamps do not increase |
| `map segments` | a map segment created after an odometry reset was lost during export |
| `gravity links` | no gravity constraints although the IMU was on, or only some nodes have them (warning) |
| `IMU agreement` | mean roll/pitch disagreement with the optimized poses above 2°, which points at wrong axes in `imuLocalTransform` |
| `IMU latency` | the disagreement grows with rotation speed, so the orientation lags the frame (warning) |
| `loop closures` | a closure was accepted with fewer inliers than `Vis/MinInliers` |
| `closure plausibility` | a closure links poses more than 2 m or 60° apart, which is implausible for the trusted depth range (warning) |
| `drift` | reports odometry against the optimized graph; warns above 5% of the travelled path |
| `covariances` | graph links with zero, negative or non-finite information |
| `timing` | RTAB-Map is not keeping up with `Rtabmap/DetectionRate` (warning) |
| `calibration` | the camera calibration changed during a run |

Thresholds are constants at the top of the script, calibrated on early runs and expected to be
refined.

## Field procedures

Each run goes into its own database, otherwise the previous one is overwritten. Plain
`./scripts/run.sh` already names every artifact after the start time; pass the paths explicitly
when a run needs a speaking name:

```bash
./scripts/run.sh --db runs/t6_1.db --cloud runs/t6_1.pcd --traj runs/t6_1_trajectory.txt
```

Accuracy requirements are deliberately loose: mark the start with tape on the floor and check
angles against a phone inclinometer. Walking the route perfectly is not required.

| Test | Procedure | Expected result |
|---|---|---|
| **T0 Static** | Camera still on a desk, textured scene, 60 s | No global-time warning; no `LOST` in the HUD; two or three nodes; odometry drift under 2 cm |
| **T1 Scale** | Walk 3 m along a tape measure and back | `drift`: path length within 5% of 6 m |
| **T2 Alignment** | A box on a desk 1–2 m away, slow 90° orbit | In `rtabmap-databaseViewer`, depth boundaries follow the object edges with no consistent offset |
| **T3 IMU axes** | 10 s each: level on a desk, tilted forward ~30°, rolled ~90° | `IMU agreement` passes; the tilt matches the inclinometer in magnitude and sign within 3° |
| **T4 IMU latency** | Rotate ±45°, first slowly (~20°/s), then quickly (~150°/s), 30 s each | `IMU latency` passes. A warning means the orientation is not sampled at the frame's timestamp |
| **T5 Tracking loss** | A 10 m route; halfway cover the lens for 3 s, then return to mapped ground | `LOST` and recovery in the HUD; `map segments` may warn, which is expected, but must not fail; a closure appears after the return |
| **T6 Loop** | A 15–30 m loop returning to the start marker within 20 cm, two laps | Closures on the second lap; `drift` shows a start-to-end gap far below the odometry gap; `closure plausibility` passes |
| **T7 Repeatability** | T6 five times over the same route under the same lighting | `./tools/validate_run.py runs/t6_*.db`: path length spread under 5%, every run closes the loop |
| **T8 IMU contribution** | Three T6 runs with the IMU against three with `--no-imu` | With the IMU: `IMU agreement` passes, the closing error is no worse, the map leans less |

Suggested order: T0, T3, T1, T6, T5, T4, T7, T8. The first four take about twenty minutes and catch
gross integration errors.

The IMU contribution can also be measured without a second capture, by reprocessing the same data
without gravity constraints:

```bash
source /opt/ros/jazzy/setup.bash         # otherwise the tools cannot find librtabmap_*.so
rtabmap-reprocess --Optimizer/GravitySigma 0 runs/t6_1.db runs/t6_1_nograv.db
./tools/validate_run.py runs/t6_1.db runs/t6_1_nograv.db
```

Two caveats. Gravity links remain in the output database — they are measurements, not constraints,
and only their weight during optimization is zeroed, so the difference appears in `IMU agreement`
rather than in the link count. On one route the 95th percentile grew from 2.5°/3.7° to 5.7°/6.6°.
Reprocessing also starts a new session, so `map segments` reports one segment more than the input.

## What the script does not cover

Per-frame data is never stored: color and depth timestamps, dropped frames, IMU rate, the number
and duration of tracking losses, per-frame odometry features and inliers. RTAB-Map's database keeps
statistics per map node only. In T0, T4 and T5 these are observed live in the HUD (`TRACKING` /
`LOST`, features, inliers, rate) and in the console status line printed every two seconds.
