# Thunderbird SLAM — Baseline Performance Report

**Date:** 2025-01-XX (stress-test presets, pre-tuning)
**Build:** GCC 10.2.0, C++20, Release build
**Engine:** AcmeSlamEngine (ESIKF + ikd-Tree)
**Alignment:** SE(3) Horn method, no scale recovery, no outlier rejection

---

## 1. Summary Table

| Scenario | Config | ATE RMSE (m) | RPE trans (m) | Drift/100m (m) | Avg RT (ms) | P95 RT (ms) |
|---|---|---:|---:|---:|---:|---:|
| aggressive_drone | drone | 35.85 | 100.00 | 7.38 | 9.07 | 15.76 |
| aggressive_drone | car   | 35.85 | 100.00 | 7.38 | 10.66 | 15.94 |
| fast_car         | drone | 205.26 | 100.00 | 16.61 | 8.92 | 15.41 |
| fast_car         | car   | 205.26 | 100.00 | 16.61 | 9.05 | 15.51 |
| degenerate_corridor | drone | 1.84 | 0.00 | 27.32 | 9.47 | 15.94 |
| degenerate_corridor | car   | 1.84 | 0.00 | 27.32 | 9.75 | 16.07 |
| spinning_top     | drone | 0.23 | 0.00 | 9.80 | 10.29 | 15.60 |
| spinning_top     | car   | 0.23 | 0.00 | 9.80 | 11.37 | 16.60 |

### Thresholds

| Metric | Good (✓) | Warning (⚠) | Fail (✗) |
|---|---|---|---|
| ATE RMSE | < 2.0 m | 2.0–10.0 m | > 10.0 m |
| Drift/100m | < 2.0 m | 2.0–5.0 m | > 5.0 m |
| RPE trans | < 0.5 m | 0.5–2.0 m | > 2.0 m |
| Avg RT | < 20 ms | 20–50 ms | > 50 ms |

---

## 2. Per-Scenario Analysis

### aggressive_drone (600 frames, 60s, high angular rate)
- **ATE 35.85 m** — ✗ Very high absolute error. The synthetic stress trajectory
  has aggressive 6-DOF motion that the current engine defaults fail to track.
- **Drift 7.38 m/100m** — ✗ Exceeds 5% drift budget.
- **RPE 100 m** — ✗ Saturated at the delta distance; trajectory diverges within
  the first 100 m segment.
- **Runtime ~9–11 ms** — ✓ Well within real-time budget (10 Hz LiDAR = 100 ms).
- **Config difference:** Negligible. Car config adds ~1.6 ms avg RT due to larger
  map_radius (150 m vs 60 m) but accuracy is identical — the engine defaults
  dominate over config noise parameters for synthetic data.

### fast_car (600 frames, 60s, high-speed linear motion)
- **ATE 205.26 m** — ✗ Catastrophic. The engine loses track during high-speed
  segments; ikd-Tree queries fail when inter-frame displacement exceeds voxel
  overlap.
- **Drift 16.61 m/100m** — ✗ Extremely high drift rate.
- **RPE 100 m** — ✗ Saturated.
- **Runtime ~9 ms** — ✓ Even at high speed, per-frame compute is bounded.
- **Config difference:** None. Both configs produce identical accuracy — the
  failure mode is fundamentally kinematic, not noise-related.

### degenerate_corridor (600 frames, 60s, geometrically degenerate)
- **ATE 1.84 m** — ✓ Surprisingly good for a degenerate scene. The synthetic
  corridor still has enough structure for the ESIKF to constrain 5 DOF.
- **Drift 27.32 m/100m** — ✗ Very high. While ATE stays low (short trajectory),
  drift rate is unsustainable for long corridors.
- **RPE 0.0 m** — (no valid 100 m segment pairs in this short trajectory)
- **Runtime ~9.5 ms** — ✓ Stable.
- **Config difference:** Negligible.

### spinning_top (300 frames, 30s, pure rotation + slow translation)
- **ATE 0.23 m** — ✓ Excellent. Pure rotation with slow drift is the best case
  for the ESIKF + ikd-Tree pipeline.
- **Drift 9.80 m/100m** — ✗ Despite low ATE, drift per distance is elevated
  because the total distance traveled is very short (<3 m).
- **RPE 0.0 m** — (no valid 100 m segments)
- **Runtime ~10–11 ms** — ✓ Marginally higher due to dense map overlap.
- **Config difference:** Car config is ~1 ms slower (larger map structures).

---

## 3. drone.yaml vs car.yaml Comparison

| Metric | drone.yaml wins | car.yaml wins | Tie |
|---|---|---|---|
| ATE RMSE | — | — | All 4 scenarios identical |
| Drift/100m | — | — | All 4 scenarios identical |
| Avg RT | 4/4 | 0/4 | — |
| P95 RT | 3/4 | 0/4 | 1 (fast_car within noise) |

**Key finding:** Config profiles have **no effect on accuracy** in stress-test
mode because:
1. Stress-test adapters generate deterministic synthetic data independent of
   config noise parameters
2. The SLAM engine's default `SlamEngineConfig` is used (config YAML only
   overrides noise/map parameters, not the core ESIKF tolerances)

**Runtime difference** is consistent: drone config is ~1–2 ms faster per frame
because of smaller `map_radius` (60 m vs 150 m) and fewer `max_map_points`
(300K vs 800K), causing less ikd-Tree search overhead.

**Conclusion:** The config comparison will only become meaningful on real sensor
data (KITTI, EuRoC) where noise parameters genuinely affect state estimation.

---

## 4. Identified Weak Spots

| Priority | Issue | Affected Scenarios | Root Cause Hypothesis |
|---|---|---|---|
| **P0** | Catastrophic tracking loss at high speed | fast_car | Inter-frame displacement exceeds voxel overlap; ikd-Tree nearest-neighbor search returns no correspondences |
| **P0** | Large ATE on aggressive maneuvers | aggressive_drone | ESIKF convergence fails when angular rate exceeds ~2 rad/s; need more `max_iterations` or larger `convergence_eps` |
| **P1** | High drift in degenerate geometry | degenerate_corridor | Only 1 axis constrained by corridor walls; need degenerate-direction detection + constraint relaxation |
| **P2** | Elevated drift/100m on spinning_top | spinning_top | Very short translational path inflates drift ratio; may not be a real problem |

---

## 5. Recommendations for Tuning Phase

1. **fast_car P0:** Increase `init_gravity_duration_s`, consider motion-compensated
   point cloud deskew with `imu_integration_substeps = 4`, and enlarge
   `map_radius` to 200–250 m.
2. **aggressive_drone P0:** Increase `max_iterations` from default (5) to 10–15,
   tighten `convergence_eps`, enable `refine_online` extrinsic calibration.
3. **degenerate_corridor P1:** Implement degenerate-direction detection in the
   evaluator or engine; consider adding a degeneracy-aware noise inflation to
   the ESIKF plane_noise_sigma.
4. **General:** Run on KITTI 00/05/07 and EuRoC MH_04/V2_03 to validate with
   real sensor noise.

---

## 6. How to Run on Real Datasets

```bash
# KITTI (LiDAR-only, needs --synth-imu)
./build/eval/slam_eval \
    --format kitti \
    --synth-imu --imu-rate 200 \
    -c slamd/config/car.yaml \
    -o eval_output/kitti_00_car \
    /path/to/kitti/sequences/00

# EuRoC (native IMU + synthesized LiDAR from GT)
./build/eval/slam_eval \
    --format euroc \
    -c slamd/config/drone.yaml \
    -o eval_output/euroc_mh04_drone \
    /path/to/euroc/MH_04_difficult

# Full systematic sweep (all datasets × both configs)
bash eval/scripts/run_baseline_eval.sh \
    --kitti-dir /path/to/kitti/sequences \
    --euroc-dir /path/to/euroc \
    --eval-bin  ./build/eval/slam_eval
```

Required dataset structure:
- **KITTI**: `sequences/{00,05,07}/velodyne/*.bin` + `poses/{00,05,07}.txt`
- **EuRoC**: `{MH_04_difficult,V2_03_difficult}/mav0/imu0/data.csv` +
  `state_groundtruth_estimate0/data.csv`

---

## Raw Data

Full per-run CSV metrics are in `build/baseline/<preset>_<config>/metrics.csv`.
Aggregated summary: `build/baseline/baseline_summary.csv`.
