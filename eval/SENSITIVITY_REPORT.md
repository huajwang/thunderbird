# Thunderbird SLAM â€” Parameter Sensitivity Analysis

**Method:** One-parameter-at-a-time sweep over stress-test presets
**Presets:** aggressive_drone (EuRoC proxy), fast_car (KITTI proxy), degenerate_corridor, spinning_top
**Baseline:** Engine compiled defaults
**Scoring:** Composite = ATE_RMSE x 1.0 + Drift/100m x 0.5 + RT_avg x 0.1  (lower is better)

## Baseline Performance

| Preset | ATE RMSE (m) | Drift/100m (m) | RPE (m) | Avg RT (ms) |
|---|---:|---:|---:|---:|
| aggressive_drone | 35.85 | 7.38 | 100.00 | 11.13 |
| spinning_top | 0.23 | 9.80 | 0.00 | 11.11 |
| fast_car | 205.26 | 16.61 | 100.00 | 8.35 |
| degenerate_corridor | 1.84 | 27.32 | 0.00 | 10.65 |

---

## Parameter Sensitivity Tables

### Dimension: IMU Noise Covariance

#### `gyro_noise`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.0e-04 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.38 | 115.34 | 69.36 |
| 5.0e-04 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.42 | 115.57 | 69.49 |
| 1.0e-03 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.35 | 115.47 | 69.41 |
| 2.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.36 | 115.46 | 69.41 |
| 5.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.40 | 115.43 | 69.42 |
| 0.01 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.42 | 115.50 | 69.46 |

**Best for drone:** `gyro_noise` = 1.0e-03 (score 23.35)
**Best for car:** `gyro_noise` = 2.0e-04 (score 115.34)

#### `accel_noise`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.35 | 115.52 | 69.43 |
| 5.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.38 | 115.45 | 69.41 |
| 0.01 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.45 | 115.46 | 69.46 |
| 0.02 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.40 | 115.50 | 69.45 |
| 0.05 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.46 | 115.42 | 69.44 |
| 0.1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.49 | 115.74 | 69.61 |

**Best for drone:** `accel_noise` = 1.0e-03 (score 23.35)
**Best for car:** `accel_noise` = 0.05 (score 115.42)

#### `gyro_bias_rw`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0e-07 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.46 | 115.55 | 69.51 |
| 1.0e-06 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.55 | 115.70 | 69.63 |
| 1.0e-05 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.44 | 115.52 | 69.48 |
| 1.0e-04 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.46 | 115.56 | 69.51 |
| 1.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.50 | 115.57 | 69.54 |

**Best for drone:** `gyro_bias_rw` = 1.0e-05 (score 23.44)
**Best for car:** `gyro_bias_rw` = 1.0e-05 (score 115.52)

#### `accel_bias_rw`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0e-06 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.40 | 115.43 | 69.41 |
| 1.0e-05 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.42 | 115.45 | 69.43 |
| 1.0e-04 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.48 | 115.73 | 69.60 |
| 1.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.39 | 115.43 | 69.41 |
| 0.01 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.34 | 115.49 | 69.42 |

**Best for drone:** `accel_bias_rw` = 0.01 (score 23.34)
**Best for car:** `accel_bias_rw` = 1.0e-06 (score 115.43)

### Dimension: Voxel Filter Resolution

#### `voxel_resolution`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.42 | 115.64 | 69.53 |
| 0.1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.45 | 115.52 | 69.49 |
| 0.2 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.46 | 115.51 | 69.48 |
| 0.3 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.47 | 115.72 | 69.60 |
| 0.5 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.55 | 115.74 | 69.65 |
| 0.8 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.53 | 115.67 | 69.60 |
| 1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.53 | 115.55 | 69.54 |

**Best for drone:** `voxel_resolution` = 0.05 (score 23.42)
**Best for car:** `voxel_resolution` = 0.2 (score 115.51)

### Dimension: Keyframe Selection Threshold

#### `convergence_eps`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0e-05 | 35.85 (0%) | 0.23 (0%) **ok** | 205.31 (+0%) | 1.84 (0%) **ok** | 23.43 | 115.67 | 69.55 |
| 1.0e-04 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.39 | 115.44 | 69.41 |
| 1.0e-03 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.43 | 115.45 | 69.44 |
| 0.01 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.40 | 115.44 | 69.42 |
| 0.1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.38 | 115.46 | 69.42 |

**Best for drone:** `convergence_eps` = 0.1 (score 23.38)
**Best for car:** `convergence_eps` = 1.0e-04 (score 115.44)

#### `min_correspondences`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.38 | 115.42 | 69.40 |
| 10 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.36 | 115.44 | 69.40 |
| 20 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.34 | 115.40 | 69.37 |
| 50 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.35 | 115.43 | 69.39 |
| 100 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.36 | 115.44 | 69.40 |

**Best for drone:** `min_correspondences` = 20 (score 23.34)
**Best for car:** `min_correspondences` = 20 (score 115.40)

### Dimension: Motion Model Constraints

#### `max_iterations`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.41 | 115.62 | 69.52 |
| 2 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.35 | 115.45 | 69.40 |
| 5 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.44 | 115.45 | 69.44 |
| 10 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.41 | 115.45 | 69.43 |
| 15 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.40 | 115.67 | 69.53 |
| 20 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.36 | 115.48 | 69.42 |

**Best for drone:** `max_iterations` = 2 (score 23.35)
**Best for car:** `max_iterations` = 5 (score 115.45)

#### `imu_integration_substeps`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.37 | 115.47 | 69.42 |
| 2 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.38 | 115.49 | 69.44 |
| 4 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.41 | 115.48 | 69.45 |
| 8 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.41 | 115.46 | 69.44 |

**Best for drone:** `imu_integration_substeps` = 1 (score 23.37)
**Best for car:** `imu_integration_substeps` = 8 (score 115.46)

#### `plane_noise_sigma`

| Value | ATE (aggressive_drone) | ATE (spinning_top) | ATE (fast_car) | ATE (degenerate_corridor) | Drone Score | Car Score | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.43 | 115.52 | 69.47 |
| 5.0e-03 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.39 | 115.46 | 69.42 |
| 0.01 **baseline** | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.37 | 115.45 | 69.41 |
| 0.05 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.46 | 115.46 | 69.46 |
| 0.1 | 35.85 (0%) | 0.23 (0%) **ok** | 205.26 (0%) | 1.84 (0%) **ok** | 23.35 | 115.45 | 69.40 |

**Best for drone:** `plane_noise_sigma` = 0.1 (score 23.35)
**Best for car:** `plane_noise_sigma` = 0.01 (score 115.45)

---

## Recommended Optimal Configurations

### Drone Profile (EuRoC-type scenarios)

| Parameter | Baseline | Recommended | Delta |
|---|---:|---:|---|
| `gyro_noise` | 1.0e-03 | 1.0e-03 | ï¿½ |
| `accel_noise` | 0.01 | **1.0e-03** | changed |
| `gyro_bias_rw` | 1.0e-05 | 1.0e-05 | ï¿½ |
| `accel_bias_rw` | 1.0e-04 | **0.01** | changed |
| `voxel_resolution` | 0.3 | **0.05** | changed |
| `convergence_eps` | 1.0e-03 | **0.1** | changed |
| `min_correspondences` | 20 | 20 | ï¿½ |
| `max_iterations` | 5 | **2** | changed |
| `imu_integration_substeps` | 1 | 1 | ï¿½ |
| `plane_noise_sigma` | 0.01 | **0.1** | changed |

### Car Profile (KITTI-type scenarios)

| Parameter | Baseline | Recommended | Delta |
|---|---:|---:|---|
| `gyro_noise` | 1.0e-03 | **2.0e-04** | changed |
| `accel_noise` | 0.01 | **0.05** | changed |
| `gyro_bias_rw` | 1.0e-05 | 1.0e-05 | ï¿½ |
| `accel_bias_rw` | 1.0e-04 | **1.0e-06** | changed |
| `voxel_resolution` | 0.3 | **0.2** | changed |
| `convergence_eps` | 1.0e-03 | **1.0e-04** | changed |
| `min_correspondences` | 20 | 20 | ï¿½ |
| `max_iterations` | 5 | 5 | ï¿½ |
| `imu_integration_substeps` | 1 | **8** | changed |
| `plane_noise_sigma` | 0.01 | 0.01 | ï¿½ |

### Cross-validation note

> These recommendations are derived from one-at-a-time sweeps on
> synthetic stress-test data. Parameter interactions are NOT captured.
> Validate on real KITTI/EuRoC sequences before deploying.
> Avoid overfitting: if a parameter only improves one preset while
> degrading others, keep the baseline value.
