# Calibration

> How to store, load, import, and compute sensor extrinsic and intrinsic
> calibration for multi-sensor setups (LiDAR + IMU + Camera).

---

## Overview

The Thunderbird SDK calibration subsystem handles the spatial and temporal
relationships between sensors.  It is split into **storage** (the
`CalibrationBundle` YAML format) and **solvers** (algorithms that compute
the transforms):

```
┌────────────────────────────────────────────────────────────┐
│                   CalibrationBundle                        │
│  imu_T_lidar · cameras[] · imu_noise · YAML load/save     │
└───────┬─────────────────────────────────────────┬──────────┘
        │  populated by                           │  consumed by
┌───────▼───────────┐  ┌────────────────────┐  ┌──▼────────────────┐
│ Kalibr importer   │  │ Offline solvers     │  │ SLAMD odometry    │
│ (camchain + imu)  │  │ Rigid-transform     │  │ Online refiner    │
│                   │  │ LiDAR-IMU (BALM)    │  │ ClockService seed │
│                   │  │ LiDAR-Camera (edge) │  │                   │
└───────────────────┘  └────────────────────┘  └───────────────────┘
```

---

## Key Types

### `SensorExtrinsic` — 6-DOF rigid transform

An SE(3) transform stored as a unit quaternion plus a translation vector:

```cpp
#include <thunderbird/types.h>

thunderbird::SensorExtrinsic T;
T.rotation    = {1.0, 0.0, 0.0, 0.0};  // [w, x, y, z]
T.translation = {0.0, 0.0, 0.28};      // metres

// Convention:  p_target = rotation * p_source + translation
auto T_inv = T.inverse();
auto T_ab  = T_a.compose(T_b);  // T_a * T_b
bool ident = T.is_identity();
```

### `CalibrationBundle` — multi-sensor calibration container

```cpp
#include <thunderbird/calibration.h>

thunderbird::CalibrationBundle bundle;

// Load from YAML
bundle.load_yaml("calibration.yaml");

// Access transforms
auto lidar_to_cam0 = bundle.lidar_T_camera(0);   // derived
auto cam0_to_lidar = bundle.camera_T_lidar(0);   // derived

// Save back
bundle.save_yaml("calibration_out.yaml");
```

### `ImuNoiseParams`

Continuous-time spectral densities matching Kalibr and SLAM engine
conventions:

| Field | Unit | Default | Description |
|---|---|---|---|
| `gyro_noise` | rad/s/√Hz | 1.0e-3 | Gyroscope white noise |
| `accel_noise` | m/s²/√Hz | 1.0e-2 | Accelerometer white noise |
| `gyro_bias_rw` | rad/s²/√Hz | 1.0e-5 | Gyroscope bias random walk |
| `accel_bias_rw` | m/s³/√Hz | 1.0e-4 | Accelerometer bias random walk |

### `CameraCalibration`

One camera in the bundle:

| Field | Type | Description |
|---|---|---|
| `label` | `string` | Human-readable name, e.g. `"cam0"` |
| `intrinsics` | `CameraIntrinsics` | Pinhole model + distortion |
| `imu_T_camera` | `SensorExtrinsic` | Camera → IMU transform |
| `time_offset_ns` | `int64_t` | `camera_ts = imu_ts + offset` |

### `CameraIntrinsics`

| Field | Type | Description |
|---|---|---|
| `fx`, `fy` | `double` | Focal lengths (pixels) |
| `cx`, `cy` | `double` | Principal point (pixels) |
| `width`, `height` | `uint32_t` | Image resolution |
| `distortion_model` | `DistortionModel` | `None`, `RadialTangential`, `Equidistant`, `FieldOfView` |
| `distortion_coeffs` | `array<double,8>` | Interpretation depends on model |

Distortion coefficient layout:

| Model | Coefficients |
|---|---|
| `RadialTangential` | `[k1, k2, p1, p2, k3, 0, 0, 0]` |
| `Equidistant` | `[k1, k2, k3, k4, 0, 0, 0, 0]` |
| `FieldOfView` | `[ω, 0, 0, 0, 0, 0, 0, 0]` |

---

## YAML Format

`CalibrationBundle` reads and writes this YAML schema:

```yaml
imu_T_lidar:
  rotation: [1.0, 0.0, 0.0, 0.0]     # [w, x, y, z] quaternion
  translation: [0.0, 0.0, 0.28]      # metres

refine_imu_T_lidar: true              # allow SLAM to refine online

imu_noise:
  gyro_noise: 8.0e-4                  # rad/s/√Hz
  accel_noise: 1.0e-2                 # m/s²/√Hz
  gyro_bias_rw: 1.0e-5               # rad/s²/√Hz
  accel_bias_rw: 1.0e-4              # m/s³/√Hz

cameras:
  - label: "cam0"
    intrinsics:
      fx: 461.6
      fy: 460.3
      cx: 362.7
      cy: 248.1
      width: 752
      height: 480
      distortion_model: radtan
      distortion_coeffs: [-0.28, 0.07, 0.0, 0.0, 0.0]
    imu_T_camera:
      rotation: [0.707, 0.0, 0.707, 0.0]
      translation: [0.05, -0.02, 0.0]
    time_offset_ns: -8234
```

---

## Importing from Kalibr

If you already have Kalibr calibration results, import them directly:

```cpp
#include <thunderbird/calibration.h>

thunderbird::CalibrationBundle bundle;

// Import camera chain (labels, intrinsics, extrinsics, time offsets)
thunderbird::calib::importKalibrCamchain("camchain-imucam.yaml", bundle);

// Import IMU noise parameters
thunderbird::calib::importKalibrImu("imu.yaml", bundle);

// Or import both at once
thunderbird::calib::importKalibrFull("camchain-imucam.yaml", "imu.yaml", bundle);

// Save in Thunderbird SDK format
bundle.save_yaml("calibration.yaml");
```

Conversion notes:
- Kalibr's `T_cam_imu` (4×4 matrix) is inverted to `imu_T_camera`
- Kalibr `timeshift_cam_imu` (seconds) converts to `time_offset_ns`
- Kalibr distortion model names (`radtan`, `equidistant`, `fov`) map
  to `DistortionModel` enum values

A complete example is in `examples/import_kalibr_demo.cpp`:

```bash
./build/examples/import_kalibr_demo camchain.yaml imu.yaml output.yaml
```

---

## Offline Calibration Solvers

### Rigid Transform (Horn's SVD)

Finds the optimal rotation and translation from corresponding 3D point
pairs:

```cpp
#include "calib/rigid_transform.h"

double src[] = {0,0,0,  1,0,0,  0,1,0,  0,0,1};
double tgt[] = {3,4,5,  4,4,5,  3,5,5,  3,4,6};

auto result = thunderbird::calib::solveRigidTransform(src, tgt, 4);
// result.transform, result.rmse, result.valid

// With RANSAC outlier rejection
auto robust = thunderbird::calib::solveRigidTransformRANSAC(
    src, tgt, 100,
    /*threshold=*/0.05,   // inlier distance (metres)
    /*max_iter=*/200,
    /*inlier_ratio=*/0.5);
```

### LiDAR-IMU Calibration (BALM)

Estimates `imu_T_lidar` from a collection of LiDAR scans with known IMU
poses.  Uses voxel-based multi-scale feature extraction and Ceres
optimization:

```cpp
#include "calib/lidar_imu_calib.h"

// Check if Ceres backend is available
if (!thunderbird::calib::isLidarImuCalibAvailable()) {
    // Build with -DTHUNDERBIRD_HAS_CERES=ON to enable
}

std::vector<thunderbird::calib::CalibFrame> frames;
// ... fill frames with points + IMU poses ...

thunderbird::calib::LidarImuCalibConfig cfg;
cfg.num_rounds = 20;
cfg.voxel_size = 1.0;

auto result = thunderbird::calib::calibrateLidarImu(
    frames, initial_extrinsic, cfg);

if (result.converged) {
    bundle.imu_T_lidar = result.imu_T_lidar;
}
```

Configuration parameters:

| Parameter | Default | Description |
|---|---|---|
| `num_rounds` | 20 | Optimization rounds |
| `voxel_size` | 1.0 | Initial voxel size (metres) |
| `max_octree_depth` | 5 | Maximum subdivision depth |
| `min_points_per_voxel` | 20 | Minimum points per voxel |
| `surface_eigen_ratio` | 16.0 | Eigenvalue ratio for surface features |
| `corner_eigen_ratio` | 9.0 | Eigenvalue ratio for corner features |
| `solver_max_iterations` | 30 | Ceres iterations per round |
| `downsample_size` | 0.2 | Voxel downsample (0 = disabled) |

### LiDAR-Camera Calibration (Edge Alignment)

Estimates `camera_T_lidar` by aligning projected LiDAR points with image
edges.  **No external dependencies** (no OpenCV, PCL, or Eigen required):

```cpp
#include "calib/lidar_camera_calib.h"

thunderbird::calib::GrayImage edge_img{gray_ptr, 752, 480};
thunderbird::calib::LidarCameraCalibConfig cfg;
cfg.num_stages = 4;
cfg.init_rot_range_deg = 1.0;
cfg.edge_threshold = 30.0;

auto result = thunderbird::calib::calibrateLidarCamera(
    points, n_points, edge_img, intrinsics, initial, cfg);

if (result.converged) {
    // result.camera_T_lidar, result.score, result.edge_matches
}
```

---

## Online Extrinsic Refiner

The `OnlineRefiner` runs continuously during SLAM operations, using
ground-plane extraction (PCA) to estimate mounting corrections:

```cpp
#include "calib/online_refiner.h"

thunderbird::calib::OnlineRefinerConfig cfg;
cfg.ground_max_height = -0.5;    // metres
cfg.min_ground_points = 100;
cfg.max_correction_deg = 2.0;    // warning threshold

thunderbird::calib::OnlineRefiner refiner(cfg);

// Feed each LiDAR scan
refiner.processFrame(points, n_points);

auto corr = refiner.correction();
// corr.rotation — accumulated correction quaternion
// corr.height   — estimated mount height above ground
// corr.warning  — true if correction exceeds safety thresholds
```

---

## B-Spline IMU Interpolation

The calibration subsystem includes a uniform cubic B-spline for smooth IMU
interpolation at arbitrary timestamps.  This is used by the SLAM time-sync
engine for scan-boundary interpolation:

```cpp
#include "calib/bspline.h"

thunderbird::calib::UniformCubicBSpline spline(6); // 6-D (accel + gyro)
spline.initFromData(t0, t1, imu_data, n_samples);

double result[6];
spline.eval(t_query, result);                     // value
spline.evalDerivative(t_query, 1, result);        // 1st derivative
spline.evalDerivative(t_query, 2, result);        // 2nd derivative (acceleration)
```

Enable B-spline boundary interpolation in the SLAM time-sync engine:

```cpp
SlamTimeSyncConfig cfg;
cfg.use_bspline_interpolation = true;  // default: false (linear)
```

This is beneficial at low IMU rates (< 200 Hz) where linear interpolation
can introduce discontinuities at scan boundaries.

---

## Converter Tools

### `.tbrec` → ROS 2 Bag

Convert a `.tbrec` recording to a ROS 2 bag with standard message types:

```bash
python python/tools/tbrec_to_rosbag.py recording.tbrec -o output_bag
```

Published topics:

| Topic | Message Type |
|---|---|
| `/thunderbird/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `/thunderbird/imu/data` | `sensor_msgs/msg/Imu` |
| `/thunderbird/camera/image` | `sensor_msgs/msg/Image` |

Requires: `pip install rosbag2-py` (falls back to individual CDR files
if `rosbag2_py` is unavailable).

### `.tbrec` → PCD + PNG

Extract individual LiDAR and camera frames for offline tools like
OpenCalib:

```bash
python python/tools/tbrec_to_pcd.py recording.tbrec -o output_dir
python python/tools/tbrec_to_pcd.py recording.tbrec --binary-pcd
```

Output structure:

```
output_dir/
├── lidar/          # .pcd files (ASCII or binary)
├── camera/         # .png files (or .pgm/.ppm/.bin fallback)
├── imu/            # .csv files (one per sample)
└── timestamps.csv  # index: sensor, index, timestamp_ns, path
```

Requires only Python stdlib.  Pillow is optional (for PNG compression;
otherwise falls back to PGM/PPM).

---

## SLAMD Configuration Profiles

Pre-tuned calibration and odometry profiles are in `slamd/config/`:

| Profile | File | Use Case |
|---|---|---|
| Car | `slamd/config/car.yaml` | Autonomous vehicles: low angular rate, high speed, roof-mounted LiDAR |
| Drone | `slamd/config/drone.yaml` | Quadrotors: high angular rate, high vibration, compact mount |

Key differences:

| Parameter | Car | Drone |
|---|---|---|
| LiDAR rate | 10 Hz | 10 Hz |
| IMU rate | 200 Hz | 400 Hz |
| Gyro noise | 8.0e-4 rad/s/√Hz | 1.6e-3 rad/s/√Hz |
| Accel noise | 1.0e-2 m/s²/√Hz | 2.0e-2 m/s²/√Hz |
| Map radius | 150 m | 60 m |
| Deskew substeps | 1 | 2 |
| Mount height | 0.28 m | -0.03 m |

---

## Further Reading

| Topic | Link |
|---|---|
| Quick Start | [getting-started/quick-start.md](../getting-started/quick-start.md) |
| Time Synchronization | [guides/time-synchronization.md](time-synchronization.md) |
| Recording & Playback | [guides/recording-playback.md](recording-playback.md) |
| Advanced Configuration | [guides/advanced-configuration.md](advanced-configuration.md) |
| API Reference | [api/overview.md](../api/overview.md) |
| Clock Sync Design | [CLOCK_SYNC_DESIGN.md](../CLOCK_SYNC_DESIGN.md) |
