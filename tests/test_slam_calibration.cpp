// ─────────────────────────────────────────────────────────────────────────────
// Test — SlamEngineConfig ↔ CalibrationBundle integration
// ─────────────────────────────────────────────────────────────────────────────
//
// Validates that CalibrationBundle embedded in SlamEngineConfig propagates
// correctly through AcmeSlamEngine::initialize().
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/odom/slam_engine.h"
#include "thunderbird/calibration.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <thread>

using namespace thunderbird;
using namespace thunderbird::odom;

static constexpr double kEps = 1e-9;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Test: default SlamEngineConfig has default CalibrationBundle ─────────────

static void test_default_config_has_default_calibration() {
    SlamEngineConfig config;

    // CalibrationBundle defaults: identity extrinsic, default noise, no cameras
    assert(config.calibration.imu_T_lidar.is_identity());
    assert(!config.calibration.refine_imu_T_lidar);
    assert(config.calibration.cameras.empty());
    assert(near(config.calibration.imu_noise.gyro_noise, 1.0e-3));
    assert(near(config.calibration.imu_noise.accel_noise, 1.0e-2));
    assert(near(config.calibration.imu_noise.gyro_bias_rw, 1.0e-5));
    assert(near(config.calibration.imu_noise.accel_bias_rw, 1.0e-4));

    std::puts("  default config has default calibration  OK");
}

// ── Test: custom noise values in SlamEngineConfig ───────────────────────────

static void test_custom_noise_in_config() {
    SlamEngineConfig config;
    config.calibration.imu_noise.gyro_noise    = 2.5e-3;
    config.calibration.imu_noise.accel_noise   = 5.0e-2;
    config.calibration.imu_noise.gyro_bias_rw  = 3.3e-5;
    config.calibration.imu_noise.accel_bias_rw = 7.7e-4;

    assert(near(config.calibration.imu_noise.gyro_noise, 2.5e-3));
    assert(near(config.calibration.imu_noise.accel_noise, 5.0e-2));
    assert(near(config.calibration.imu_noise.gyro_bias_rw, 3.3e-5));
    assert(near(config.calibration.imu_noise.accel_bias_rw, 7.7e-4));

    std::puts("  custom noise values in config           OK");
}

// ── Test: non-identity extrinsic in SlamEngineConfig ────────────────────────

static void test_nonidentity_extrinsic_in_config() {
    SlamEngineConfig config;
    config.calibration.imu_T_lidar.translation = {0.1, -0.05, 0.03};
    // 45° around Z axis
    config.calibration.imu_T_lidar.rotation = {
        std::cos(std::numbers::pi / 8), 0.0, 0.0, std::sin(std::numbers::pi / 8)};

    assert(!config.calibration.imu_T_lidar.is_identity());
    assert(near(config.calibration.imu_T_lidar.translation[0], 0.1));
    assert(near(config.calibration.imu_T_lidar.translation[1], -0.05));
    assert(near(config.calibration.imu_T_lidar.translation[2], 0.03));

    std::puts("  non-identity extrinsic in config        OK");
}

// ── Test: cameras in SlamEngineConfig ───────────────────────────────────────

static void test_cameras_in_config() {
    SlamEngineConfig config;

    CameraCalibration cam;
    cam.label = "cam0";
    cam.intrinsics.fx = 500; cam.intrinsics.fy = 500;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.imu_T_camera.translation = {0.05, -0.03, 0.01};
    cam.time_offset_ns = -2000;
    config.calibration.cameras.push_back(cam);

    assert(config.calibration.cameras.size() == 1);
    assert(config.calibration.cameras[0].label == "cam0");
    assert(config.calibration.cameras[0].intrinsics.valid());
    assert(near(config.calibration.cameras[0].imu_T_camera.translation[0], 0.05));
    assert(config.calibration.cameras[0].time_offset_ns == -2000);

    std::puts("  cameras in config                       OK");
}

// ── Test: refine_imu_T_lidar flag ───────────────────────────────────────────

static void test_refine_flag_in_config() {
    SlamEngineConfig config;
    assert(!config.calibration.refine_imu_T_lidar);  // default false

    config.calibration.refine_imu_T_lidar = true;
    assert(config.calibration.refine_imu_T_lidar);

    std::puts("  refine_imu_T_lidar flag                 OK");
}

// ── Test: engine initializes with default CalibrationBundle ─────────────────

static void test_engine_init_default_calibration() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;

    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::puts("  engine init default calibration         OK");
}

// ── Test: engine initializes with custom noise model ────────────────────────

static void test_engine_init_custom_noise() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;

    config.calibration.imu_noise.gyro_noise    = 5.0e-3;
    config.calibration.imu_noise.accel_noise   = 3.0e-2;
    config.calibration.imu_noise.gyro_bias_rw  = 2.0e-5;
    config.calibration.imu_noise.accel_bias_rw = 6.0e-4;

    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::puts("  engine init custom noise                OK");
}

// ── Test: engine initializes with non-identity extrinsic ────────────────────

static void test_engine_init_nonidentity_extrinsic() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;

    config.calibration.imu_T_lidar.translation = {0.2, -0.1, 0.05};
    config.calibration.imu_T_lidar.rotation = {
        std::cos(std::numbers::pi / 4), 0.0, 0.0, std::sin(std::numbers::pi / 4)};  // 90° yaw

    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::puts("  engine init non-identity extrinsic      OK");
}

// ── Test: engine initializes with refine flag set ───────────────────────────

static void test_engine_init_with_refine() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;

    config.calibration.refine_imu_T_lidar = true;
    config.calibration.imu_T_lidar.translation = {0.05, 0.0, 0.0};

    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::puts("  engine init with refine flag            OK");
}

// ── Test: engine initializes with full CalibrationBundle (noise + extrinsic + camera)

static void test_engine_init_full_calibration() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;

    // Non-default noise
    config.calibration.imu_noise.gyro_noise    = 1.2e-3;
    config.calibration.imu_noise.accel_noise   = 1.5e-2;
    config.calibration.imu_noise.gyro_bias_rw  = 8.0e-6;
    config.calibration.imu_noise.accel_bias_rw = 3.0e-4;

    // Non-identity extrinsic
    config.calibration.imu_T_lidar.translation = {0.1, -0.05, 0.03};
    config.calibration.refine_imu_T_lidar = true;

    // Camera
    CameraCalibration cam;
    cam.label = "front_camera";
    cam.intrinsics.fx = 600; cam.intrinsics.fy = 600;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.imu_T_camera.translation = {0.0, 0.05, -0.02};
    cam.time_offset_ns = -1500;
    config.calibration.cameras.push_back(cam);

    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::puts("  engine init full calibration            OK");
}

// ── Test: engine init with YAML-loaded CalibrationBundle ────────────────────

static void test_engine_init_from_yaml() {
    // Create a temp calibration YAML
    const auto unique_id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() /
                      ("test_slam_calib_" + unique_id + ".yaml");
    {
        std::ofstream f(path);
        f << "imu_T_lidar:\n";
        f << "  rotation: [0.9999, 0.0, 0.01, 0.0]\n";
        f << "  translation: [0.15, -0.02, 0.08]\n";
        f << "refine_imu_T_lidar: true\n";
        f << "imu_noise:\n";
        f << "  gyro_noise: 2.0e-3\n";
        f << "  accel_noise: 3.0e-2\n";
        f << "  gyro_bias_rw: 4.0e-5\n";
        f << "  accel_bias_rw: 5.0e-4\n";
    }

    SlamEngineConfig config;
    bool loaded = config.calibration.load_yaml(path.string());
    assert(loaded);

    // Verify the calibration was loaded into the config
    assert(near(config.calibration.imu_T_lidar.translation[0], 0.15, 1e-6));
    assert(near(config.calibration.imu_T_lidar.translation[1], -0.02, 1e-6));
    assert(near(config.calibration.imu_T_lidar.translation[2], 0.08, 1e-6));
    assert(config.calibration.refine_imu_T_lidar);
    assert(near(config.calibration.imu_noise.gyro_noise, 2.0e-3, 1e-8));
    assert(near(config.calibration.imu_noise.accel_noise, 3.0e-2, 1e-8));
    assert(near(config.calibration.imu_noise.gyro_bias_rw, 4.0e-5, 1e-8));
    assert(near(config.calibration.imu_noise.accel_bias_rw, 5.0e-4, 1e-8));

    // Engine should initialize successfully with YAML-loaded calibration
    AcmeSlamEngine engine;
    bool ok = engine.initialize(config);
    assert(ok);

    engine.shutdown();
    std::filesystem::remove(path);
    std::puts("  engine init from YAML calibration       OK");
}

// ── Test: CalibrationBundle config survives copy ────────────────────────────

static void test_config_copy_preserves_calibration() {
    SlamEngineConfig config1;
    config1.calibration.imu_noise.gyro_noise = 9.9e-3;
    config1.calibration.imu_T_lidar.translation = {1.0, 2.0, 3.0};
    config1.calibration.refine_imu_T_lidar = true;

    CameraCalibration cam;
    cam.label = "test_cam";
    cam.intrinsics.fx = 400;
    config1.calibration.cameras.push_back(cam);

    // Copy
    SlamEngineConfig config2 = config1;

    assert(near(config2.calibration.imu_noise.gyro_noise, 9.9e-3));
    assert(near(config2.calibration.imu_T_lidar.translation[0], 1.0));
    assert(near(config2.calibration.imu_T_lidar.translation[1], 2.0));
    assert(near(config2.calibration.imu_T_lidar.translation[2], 3.0));
    assert(config2.calibration.refine_imu_T_lidar);
    assert(config2.calibration.cameras.size() == 1);
    assert(config2.calibration.cameras[0].label == "test_cam");

    std::puts("  config copy preserves calibration       OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("SlamCalibration:");
    test_default_config_has_default_calibration();
    test_custom_noise_in_config();
    test_nonidentity_extrinsic_in_config();
    test_cameras_in_config();
    test_refine_flag_in_config();
    test_engine_init_default_calibration();
    test_engine_init_custom_noise();
    test_engine_init_nonidentity_extrinsic();
    test_engine_init_with_refine();
    test_engine_init_full_calibration();
    test_engine_init_from_yaml();
    test_config_copy_preserves_calibration();
    std::puts("SlamCalibration: ALL TESTS PASSED");
    return 0;
}
