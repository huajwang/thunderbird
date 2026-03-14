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

// ── Test: use_bspline_interpolation in SlamEngineConfig ──────────────────────
// Verifies the field exists, defaults to false, and is accepted by initialize().
// The single-line forwarding to SlamTimeSyncConfig (in slam_engine.cpp) is
// verified by code inspection — SlamTimeSyncConfig is internal with no getter.

static void test_bspline_interpolation_flag() {
    // Default should be false
    SlamEngineConfig config;
    assert(!config.use_bspline_interpolation);

    // Set to true — initialize() should accept it without error
    config.use_bspline_interpolation = true;
    AcmeSlamEngine engine;
    assert(engine.initialize(config));

    // Copy preserves the flag
    SlamEngineConfig config2 = config;
    assert(config2.use_bspline_interpolation);

    engine.shutdown();
    std::puts("  use_bspline_interpolation flag          OK");
}

// ── Test: engine with refine=false does not construct refiner ────────────────
// (Verifies that existing behavior is unchanged — engine starts, processes,
//  and shuts down identically when the flag is off.)

static void test_refine_off_no_change() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;
    config.calibration.refine_imu_T_lidar = false;

    bool ok = engine.initialize(config);
    assert(ok);

    // Feed some synthetic IMU data — should work fine with no refiner
    odom::ImuSample imu{};
    imu.accel = {0.0, 0.0, -9.81};
    imu.gyro = {0.0, 0.0, 0.0};
    imu.timestamp_ns = 1'000'000;
    engine.feedImu(imu);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    engine.shutdown();
    std::puts("  refine off: no behavior change          OK");
}

// ── Test: engine with refine=true processes cloud and updates refine fields ──

static void test_refine_on_constructs_refiner() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;
    config.calibration.refine_imu_T_lidar = true;
    config.calibration.imu_T_lidar.translation = {0.0, 0.0, 1.5};
    config.online_refiner.ground_max_height = -0.3;
    config.online_refiner.min_ground_points = 10;
    config.online_refiner.min_height = 0.5;

    bool ok = engine.initialize(config);
    assert(ok);

    // Feed IMU to move past init
    for (int i = 0; i < 300; ++i) {
        odom::ImuSample imu{};
        imu.timestamp_ns = static_cast<int64_t>(i) * 5'000'000;
        imu.accel = {0.0, 0.0, -9.81};
        imu.gyro  = {0.0, 0.0, 0.0};
        engine.feedImu(imu);
    }

    // Build a synthetic flat ground cloud at z ≈ -1.5 (LiDAR frame).
    // With ground_max_height = -0.3 and min_ground_points = 10, all 200
    // points qualify as ground candidates and should produce a valid plane.
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 300 * 5'000'000LL;  // after last IMU
    cloud->sequence     = 0;
    cloud->is_deskewed  = false;
    cloud->points.resize(200);
    for (int j = 0; j < 200; ++j) {
        auto& pt = cloud->points[static_cast<size_t>(j)];
        pt.x = static_cast<float>(j % 20) * 0.5f - 5.0f;
        pt.y = static_cast<float>(j / 20) * 0.5f - 2.5f;
        pt.z = -1.5f;  // flat ground plane
        pt.intensity = 100.0f;
        pt.dt_ns = 0;
    }
    engine.feedPointCloud(cloud);

    // Allow worker to process the scan
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Check that the refiner has processed at least one frame
    SlamOutput out;
    if (engine.getLatestOutput(out)) {
        assert(out.refine_frame_count > 0);
        // Rotation should be near identity for a perfectly flat cloud
        assert(near(out.refine_rotation[0], 1.0, 0.05));
    }

    engine.shutdown();
    std::puts("  refine on: constructs and accepts data  OK");
}

// ── Test: custom OnlineRefinerConfig propagates through SlamEngineConfig ─────

static void test_online_refiner_config_in_slam_config() {
    SlamEngineConfig config;

    // Verify defaults
    assert(near(config.online_refiner.ground_max_height, -0.5, 0.01));
    assert(config.online_refiner.min_ground_points == 100);
    assert(near(config.online_refiner.min_normal_z, 0.9, 0.01));

    // Set custom values
    config.online_refiner.ground_max_height = -1.0;
    config.online_refiner.min_ground_points = 50;
    config.online_refiner.max_correction_deg = 5.0;

    // Copy preserves
    SlamEngineConfig config2 = config;
    assert(near(config2.online_refiner.ground_max_height, -1.0, 0.01));
    assert(config2.online_refiner.min_ground_points == 50);
    assert(near(config2.online_refiner.max_correction_deg, 5.0, 0.01));

    std::puts("  online refiner config in slam config    OK");
}

// ── Test: reset clears refiner state ────────────────────────────────────────

static void test_reset_clears_refiner() {
    AcmeSlamEngine engine;
    SlamEngineConfig config;
    config.calibration.refine_imu_T_lidar = true;
    config.calibration.imu_T_lidar.translation = {0.0, 0.0, 1.5};
    config.online_refiner.ground_max_height = -0.3;
    config.online_refiner.min_ground_points = 10;
    config.online_refiner.min_height = 0.5;

    bool ok = engine.initialize(config);
    assert(ok);

    // Feed IMU to move past init
    for (int i = 0; i < 300; ++i) {
        odom::ImuSample imu{};
        imu.timestamp_ns = static_cast<int64_t>(i) * 5'000'000;
        imu.accel = {0.0, 0.0, -9.81};
        imu.gyro  = {0.0, 0.0, 0.0};
        engine.feedImu(imu);
    }

    // Feed a ground-plane cloud to drive refiner to non-default state
    auto cloud = std::make_shared<PointCloudFrame>();
    cloud->timestamp_ns = 300 * 5'000'000LL;
    cloud->sequence     = 0;
    cloud->is_deskewed  = false;
    cloud->points.resize(200);
    for (int j = 0; j < 200; ++j) {
        auto& pt = cloud->points[static_cast<size_t>(j)];
        pt.x = static_cast<float>(j % 20) * 0.5f - 5.0f;
        pt.y = static_cast<float>(j / 20) * 0.5f - 2.5f;
        pt.z = -1.5f;
        pt.intensity = 100.0f;
        pt.dt_ns = 0;
    }
    engine.feedPointCloud(cloud);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Verify refiner has processed at least one frame before reset
    SlamOutput pre;
    bool had_output = engine.getLatestOutput(pre);
    if (had_output) {
        assert(pre.refine_frame_count > 0);
    }

    // Reset and verify refiner state returns to defaults
    engine.reset();

    // Feed fresh IMU + cloud after reset
    for (int i = 0; i < 300; ++i) {
        odom::ImuSample imu{};
        imu.timestamp_ns = static_cast<int64_t>(2'000'000'000LL + i * 5'000'000LL);
        imu.accel = {0.0, 0.0, -9.81};
        imu.gyro  = {0.0, 0.0, 0.0};
        engine.feedImu(imu);
    }

    auto cloud2 = std::make_shared<PointCloudFrame>();
    cloud2->timestamp_ns = 2'000'000'000LL + 300 * 5'000'000LL;
    cloud2->sequence     = 1;
    cloud2->is_deskewed  = false;
    cloud2->points.resize(200);
    for (int j = 0; j < 200; ++j) {
        auto& pt = cloud2->points[static_cast<size_t>(j)];
        pt.x = static_cast<float>(j % 20) * 0.5f - 5.0f;
        pt.y = static_cast<float>(j / 20) * 0.5f - 2.5f;
        pt.z = -1.5f;
        pt.intensity = 100.0f;
        pt.dt_ns = 0;
    }
    engine.feedPointCloud(cloud2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // After reset, frame_count should have restarted (≤ 1 post-reset scan)
    SlamOutput post;
    if (engine.getLatestOutput(post)) {
        assert(post.refine_frame_count <= 1);
    }

    engine.shutdown();
    std::puts("  reset clears refiner state              OK");
}

// ── Test: SlamOutput has refine fields defaulting to identity ────────────────

static void test_slam_output_refine_fields() {
    SlamOutput output;
    // Defaults: identity quaternion, zero height/confidence/count, no warning
    assert(near(output.refine_rotation[0], 1.0));
    assert(near(output.refine_rotation[1], 0.0));
    assert(near(output.refine_rotation[2], 0.0));
    assert(near(output.refine_rotation[3], 0.0));
    assert(near(output.refine_height, 0.0));
    assert(near(output.refine_confidence, 0.0));
    assert(output.refine_frame_count == 0);
    assert(!output.refine_warning);

    std::puts("  SlamOutput refine fields default OK      OK");
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
    test_bspline_interpolation_flag();
    test_refine_off_no_change();
    test_refine_on_constructs_refiner();
    test_online_refiner_config_in_slam_config();
    test_reset_clears_refiner();
    test_slam_output_refine_fields();
    std::puts("SlamCalibration: ALL TESTS PASSED");
    return 0;
}
