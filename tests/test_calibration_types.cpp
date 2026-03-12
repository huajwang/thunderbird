// ─────────────────────────────────────────────────────────────────────────────
// Test — Calibration Types (CameraIntrinsics, SensorExtrinsic, CalibrationBundle)
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/types.h"
#include "thunderbird/calibration.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

using namespace thunderbird;

static constexpr double kEps = 1e-9;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── CameraIntrinsics ────────────────────────────────────────────────────────

static void test_intrinsics_valid() {
    CameraIntrinsics ci;
    assert(!ci.valid());  // default is invalid

    ci.fx = 500; ci.fy = 500; ci.cx = 320; ci.cy = 240;
    ci.width = 640; ci.height = 480;
    assert(ci.valid());
    std::puts("  CameraIntrinsics::valid()    OK");
}

// ── SensorExtrinsic ─────────────────────────────────────────────────────────

static void test_extrinsic_identity() {
    SensorExtrinsic e;
    assert(e.is_identity());
    assert(near(e.rotation[0], 1.0));
    assert(near(e.rotation[1], 0.0));
    assert(near(e.translation[0], 0.0));
    std::puts("  SensorExtrinsic::is_identity()    OK");
}

static void test_extrinsic_inverse() {
    // Pure translation along X
    SensorExtrinsic e;
    e.translation = {1.0, 0.0, 0.0};
    auto inv = e.inverse();
    assert(near(inv.translation[0], -1.0));
    assert(near(inv.translation[1], 0.0));

    // Compose with inverse should yield identity
    auto id = e.compose(inv);
    assert(near(id.rotation[0], 1.0, 1e-6));
    assert(near(id.translation[0], 0.0, 1e-6));
    assert(near(id.translation[1], 0.0, 1e-6));
    assert(near(id.translation[2], 0.0, 1e-6));
    std::puts("  SensorExtrinsic::inverse()         OK");
}

static void test_extrinsic_compose() {
    // Two translations should add
    SensorExtrinsic a, b;
    a.translation = {1.0, 0.0, 0.0};
    b.translation = {0.0, 2.0, 0.0};
    auto c = a.compose(b);
    assert(near(c.translation[0], 1.0, 1e-6));
    assert(near(c.translation[1], 2.0, 1e-6));

    // 90° rotation around Z: q = [cos(45°), 0, 0, sin(45°)]
    SensorExtrinsic rot;
    rot.rotation = {std::cos(std::numbers::pi / 4), 0.0, 0.0,
                    std::sin(std::numbers::pi / 4)};
    rot.translation = {0.0, 0.0, 0.0};

    // Compose two 90° rotations = 180° rotation
    auto rot180 = rot.compose(rot);
    // 180° around Z: q ≈ [0, 0, 0, 1]
    assert(near(std::abs(rot180.rotation[3]), 1.0, 1e-6));
    std::puts("  SensorExtrinsic::compose()         OK");
}

// ── CalibrationBundle YAML I/O ──────────────────────────────────────────────

static void test_calibration_yaml_roundtrip() {
    CalibrationBundle bundle;
    bundle.imu_T_lidar.translation = {0.1, 0.2, 0.3};

    CameraCalibration cam;
    cam.label = "cam0";
    cam.intrinsics.fx = 500; cam.intrinsics.fy = 500;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.intrinsics.distortion_model = DistortionModel::RadialTangential;
    cam.intrinsics.distortion_coeffs = {0.1, -0.2, 0.001, 0.002, 0, 0, 0, 0};
    cam.imu_T_camera.translation = {0.05, -0.03, 0.01};
    cam.time_offset_ns = -5000;
    bundle.cameras.push_back(cam);

    bundle.imu_noise.gyro_noise = 1.5e-3;
    bundle.imu_noise.accel_noise = 2.0e-2;

    // Save
    const auto unique_id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() /
                      ("test_calib_roundtrip_" + unique_id + ".yaml");
    bool ok = bundle.save_yaml(path.string());
    assert(ok);

    // Load into fresh bundle
    CalibrationBundle loaded;
    ok = loaded.load_yaml(path.string());
    assert(ok);

    // Verify
    assert(near(loaded.imu_T_lidar.translation[0], 0.1, 1e-6));
    assert(near(loaded.imu_T_lidar.translation[1], 0.2, 1e-6));
    assert(near(loaded.imu_T_lidar.translation[2], 0.3, 1e-6));

    assert(loaded.cameras.size() == 1);
    assert(loaded.cameras[0].label == "cam0");
    assert(near(loaded.cameras[0].intrinsics.fx, 500.0, 1e-6));
    assert(loaded.cameras[0].intrinsics.width == 640);
    assert(loaded.cameras[0].intrinsics.distortion_model == DistortionModel::RadialTangential);
    assert(near(loaded.cameras[0].intrinsics.distortion_coeffs[0], 0.1, 1e-6));
    assert(loaded.cameras[0].time_offset_ns == -5000);

    assert(near(loaded.imu_noise.gyro_noise, 1.5e-3, 1e-8));
    assert(near(loaded.imu_noise.accel_noise, 2.0e-2, 1e-8));

    // Cleanup
    std::filesystem::remove(path);
    std::puts("  CalibrationBundle YAML roundtrip   OK");
}

static void test_calibration_derived_transforms() {
    CalibrationBundle bundle;
    // Identity imu_T_lidar
    // cam with 1m Z offset from IMU
    CameraCalibration cam;
    cam.label = "cam0";
    cam.intrinsics.fx = 500; cam.intrinsics.fy = 500;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.imu_T_camera.translation = {0.0, 0.0, 1.0};
    bundle.cameras.push_back(cam);

    // lidar_T_camera = imu_T_lidar^-1 * imu_T_camera
    auto ltc = bundle.lidar_T_camera(0);
    assert(near(ltc.translation[2], 1.0, 1e-6));

    // camera_T_lidar = lidar_T_camera^-1
    auto ctl = bundle.camera_T_lidar(0);
    assert(near(ctl.translation[2], -1.0, 1e-6));

    std::puts("  CalibrationBundle derived xforms   OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("CalibrationTypes:");
    test_intrinsics_valid();
    test_extrinsic_identity();
    test_extrinsic_inverse();
    test_extrinsic_compose();
    test_calibration_yaml_roundtrip();
    test_calibration_derived_transforms();
    std::puts("CalibrationTypes: ALL TESTS PASSED");
    return 0;
}
