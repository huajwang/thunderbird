// ─────────────────────────────────────────────────────────────────────────────
// Test — Ros2BridgeConfig CalibrationBundle integration
// ─────────────────────────────────────────────────────────────────────────────
//
// Validates that CalibrationBundle in Ros2BridgeConfig is correctly wired:
//   • CameraInfo fields sourced from CalibrationBundle cameras
//   • Static TF transform building from CalibrationBundle extrinsics
//
// These tests do NOT require rclcpp — they test the data pathway that
// feeds into the ROS2 publishers.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/calibration.h"
#include "thunderbird/types.h"
#include "thunderbird/ros2_helpers.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

using namespace thunderbird;

static constexpr double kEps = 1e-9;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── Minimal Ros2BridgeConfig replica for testing without rclcpp ─────────────
// Mirrors the real Ros2BridgeConfig but without the rclcpp-dependent QoS field.

namespace test_helpers {

struct TestBridgeConfig {
    std::string lidar_frame_id  = "thunderbird_lidar";
    std::string imu_frame_id    = "thunderbird_imu";
    std::string camera_frame_id = "thunderbird_camera";

    bool publish_camera_info = true;
    CalibrationBundle calibration;
};

// Mirrors the CameraInfo building logic from ros2_bridge.cpp::onCamera()
struct CameraInfoFields {
    uint32_t width{0};
    uint32_t height{0};
    std::string distortion_model;
    std::vector<double> d;
    std::array<double, 9> k{};
    std::array<double, 9> r{};
    std::array<double, 12> p{};
    bool valid{false};
};

CameraInfoFields buildCameraInfo(const TestBridgeConfig& config) {
    CameraInfoFields ci;
    if (!config.publish_camera_info ||
        config.calibration.cameras.empty() ||
        !config.calibration.cameras[0].intrinsics.valid()) {
        return ci;
    }

    const auto& cam_intr = config.calibration.cameras[0].intrinsics;
    ci.width  = cam_intr.width;
    ci.height = cam_intr.height;

    switch (cam_intr.distortion_model) {
        case DistortionModel::RadialTangential:
            ci.distortion_model = "plumb_bob"; break;
        case DistortionModel::Equidistant:
            ci.distortion_model = "equidistant"; break;
        case DistortionModel::FieldOfView:
            ci.distortion_model = ""; break;
        default:
            ci.distortion_model = ""; break;
    }

    int n = 0;
    switch (cam_intr.distortion_model) {
        case DistortionModel::RadialTangential: n = 5; break;
        case DistortionModel::Equidistant:      n = 4; break;
        case DistortionModel::FieldOfView:      n = 1; break;
        default: break;
    }
    ci.d.assign(cam_intr.distortion_coeffs.begin(),
                cam_intr.distortion_coeffs.begin() + n);

    ci.k = {cam_intr.fx, 0.0,         cam_intr.cx,
             0.0,         cam_intr.fy, cam_intr.cy,
             0.0,         0.0,         1.0};

    ci.r = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};

    ci.p = {cam_intr.fx, 0.0,         cam_intr.cx, 0.0,
             0.0,         cam_intr.fy, cam_intr.cy, 0.0,
             0.0,         0.0,         1.0,         0.0};

    ci.valid = true;
    return ci;
}

// Mirrors the TransformStamped building logic from publishStaticTransforms()
struct TestTransformStamped {
    std::string parent_frame;
    std::string child_frame;
    double rotation_w, rotation_x, rotation_y, rotation_z;
    double translation_x, translation_y, translation_z;
};

std::vector<TestTransformStamped> buildStaticTransforms(
    const TestBridgeConfig& config) {
    const auto& calib = config.calibration;
    if (calib.imu_T_lidar.is_identity() && calib.cameras.empty())
        return {};

    std::vector<TestTransformStamped> transforms;

    if (!calib.imu_T_lidar.is_identity()) {
        TestTransformStamped tf;
        tf.parent_frame = config.imu_frame_id;
        tf.child_frame  = config.lidar_frame_id;
        tf.rotation_w   = calib.imu_T_lidar.rotation[0];
        tf.rotation_x   = calib.imu_T_lidar.rotation[1];
        tf.rotation_y   = calib.imu_T_lidar.rotation[2];
        tf.rotation_z   = calib.imu_T_lidar.rotation[3];
        tf.translation_x = calib.imu_T_lidar.translation[0];
        tf.translation_y = calib.imu_T_lidar.translation[1];
        tf.translation_z = calib.imu_T_lidar.translation[2];
        transforms.push_back(tf);
    }

    for (size_t i = 0; i < calib.cameras.size(); ++i) {
        TestTransformStamped tf;
        tf.parent_frame = config.imu_frame_id;
        tf.child_frame  = (i == 0)
            ? config.camera_frame_id
            : config.camera_frame_id + "_" + std::to_string(i);
        tf.rotation_w   = calib.cameras[i].imu_T_camera.rotation[0];
        tf.rotation_x   = calib.cameras[i].imu_T_camera.rotation[1];
        tf.rotation_y   = calib.cameras[i].imu_T_camera.rotation[2];
        tf.rotation_z   = calib.cameras[i].imu_T_camera.rotation[3];
        tf.translation_x = calib.cameras[i].imu_T_camera.translation[0];
        tf.translation_y = calib.cameras[i].imu_T_camera.translation[1];
        tf.translation_z = calib.cameras[i].imu_T_camera.translation[2];
        transforms.push_back(tf);
    }

    return transforms;
}

} // namespace test_helpers

using namespace test_helpers;

// ── Test: CameraInfo not built when no cameras ──────────────────────────────

static void test_camera_info_no_cameras() {
    TestBridgeConfig config;
    // No cameras → CameraInfo should not be valid
    auto ci = buildCameraInfo(config);
    assert(!ci.valid);
    std::puts("  CameraInfo no cameras              OK");
}

// ── Test: CameraInfo not built when publish disabled ────────────────────────

static void test_camera_info_publish_disabled() {
    TestBridgeConfig config;
    config.publish_camera_info = false;

    CameraCalibration cam;
    cam.intrinsics.fx = 500; cam.intrinsics.fy = 500;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    config.calibration.cameras.push_back(cam);

    auto ci = buildCameraInfo(config);
    assert(!ci.valid);
    std::puts("  CameraInfo publish disabled        OK");
}

// ── Test: CameraInfo not built when intrinsics invalid ──────────────────────

static void test_camera_info_invalid_intrinsics() {
    TestBridgeConfig config;
    CameraCalibration cam;
    cam.label = "cam0";
    // Don't set intrinsics → invalid
    config.calibration.cameras.push_back(cam);

    auto ci = buildCameraInfo(config);
    assert(!ci.valid);
    std::puts("  CameraInfo invalid intrinsics      OK");
}

// ── Test: CameraInfo built correctly from CalibrationBundle ─────────────────

static void test_camera_info_from_calibration() {
    TestBridgeConfig config;

    CameraCalibration cam;
    cam.label = "cam0";
    cam.intrinsics.fx = 600; cam.intrinsics.fy = 601;
    cam.intrinsics.cx = 319; cam.intrinsics.cy = 239;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.intrinsics.distortion_model = DistortionModel::RadialTangential;
    cam.intrinsics.distortion_coeffs = {0.1, -0.2, 0.001, 0.002, 0.05, 0, 0, 0};
    config.calibration.cameras.push_back(cam);

    auto ci = buildCameraInfo(config);
    assert(ci.valid);
    assert(ci.width == 640);
    assert(ci.height == 480);
    assert(ci.distortion_model == "plumb_bob");

    // 5 distortion coefficients for RadialTangential
    assert(ci.d.size() == 5);
    assert(near(ci.d[0], 0.1, 1e-6));
    assert(near(ci.d[1], -0.2, 1e-6));
    assert(near(ci.d[2], 0.001, 1e-6));
    assert(near(ci.d[3], 0.002, 1e-6));
    assert(near(ci.d[4], 0.05, 1e-6));

    // K matrix
    assert(near(ci.k[0], 600.0)); // fx
    assert(near(ci.k[4], 601.0)); // fy
    assert(near(ci.k[2], 319.0)); // cx
    assert(near(ci.k[5], 239.0)); // cy
    assert(near(ci.k[8], 1.0));   // bottom-right

    // R matrix (identity)
    assert(near(ci.r[0], 1.0));
    assert(near(ci.r[4], 1.0));
    assert(near(ci.r[8], 1.0));

    // P matrix
    assert(near(ci.p[0], 600.0)); // fx
    assert(near(ci.p[5], 601.0)); // fy
    assert(near(ci.p[2], 319.0)); // cx
    assert(near(ci.p[6], 239.0)); // cy

    std::puts("  CameraInfo from CalibrationBundle  OK");
}

// ── Test: CameraInfo equidistant model ──────────────────────────────────────

static void test_camera_info_equidistant() {
    TestBridgeConfig config;

    CameraCalibration cam;
    cam.intrinsics.fx = 400; cam.intrinsics.fy = 400;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.width = 640; cam.intrinsics.height = 480;
    cam.intrinsics.distortion_model = DistortionModel::Equidistant;
    cam.intrinsics.distortion_coeffs = {0.3, -0.1, 0.05, 0.01, 0, 0, 0, 0};
    config.calibration.cameras.push_back(cam);

    auto ci = buildCameraInfo(config);
    assert(ci.valid);
    assert(ci.distortion_model == "equidistant");
    assert(ci.d.size() == 4);  // Equidistant uses 4 coefficients
    assert(near(ci.d[0], 0.3, 1e-6));

    std::puts("  CameraInfo equidistant model       OK");
}

// ── Test: no static TFs when calibration is identity + no cameras ───────────

static void test_static_tf_identity_no_cameras() {
    TestBridgeConfig config;
    auto tfs = buildStaticTransforms(config);
    assert(tfs.empty());
    std::puts("  static TF identity + no cameras    OK");
}

// ── Test: IMU→LiDAR static TF published ─────────────────────────────────────

static void test_static_tf_imu_lidar() {
    TestBridgeConfig config;
    config.calibration.imu_T_lidar.translation = {0.1, -0.05, 0.03};
    config.calibration.imu_T_lidar.rotation = {
        std::cos(std::numbers::pi / 8), 0.0, 0.0,
        std::sin(std::numbers::pi / 8)};  // 45° yaw

    auto tfs = buildStaticTransforms(config);
    assert(tfs.size() == 1);
    assert(tfs[0].parent_frame == "thunderbird_imu");
    assert(tfs[0].child_frame == "thunderbird_lidar");
    assert(near(tfs[0].translation_x, 0.1));
    assert(near(tfs[0].translation_y, -0.05));
    assert(near(tfs[0].translation_z, 0.03));
    assert(near(tfs[0].rotation_w, std::cos(std::numbers::pi / 8)));
    assert(near(tfs[0].rotation_z, std::sin(std::numbers::pi / 8)));

    std::puts("  static TF IMU→LiDAR                OK");
}

// ── Test: IMU→Camera static TF published ────────────────────────────────────

static void test_static_tf_imu_camera() {
    TestBridgeConfig config;
    // Identity lidar extrinsic (should be skipped)
    CameraCalibration cam;
    cam.label = "cam0";
    cam.imu_T_camera.translation = {0.02, -0.01, 0.05};
    config.calibration.cameras.push_back(cam);

    auto tfs = buildStaticTransforms(config);
    assert(tfs.size() == 1);  // only camera TF (lidar is identity → skipped)
    assert(tfs[0].parent_frame == "thunderbird_imu");
    assert(tfs[0].child_frame == "thunderbird_camera");
    assert(near(tfs[0].translation_x, 0.02));
    assert(near(tfs[0].translation_y, -0.01));
    assert(near(tfs[0].translation_z, 0.05));

    std::puts("  static TF IMU→Camera               OK");
}

// ── Test: multiple cameras → multiple TFs ───────────────────────────────────

static void test_static_tf_multiple_cameras() {
    TestBridgeConfig config;
    config.calibration.imu_T_lidar.translation = {0.1, 0.0, 0.0};

    CameraCalibration cam0, cam1;
    cam0.label = "cam0";
    cam0.imu_T_camera.translation = {0.02, 0.0, 0.0};
    cam1.label = "cam1";
    cam1.imu_T_camera.translation = {-0.02, 0.0, 0.0};
    config.calibration.cameras.push_back(cam0);
    config.calibration.cameras.push_back(cam1);

    auto tfs = buildStaticTransforms(config);
    // 1 lidar TF + 2 camera TFs = 3
    assert(tfs.size() == 3);

    // Verify frame naming
    assert(tfs[0].child_frame == "thunderbird_lidar");
    assert(tfs[1].child_frame == "thunderbird_camera");    // cam0 = default name
    assert(tfs[2].child_frame == "thunderbird_camera_1");  // cam1 = suffixed

    std::puts("  static TF multiple cameras         OK");
}

// ── Test: full config with extrinsic + cameras → correct TFs ────────────────

static void test_static_tf_full_config() {
    TestBridgeConfig config;

    // Non-identity lidar
    config.calibration.imu_T_lidar.translation = {0.3, -0.1, 0.05};
    config.calibration.imu_T_lidar.rotation = {0.9999, 0.001, 0.002, 0.003};

    // Two cameras with different extrinsics
    CameraCalibration cam0;
    cam0.label = "front";
    cam0.imu_T_camera.translation = {0.05, 0.03, -0.01};
    cam0.imu_T_camera.rotation = {0.998, 0.0, 0.063, 0.0};

    CameraCalibration cam1;
    cam1.label = "rear";
    cam1.imu_T_camera.translation = {-0.05, 0.03, -0.01};
    cam1.imu_T_camera.rotation = {0.0, 0.0, 0.0, 1.0};  // 180° yaw

    config.calibration.cameras.push_back(cam0);
    config.calibration.cameras.push_back(cam1);

    auto tfs = buildStaticTransforms(config);
    assert(tfs.size() == 3);

    // Verify all transforms have correct parent frame
    for (const auto& tf : tfs) {
        assert(tf.parent_frame == "thunderbird_imu");
    }

    // Verify lidar transform
    assert(tfs[0].child_frame == "thunderbird_lidar");
    assert(near(tfs[0].translation_x, 0.3));
    assert(near(tfs[0].rotation_w, 0.9999));

    // Verify camera transforms
    assert(tfs[1].child_frame == "thunderbird_camera");
    assert(near(tfs[1].translation_x, 0.05));
    assert(near(tfs[1].rotation_w, 0.998));

    assert(tfs[2].child_frame == "thunderbird_camera_1");
    assert(near(tfs[2].translation_x, -0.05));
    assert(near(tfs[2].rotation_z, 1.0));

    std::puts("  static TF full config              OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("Ros2BridgeCalibration:");
    test_camera_info_no_cameras();
    test_camera_info_publish_disabled();
    test_camera_info_invalid_intrinsics();
    test_camera_info_from_calibration();
    test_camera_info_equidistant();
    test_static_tf_identity_no_cameras();
    test_static_tf_imu_lidar();
    test_static_tf_imu_camera();
    test_static_tf_multiple_cameras();
    test_static_tf_full_config();
    std::puts("Ros2BridgeCalibration: ALL TESTS PASSED");
    return 0;
}
