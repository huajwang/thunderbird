// ─────────────────────────────────────────────────────────────────────────────
// Test — slamd::loadConfig calibration_file parsing
// ─────────────────────────────────────────────────────────────────────────────
//
// Validates that the calibration_file parsing logic from loadConfig()
// correctly loads CalibrationBundle from a config YAML.
//
// This test reimplements the calibration_file parsing logic found in
// slamd/src/acme_slamd.cpp::loadConfig() to test it independently
// without linking the full slamd daemon implementation.
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/calibration.h"
#include "thunderbird/odom/slam_engine.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace thunderbird;
using namespace thunderbird::odom;

static constexpr double kEps = 1e-9;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// Helper to create a temp directory with unique name
static std::filesystem::path makeTempDir(const std::string& prefix) {
    auto unique_id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    auto dir = std::filesystem::temp_directory_path() / (prefix + "_" + unique_id);
    std::filesystem::create_directories(dir);
    return dir;
}

// ── Standalone calibration_file parser ──────────────────────────────────────
// This mirrors the logic from slamd/src/acme_slamd.cpp::loadConfig()
// for the calibration_file key parsing.

struct SlamdTestConfig {
    SlamEngineConfig slam;
};

static bool parseCalibrationFile(const std::string& config_path,
                                  SlamdTestConfig& config,
                                  std::string& error) {
    std::ifstream fs(config_path);
    if (!fs.is_open()) {
        error = "Cannot open config file: " + config_path;
        return false;
    }

    std::string line;
    while (std::getline(fs, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        // Trim leading whitespace and require key at the start of the line.
        size_t first_non_ws = line.find_first_not_of(" \t");
        if (first_non_ws == std::string::npos) continue;
        std::string trimmed = line.substr(first_non_ws);
        constexpr const char* kCalibKey = "calibration_file:";
        constexpr size_t kCalibKeyLen = 17;
        if (trimmed.size() < kCalibKeyLen ||
            trimmed.compare(0, kCalibKeyLen, kCalibKey) != 0) continue;
        std::string val = trimmed.substr(kCalibKeyLen);
        // Trim whitespace and quotes.
        size_t vs = val.find_first_not_of(" \t\"'");
        if (vs == std::string::npos) continue;
        val = val.substr(vs);
        size_t ve = val.find_last_not_of(" \t\r\n\"'");
        if (ve != std::string::npos) val = val.substr(0, ve + 1);
        if (val.empty()) continue;

        // Resolve relative path against config file directory.
        std::error_code ec;
        auto config_dir = std::filesystem::path(config_path).parent_path();
        auto calib_path = std::filesystem::path(val);
        if (calib_path.is_relative())
            calib_path = config_dir / calib_path;
        auto canonical = std::filesystem::canonical(calib_path, ec);
        if (ec || !std::filesystem::is_regular_file(canonical, ec)) {
            error = "Calibration file not found: " + calib_path.string();
            return false;
        }
        if (!config.slam.calibration.load_yaml(canonical.string())) {
            error = "Failed to parse calibration file: " + canonical.string();
            return false;
        }
        break;
    }

    error.clear();
    return true;
}

// ── Test: loadConfig with valid calibration_file ────────────────────────────

static void test_load_config_with_calibration_file() {
    auto dir = makeTempDir("test_lc_valid");

    // Write a calibration YAML
    {
        std::ofstream f(dir / "calibration.yaml");
        f << "imu_T_lidar:\n";
        f << "  rotation: [1.0, 0.0, 0.0, 0.0]\n";
        f << "  translation: [0.1, 0.2, 0.3]\n";
        f << "refine_imu_T_lidar: true\n";
        f << "imu_noise:\n";
        f << "  gyro_noise: 2.5e-3\n";
        f << "  accel_noise: 3.5e-2\n";
        f << "  gyro_bias_rw: 1.1e-5\n";
        f << "  accel_bias_rw: 2.2e-4\n";
    }

    // Write a daemon config YAML referencing the calibration file
    auto config_path = dir / "config.yaml";
    {
        std::ofstream f(config_path);
        f << "# Daemon config\n";
        f << "calibration_file: \"calibration.yaml\"\n";
        f << "sensor:\n";
        f << "  device_uri: \"\"\n";
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(ok);

    // Verify the CalibrationBundle was loaded into config.slam.calibration
    assert(near(config.slam.calibration.imu_T_lidar.translation[0], 0.1, 1e-6));
    assert(near(config.slam.calibration.imu_T_lidar.translation[1], 0.2, 1e-6));
    assert(near(config.slam.calibration.imu_T_lidar.translation[2], 0.3, 1e-6));
    assert(config.slam.calibration.refine_imu_T_lidar);
    assert(near(config.slam.calibration.imu_noise.gyro_noise, 2.5e-3, 1e-8));
    assert(near(config.slam.calibration.imu_noise.accel_noise, 3.5e-2, 1e-8));
    assert(near(config.slam.calibration.imu_noise.gyro_bias_rw, 1.1e-5, 1e-8));
    assert(near(config.slam.calibration.imu_noise.accel_bias_rw, 2.2e-4, 1e-8));

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig with calibration_file       OK");
}

// ── Test: loadConfig with missing calibration_file → error ──────────────────

static void test_load_config_missing_calibration_file() {
    auto dir = makeTempDir("test_lc_missing");

    auto config_path = dir / "config.yaml";
    {
        std::ofstream f(config_path);
        f << "calibration_file: \"nonexistent_calib.yaml\"\n";
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(!ok);
    assert(!error.empty());

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig missing calibration_file    OK");
}

// ── Test: loadConfig with no calibration_file key → defaults ────────────────

static void test_load_config_no_calibration_key() {
    auto dir = makeTempDir("test_lc_nokey");

    auto config_path = dir / "config.yaml";
    {
        std::ofstream f(config_path);
        f << "# No calibration_file key\n";
        f << "sensor:\n";
        f << "  device_uri: \"\"\n";
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(ok);

    // Should have default CalibrationBundle (identity extrinsic, default noise)
    assert(config.slam.calibration.imu_T_lidar.is_identity());
    assert(near(config.slam.calibration.imu_noise.gyro_noise, 1.0e-3));

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig no calibration_file key     OK");
}

// ── Test: loadConfig with nonexistent config file → error ───────────────────

static void test_load_config_nonexistent_file() {
    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile("/nonexistent/path/config.yaml", config, error);
    assert(!ok);
    assert(!error.empty());

    std::puts("  loadConfig nonexistent config file     OK");
}

// ── Test: loadConfig with calibration file containing cameras ───────────────

static void test_load_config_calibration_with_cameras() {
    auto dir = makeTempDir("test_lc_cam");

    // Write calibration YAML with a camera
    {
        std::ofstream f(dir / "calib_cam.yaml");
        f << "imu_T_lidar:\n";
        f << "  translation: [0.05, 0.0, 0.0]\n";
        f << "imu_noise:\n";
        f << "  gyro_noise: 1.0e-3\n";
        f << "  accel_noise: 1.0e-2\n";
        f << "cameras:\n";
        f << "  - label: cam0\n";
        f << "    intrinsics:\n";
        f << "      fx: 500\n";
        f << "      fy: 500\n";
        f << "      cx: 320\n";
        f << "      cy: 240\n";
        f << "      width: 640\n";
        f << "      height: 480\n";
        f << "    imu_T_camera:\n";
        f << "      translation: [0.02, -0.01, 0.0]\n";
        f << "    time_offset_ns: -3000\n";
    }

    auto config_path = dir / "daemon.yaml";
    {
        std::ofstream f(config_path);
        f << "calibration_file: \"calib_cam.yaml\"\n";
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(ok);

    // Verify camera was loaded
    assert(config.slam.calibration.cameras.size() == 1);
    assert(config.slam.calibration.cameras[0].label == "cam0");
    assert(near(config.slam.calibration.cameras[0].intrinsics.fx, 500.0, 1e-6));
    assert(config.slam.calibration.cameras[0].time_offset_ns == -3000);
    assert(near(config.slam.calibration.cameras[0].imu_T_camera.translation[0],
                0.02, 1e-6));

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig calibration with cameras    OK");
}

// ── Test: loadConfig with commented-out calibration_file ────────────────────

static void test_load_config_commented_calibration_file() {
    auto dir = makeTempDir("test_lc_comment");

    auto config_path = dir / "config.yaml";
    {
        std::ofstream f(config_path);
        f << "# calibration_file: \"should_be_ignored.yaml\"\n";
        f << "sensor:\n";
        f << "  device_uri: \"\"\n";
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(ok);

    // Comment should be stripped — calibration_file key should not be found
    assert(config.slam.calibration.imu_T_lidar.is_identity());

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig commented calibration_file  OK");
}

// ── Test: loadConfig with calibration_file with quoted value ────────────────

static void test_load_config_quoted_calibration_path() {
    auto dir = makeTempDir("test_lc_quoted");

    // Write calibration YAML
    {
        std::ofstream f(dir / "my_calib.yaml");
        f << "imu_T_lidar:\n";
        f << "  translation: [0.5, 0.0, 0.0]\n";
    }

    auto config_path = dir / "config.yaml";
    {
        std::ofstream f(config_path);
        f << "calibration_file: 'my_calib.yaml'\n";  // single quotes
    }

    SlamdTestConfig config;
    std::string error;
    bool ok = parseCalibrationFile(config_path.string(), config, error);
    assert(ok);
    assert(near(config.slam.calibration.imu_T_lidar.translation[0], 0.5, 1e-6));

    std::filesystem::remove_all(dir);
    std::puts("  loadConfig quoted calibration path     OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("SlamdLoadConfig:");
    test_load_config_with_calibration_file();
    test_load_config_missing_calibration_file();
    test_load_config_no_calibration_key();
    test_load_config_nonexistent_file();
    test_load_config_calibration_with_cameras();
    test_load_config_commented_calibration_file();
    test_load_config_quoted_calibration_path();
    std::puts("SlamdLoadConfig: ALL TESTS PASSED");
    return 0;
}
