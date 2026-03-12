// ─────────────────────────────────────────────────────────────────────────────
// Test — Kalibr Import (importKalibrImu / importKalibrCamchain)
// ─────────────────────────────────────────────────────────────────────────────
#include "calib/kalibr_import.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace thunderbird;
using namespace thunderbird::calib;

static constexpr double kEps = 1e-9;

static bool near(double a, double b, double tol = kEps) {
    return std::abs(a - b) < tol;
}

// ── importKalibrImu: no imu section → returns false ─────────────────────────

static void test_import_imu_no_section() {
    const auto unique_id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() /
                      ("test_no_imu_" + unique_id + ".yaml");
    {
        std::ofstream f(path);
        f << "# Empty YAML — no imu0 section\n";
        f << "some_other_key: 42\n";
    }

    CalibrationBundle bundle;
    bool ok = importKalibrImu(path.string(), bundle);
    assert(!ok);  // must return false when no IMU data parsed

    std::filesystem::remove(path);
    std::puts("  importKalibrImu no section      OK");
}

// ── importKalibrImu: valid imu section → returns true ───────────────────────

static void test_import_imu_valid() {
    const auto unique_id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() /
                      ("test_valid_imu_" + unique_id + ".yaml");
    {
        std::ofstream f(path);
        f << "imu0:\n";
        f << "  accelerometer_noise_density: 0.01\n";
        f << "  accelerometer_random_walk: 0.0002\n";
        f << "  gyroscope_noise_density: 0.001\n";
        f << "  gyroscope_random_walk: 1.0e-05\n";
        f << "  model: calibrated\n";
        f << "  update_rate: 200.0\n";
    }

    CalibrationBundle bundle;
    bool ok = importKalibrImu(path.string(), bundle);
    assert(ok);
    assert(near(bundle.imu_noise.accel_noise, 0.01));
    assert(near(bundle.imu_noise.accel_bias_rw, 0.0002));
    assert(near(bundle.imu_noise.gyro_noise, 0.001));
    assert(near(bundle.imu_noise.gyro_bias_rw, 1.0e-5));

    std::filesystem::remove(path);
    std::puts("  importKalibrImu valid section   OK");
}

// ── importKalibrImu: file not found → returns false ─────────────────────────

static void test_import_imu_missing_file() {
    CalibrationBundle bundle;
    bool ok = importKalibrImu("/nonexistent/path/imu.yaml", bundle);
    assert(!ok);
    std::puts("  importKalibrImu missing file    OK");
}

// ── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::puts("KalibrImport:");
    test_import_imu_no_section();
    test_import_imu_valid();
    test_import_imu_missing_file();
    std::puts("KalibrImport: ALL TESTS PASSED");
    return 0;
}
