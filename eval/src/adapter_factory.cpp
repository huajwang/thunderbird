// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: Adapter Factory
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/dataset_adapter.h"
#include "eval/imu_interpolator.h"
#include "eval/stress_test_adapter.h"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace fs = std::filesystem;

namespace eval {

// Forward-declared factory functions (defined in each adapter .cpp).
std::unique_ptr<DatasetAdapter> make_kitti_adapter(
    bool enable_imu = false, ImuInterpolatorConfig imu_cfg = {});
std::unique_ptr<DatasetAdapter> make_euroc_adapter();
// std::unique_ptr<DatasetAdapter> make_tbrec_adapter();   // TODO

/// Stub adapter for formats not yet implemented.
class StubAdapter final : public DatasetAdapter {
public:
    explicit StubAdapter(std::string fmt) : fmt_(std::move(fmt)) {}
    bool open(const std::string&, size_t) override {
        std::cerr << "[eval] adapter '" << fmt_ << "' not yet implemented\n";
        return false;
    }
    DatasetInfo info() const override { return {}; }
    std::optional<StreamEvent> next() override { return std::nullopt; }
    std::vector<GtPose> loadGroundTruth() override { return {}; }
    void rewind() override {}
    size_t framesEmitted() const override { return 0; }

private:
    std::string fmt_;
};

std::unique_ptr<DatasetAdapter> createAdapter(const std::string& format) {
    return createAdapter(format, false, {});
}

std::unique_ptr<DatasetAdapter> createAdapter(const std::string& format,
                                               bool enable_imu,
                                               ImuInterpolatorConfig imu_cfg) {
    if (format == "kitti") {
        return make_kitti_adapter(enable_imu, imu_cfg);
    }
    if (format == "euroc") {
        return make_euroc_adapter();
    }
    if (format == "tbrec") {
        return std::make_unique<StubAdapter>("tbrec");
    }
    // Stress-test presets.
    if (format == "stress" || format == "stress_custom") {
        return make_stress_adapter();
    }
    if (format == "stress_drone" || format == "aggressive_drone") {
        return make_stress_adapter(StressConfig::aggressiveDrone());
    }
    if (format == "stress_car" || format == "fast_car") {
        return make_stress_adapter(StressConfig::fastCar());
    }
    if (format == "stress_corridor" || format == "degenerate_corridor") {
        return make_stress_adapter(StressConfig::degenerateCorridor());
    }
    if (format == "stress_spin" || format == "spinning_top") {
        return make_stress_adapter(StressConfig::spinningTop());
    }
    std::cerr << "[eval] unknown format: " << format << "\n";
    return nullptr;
}

std::unique_ptr<DatasetAdapter> createAdapterFromPath(const std::string& path) {
    return createAdapterFromPath(path, false, {});
}

std::unique_ptr<DatasetAdapter> createAdapterFromPath(const std::string& path,
                                                       bool enable_imu,
                                                       ImuInterpolatorConfig imu_cfg) {
    // Heuristic: check for format-specific markers.
    const fs::path p(path);

    // .tbrec file
    if (p.extension() == ".tbrec") {
        return createAdapter("tbrec", enable_imu, imu_cfg);
    }

    // KITTI: directory containing velodyne/
    if (fs::is_directory(p / "velodyne")) {
        return createAdapter("kitti", enable_imu, imu_cfg);
    }

    // EuRoC: directory containing mav0/ or imu0/
    if (fs::is_directory(p / "mav0") || fs::is_directory(p / "imu0")) {
        return createAdapter("euroc", enable_imu, imu_cfg);
    }

    std::cerr << "[eval] cannot detect dataset format for: " << path << "\n";
    return nullptr;
}

} // namespace eval
