// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — GPU detector class declarations (internal)
// ─────────────────────────────────────────────────────────────────────────────
//
// Shared header so that both the factory (cpu_cluster_detector.cpp) and the
// GPU method definitions (gpu_pillar_detector.cpp) see the same class
// declarations when THUNDERBIRD_HAS_GPU_PERCEPTION is enabled.
//
// This is an internal (non-public) header.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#ifdef THUNDERBIRD_HAS_GPU_PERCEPTION

#include "thunderbird/perception/object_detector.h"

namespace thunderbird::perception {

class GpuPillarDetector final : public ObjectDetector {
public:
    bool initialize(const PerceptionConfig& config) override;
    void teardown() override;
    DetectionFrame detect(const DetectionInput& input) override;
    [[nodiscard]] const char* name() const noexcept override { return "GpuPillarDetector (TensorRT)"; }
    [[nodiscard]] bool uses_gpu() const noexcept override { return true; }
};

class GpuCenterPointDetector final : public ObjectDetector {
public:
    bool initialize(const PerceptionConfig& config) override;
    void teardown() override;
    DetectionFrame detect(const DetectionInput& input) override;
    [[nodiscard]] const char* name() const noexcept override { return "GpuCenterPointDetector (TensorRT)"; }
    [[nodiscard]] bool uses_gpu() const noexcept override { return true; }
};

} // namespace thunderbird::perception

#endif // THUNDERBIRD_HAS_GPU_PERCEPTION
