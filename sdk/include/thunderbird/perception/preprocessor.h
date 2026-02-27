// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Point cloud preprocessor interface
// ─────────────────────────────────────────────────────────────────────────────
//
// Stateless point cloud preprocessing: voxel downsampling, ROI crop,
// ground removal, and Euclidean clustering.  Runs on CPU only.
//
// This is a PImpl wrapper so that internal dependencies (nanoflann,
// PCL, or custom spatial indices) never leak into the public header.
//
// Thread safety:
//   • process() is called from a SINGLE thread (T1 preprocessor thread).
//   • It is NOT thread-safe — do not call from multiple threads.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/perception/perception_config.h"
#include "thunderbird/perception/object_detector.h"     // DetectionInput
#include "thunderbird/odom/slam_types.h"

#include <memory>

namespace thunderbird::perception {

class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(const PreprocessorConfig& config);
    ~PointCloudPreprocessor();

    // Non-copyable.
    PointCloudPreprocessor(const PointCloudPreprocessor&) = delete;
    PointCloudPreprocessor& operator=(const PointCloudPreprocessor&) = delete;

    /// Process a deskewed point cloud into a DetectionInput.
    ///
    /// Steps (in order):
    ///   1. ROI crop (remove points outside radius / z-bounds)
    ///   2. Voxel downsample (uniform grid filter)
    ///   3. Ground removal (height threshold or RANSAC)
    ///   4. Euclidean clustering on non-ground points
    ///
    /// The output shares no mutable state with the input cloud (the
    /// filtered_cloud inside DetectionInput is a new allocation).
    ///
    /// @param cloud        Motion-compensated (deskewed) LiDAR frame.
    /// @param ego_pose     Ego pose at the cloud's timestamp.
    /// @param timestamp_ns Scan timestamp.
    /// @return DetectionInput ready for ObjectDetector::detect().
    DetectionInput process(
        std::shared_ptr<const odom::PointCloudFrame> cloud,
        const odom::Pose6D& ego_pose,
        int64_t timestamp_ns);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace thunderbird::perception
