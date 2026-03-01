// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — LiDAR Frame Assembler
// ─────────────────────────────────────────────────────────────────────────────
//
// Accumulates per-packet LidarFrame data (50–200 points) into complete 360°
// PointCloudFrame sweeps (~28,000 points).  Handles:
//   • Revolution boundary detection via azimuth-wrap
//   • Time-based frame completion for solid-state LiDARs
//   • Packet-drop detection via azimuth-gap analysis
//   • Per-point dt_ns population for motion compensation
//   • Partial-frame timeout for stalled sensors
//
// Threading: all methods are called from the I/O thread (single-threaded).
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include "thunderbird/odom/slam_types.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

namespace thunderbird {

// ── Completion detection mode ───────────────────────────────────────────────

enum class CompletionMode : uint8_t {
    /// Spinning LiDAR: detect revolution boundary via azimuth wrap.
    AzimuthWrap = 0,

    /// Solid-state / non-repetitive scan pattern: emit on fixed timer.
    TimeBased = 1,
};

// ── Configuration ───────────────────────────────────────────────────────────

struct FrameAssemblerConfig {
    CompletionMode mode{CompletionMode::AzimuthWrap};

    /// Expected LiDAR rotation rate in Hz.
    double expected_rate_hz{10.0};

    /// [TimeBased mode only] Fixed accumulation period (ns).
    int64_t frame_period_ns{100'000'000};

    /// Maximum number of points per assembled frame (safety cap).
    size_t max_points_per_frame{100'000};

    /// Maximum number of packets per frame (safety cap).
    uint32_t max_packets_per_frame{200};

    /// Partial-frame timeout multiplier relative to expected period.
    double partial_timeout_mul{2.0};

    /// Minimum points for a frame to be emitted.
    size_t min_points_to_emit{100};

    /// Azimuth wrap hysteresis (degrees).
    float azimuth_wrap_threshold_deg{180.0f};

    /// Pre-allocate point buffer to this capacity.
    size_t point_reserve{32'768};
};

// ── Assembly metadata ───────────────────────────────────────────────────────

struct FrameAssemblyMeta {
    int64_t  scan_start_ns{0};
    int64_t  scan_end_ns{0};
    int64_t  scan_duration_ns{0};
    int64_t  host_arrival_ns{0};

    uint32_t total_packets{0};
    uint32_t expected_packets{0};

    float    azimuth_coverage_deg{0};
    float    azimuth_start_deg{0};
    float    azimuth_end_deg{0};

    uint32_t dropped_packets{0};
    double   drop_rate{0};

    bool     is_partial{false};
    bool     is_first_frame{false};
    uint32_t sequence{0};
};

// ── Output callback ─────────────────────────────────────────────────────────

using AssembledFrameCallback = std::function<void(
    std::shared_ptr<const odom::PointCloudFrame> frame,
    const FrameAssemblyMeta& meta)>;

// ── Assembler statistics ────────────────────────────────────────────────────

struct AssemblerStats {
    uint64_t frames_emitted{0};
    uint64_t partial_frames{0};
    uint64_t packets_ingested{0};
    uint64_t points_ingested{0};
    uint64_t dropped_packets_total{0};
    uint64_t runt_frames_discarded{0};
    double   avg_points_per_frame{0};
    double   avg_packets_per_frame{0};
    double   avg_frame_period_ms{0};
    double   measured_rate_hz{0};
};

// ─────────────────────────────────────────────────────────────────────────────
// LidarFrameAssembler — accumulates per-packet data into full sweeps
// ─────────────────────────────────────────────────────────────────────────────

class LidarFrameAssembler {
public:
    explicit LidarFrameAssembler(const FrameAssemblerConfig& config = {})
        : config_(config)
    {
        accum_points_.reserve(config_.point_reserve);
        packet_metas_.reserve(config_.max_packets_per_frame);
    }

    // ── Packet ingestion ────────────────────────────────────────────────

    /// Feed one decoded LiDAR packet (azimuth-wrap mode).
    void feed(std::shared_ptr<const LidarFrame> frame,
              float azimuth_start,
              float azimuth_end)
    {
        if (!frame || frame->points.empty()) return;

        ++stats_.packets_ingested;
        stats_.points_ingested += frame->points.size();

        int64_t hw_ns = frame->timestamp.nanoseconds;
        int64_t host_ns = frame->host_timestamp.nanoseconds;

        // Detect revolution boundary (azimuth wrap).
        if (have_first_packet_ && config_.mode == CompletionMode::AzimuthWrap) {
            float delta = prev_azimuth_end_ - azimuth_start;
            if (delta > config_.azimuth_wrap_threshold_deg) {
                // Revolution complete — emit accumulated frame.
                emit_frame(false, host_ns);
            }
        }

        // Safety: cap accumulation.
        if (accum_points_.size() + frame->points.size() > config_.max_points_per_frame ||
            packets_in_frame_ >= config_.max_packets_per_frame) {
            emit_frame(false, host_ns);
        }

        // Track per-packet metadata.
        PacketMeta pm;
        pm.azimuth_start = azimuth_start;
        pm.azimuth_end = azimuth_end;
        pm.hw_timestamp_ns = hw_ns;
        pm.host_timestamp_ns = host_ns;
        pm.point_start_idx = static_cast<uint32_t>(accum_points_.size());
        pm.point_count = static_cast<uint32_t>(frame->points.size());
        packet_metas_.push_back(pm);

        // Append points to accumulation buffer.
        for (const auto& pt : frame->points) {
            odom::PointXYZIT p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.dt_ns = 0;  // populated at emit time
            accum_points_.push_back(p);
        }

        // Update tracking state.
        if (!have_first_packet_) {
            first_packet_hw_ns_ = hw_ns;
            frame_azimuth_start_ = azimuth_start;
        }
        last_packet_hw_ns_ = hw_ns;
        last_packet_host_ns_ = host_ns;
        prev_azimuth_end_ = azimuth_end;
        frame_azimuth_end_ = azimuth_end;
        have_first_packet_ = true;
        ++packets_in_frame_;
    }

    /// Feed one decoded LiDAR packet (time-based mode).
    void feed_timed(std::shared_ptr<const LidarFrame> frame) {
        if (!frame || frame->points.empty()) return;

        ++stats_.packets_ingested;
        stats_.points_ingested += frame->points.size();

        int64_t hw_ns = frame->timestamp.nanoseconds;
        int64_t host_ns = frame->host_timestamp.nanoseconds;

        // Check if the accumulation window has elapsed.
        if (have_first_packet_) {
            int64_t elapsed = hw_ns - first_packet_hw_ns_;
            if (elapsed >= config_.frame_period_ns) {
                emit_frame(false, host_ns);
            }
        }

        // Append points.
        PacketMeta pm;
        pm.azimuth_start = 0;
        pm.azimuth_end = 0;
        pm.hw_timestamp_ns = hw_ns;
        pm.host_timestamp_ns = host_ns;
        pm.point_start_idx = static_cast<uint32_t>(accum_points_.size());
        pm.point_count = static_cast<uint32_t>(frame->points.size());
        packet_metas_.push_back(pm);

        for (const auto& pt : frame->points) {
            odom::PointXYZIT p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.dt_ns = 0;
            accum_points_.push_back(p);
        }

        if (!have_first_packet_) {
            first_packet_hw_ns_ = hw_ns;
        }
        last_packet_hw_ns_ = hw_ns;
        last_packet_host_ns_ = host_ns;
        have_first_packet_ = true;
        ++packets_in_frame_;
    }

    /// Check for partial-frame timeout.
    void check_timeout(int64_t now_ns) {
        if (!have_first_packet_ || accum_points_.empty()) return;
        if (config_.expected_rate_hz <= 0) return;  // guard div-by-zero

        double expected_period_ns = 1e9 / config_.expected_rate_hz;
        int64_t timeout_ns = static_cast<int64_t>(
            expected_period_ns * config_.partial_timeout_mul);

        if (now_ns - last_packet_host_ns_ > timeout_ns) {
            emit_frame(true, now_ns);
        }
    }

    // ── Output ──────────────────────────────────────────────────────────

    void on_frame(AssembledFrameCallback cb) {
        frame_cb_ = std::move(cb);
    }

    // ── Diagnostics ─────────────────────────────────────────────────────

    [[nodiscard]] AssemblerStats stats() const { return stats_; }

    // ── Lifecycle ───────────────────────────────────────────────────────

    void reset() {
        accum_points_.clear();
        packet_metas_.clear();
        have_first_packet_ = false;
        packets_in_frame_ = 0;
        first_packet_hw_ns_ = 0;
        last_packet_hw_ns_ = 0;
        last_packet_host_ns_ = 0;
        prev_azimuth_end_ = 0;
        frame_azimuth_start_ = 0;
        frame_azimuth_end_ = 0;
        // Don't reset stats or frame_sequence_ (cumulative).
    }

private:
    // ── Per-packet metadata ─────────────────────────────────────────────

    struct PacketMeta {
        float    azimuth_start;
        float    azimuth_end;
        int64_t  hw_timestamp_ns;
        int64_t  host_timestamp_ns;
        uint32_t point_start_idx;
        uint32_t point_count;
    };

    // ── Frame emission ──────────────────────────────────────────────────

    void emit_frame(bool is_partial, int64_t host_now_ns) {
        if (accum_points_.size() < config_.min_points_to_emit) {
            ++stats_.runt_frames_discarded;
            clear_accumulator();
            return;
        }

        // Populate per-point dt_ns (Strategy A: packet-level interpolation).
        populate_dt_ns();

        // Detect dropped packets from azimuth gaps.
        uint32_t drops = detect_drops();

        // Build PointCloudFrame.
        auto pcf = std::make_shared<odom::PointCloudFrame>();
        pcf->timestamp_ns = first_packet_hw_ns_;
        pcf->sequence = frame_sequence_;
        pcf->is_deskewed = false;
        pcf->points = std::move(accum_points_);

        // Build metadata.
        FrameAssemblyMeta meta;
        meta.scan_start_ns = first_packet_hw_ns_;
        meta.scan_end_ns = last_packet_hw_ns_;
        meta.scan_duration_ns = last_packet_hw_ns_ - first_packet_hw_ns_;
        meta.host_arrival_ns = host_now_ns;
        meta.total_packets = packets_in_frame_;
        meta.expected_packets = packets_in_frame_ + drops;
        meta.azimuth_start_deg = frame_azimuth_start_;
        meta.azimuth_end_deg = frame_azimuth_end_;
        meta.azimuth_coverage_deg = compute_azimuth_coverage();
        meta.dropped_packets = drops;
        meta.drop_rate = (drops + packets_in_frame_) > 0
            ? static_cast<double>(drops) /
              static_cast<double>(drops + packets_in_frame_)
            : 0.0;
        meta.is_partial = is_partial;
        meta.is_first_frame = (frame_sequence_ == 0);
        meta.sequence = frame_sequence_;

        // Update statistics.
        ++stats_.frames_emitted;
        if (is_partial) ++stats_.partial_frames;
        stats_.dropped_packets_total += drops;

        uint64_t n = stats_.frames_emitted;
        double pt_count = static_cast<double>(pcf->points.size());
        double pkt_count = static_cast<double>(packets_in_frame_);
        sum_points_ += pt_count;
        sum_packets_ += pkt_count;
        stats_.avg_points_per_frame = sum_points_ / static_cast<double>(n);
        stats_.avg_packets_per_frame = sum_packets_ / static_cast<double>(n);

        if (prev_frame_emit_host_ns_ > 0) {
            double period_ms = static_cast<double>(
                host_now_ns - prev_frame_emit_host_ns_) / 1e6;
            sum_period_ms_ += period_ms;
            stats_.avg_frame_period_ms = sum_period_ms_ / static_cast<double>(n - 1);
            if (stats_.avg_frame_period_ms > 0) {
                stats_.measured_rate_hz = 1000.0 / stats_.avg_frame_period_ms;
            }
        }
        prev_frame_emit_host_ns_ = host_now_ns;

        ++frame_sequence_;

        // Reset accumulator (reuse allocation — clear() doesn't free).
        accum_points_.clear();
        accum_points_.reserve(config_.point_reserve);
        packet_metas_.clear();
        have_first_packet_ = false;
        packets_in_frame_ = 0;
        first_packet_hw_ns_ = 0;
        last_packet_hw_ns_ = 0;
        prev_azimuth_end_ = 0;
        frame_azimuth_start_ = 0;
        frame_azimuth_end_ = 0;

        // Fire callback.
        if (frame_cb_) {
            frame_cb_(std::move(pcf), meta);
        }
    }

    void clear_accumulator() {
        accum_points_.clear();
        packet_metas_.clear();
        have_first_packet_ = false;
        packets_in_frame_ = 0;
        first_packet_hw_ns_ = 0;
        last_packet_hw_ns_ = 0;
        prev_azimuth_end_ = 0;
        frame_azimuth_start_ = 0;
        frame_azimuth_end_ = 0;
    }

    // ── Per-point dt_ns (Strategy A: packet-level interpolation) ────────

    void populate_dt_ns() {
        for (const auto& m : packet_metas_) {
            int32_t dt = static_cast<int32_t>(
                m.hw_timestamp_ns - first_packet_hw_ns_);
            for (uint32_t i = m.point_start_idx;
                 i < m.point_start_idx + m.point_count &&
                 i < static_cast<uint32_t>(accum_points_.size()); ++i) {
                accum_points_[i].dt_ns = dt;
            }
        }
    }

    // ── Packet drop detection ───────────────────────────────────────────

    uint32_t detect_drops() const {
        if (packet_metas_.size() < 2) return 0;

        // Estimate expected packet azimuth span.
        float total_coverage = compute_azimuth_coverage();
        float expected_span = total_coverage /
            static_cast<float>(packet_metas_.size());
        if (expected_span <= 0) return 0;

        uint32_t drops = 0;
        for (size_t i = 1; i < packet_metas_.size(); ++i) {
            float gap = packet_metas_[i].azimuth_start -
                        packet_metas_[i - 1].azimuth_end;
            if (gap < 0) gap += 360.0f;

            if (gap > expected_span * 1.5f) {
                drops += static_cast<uint32_t>(gap / expected_span + 0.5f) - 1;
            }
        }
        return drops;
    }

    float compute_azimuth_coverage() const {
        if (packet_metas_.empty()) return 0.0f;
        float coverage = frame_azimuth_end_ - frame_azimuth_start_;
        if (coverage < 0) coverage += 360.0f;
        return coverage;
    }

    // ── Members ─────────────────────────────────────────────────────────

    FrameAssemblerConfig config_;

    // Accumulation buffer (reused across frames).
    std::vector<odom::PointXYZIT> accum_points_;
    std::vector<PacketMeta> packet_metas_;

    // Revolution tracking.
    float    prev_azimuth_end_{0.0f};
    float    frame_azimuth_start_{0.0f};
    float    frame_azimuth_end_{0.0f};
    int64_t  first_packet_hw_ns_{0};
    int64_t  last_packet_hw_ns_{0};
    int64_t  last_packet_host_ns_{0};
    uint32_t packets_in_frame_{0};
    bool     have_first_packet_{false};

    // Frame counter + timing.
    uint32_t frame_sequence_{0};
    int64_t  prev_frame_emit_host_ns_{0};

    // Statistics accumulators.
    AssemblerStats stats_{};
    double sum_points_{0};
    double sum_packets_{0};
    double sum_period_ms_{0};

    // Output callback.
    AssembledFrameCallback frame_cb_;
};

} // namespace thunderbird
