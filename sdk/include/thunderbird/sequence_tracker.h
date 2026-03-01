// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Per-channel packet sequence tracker
// ─────────────────────────────────────────────────────────────────────────────
//
// Tracks a per-channel monotonic sequence number and detects gaps (dropped
// packets).  No heap allocation.  No locking (single-threaded I/O path).
//
// Usage:
//   SequenceTracker lidar_seq;
//   uint32_t dropped = lidar_seq.update(hdr.sequence);
//   stats.packets_dropped += dropped;
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>

namespace thunderbird {

class SequenceTracker {
public:
    /// Record a received sequence number.
    /// Returns the number of dropped packets (0 = no loss).
    uint32_t update(uint32_t seq) {
        if (!initialised_) {
            prev_seq_    = seq;
            initialised_ = true;
            return 0;
        }

        // Expected: prev + 1.  Handle wrap-around (uint32 counter).
        uint32_t expected = prev_seq_ + 1;
        uint32_t gap = seq - expected;  // unsigned subtraction handles wrap

        prev_seq_ = seq;

        // If gap > threshold, treat as re-initialisation (device restart)
        // rather than massive loss.
        if (gap > kMaxReasonableGap) {
            ++total_resets_;
            return 0;
        }

        total_dropped_ += gap;
        return gap;
    }

    void reset() {
        initialised_  = false;
        prev_seq_     = 0;
        total_dropped_ = 0;
        total_resets_  = 0;
    }

    uint64_t total_dropped() const { return total_dropped_; }
    uint32_t total_resets()  const { return total_resets_; }
    bool     initialised()   const { return initialised_; }

private:
    static constexpr uint32_t kMaxReasonableGap = 1000;

    bool     initialised_{false};
    uint32_t prev_seq_{0};
    uint64_t total_dropped_{0};
    uint32_t total_resets_{0};
};

} // namespace thunderbird
