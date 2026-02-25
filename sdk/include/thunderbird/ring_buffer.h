// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Lock-free ring buffer for high-throughput sensor data
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace thunderbird {

/// Single-producer / single-consumer ring buffer.
/// `T` is typically a shared_ptr to a sensor frame.
/// Capacity MUST be a power of two.
template <typename T, size_t Capacity>
class RingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power-of-two");
public:
    RingBuffer() = default;

    /// Try to push an item.  Returns false if the buffer is full (oldest lost).
    bool push(T item) {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) & kMask;

        // If full, advance tail (drop oldest)
        size_t tail = tail_.load(std::memory_order_acquire);
        if (next == tail) {
            // Drop oldest — move tail forward
            tail_.store((tail + 1) & kMask, std::memory_order_release);
            ++dropped_;
        }

        buf_[head] = std::move(item);
        head_.store(next, std::memory_order_release);
        return true;
    }

    /// Try to pop the oldest item.  Returns std::nullopt if empty.
    std::optional<T> pop() {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
            return std::nullopt; // empty

        T item = std::move(buf_[tail]);
        tail_.store((tail + 1) & kMask, std::memory_order_release);
        return item;
    }

    size_t size() const {
        const size_t h = head_.load(std::memory_order_acquire);
        const size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & kMask;
    }

    bool empty() const { return size() == 0; }

    size_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

    void clear() {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }

private:
    static constexpr size_t kMask = Capacity - 1;

    std::array<T, Capacity>  buf_{};
    std::atomic<size_t>      head_{0};
    std::atomic<size_t>      tail_{0};
    std::atomic<size_t>      dropped_{0};
};

} // namespace thunderbird
