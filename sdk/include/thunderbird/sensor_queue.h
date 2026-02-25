// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Thread-safe bounded sensor queue
// ─────────────────────────────────────────────────────────────────────────────
//
// A fixed-capacity, multi-producer / single-consumer queue used by the
// Data Abstraction Layer to buffer sensor frames between the parser thread
// (producer) and the user's polling thread (consumer).
//
// Design rationale:
//   • **Bounded**: a fixed maximum depth prevents unbounded memory growth
//     when the consumer cannot keep up.  When full the oldest frame is
//     silently dropped (drop-oldest / ring-buffer policy), matching
//     real-time sensor semantics where stale data is worthless.
//   • **Mutex + condvar**: acceptable for PoC; the lock is held only for
//     the duration of a pointer move.  Can be replaced with a lock-free
//     queue in future if profiling shows contention.
//   • **Blocking pop with timeout**: supports both busy-poll (`try_pop`)
//     and wait-style (`pop_for`) usage patterns.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <condition_variable>
#include <chrono>
#include <atomic>

namespace thunderbird {

template <typename T>
class SensorQueue {
public:
    /// @param capacity Maximum number of items.  When exceeded the oldest
    ///                 item is dropped to make room (drop-oldest policy).
    explicit SensorQueue(size_t capacity = 128)
        : capacity_(capacity) {}

    // ── Producer side (called from parser / driver thread) ──────────────

    /// Push a new item.  If the queue is at capacity the **oldest** item is
    /// silently dropped and the drop counter is incremented.
    void push(T item) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (queue_.size() >= capacity_) {
                queue_.pop_front();   // drop oldest
                ++dropped_;
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    // ── Consumer side (called from user thread) ─────────────────────────

    /// Non-blocking pop.  Returns std::nullopt if the queue is empty.
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    /// Blocking pop with timeout.  Returns std::nullopt on timeout.
    template <typename Rep, typename Period>
    std::optional<T> pop_for(std::chrono::duration<Rep, Period> timeout) {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!cv_.wait_for(lk, timeout, [this]{ return !queue_.empty(); }))
            return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    // ── Query ───────────────────────────────────────────────────────────

    size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return queue_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return queue_.empty();
    }

    /// Number of items that were dropped because the queue was full.
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

    /// Drain all pending items.
    void clear() {
        std::lock_guard<std::mutex> lk(mtx_);
        queue_.clear();
    }

    size_t capacity() const { return capacity_; }

private:
    size_t                   capacity_;
    std::deque<T>            queue_;
    mutable std::mutex       mtx_;
    std::condition_variable  cv_;
    std::atomic<uint64_t>    dropped_{0};
};

} // namespace thunderbird
