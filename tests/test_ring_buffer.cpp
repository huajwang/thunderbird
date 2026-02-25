// ─────────────────────────────────────────────────────────────────────────────
// Test — RingBuffer
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/ring_buffer.h"

#include <cassert>
#include <cstdio>
#include <memory>

int main() {
    using namespace thunderbird;

    // ── Basic push/pop ──────────────────────────────────────────────────────
    {
        RingBuffer<int, 4> rb;  // capacity 4 (power-of-2)
        assert(rb.empty());
        assert(rb.size() == 0);

        rb.push(10);
        rb.push(20);
        rb.push(30);
        assert(rb.size() == 3);

        auto v = rb.pop();
        assert(v.has_value() && *v == 10);
        v = rb.pop();
        assert(v.has_value() && *v == 20);
        v = rb.pop();
        assert(v.has_value() && *v == 30);
        v = rb.pop();
        assert(!v.has_value());
    }

    // ── Overflow drops oldest ───────────────────────────────────────────────
    {
        RingBuffer<int, 4> rb;
        rb.push(1);
        rb.push(2);
        rb.push(3);
        // Buffer full (3 items in 4-slot ring), push one more → drops 1
        rb.push(4);
        assert(rb.dropped() >= 1);

        auto v = rb.pop();
        assert(v.has_value() && *v == 2); // oldest surviving
    }

    // ── Shared pointers ─────────────────────────────────────────────────────
    {
        RingBuffer<std::shared_ptr<int>, 8> rb;
        auto p = std::make_shared<int>(42);
        rb.push(p);
        assert(p.use_count() == 2); // held by rb + local

        auto out = rb.pop();
        assert(out.has_value() && **out == 42);
    }

    std::puts("RingBuffer: ALL TESTS PASSED");
    return 0;
}
