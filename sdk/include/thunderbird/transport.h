// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Transport interface (USB / Ethernet / simulated)
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/types.h"
#include <cstddef>
#include <string>

namespace thunderbird {

/// Abstract byte-level transport for device communication.
/// Concrete implementations: USB, Ethernet/UDP, or simulated loopback.
class ITransport {
public:
    virtual ~ITransport() = default;

    /// Open the transport to the given URI (e.g. "usb://0", "udp://192.168.1.100:7500")
    virtual Status open(const std::string& uri) = 0;

    /// Close the transport.
    virtual void close() = 0;

    /// Returns true when the transport is open and healthy.
    virtual bool is_open() const = 0;

    /// Blocking read up to `max_bytes`. Returns bytes actually read, or 0 on timeout.
    virtual size_t read(uint8_t* buf, size_t max_bytes, uint32_t timeout_ms) = 0;

    /// Blocking write. Returns bytes actually written.
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};

} // namespace thunderbird
