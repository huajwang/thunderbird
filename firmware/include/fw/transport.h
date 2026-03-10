// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Host transport abstraction
// ─────────────────────────────────────────────────────────────────────────────
//
// Abstracts the outbound link to the host (Ethernet, USB, etc.).
// On real hardware, implement against your SoC's networking stack.
// The stub implementation uses POSIX TCP sockets for desktop testing.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque handle for a host transport.
typedef struct fw_transport fw_transport_t;

/// Allocate and return a stub (TCP server) transport for desktop testing.
/// The transport listens on `port` and accepts one client at a time.
fw_transport_t* fw_transport_create_tcp(uint16_t port);

/// Wait for a host to connect.  Blocks until a client connects or an
/// error occurs.  Returns 0 on success, negative on error.
int fw_transport_accept(fw_transport_t* t);

/// Send `len` bytes to the connected host.
/// Returns bytes sent, or negative on error.
int fw_transport_send(fw_transport_t* t, const uint8_t* data, size_t len);

/// Receive up to `max_len` bytes from the host.
/// @param timeout_ms  0 = non-blocking, >0 = block up to this many ms.
/// Returns bytes received, 0 if timeout/no data, negative on error.
int fw_transport_recv(fw_transport_t* t, uint8_t* buf, size_t max_len,
                      uint32_t timeout_ms);

/// Check if a host is currently connected.
int fw_transport_is_connected(const fw_transport_t* t);

/// Close the current client connection (but keep listening).
void fw_transport_close_client(fw_transport_t* t);

/// Destroy the transport and free resources.
void fw_transport_destroy(fw_transport_t* t);

#ifdef __cplusplus
}
#endif
