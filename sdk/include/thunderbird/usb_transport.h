// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — USB transport (libusb-based)
// ─────────────────────────────────────────────────────────────────────────────
//
// Design rationale:
//   • USB sensor devices typically expose one or more bulk endpoints.
//     We model this as a single ITransport that maps:
//       – write() → bulk OUT endpoint (control commands)
//       – read()  → bulk IN endpoint (sensor data)
//   • We abstract the actual libusb dependency behind a thin wrapper so the
//     rest of the SDK never includes <libusb.h> directly.  If libusb is not
//     available at build time, this file compiles as a stub that returns
//     Status::NotSupported.
//   • Hotplug: rather than blocking on open(), we support a URI-based device
//     selector so the caller can target a specific serial or bus address:
//       "usb://0"                  → first Thunderbird device found
//       "usb://serial:ABC123"      → device with matching serial string
//       "usb://bus:1,addr:4"       → specific bus/address
//
// Note: this PoC provides the COMPLETE class API and a stub implementation.
//       Swapping in real libusb calls is a linear fill-in exercise.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/transport.h"
#include "thunderbird/types.h"

#include <cstdint>
#include <string>
#include <mutex>

namespace thunderbird {

class UsbTransport final : public ITransport {
public:
    UsbTransport() = default;
    ~UsbTransport() override { close(); }

    // ── ITransport implementation ───────────────────────────────────────────

    Status open(const std::string& uri) override {
        std::lock_guard lk(mu_);
        if (open_) return Status::AlreadyStreaming;

        // Parse the URI to extract selector
        selector_ = parse_selector(uri);

        //
        // ── Real implementation outline (disabled until libusb is linked):
        //
        //  1. libusb_init(&ctx_);
        //  2. Enumerate devices with libusb_get_device_list().
        //  3. Match by VID/PID (Thunderbird-specific) and optional serial.
        //  4. libusb_open(dev, &handle_);
        //  5. libusb_claim_interface(handle_, kInterfaceNumber);
        //  6. Record the bulk-IN and bulk-OUT endpoint addresses.
        //
        // For now, return NotSupported to signal that no USB backend is linked.
        //

#if defined(THUNDERBIRD_HAS_LIBUSB)
        // --- Real libusb init here ---
        // (omitted — fill in when hardware is available)
        open_ = true;
        return Status::OK;
#else
        return Status::NotSupported;
#endif
    }

    void close() override {
        std::lock_guard lk(mu_);
        if (!open_) return;

#if defined(THUNDERBIRD_HAS_LIBUSB)
        // libusb_release_interface(handle_, kInterfaceNumber);
        // libusb_close(handle_);
        // libusb_exit(ctx_);
#endif

        open_ = false;
    }

    bool is_open() const override {
        std::lock_guard lk(mu_);
        return open_;
    }

    size_t read(uint8_t* buf, size_t max_bytes, uint32_t timeout_ms) override {
        std::lock_guard lk(mu_);
        if (!open_) return 0;

#if defined(THUNDERBIRD_HAS_LIBUSB)
        //
        // int transferred = 0;
        // int rc = libusb_bulk_transfer(
        //     handle_, kBulkInEndpoint,
        //     buf, static_cast<int>(max_bytes),
        //     &transferred, timeout_ms);
        // if (rc == 0 || rc == LIBUSB_ERROR_TIMEOUT)
        //     return static_cast<size_t>(transferred);
        // return 0;
        //
        (void)buf; (void)max_bytes; (void)timeout_ms;
        return 0;
#else
        (void)buf; (void)max_bytes; (void)timeout_ms;
        return 0;
#endif
    }

    size_t write(const uint8_t* buf, size_t len) override {
        std::lock_guard lk(mu_);
        if (!open_) return 0;

#if defined(THUNDERBIRD_HAS_LIBUSB)
        //
        // int transferred = 0;
        // int rc = libusb_bulk_transfer(
        //     handle_, kBulkOutEndpoint,
        //     const_cast<uint8_t*>(buf), static_cast<int>(len),
        //     &transferred, 1000 /* ms */);
        // return (rc == 0) ? static_cast<size_t>(transferred) : 0;
        //
        (void)buf; (void)len;
        return 0;
#else
        (void)buf; (void)len;
        return 0;
#endif
    }

private:
    // ── USB constants (device-specific — adjust per hardware) ───────────────
    // static constexpr uint16_t kVendorId        = 0x1234;
    // static constexpr uint16_t kProductId       = 0x5678;
    // static constexpr int      kInterfaceNumber = 0;
    // static constexpr uint8_t  kBulkInEndpoint  = 0x81;
    // static constexpr uint8_t  kBulkOutEndpoint = 0x02;

    struct Selector {
        int         index{0};          // "usb://0"
        std::string serial;            // "usb://serial:ABC123"
        int         bus{-1};
        int         addr{-1};
    };

    static Selector parse_selector(const std::string& uri) {
        Selector s;
        std::string body = uri;
        const std::string scheme = "usb://";
        if (body.substr(0, scheme.size()) == scheme)
            body = body.substr(scheme.size());

        if (body.substr(0, 7) == "serial:") {
            s.serial = body.substr(7);
        } else if (body.substr(0, 4) == "bus:") {
            // "bus:1,addr:4"
            auto comma = body.find(',');
            s.bus  = std::stoi(body.substr(4, comma - 4));
            if (comma != std::string::npos && body.substr(comma + 1, 5) == "addr:")
                s.addr = std::stoi(body.substr(comma + 6));
        } else {
            s.index = std::stoi(body.empty() ? "0" : body);
        }
        return s;
    }

    mutable std::mutex  mu_;
    bool                open_{false};
    Selector            selector_;

    // libusb handles (opaque when THUNDERBIRD_HAS_LIBUSB is not defined)
    // libusb_context*       ctx_{nullptr};
    // libusb_device_handle* handle_{nullptr};
};

} // namespace thunderbird
