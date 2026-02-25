// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Ethernet transport (UDP data channel + TCP control)
// ─────────────────────────────────────────────────────────────────────────────
//
// Design rationale:
//   • **Dual-channel model**: a TCP socket handles control / handshake / heartbeat
//     (reliable, ordered), while a UDP socket carries the bulk sensor data
//     (lower latency, kernel bypass-friendly in the future).
//   • Sensor data on UDP avoids TCP head-of-line blocking: a single dropped
//     LiDAR packet shouldn't stall IMU or camera frames.
//   • The ITransport interface is byte-oriented, so this class multiplexes
//     both channels behind a single read()/write() surface:
//       – write() always goes to the TCP control channel.
//       – read() first drains the UDP data socket, then falls back to TCP.
//   • Socket I/O uses BSD-style calls wrapped behind platform macros so the
//     same source compiles on Linux (native sockets) and Windows (Winsock).
//   • All operations are blocking with configurable timeouts — the SDK's
//     driver threads own the blocking context.
//
// URI format:
//   "eth://<ip>:<control_port>"          (data port = control_port + 1)
//   "eth://192.168.1.100:7500"           → TCP 7500, UDP 7501
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "thunderbird/transport.h"
#include "thunderbird/types.h"

#include <array>
#include <cstring>
#include <string>

// ── Platform socket includes ────────────────────────────────────────────────
#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    using socket_t = SOCKET;
    inline constexpr socket_t kInvalidSock = INVALID_SOCKET;
    inline int sock_errno() { return WSAGetLastError(); }
    inline void sock_close(socket_t s) { closesocket(s); }
#else
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <cerrno>
    using socket_t = int;
    inline constexpr socket_t kInvalidSock = -1;
    inline int sock_errno() { return errno; }
    inline void sock_close(socket_t s) { ::close(s); }
#endif

namespace thunderbird {

/// RAII helper: initialises Winsock on Windows, no-op on POSIX.
struct SocketInit {
    SocketInit() {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    }
    ~SocketInit() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

// ─────────────────────────────────────────────────────────────────────────────

class EthernetTransport final : public ITransport {
public:
    EthernetTransport() = default;

    ~EthernetTransport() override { close(); }

    // ── ITransport ──────────────────────────────────────────────────────────

    Status open(const std::string& uri) override {
        // Parse "eth://host:port"
        auto host_port = parse_uri(uri);
        if (host_port.first.empty()) return Status::InvalidParameter;

        const std::string& host = host_port.first;
        uint16_t ctrl_port = host_port.second;
        uint16_t data_port = ctrl_port + 1;

        sock_init_ = std::make_unique<SocketInit>();

        // ── TCP control channel ─────────────────────────────────────────
        tcp_sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp_sock_ == kInvalidSock) return Status::TransportError;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(ctrl_port);
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            sock_close(tcp_sock_); tcp_sock_ = kInvalidSock;
            return Status::InvalidParameter;
        }

        if (::connect(tcp_sock_, reinterpret_cast<sockaddr*>(&addr),
                      sizeof(addr)) != 0) {
            sock_close(tcp_sock_); tcp_sock_ = kInvalidSock;
            return Status::TransportError;
        }

        // ── UDP data channel ────────────────────────────────────────────
        udp_sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udp_sock_ == kInvalidSock) {
            sock_close(tcp_sock_); tcp_sock_ = kInvalidSock;
            return Status::TransportError;
        }

        // Bind UDP to receive from device
        sockaddr_in udp_bind{};
        udp_bind.sin_family      = AF_INET;
        udp_bind.sin_port        = htons(data_port);
        udp_bind.sin_addr.s_addr = INADDR_ANY;

        if (::bind(udp_sock_, reinterpret_cast<sockaddr*>(&udp_bind),
                   sizeof(udp_bind)) != 0) {
            sock_close(udp_sock_); udp_sock_ = kInvalidSock;
            sock_close(tcp_sock_); tcp_sock_ = kInvalidSock;
            return Status::TransportError;
        }

        // Store device address for sendto() on UDP
        device_addr_ = addr;
        device_addr_.sin_port = htons(data_port);

        open_ = true;
        return Status::OK;
    }

    void close() override {
        open_ = false;
        if (tcp_sock_ != kInvalidSock) { sock_close(tcp_sock_); tcp_sock_ = kInvalidSock; }
        if (udp_sock_ != kInvalidSock) { sock_close(udp_sock_); udp_sock_ = kInvalidSock; }
    }

    bool is_open() const override { return open_; }

    /// Read: prioritise the UDP data socket, fall back to TCP.
    size_t read(uint8_t* buf, size_t max_bytes, uint32_t timeout_ms) override {
        if (!open_) return 0;

        // Use poll/select to wait on BOTH sockets with a single timeout.
#ifdef _WIN32
        // Windows: use select()
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(udp_sock_, &fds);
        FD_SET(tcp_sock_, &fds);

        timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ready = ::select(0 /*ignored on Windows*/, &fds, nullptr, nullptr, &tv);
#else
        // POSIX: use poll()
        pollfd pfds[2];
        pfds[0].fd     = udp_sock_;
        pfds[0].events = POLLIN;
        pfds[1].fd     = tcp_sock_;
        pfds[1].events = POLLIN;

        int ready = ::poll(pfds, 2, static_cast<int>(timeout_ms));
#endif
        if (ready <= 0) return 0; // timeout or error

        // Prefer UDP (sensor data)
#ifdef _WIN32
        if (FD_ISSET(udp_sock_, &fds)) {
#else
        if (pfds[0].revents & POLLIN) {
#endif
            auto n = ::recvfrom(udp_sock_, reinterpret_cast<char*>(buf),
                                static_cast<int>(max_bytes), 0, nullptr, nullptr);
            if (n > 0) return static_cast<size_t>(n);
        }

        // Then TCP (control)
#ifdef _WIN32
        if (FD_ISSET(tcp_sock_, &fds)) {
#else
        if (pfds[1].revents & POLLIN) {
#endif
            auto n = ::recv(tcp_sock_, reinterpret_cast<char*>(buf),
                            static_cast<int>(max_bytes), 0);
            if (n > 0) return static_cast<size_t>(n);
        }

        return 0;
    }

    /// Write: always goes to the TCP control channel.
    size_t write(const uint8_t* buf, size_t len) override {
        if (!open_ || tcp_sock_ == kInvalidSock) return 0;
        auto n = ::send(tcp_sock_, reinterpret_cast<const char*>(buf),
                        static_cast<int>(len), 0);
        return (n > 0) ? static_cast<size_t>(n) : 0;
    }

private:
    /// Parse "eth://host:port" → {host, port}.
    static std::pair<std::string, uint16_t> parse_uri(const std::string& uri) {
        // Strip scheme
        std::string body = uri;
        const std::string scheme = "eth://";
        if (body.substr(0, scheme.size()) == scheme)
            body = body.substr(scheme.size());

        auto colon = body.rfind(':');
        if (colon == std::string::npos) return {"", 0};

        std::string host = body.substr(0, colon);
        uint16_t port = static_cast<uint16_t>(std::stoi(body.substr(colon + 1)));
        return {host, port};
    }

    std::unique_ptr<SocketInit> sock_init_;
    socket_t        tcp_sock_{kInvalidSock};
    socket_t        udp_sock_{kInvalidSock};
    sockaddr_in     device_addr_{};
    bool            open_{false};
};

} // namespace thunderbird
