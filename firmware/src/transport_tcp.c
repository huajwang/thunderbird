// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird Firmware — Transport implementation (TCP stub for desktop)
// ─────────────────────────────────────────────────────────────────────────────
//
// Uses POSIX/Winsock TCP server sockets so the firmware can be tested on a
// desktop machine against the real SDK.  On real hardware, replace with
// your SoC's Ethernet MAC / USB CDC driver.
// ─────────────────────────────────────────────────────────────────────────────
#include "fw/transport.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef SOCKET socket_t;
  #define INVALID_SOCK INVALID_SOCKET
  #define CLOSE_SOCKET closesocket
#else
  #include <unistd.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <poll.h>
  #include <errno.h>
  typedef int socket_t;
  #define INVALID_SOCK (-1)
  #define CLOSE_SOCKET close
#endif

struct fw_transport {
    socket_t listen_fd;
    socket_t client_fd;
    uint16_t port;
};

// ─── Winsock init helper ────────────────────────────────────────────────────

#ifdef _WIN32
static int s_wsa_init = 0;
static void ensure_wsa(void) {
    if (!s_wsa_init) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        s_wsa_init = 1;
    }
}
#else
static void ensure_wsa(void) {}
#endif

// ─── Create ─────────────────────────────────────────────────────────────────

fw_transport_t* fw_transport_create_tcp(uint16_t port) {
    ensure_wsa();

    fw_transport_t* t = (fw_transport_t*)calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->client_fd = INVALID_SOCK;
    t->port      = port;

    t->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (t->listen_fd == INVALID_SOCK) { free(t); return NULL; }

    int opt = 1;
    setsockopt(t->listen_fd, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (bind(t->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        CLOSE_SOCKET(t->listen_fd);
        free(t);
        return NULL;
    }

    if (listen(t->listen_fd, 1) != 0) {
        CLOSE_SOCKET(t->listen_fd);
        free(t);
        return NULL;
    }

    return t;
}

// ─── Accept ─────────────────────────────────────────────────────────────────

int fw_transport_accept(fw_transport_t* t) {
    if (!t) return -1;
    if (t->client_fd != INVALID_SOCK) return 0;  // already connected

    t->client_fd = accept(t->listen_fd, NULL, NULL);
    return (t->client_fd == INVALID_SOCK) ? -1 : 0;
}

// ─── Send ───────────────────────────────────────────────────────────────────

int fw_transport_send(fw_transport_t* t, const uint8_t* data, size_t len) {
    if (!t || t->client_fd == INVALID_SOCK) return -1;
    size_t total_sent = 0;
    while (total_sent < len) {
        int n = send(t->client_fd, (const char*)(data + total_sent),
                     (int)(len - total_sent), 0);
        if (n <= 0) {
#ifndef _WIN32
            if (errno == EINTR) continue;
#endif
            return -1;
        }
        total_sent += (size_t)n;
    }
    return (int)total_sent;
}

// ─── Receive ────────────────────────────────────────────────────────────────

int fw_transport_recv(fw_transport_t* t, uint8_t* buf, size_t max_len,
                      uint32_t timeout_ms) {
    if (!t || t->client_fd == INVALID_SOCK) return -1;

#ifdef _WIN32
    // Use select() for timeout on Windows
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(t->client_fd, &fds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int ready = select(0, &fds, NULL, NULL, &tv);
    if (ready < 0) return -1;   // error
    if (ready == 0) return 0;   // timeout / no data
#else
    struct pollfd pfd;
    pfd.fd = t->client_fd;
    pfd.events = POLLIN;
    int ready = poll(&pfd, 1, (int)timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) return 0;
        return -1;   // error
    }
    if (ready == 0) return 0;   // timeout / no data
#endif

    int n = recv(t->client_fd, (char*)buf, (int)max_len, 0);
    if (n <= 0) return -1;  // disconnected or error
    return n;
}

// ─── Status / cleanup ───────────────────────────────────────────────────────

int fw_transport_is_connected(const fw_transport_t* t) {
    return (t && t->client_fd != INVALID_SOCK) ? 1 : 0;
}

void fw_transport_close_client(fw_transport_t* t) {
    if (t && t->client_fd != INVALID_SOCK) {
        CLOSE_SOCKET(t->client_fd);
        t->client_fd = INVALID_SOCK;
    }
}

void fw_transport_destroy(fw_transport_t* t) {
    if (!t) return;
    fw_transport_close_client(t);
    if (t->listen_fd != INVALID_SOCK) CLOSE_SOCKET(t->listen_fd);
    free(t);
}
