// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/transport/tcp_stream_client.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socket_t = SOCKET;
static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
static inline void close_socket(socket_t s) { closesocket(s); }
static inline bool socket_valid(socket_t s) { return s != INVALID_SOCKET; }
#else
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
using socket_t = int;
static constexpr socket_t kInvalidSocket = -1;
static inline void close_socket(socket_t s) { ::close(s); }
static inline bool socket_valid(socket_t s) { return s >= 0; }
#endif

namespace afi920_driver {

TcpStreamClient::TcpStreamClient(const std::string& sensor_ip, uint16_t port,
                                 size_t recv_buf_size, int connect_timeout_ms)
    : sensor_ip_(sensor_ip),
      port_(port),
      recv_buf_size_(recv_buf_size),
      connect_timeout_ms_(connect_timeout_ms) {}

TcpStreamClient::~TcpStreamClient() {
    Close();
}

int TcpStreamClient::Connect() {
#ifdef _WIN32
    socket_t fd = socket_fd_;
    if (fd != static_cast<socket_t>(static_cast<uintptr_t>(~0))) {
#else
    int fd = socket_fd_;
    if (fd >= 0) {
#endif
        close_socket(fd);
        open_ = false;
    }

    socket_t new_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (!socket_valid(new_fd)) {
        return -1;
    }

    // Set non-blocking for connect with timeout
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(new_fd, FIONBIO, &mode);
#else
    int flags = fcntl(new_fd, F_GETFL, 0);
    fcntl(new_fd, F_SETFL, flags | O_NONBLOCK);
#endif

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, sensor_ip_.c_str(), &addr.sin_addr) <= 0) {
        close_socket(new_fd);
        return -1;
    }

    int ret = ::connect(new_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));

#ifdef _WIN32
    if (ret == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK) {
        close_socket(new_fd);
        return -1;
    }
#else
    if (ret < 0 && errno != EINPROGRESS) {
        close_socket(new_fd);
        return -1;
    }
#endif

    // Wait for connect with timeout using select
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(new_fd, &wfds);
    struct timeval tv;
    tv.tv_sec = connect_timeout_ms_ / 1000;
    tv.tv_usec = (connect_timeout_ms_ % 1000) * 1000;

#ifdef _WIN32
    int sel = ::select(0, nullptr, &wfds, nullptr, &tv);
#else
    int sel = ::select(static_cast<int>(new_fd) + 1, nullptr, &wfds, nullptr, &tv);
#endif
    if (sel <= 0) {
        close_socket(new_fd);
        return -1;
    }

    // Check SO_ERROR to confirm connect succeeded
    int err = 0;
    socklen_t errlen = sizeof(err);
    getsockopt(new_fd, SOL_SOCKET, SO_ERROR,
               reinterpret_cast<char*>(&err), &errlen);
    if (err != 0) {
        close_socket(new_fd);
        return -1;
    }

    // Restore blocking mode for recv
#ifdef _WIN32
    u_long block = 0;
    ioctlsocket(new_fd, FIONBIO, &block);
#else
    int blk_flags = fcntl(new_fd, F_GETFL, 0);
    fcntl(new_fd, F_SETFL, blk_flags & ~O_NONBLOCK);
#endif

#ifdef _WIN32
    socket_fd_ = static_cast<uintptr_t>(new_fd);
#else
    socket_fd_ = new_fd;
#endif

    configure_socket();
    open_ = true;
    reconnect_delay_ms_ = kMinReconnectDelay;
    return 0;
}

int TcpStreamClient::Receive(uint8_t* buf, size_t buf_size) {
    if (!open_) {
        return try_reconnect();
    }

    // Read exactly 8 bytes: SOME/IP header first half (MessageID[4] + Length[4])
    uint8_t hdr[8];
    if (recv_exact(hdr, 8) != 0) {
        open_ = false;
        return -1;
    }

    // Parse Length field (big-endian uint32 at offset 4)
    uint32_t length = (static_cast<uint32_t>(hdr[4]) << 24) |
                      (static_cast<uint32_t>(hdr[5]) << 16) |
                      (static_cast<uint32_t>(hdr[6]) << 8)  |
                      static_cast<uint32_t>(hdr[7]);

    size_t total = 8u + static_cast<size_t>(length);
    if (total > buf_size) {
        // Buffer too small — drain and return error
        open_ = false;
        return -1;
    }

    // Copy header into caller's buffer
    memcpy(buf, hdr, 8);

    // Read the remaining `length` bytes
    if (recv_exact(buf + 8, static_cast<size_t>(length)) != 0) {
        open_ = false;
        return -1;
    }

    return static_cast<int>(total);
}

void TcpStreamClient::Close() {
    running_ = false;
    open_ = false;
#ifdef _WIN32
    socket_t fd = static_cast<socket_t>(socket_fd_);
    if (fd != INVALID_SOCKET) {
        close_socket(fd);
        socket_fd_ = static_cast<uintptr_t>(~0);
    }
#else
    if (socket_fd_ >= 0) {
        close_socket(socket_fd_);
        socket_fd_ = -1;
    }
#endif
}

bool TcpStreamClient::IsOpen() const {
    return open_;
}

int TcpStreamClient::recv_exact(uint8_t* buf, size_t n) {
    size_t received = 0;
    while (received < n) {
#ifdef _WIN32
        socket_t fd = static_cast<socket_t>(socket_fd_);
        int ret = ::recv(fd, reinterpret_cast<char*>(buf + received),
                         static_cast<int>(n - received), 0);
        if (ret <= 0) {
            return -1;
        }
#else
        ssize_t ret = ::recv(socket_fd_, buf + received, n - received, 0);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (ret == 0) {
            return -1;  // connection closed
        }
#endif
        received += static_cast<size_t>(ret);
    }
    return 0;
}

void TcpStreamClient::configure_socket() {
#ifdef _WIN32
    socket_t fd = static_cast<socket_t>(socket_fd_);
#else
    int fd = socket_fd_;
#endif

    // TCP_NODELAY: disable Nagle's algorithm
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,
               reinterpret_cast<const char*>(&flag), sizeof(flag));

    // SO_KEEPALIVE
    int keepalive = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE,
               reinterpret_cast<const char*>(&keepalive), sizeof(keepalive));

#ifdef _WIN32
    // Windows: use SIO_KEEPALIVE_VALS via WSAIoctl
    struct tcp_keepalive {
        u_long onoff;
        u_long keepalivetime;
        u_long keepaliveinterval;
    } ka;
    ka.onoff = 1;
    ka.keepalivetime = 5000;     // 5s idle
    ka.keepaliveinterval = 1000;  // 1s interval
    DWORD bytes_returned = 0;
    WSAIoctl(fd, SIO_KEEPALIVE_VALS, &ka, sizeof(ka),
             nullptr, 0, &bytes_returned, nullptr, nullptr);
#else
    int idle = 5;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    int intvl = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    int cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif

    // SO_RCVBUF
    int rcvbuf = static_cast<int>(recv_buf_size_);
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF,
               reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));
}

int TcpStreamClient::try_reconnect() {
    if (!running_) return -1;

#ifdef _WIN32
    socket_t fd = static_cast<socket_t>(socket_fd_);
    if (fd != INVALID_SOCKET) {
        close_socket(fd);
        socket_fd_ = static_cast<uintptr_t>(~0);
    }
#else
    if (socket_fd_ >= 0) {
        close_socket(socket_fd_);
        socket_fd_ = -1;
    }
#endif
    open_ = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));

    if (Connect() == 0) {
        reconnect_delay_ms_ = kMinReconnectDelay;
        return 0;
    }

    // Exponential backoff, cap at kMaxReconnectDelay
    reconnect_delay_ms_ = std::min(reconnect_delay_ms_ * 2, kMaxReconnectDelay);
    return -1;
}

}  // namespace afi920_driver
