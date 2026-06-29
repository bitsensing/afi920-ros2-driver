// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/transport/udp_receiver.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

namespace afi920_driver {

UdpReceiver::UdpReceiver(uint16_t port, uint32_t buffer_size, int timeout_ms)
    : port_(port), buffer_size_(buffer_size), timeout_ms_(timeout_ms) {}

UdpReceiver::~UdpReceiver() {
    Close();
}

int UdpReceiver::Connect() {
    if (socket_fd_ >= 0) {
        Close();
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd_ < 0) {
        return -1;
    }

    // Allow port reuse
    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Set receive buffer size
    int buf_sz = static_cast<int>(buffer_size_);
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &buf_sz, sizeof(buf_sz));

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms_ / 1000;
    tv.tv_usec = (timeout_ms_ % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Bind to port
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        return -1;
    }

    return 0;
}

int UdpReceiver::Receive(uint8_t* buf, size_t buf_size) {
    if (socket_fd_ < 0) return -1;

    ssize_t n = ::recvfrom(socket_fd_, buf, buf_size, 0, nullptr, nullptr);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;  // timeout
        return -1;
    }

    return static_cast<int>(n);
}

void UdpReceiver::Close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

}  // namespace afi920_driver
