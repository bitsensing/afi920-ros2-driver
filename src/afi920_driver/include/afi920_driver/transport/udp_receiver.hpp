// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file udp_receiver.hpp
 * @brief UDP socket receiver for data streams (POSIX)
 *
 * Simple blocking UDP receiver with timeout support.
 * One instance per data stream (RDI/SHII/SPI).
 */
#pragma once

#include "afi920_driver/transport/stream_receiver.hpp"
#include <cstdint>
#include <cstddef>

namespace afi920_driver {

class UdpReceiver : public StreamReceiver {
public:
    /**
     * @brief Construct a UdpReceiver with all socket parameters
     * @param port        UDP port to bind
     * @param buffer_size SO_RCVBUF size (default 16 MB)
     * @param timeout_ms  Receive timeout in milliseconds
     */
    explicit UdpReceiver(uint16_t port,
                         uint32_t buffer_size = 16 * 1024 * 1024,
                         int timeout_ms = 1000);
    ~UdpReceiver() override;

    UdpReceiver(const UdpReceiver&) = delete;
    UdpReceiver& operator=(const UdpReceiver&) = delete;

    /**
     * @brief Open UDP socket and bind to the configured port
     * @return 0 on success, negative on error
     */
    int Connect() override;

    /**
     * @brief Receive a single UDP datagram (blocking with timeout)
     * @return Number of bytes received, 0 on timeout, -1 on error
     */
    int Receive(uint8_t* buf, size_t buf_size) override;

    /**
     * @brief Close the socket (unblocks any pending Receive)
     */
    void Close() override;

    bool IsOpen() const override { return socket_fd_ >= 0; }

private:
    uint16_t port_;
    uint32_t buffer_size_;
    int timeout_ms_;
    int socket_fd_ = -1;
};

}  // namespace afi920_driver
