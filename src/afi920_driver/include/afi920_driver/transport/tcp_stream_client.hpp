// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#pragma once

#include "afi920_driver/transport/stream_receiver.hpp"
#include <string>
#include <cstdint>
#include <atomic>
#include <vector>

namespace afi920_driver {

/**
 * @brief TCP stream client for receiving SOME/IP messages over TCP
 *
 * Connects to the sensor's TCP server port and receives SOME/IP framed messages.
 * TCP framing: read 8B (MessageID + Length), then read Length more bytes.
 * Total message = 8 + Length bytes.
 *
 * Auto-reconnects on connection drop with exponential backoff.
 * Configures TCP keepalive: IDLE=5s, INTVL=1s, CNT=3.
 */
class TcpStreamClient : public StreamReceiver {
public:
    /**
     * @param sensor_ip          Sensor IP address to connect to
     * @param port               TCP port number
     * @param recv_buf_size      Application receive buffer size (default 512KB for RDI)
     * @param connect_timeout_ms Connection timeout in milliseconds
     */
    explicit TcpStreamClient(const std::string& sensor_ip, uint16_t port,
                             size_t recv_buf_size = 512 * 1024,
                             int connect_timeout_ms = 5000);
    ~TcpStreamClient() override;

    int Connect() override;
    int Receive(uint8_t* buf, size_t buf_size) override;
    void Close() override;
    bool IsOpen() const override;

private:
    /// Read exactly n bytes from socket. Returns 0 on success, -1 on error/disconnect.
    int recv_exact(uint8_t* buf, size_t n);

    /// Configure TCP keepalive and TCP_NODELAY on the socket
    void configure_socket();

    /// Attempt reconnection with exponential backoff
    int try_reconnect();

    std::string sensor_ip_;
    uint16_t port_;
    size_t recv_buf_size_;
    int connect_timeout_ms_;

#ifdef _WIN32
    uintptr_t socket_fd_{static_cast<uintptr_t>(~0)};  // INVALID_SOCKET
#else
    int socket_fd_{-1};
#endif
    std::atomic<bool> open_{false};
    std::atomic<bool> running_{true};

    int reconnect_delay_ms_{1000};
    static constexpr int kMaxReconnectDelay = 30000;
    static constexpr int kMinReconnectDelay = 1000;
};

}  // namespace afi920_driver
