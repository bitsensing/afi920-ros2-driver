// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#pragma once

#include <cstdint>
#include <cstddef>

namespace afi920_driver {

/**
 * @brief Abstract base class for stream receivers (UDP or TCP)
 *
 * Configuration (IP, port, buffer sizes) is passed via each subclass constructor.
 * The base interface provides only connection lifecycle and data reception.
 */
class StreamReceiver {
public:
    virtual ~StreamReceiver() = default;

    /// Connect/bind to the configured endpoint. Returns 0 on success, negative on error.
    virtual int Connect() = 0;

    /// Receive one complete SOME/IP message into buf.
    /// Returns number of bytes received, 0 on timeout, negative on error.
    virtual int Receive(uint8_t* buf, size_t buf_size) = 0;

    /// Close the connection/socket.
    virtual void Close() = 0;

    /// Check if the receiver is connected/open.
    virtual bool IsOpen() const = 0;
};

}  // namespace afi920_driver
