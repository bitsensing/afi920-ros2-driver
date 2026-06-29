// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file someip_parser.hpp
 * @brief SOME/IP header deserialization and TP reassembly
 *
 * SOME/IP header: 16 bytes, big-endian.
 * TP header: 4 bytes after SOME/IP header when TP flag is set.
 * TP reassembly: needed for RDI frames with >~26 detections (>1392 bytes).
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <cstddef>
#include <vector>

namespace afi920_driver {

// ─── Constants ───

static constexpr size_t   kSomeipHeaderSize          = 16;
static constexpr size_t   kSomeipTpHeaderSize        = 4;
static constexpr uint8_t  kSomeipTpFlag              = 0x20;
static constexpr uint16_t kSomeipServiceId           = 0x6000;
static constexpr uint8_t  kSomeipMsgTypeNotification = 0x02;

// Event IDs for data streams
static constexpr uint16_t kEventRdi     = 0x8002;
static constexpr uint16_t kEventShii    = 0x8003;
static constexpr uint16_t kEventSpi     = 0x8004;

// ─── SOME/IP Header ───

struct SomeipHeader {
    uint16_t service_id;
    uint16_t method_id;
    uint32_t length;
    uint16_t client_id;
    uint16_t session_id;
    uint8_t  protocol_version;
    uint8_t  interface_version;
    uint8_t  message_type;
    uint8_t  return_code;
};

/**
 * @brief Deserialize 16-byte SOME/IP header (big-endian)
 * @return 0 on success, -1 on error
 */
int someip_deserialize(const uint8_t* buf, size_t len, SomeipHeader& out);

inline bool someip_is_tp(uint8_t message_type) {
    return (message_type & kSomeipTpFlag) != 0;
}

/// Validate SOME/IP header fields (service_id, message_type)
/// Returns true if valid, false if packet should be discarded
bool someip_validate(const SomeipHeader& hdr);

// ─── SOME/IP-TP Header ───

struct SomeipTpHeader {
    uint32_t offset_bytes;
    bool     more_segments;
};

/**
 * @brief Deserialize 4-byte TP header (big-endian)
 */
void someip_tp_deserialize(const uint8_t* buf, SomeipTpHeader& out);

// ─── TP Reassembler ───

class SomeipTpReassembler {
public:
    SomeipTpReassembler() = default;

    /**
     * @brief Feed a TP segment
     * @param session_id  SOME/IP session_id from the header; resets reassembly on change
     * @return true if message is now complete
     */
    bool Feed(const uint8_t* segment_data, size_t data_len,
              uint32_t offset_bytes, bool more_segments,
              uint16_t session_id = 0);

    const uint8_t* GetPayload() const { return buffer_.data(); }
    size_t GetPayloadSize() const { return total_size_; }

    /**
     * @brief Returns true if reassembly has been in progress longer than timeout
     * @param timeout  maximum allowed reassembly duration (default 100ms per spec)
     */
    bool IsTimedOut(std::chrono::milliseconds timeout =
                        std::chrono::milliseconds(100)) const;

    /** @brief Reset for next message (buffer kept for reuse) */
    void Reset();

private:
    std::vector<uint8_t> buffer_;
    size_t total_size_    = 0;
    size_t expected_total_ = 0;
    bool   in_progress_   = false;
    uint16_t session_id_  = 0;
    std::chrono::steady_clock::time_point started_at_;
};

}  // namespace afi920_driver
