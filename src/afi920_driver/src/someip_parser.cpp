// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/someip/someip_parser.hpp"
#include "afi920_driver/parsers/endian_util.hpp"
#include <cstring>

namespace afi920_driver {

// ─── SOME/IP Header Deserialization ─────────────────────────────────────────

int someip_deserialize(const uint8_t* buf, size_t len, SomeipHeader& out) {
    if (!buf || len < kSomeipHeaderSize) return -1;

    uint32_t message_id = detail::unpack_be32(buf + 0);
    out.service_id = static_cast<uint16_t>((message_id >> 16) & 0xFFFF);
    out.method_id  = static_cast<uint16_t>(message_id & 0xFFFF);
    out.length     = detail::unpack_be32(buf + 4);

    uint32_t request_id = detail::unpack_be32(buf + 8);
    out.client_id  = static_cast<uint16_t>((request_id >> 16) & 0xFFFF);
    out.session_id = static_cast<uint16_t>(request_id & 0xFFFF);

    out.protocol_version  = buf[12];
    out.interface_version = buf[13];
    out.message_type      = buf[14];
    out.return_code       = buf[15];

    return 0;
}

// ─── SOME/IP-TP Header Deserialization ──────────────────────────────────────

void someip_tp_deserialize(const uint8_t* buf, SomeipTpHeader& out) {
    uint32_t raw = detail::unpack_be32(buf);
    out.offset_bytes  = (raw >> 4) * 16;
    out.more_segments = (raw & 0x01) != 0;
}

// ─── SOME/IP Validation ──────────────────────────────────────────────────────

bool someip_validate(const SomeipHeader& hdr) {
    // Check Service ID
    if (hdr.service_id != kSomeipServiceId) {
        return false;
    }
    // Check Message Type: must be Notification (0x02) or TP-Notification (0x22)
    uint8_t base_type = hdr.message_type & 0x1F;
    if (base_type != kSomeipMsgTypeNotification) {
        return false;
    }
    return true;
}

// ─── TP Reassembler ─────────────────────────────────────────────────────────

bool SomeipTpReassembler::Feed(const uint8_t* segment_data, size_t data_len,
                                uint32_t offset_bytes, bool more_segments,
                                uint16_t session_id) {
    if (!segment_data || data_len == 0) return false;

    // Session ID mismatch: new message started, reset and begin fresh
    if (in_progress_ && session_id != session_id_) {
        Reset();
    }

    // Timeout check: stale reassembly, discard and start fresh
    if (in_progress_ && IsTimedOut()) {
        Reset();
    }

    if (!in_progress_) {
        in_progress_ = true;
        session_id_  = session_id;
        started_at_  = std::chrono::steady_clock::now();
        expected_total_ = 0;
    }

    size_t end = offset_bytes + data_len;
    if (end > buffer_.size()) {
        buffer_.resize(end);
    }

    std::memcpy(buffer_.data() + offset_bytes, segment_data, data_len);

    if (!more_segments) {
        expected_total_ = end;
        if (buffer_.size() >= expected_total_) {
            total_size_  = expected_total_;
            in_progress_ = false;
            return true;
        }
    }

    return false;
}

bool SomeipTpReassembler::IsTimedOut(std::chrono::milliseconds timeout) const {
    if (!in_progress_) return false;
    auto elapsed = std::chrono::steady_clock::now() - started_at_;
    return elapsed > timeout;
}

void SomeipTpReassembler::Reset() {
    total_size_     = 0;
    expected_total_ = 0;
    in_progress_    = false;
    session_id_     = 0;
}

}  // namespace afi920_driver
