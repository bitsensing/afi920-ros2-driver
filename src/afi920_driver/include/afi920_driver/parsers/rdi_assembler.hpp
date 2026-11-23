// Copyright (c) 2026 bitsensing Inc. All rights reserved.
/**
 * @file rdi_assembler.hpp
 * @brief Interface v1.2.0 RDI scan assembly from header and chunk messages
 *
 * One scan is header A (Event 0x8002), chunk k (Event 0x8100 + k) for
 * k = 0..K-1 and header B (Event 0x8002 again), all under one E2E Counter.
 * The assembler follows Interface Overview §3.2.3:
 *   - A message whose E2E Counter equals one of the last two closed scans is
 *     late and is dropped without opening or closing a scan.
 *   - A message with another E2E Counter closes the open scan and opens one.
 *   - The first 0x8002 of a scan is header A, the second header B.
 *   - A scan closes when one header and ChunkCount chunks are in, on header B,
 *     on a message of another E2E Counter, or 100 ms after its first message.
 *   - A closed scan without a usable header has no detections; one with
 *     missing chunks keeps the detections that arrived.
 *   - A chunk is dropped when its size, MessageCounter or Event index does
 *     not fit the header, or its FirstEntityIndex is not a multiple of
 *     EntitiesPerChunk. Kept chunks are placed in FirstEntityIndex order.
 */
#pragma once

#include "afi920_driver/parsers/rdi_parser.hpp"
#include "afi920_driver/radar_types.hpp"
#include "afi920_driver/someip/e2e.hpp"
#include "afi920_driver/someip/someip_parser.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace afi920_driver {

/// E2E check result of one header or chunk message.
struct RdiMessageE2e {
    E2eStatus status = E2eStatus::kOk;
    E2eCrcFormat format = E2eCrcFormat::kUnknown;
    bool length_ok = true;
    bool data_id_ok = true;
};

/// One closed scan.
struct RdiScan {
    RdiFrame frame;                     ///< valid only when has_header
    bool has_header = false;            ///< a 40-byte header message was received
    uint32_t e2e_counter = 0;
    uint16_t chunk_count = 0;           ///< ChunkCount of the header
    uint16_t chunks_received = 0;       ///< chunks whose detections were used
    uint16_t chunks_rejected = 0;       ///< chunks dropped for size, MessageCounter, index or FirstEntityIndex
    uint16_t announced_detections = 0;  ///< NumberOfValidDetections of the header
    /// Worst result over every message of the scan: CRC mismatch, then CRC
    /// zero, then OK. length_ok / data_id_ok are false when any message failed.
    RdiMessageE2e e2e;

    bool IsComplete() const { return has_header && chunks_received == chunk_count; }
};

class RdiScanAssembler {
public:
    using Emit = std::function<void(RdiScan&)>;

    static constexpr uint64_t kDefaultTimeoutNs = 100000000ull;  // 100 ms

    explicit RdiScanAssembler(uint64_t timeout_ns = kDefaultTimeoutNs)
        : timeout_ns_(timeout_ns) {}

    /// Header message body (after the E2E header) of Event 0x8002.
    void OnHeader(uint32_t e2e_counter, const RdiMessageE2e& e2e,
                  const uint8_t* body, size_t len, uint64_t now_ns,
                  const Emit& emit);

    /// Chunk message body (after the E2E header) of Event 0x8100 + chunk_index.
    void OnChunk(uint32_t e2e_counter, const RdiMessageE2e& e2e, uint16_t chunk_index,
                 const uint8_t* body, size_t len, uint64_t now_ns,
                 const Emit& emit);

    /// Close the open scan once its first message is older than the timeout.
    void Poll(uint64_t now_ns, const Emit& emit);

    /// Drop the open scan and the closed-scan history.
    void Reset();

    uint64_t LateMessages() const { return late_messages_; }

private:
    struct Chunk {
        bool present = false;
        bool e2e_ok = false;
        std::vector<uint8_t> body;
    };

    // §3.2.3 rules (1) to (3); false when the message is late.
    bool Admit(uint32_t e2e_counter, const RdiMessageE2e& e2e, uint64_t now_ns,
               const Emit& emit);
    bool IsComplete() const;
    void Close(const Emit& emit);

    uint64_t timeout_ns_;

    bool     open_ = false;
    uint32_t counter_ = 0;
    uint64_t first_ns_ = 0;
    int      headers_seen_ = 0;
    bool     header_e2e_ok_ = false;
    std::vector<uint8_t> header_;        // body of the header used (empty: none)
    std::array<Chunk, kRdiMaxChunks> chunks_{};
    RdiMessageE2e e2e_;                  // aggregate of the open scan

    uint32_t closed_[2] = {0, 0};        // E2E Counters of the last two closed scans
    size_t   closed_count_ = 0;

    uint64_t late_messages_ = 0;
};

}  // namespace afi920_driver
