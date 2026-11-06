// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file e2e.hpp
 * @brief E2E header (20 bytes, big-endian) parsing and CRC validation.
 *
 * RDI/SHII/SPI payloads carry a 20-byte E2E header between the SOME/IP
 * header and the ISO 23150 payload. Two CRC formats exist on the wire and
 * are told apart per frame:
 *
 *   Interface v1.1.0+ : bytes 0..3 = 0, bytes 4..7 = IEEE 802.3 CRC-32 over
 *                       the body only (the Length bytes after the header).
 *   Interface v1.0.x  : bytes 0..7 = CRC-64/XZ over header bytes 8..19 and
 *                       the body.
 *
 * Length / Counter / Data ID keep the same offsets in both formats.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace afi920_driver {

static constexpr size_t kE2eHeaderSize = 20;
static constexpr size_t kE2eOffsetCrc = 0;
static constexpr size_t kE2eOffsetCrc32 = 4;
static constexpr size_t kE2eOffsetLength = 8;
static constexpr size_t kE2eOffsetCounter = 12;
static constexpr size_t kE2eOffsetDataId = 16;

static constexpr uint64_t kE2eCrc64Init = 0xFFFFFFFFFFFFFFFFull;
static constexpr uint64_t kE2eCrc64Xorout = 0xFFFFFFFFFFFFFFFFull;

static constexpr uint32_t kE2eDataIdServiceBase = 0x60000000u;
static constexpr uint32_t kE2eDataIdCsii = 0x60008001u;
static constexpr uint32_t kE2eDataIdRdi = 0x60008002u;
static constexpr uint32_t kE2eDataIdShii = 0x60008003u;
static constexpr uint32_t kE2eDataIdSpi = 0x60008004u;

/// CRC format that matched the frame (kUnknown when no format matched).
enum class E2eCrcFormat : uint8_t {
    kUnknown = 0,
    kCrc32,     ///< Interface v1.1.0+
    kCrc64Xz,   ///< Interface v1.0.x
};

struct E2eHeader {
    uint64_t crc = 0;      ///< bytes 0..7 as u64 (equals the CRC-32 value on v1.1.0+)
    uint32_t length = 0;
    uint32_t counter = 0;
    uint32_t data_id = 0;
    E2eCrcFormat format = E2eCrcFormat::kUnknown;
};

enum class E2eStatus {
    kOk = 0,
    kCrcMismatch,
    kCrcZero,     ///< CRC field is 0; never treated as a pass
    kTooShort,
};

int e2e_parse(const uint8_t* buf, size_t len, E2eHeader& out);
E2eStatus e2e_validate(const uint8_t* buf, size_t len, E2eHeader& out);

/// IEEE 802.3 CRC-32 (zlib crc32). Chained calls start from 0.
uint32_t e2e_crc32_update(uint32_t crc, const uint8_t* data, size_t len);
uint32_t e2e_crc32(const uint8_t* data, size_t len);

uint64_t e2e_crc64_xz_update(uint64_t crc, const uint8_t* data, size_t len);
uint64_t e2e_crc64_xz(const uint8_t* data, size_t len);

/// Writes a v1.1.0 E2E header (CRC-32 over body) into out[0..19].
void e2e_build_header(uint8_t* out, uint32_t data_id, uint32_t counter,
                      const uint8_t* body, size_t body_len);

const char* e2e_crc_format_name(E2eCrcFormat format);

/// Result of feeding one E2E Counter into E2eCounterTracker.
struct E2eCounterUpdate {
    uint32_t missing = 0;     ///< counters skipped since the previous message
    bool restarted = false;   ///< counter decreased: sender restarted, baseline reset
};

/**
 * @brief Tracks the per-stream E2E Counter to count missing messages.
 *
 * v1.0.x increments the counter per message and v1.1.0+ per send slot, so in
 * both formats a skipped value is a message that did not arrive. A decrease
 * marks a sender restart and is not counted as loss.
 */
class E2eCounterTracker {
public:
    E2eCounterUpdate Update(uint32_t counter);
    void Reset() { has_last_ = false; }

private:
    bool has_last_ = false;
    uint32_t last_ = 0;
};

}  // namespace afi920_driver
