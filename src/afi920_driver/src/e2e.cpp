// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/someip/e2e.hpp"

#include <array>

namespace afi920_driver {

namespace {

uint64_t unpack_be64(const uint8_t* p)
{
    return (static_cast<uint64_t>(p[0]) << 56) |
           (static_cast<uint64_t>(p[1]) << 48) |
           (static_cast<uint64_t>(p[2]) << 40) |
           (static_cast<uint64_t>(p[3]) << 32) |
           (static_cast<uint64_t>(p[4]) << 24) |
           (static_cast<uint64_t>(p[5]) << 16) |
           (static_cast<uint64_t>(p[6]) << 8) |
           static_cast<uint64_t>(p[7]);
}

uint32_t unpack_be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

void pack_be32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

// RDI bodies reach ~100 KB per frame, so the byte-wise table is worth its 1 KB.
const std::array<uint32_t, 256>& crc32_table()
{
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int bit = 0; bit < 8; ++bit) {
                c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
            }
            t[i] = c;
        }
        return t;
    }();
    return table;
}

}  // namespace

int e2e_parse(const uint8_t* buf, size_t len, E2eHeader& out)
{
    if (!buf || len < kE2eHeaderSize) {
        return -1;
    }

    out.crc = unpack_be64(buf + kE2eOffsetCrc);
    out.length = unpack_be32(buf + kE2eOffsetLength);
    out.counter = unpack_be32(buf + kE2eOffsetCounter);
    out.data_id = unpack_be32(buf + kE2eOffsetDataId);
    out.format = E2eCrcFormat::kUnknown;
    return 0;
}

uint32_t e2e_crc32_update(uint32_t crc, const uint8_t* data, size_t len)
{
    if (!data) {
        return crc;
    }

    const auto& table = crc32_table();
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc = (crc >> 8) ^ table[(crc ^ data[i]) & 0xFFu];
    }
    return ~crc;
}

uint32_t e2e_crc32(const uint8_t* data, size_t len)
{
    return e2e_crc32_update(0u, data, len);
}

uint64_t e2e_crc64_xz_update(uint64_t crc, const uint8_t* data, size_t len)
{
    static constexpr uint64_t kReflectedPoly = 0xC96C5795D7870F42ull;

    if (!data && len > 0) {
        return crc;
    }

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? ((crc >> 1) ^ kReflectedPoly) : (crc >> 1);
        }
    }
    return crc;
}

uint64_t e2e_crc64_xz(const uint8_t* data, size_t len)
{
    return e2e_crc64_xz_update(kE2eCrc64Init, data, len) ^ kE2eCrc64Xorout;
}

E2eStatus e2e_validate(const uint8_t* buf, size_t len, E2eHeader& out)
{
    if (e2e_parse(buf, len, out) < 0) {
        return E2eStatus::kTooShort;
    }
    if (out.crc == 0) {
        return E2eStatus::kCrcZero;
    }

    // v1.1.0+ keeps the upper CRC word at 0. A v1.0.x CRC-64 has a zero upper
    // word with probability 2^-32, so a CRC-32 miss still falls through.
    if ((out.crc >> 32) == 0) {
        const uint32_t crc32 = e2e_crc32(buf + kE2eHeaderSize, len - kE2eHeaderSize);
        if (crc32 == static_cast<uint32_t>(out.crc)) {
            out.format = E2eCrcFormat::kCrc32;
            return E2eStatus::kOk;
        }
    }

    const uint64_t crc64 = e2e_crc64_xz(buf + kE2eOffsetLength, len - kE2eOffsetLength);
    if (crc64 == out.crc) {
        out.format = E2eCrcFormat::kCrc64Xz;
        return E2eStatus::kOk;
    }
    return E2eStatus::kCrcMismatch;
}

void e2e_build_header(uint8_t* out, uint32_t data_id, uint32_t counter,
                      const uint8_t* body, size_t body_len)
{
    pack_be32(out + kE2eOffsetCrc, 0u);
    pack_be32(out + kE2eOffsetCrc32, e2e_crc32(body, body_len));
    pack_be32(out + kE2eOffsetLength, static_cast<uint32_t>(body_len));
    pack_be32(out + kE2eOffsetCounter, counter);
    pack_be32(out + kE2eOffsetDataId, data_id);
}

const char* e2e_crc_format_name(E2eCrcFormat format)
{
    switch (format) {
        case E2eCrcFormat::kCrc32:
            return "CRC-32 (interface v1.1.0+)";
        case E2eCrcFormat::kCrc64Xz:
            return "CRC-64/XZ (interface v1.0.x)";
        default:
            return "unknown";
    }
}

E2eCounterUpdate E2eCounterTracker::Update(uint32_t counter)
{
    E2eCounterUpdate result;
    if (has_last_) {
        const uint32_t delta = counter - last_;
        if (static_cast<int32_t>(delta) < 0) {
            result.restarted = true;
        } else if (delta > 1u) {
            result.missing = delta - 1u;
        }
    }
    has_last_ = true;
    last_ = counter;
    return result;
}

}  // namespace afi920_driver
