// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/someip/e2e.hpp"

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
    return 0;
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

    const uint64_t crc = e2e_crc64_xz(buf + kE2eOffsetLength, len - kE2eOffsetLength);
    return (crc == out.crc) ? E2eStatus::kOk : E2eStatus::kCrcMismatch;
}

}  // namespace afi920_driver
