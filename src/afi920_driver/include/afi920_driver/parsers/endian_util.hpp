// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file endian_util.hpp
 * @brief Endianness unpack helpers for protocol deserialization
 *
 * Big-endian:  SOME/IP header (16 bytes)
 * Little-endian: ISO 23150 payload (RDI/SHII/SPI)
 */
#pragma once

#include <cstdint>
#include <cstring>

namespace afi920_driver {
namespace detail {

// ─── Big-endian unpack (for SOME/IP header) ───

inline uint16_t unpack_be16(const uint8_t* p) {
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

inline uint32_t unpack_be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
           (static_cast<uint32_t>(p[3]));
}

// ─── Little-endian unpack (for ISO 23150 payload) ───

inline uint16_t unpack_le16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline uint32_t unpack_le32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]))       |
           (static_cast<uint32_t>(p[1]) << 8)  |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t unpack_le64(const uint8_t* p) {
    return (static_cast<uint64_t>(p[0]))       |
           (static_cast<uint64_t>(p[1]) << 8)  |
           (static_cast<uint64_t>(p[2]) << 16) |
           (static_cast<uint64_t>(p[3]) << 24) |
           (static_cast<uint64_t>(p[4]) << 32) |
           (static_cast<uint64_t>(p[5]) << 40) |
           (static_cast<uint64_t>(p[6]) << 48) |
           (static_cast<uint64_t>(p[7]) << 56);
}

inline float unpack_le_f32(const uint8_t* p) {
    uint32_t u = unpack_le32(p);
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

}  // namespace detail
}  // namespace afi920_driver
