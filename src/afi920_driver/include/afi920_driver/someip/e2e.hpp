// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file e2e.hpp
 * @brief AUTOSAR E2E Profile 7 header parsing and CRC validation.
 *
 * RDI/SHII/SPI payloads carry a 20-byte E2E header between the SOME/IP
 * header and the ISO 23150 payload. The E2E fields are big-endian.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace afi920_driver {

static constexpr size_t kE2eHeaderSize = 20;
static constexpr size_t kE2eOffsetCrc = 0;
static constexpr size_t kE2eOffsetLength = 8;
static constexpr size_t kE2eOffsetCounter = 12;
static constexpr size_t kE2eOffsetDataId = 16;

static constexpr uint64_t kE2eCrc64Init = 0xFFFFFFFFFFFFFFFFull;
static constexpr uint64_t kE2eCrc64Xorout = 0xFFFFFFFFFFFFFFFFull;

static constexpr uint32_t kE2eDataIdCsii = 0x60008001u;
static constexpr uint32_t kE2eDataIdRdi = 0x60008002u;
static constexpr uint32_t kE2eDataIdShii = 0x60008003u;
static constexpr uint32_t kE2eDataIdSpi = 0x60008004u;

struct E2eHeader {
    uint64_t crc = 0;
    uint32_t length = 0;
    uint32_t counter = 0;
    uint32_t data_id = 0;
};

enum class E2eStatus {
    kOk = 0,
    kCrcMismatch,
    kTooShort,
};

int e2e_parse(const uint8_t* buf, size_t len, E2eHeader& out);
E2eStatus e2e_validate(const uint8_t* buf, size_t len, E2eHeader& out);

uint64_t e2e_crc64_xz_update(uint64_t crc, const uint8_t* data, size_t len);
uint64_t e2e_crc64_xz(const uint8_t* data, size_t len);

}  // namespace afi920_driver
