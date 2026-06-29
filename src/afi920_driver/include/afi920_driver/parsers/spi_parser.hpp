// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file spi_parser.hpp
 * @brief SPI (Sensor Performance Interface) payload parser
 *
 * Variable-length payload: header + pose + FOV segments +
 * recognisable objects + reference targets. Little-endian.
 */
#pragma once

#include "afi920_driver/radar_types.hpp"
#include <cstdint>
#include <cstddef>

namespace afi920_driver {

class SpiParser {
public:
    /**
     * @brief Parse SPI payload (variable length, little-endian)
     * @return 0 on success, negative on error
     */
    int Parse(const uint8_t* payload, size_t len, SpiMessage& msg);
};

}  // namespace afi920_driver
