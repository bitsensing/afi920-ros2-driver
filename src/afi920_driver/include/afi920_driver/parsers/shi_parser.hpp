// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file shi_parser.hpp
 * @brief SHII (Sensor Health Information Interface) payload parser
 *
 * Variable-length payload, little-endian.
 */
#pragma once

#include "afi920_driver/radar_types.hpp"
#include <cstdint>
#include <cstddef>

namespace afi920_driver {

class ShiParser {
public:
    /**
     * @brief Parse SHII payload (variable length, little-endian)
     * @return 0 on success, negative on error
     */
    int Parse(const uint8_t* payload, size_t len, ShiMessage& msg);
};

}  // namespace afi920_driver
