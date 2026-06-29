// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file rdi_parser.hpp
 * @brief RDI (Radar Detection Interface) payload parser
 *
 * Deserializes ISO 23150 RDI binary payload into RdiFrame.
 * Header: 36 bytes + Detections: 51 bytes each (little-endian).
 */
#pragma once

#include "afi920_driver/radar_types.hpp"
#include <cstdint>
#include <cstddef>

namespace afi920_driver {

class RdiParser {
public:
    /**
     * @brief Parse RDI payload (after SOME/IP header removal & TP reassembly)
     * @param payload Raw RDI payload bytes (little-endian)
     * @param len Payload length
     * @param[out] frame Output frame (detections_storage will be populated)
     * @return 0 on success, negative on error
     */
    int Parse(const uint8_t* payload, size_t len, RdiFrame& frame);

private:
    static int ParseDetection(const uint8_t* data, bts_rdi_detection_t& det);
};

}  // namespace afi920_driver
