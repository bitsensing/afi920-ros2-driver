// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file rdi_parser.hpp
 * @brief RDI (Radar Detection Interface) payload parser
 *
 * Deserializes ISO 23150 RDI binary payload into RdiFrame.
 * Header: 36 bytes + Detections: 51 bytes each (little-endian).
 * Interface v1.2.0 header messages add a 4-byte List Layout after the header.
 */
#pragma once

#include "afi920_driver/radar_types.hpp"
#include <cstdint>
#include <cstddef>

namespace afi920_driver {

/// List Layout of an interface v1.2.0 RDI header message.
struct RdiListLayout {
    uint16_t entities_per_chunk = 0;
    uint16_t chunk_count = 0;
};

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

    /**
     * @brief Parse an interface v1.2.0 header message body (after the E2E header)
     * @param body Detection Header (36B) + List Layout (4B)
     * @param len Body length; must be BTS_RDI_HEADER_MSG_SIZE
     * @param[out] msg Header fields; num_detections is NumberOfValidDetections
     *                 and detections is left untouched
     * @param[out] layout EntitiesPerChunk and ChunkCount
     * @return 0 on success, -1 on a wrong length
     */
    static int ParseHeaderMessage(const uint8_t* body, size_t len,
                                  bts_rdi_message_t& msg, RdiListLayout& layout);

    /// Parse one 51-byte detection entity.
    static void ParseDetection(const uint8_t* data, bts_rdi_detection_t& det);

private:
    static void ParseDetectionHeader(const uint8_t* data, bts_rdi_message_t& msg);
};

}  // namespace afi920_driver
