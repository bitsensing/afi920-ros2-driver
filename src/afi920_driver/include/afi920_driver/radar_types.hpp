// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file radar_types.hpp
 * @brief Data types for AFI920 radar streams
 *
 * Replaces afi_types.h from the SDK. Contains only the types
 * needed for data streaming (no Config API types).
 */
#pragma once

#include <cstdint>
#include <vector>

#include "afi920_driver/interface/bts_iso23150.h"
#include "afi920_driver/interface/AFI920/bts_iso23150_rdi.h"
#include "afi920_driver/interface/AFI920/bts_iso23150_shi.h"
#include "afi920_driver/interface/AFI920/bts_iso23150_spi.h"

namespace afi920_driver {

/**
 * @brief RDI frame with host-side metadata
 *
 * The `detections` pointer in `message` points into `detections_storage`.
 */
struct RdiFrame {
    bts_rdi_message_t message{};
    uint64_t          recv_timestamp_ns = 0;
    uint32_t          frame_seq = 0;
    std::vector<bts_rdi_detection_t> detections_storage;
};

/// SHII message (direct C struct usage)
using ShiMessage = bts_shi_message_t;

/// SPI message (direct C struct usage)
using SpiMessage = bts_spi_message_t;

}  // namespace afi920_driver
