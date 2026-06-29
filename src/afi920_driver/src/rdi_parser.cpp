// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/parsers/rdi_parser.hpp"

#include "afi920_driver/parsers/endian_util.hpp"

#include <cstring>

namespace afi920_driver {

static constexpr size_t kRdiHeaderSize = BTS_RDI_HEADER_SIZE;
static constexpr size_t kRdiDetectionSize = BTS_RDI_DETECTION_SIZE;
static constexpr size_t kRdiMaxDetections = BTS_RDI_MAX_DETECTIONS;

int RdiParser::Parse(const uint8_t* payload, size_t len, RdiFrame& frame)
{
    if (!payload || len < kRdiHeaderSize) {
        return -1;
    }

    auto& msg = frame.message;
    const uint8_t* p = payload;

    // Common Interface Header (24 bytes)
    msg.interface_version.major = p[0];
    msg.interface_version.minor = p[1];
    msg.interface_version.patch = p[2];
    msg.interface_id = p[3];
    msg.num_serving_sensors = p[4];
    msg.sensor_id = p[5];
    msg.timestamp = detail::unpack_le64(p + 6);
    msg.message_counter = detail::unpack_le32(p + 14);
    msg.interface_cycle_time = detail::unpack_le32(p + 18);
    msg.interface_cycle_time_variation = p[22];
    msg.data_qualifier = static_cast<bts_data_qualifier_t>(p[23]);

    // Current AFI920 RDI wire header:
    //   24B common + 8B radial velocity ambiguity + 4B detection list info.
    msg.ambiguity_domain.radial_velocity_begin = detail::unpack_le_f32(p + 24);
    msg.ambiguity_domain.radial_velocity_end = detail::unpack_le_f32(p + 28);

    // These fields existed in an older ROS message surface, but current RDI
    // does not carry range/azimuth/elevation ambiguity domains.
    msg.ambiguity_domain.range_begin = 0.0f;
    msg.ambiguity_domain.range_end = 0.0f;
    msg.ambiguity_domain.azimuth_begin = 0.0f;
    msg.ambiguity_domain.azimuth_end = 0.0f;
    msg.ambiguity_domain.elevation_begin = 0.0f;
    msg.ambiguity_domain.elevation_end = 0.0f;

    msg.recognised_detections_capability = detail::unpack_le16(p + 32);
    msg.recognised_detections_status = 0;
    msg.num_detections = detail::unpack_le16(p + 34);

    if (msg.num_detections > kRdiMaxDetections) {
        msg.num_detections = static_cast<uint16_t>(kRdiMaxDetections);
    }

    const size_t expected = kRdiHeaderSize +
        static_cast<size_t>(msg.num_detections) * kRdiDetectionSize;
    if (len < expected) {
        msg.num_detections = static_cast<uint16_t>(
            (len - kRdiHeaderSize) / kRdiDetectionSize);
    }

    frame.detections_storage.resize(msg.num_detections);
    for (uint16_t i = 0; i < msg.num_detections; i++) {
        const uint8_t* det_ptr = payload + kRdiHeaderSize + i * kRdiDetectionSize;
        ParseDetection(det_ptr, frame.detections_storage[i]);
    }

    msg.detections = frame.detections_storage.data();

    return 0;
}

int RdiParser::ParseDetection(const uint8_t* d, bts_rdi_detection_t& det)
{
    std::memset(&det, 0, sizeof(det));

    // Status (9 bytes)
    det.existence_probability = d[0];
    det.detection_id = detail::unpack_le16(d + 1);
    det.object_id_reference = detail::unpack_le16(d + 3);
    det.timestamp_difference = detail::unpack_le_f32(d + 5);

    // Information (10 bytes)
    det.radar_cross_section = detail::unpack_le_f32(d + 9);
    det.signal_to_noise_ratio = detail::unpack_le_f32(d + 13);
    det.ambiguity_grouping_id = detail::unpack_le16(d + 17);

    // Position (24 bytes)
    det.position_radial_distance = detail::unpack_le_f32(d + 19);
    det.position_azimuth = detail::unpack_le_f32(d + 23);
    det.position_elevation = detail::unpack_le_f32(d + 27);
    det.position_radial_distance_error = detail::unpack_le_f32(d + 31);
    det.position_azimuth_error = detail::unpack_le_f32(d + 35);
    det.position_elevation_error = detail::unpack_le_f32(d + 39);

    // Dynamics (8 bytes)
    det.relative_velocity_radial = detail::unpack_le_f32(d + 43);
    det.relative_velocity_radial_error = detail::unpack_le_f32(d + 47);

    return 0;
}

}  // namespace afi920_driver
