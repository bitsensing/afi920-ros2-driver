// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/parsers/spi_parser.hpp"

#include "afi920_driver/parsers/endian_util.hpp"

#include <algorithm>
#include <cstring>

namespace afi920_driver {

static constexpr size_t kCommonHeaderSize = 24;
static constexpr size_t kVehicleCoordinateSize = 1;
static constexpr size_t kSpiMinSize =
    kCommonHeaderSize + kVehicleCoordinateSize + BTS_SPI_SENSOR_POSE_SIZE;

int SpiParser::Parse(const uint8_t* payload, size_t len, SpiMessage& msg)
{
    if (!payload || len < kSpiMinSize) {
        return -1;
    }

    std::memset(&msg, 0, sizeof(msg));

    const uint8_t* p = payload;
    size_t offset = 0;

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
    offset = kCommonHeaderSize;

    // Vehicle Coordinate System (1 byte)
    msg.vehicle_coordinate_system =
        static_cast<bts_spi_vehicle_coordinate_system_t>(p[offset]);
    offset += kVehicleCoordinateSize;

    // Current interface sensor pose (32 bytes).
    msg.sensor_pose.origin_point_x = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.origin_point_y = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.origin_point_z = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.orientation_yaw = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.orientation_pitch = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.orientation_roll = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.orientation_error_yaw = detail::unpack_le_f32(p + offset); offset += 4;
    msg.sensor_pose.orientation_error_pitch = detail::unpack_le_f32(p + offset); offset += 4;

    // Current wire payload does not carry origin errors or roll error.
    msg.sensor_pose.origin_point_error_x = 0.0f;
    msg.sensor_pose.origin_point_error_y = 0.0f;
    msg.sensor_pose.origin_point_error_z = 0.0f;
    msg.sensor_pose.orientation_error_roll = 0.0f;

    // FOV Segments (variable)
    if (offset >= len) return 0;
    const uint8_t fov_declared = p[offset++];
    const uint8_t n_fov = std::min(fov_declared, static_cast<uint8_t>(BTS_SPI_MAX_FOV_SEGMENTS));
    if (offset + static_cast<size_t>(fov_declared) * BTS_SPI_FOV_SEGMENT_SIZE > len) {
        return -1;
    }
    msg.num_fov_segments = n_fov;
    for (uint8_t i = 0; i < fov_declared; i++) {
        if (i < n_fov) {
            auto& seg = msg.fov_segments[i];
            seg.azimuth_begin = detail::unpack_le_f32(p + offset);
            seg.azimuth_end = detail::unpack_le_f32(p + offset + 4);
            seg.elevation_begin = detail::unpack_le_f32(p + offset + 8);
            seg.elevation_end = detail::unpack_le_f32(p + offset + 12);
            seg.blockage_status = static_cast<bts_spi_blockage_status_t>(p[offset + 16]);
        }
        offset += BTS_SPI_FOV_SEGMENT_SIZE;
    }

    // Recognisable Object Types (variable)
    if (offset >= len) return 0;
    const uint8_t obj_declared = p[offset++];
    const uint8_t n_obj = std::min(obj_declared, static_cast<uint8_t>(BTS_SPI_MAX_OBJECT_TYPES));
    if (offset + static_cast<size_t>(obj_declared) * BTS_SPI_OBJECT_TYPE_SIZE > len) {
        return -1;
    }
    msg.num_recognisable_object_types = n_obj;
    for (uint8_t i = 0; i < obj_declared; i++) {
        if (i < n_obj) {
            auto& obj = msg.recognisable_object_types[i];
            obj.object_type = static_cast<bts_spi_recognised_object_type_t>(p[offset]);
            obj.detection_range_begin = detail::unpack_le_f32(p + offset + 1);
            obj.detection_range_end = detail::unpack_le_f32(p + offset + 5);
        }
        offset += BTS_SPI_OBJECT_TYPE_SIZE;
    }

    // Reference Target Types (variable)
    if (offset >= len) return 0;
    const uint8_t ref_declared = p[offset++];
    const uint8_t n_ref = std::min(ref_declared, static_cast<uint8_t>(BTS_SPI_MAX_REF_TARGETS));
    if (offset + static_cast<size_t>(ref_declared) * BTS_SPI_REF_TARGET_SIZE > len) {
        return -1;
    }
    msg.num_reference_target_types = n_ref;
    for (uint8_t i = 0; i < ref_declared; i++) {
        if (i < n_ref) {
            auto& ref = msg.reference_target_types[i];
            ref.radar_cross_section = detail::unpack_le_f32(p + offset);
            ref.detection_range_begin = detail::unpack_le_f32(p + offset + 4);
            ref.detection_range_end = detail::unpack_le_f32(p + offset + 8);
            ref.signal_to_noise_ratio = detail::unpack_le_f32(p + offset + 12);
        }
        offset += BTS_SPI_REF_TARGET_SIZE;
    }

    return 0;
}

}  // namespace afi920_driver
