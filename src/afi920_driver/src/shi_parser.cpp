// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/parsers/shi_parser.hpp"

#include "afi920_driver/parsers/endian_util.hpp"

#include <algorithm>
#include <cstring>

namespace afi920_driver {

static constexpr size_t kCommonHeaderSize = 24;
static constexpr size_t kShiMinSize = 32;

int ShiParser::Parse(const uint8_t* payload, size_t len, ShiMessage& msg)
{
    if (!payload || len < kShiMinSize) {
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

    // Operation Modes
    if (offset >= len) return -1;
    const uint8_t op_declared = p[offset++];
    if (offset + op_declared > len) return -1;
    const uint8_t op_count = std::min(op_declared, static_cast<uint8_t>(BTS_SHI_MAX_OPERATION_MODES));
    msg.num_valid_operation_modes = op_count;
    std::memcpy(msg.sensor_operation_modes, p + offset, op_count);
    offset += op_declared;

    // Defect, supply, and temperature status
    if (offset + 4 > len) return -1;
    msg.sensor_defect_recognised =
        static_cast<bts_shi_sensor_defect_recognised_t>(p[offset++]);
    msg.sensor_defect_reason =
        static_cast<bts_shi_sensor_defect_reason_t>(p[offset++]);
    msg.supply_voltage_status =
        static_cast<bts_shi_supply_voltage_status_t>(p[offset++]);
    msg.sensor_temperature_status =
        static_cast<bts_shi_sensor_temperature_status_t>(p[offset++]);

    // Input Signal Status
    if (offset >= len) return -1;
    const uint8_t input_declared = p[offset++];
    if (offset + static_cast<size_t>(input_declared) * 2 > len) return -1;
    const uint8_t input_count =
        std::min(input_declared, static_cast<uint8_t>(BTS_SHI_MAX_INPUT_SIGNALS));
    msg.num_valid_input_signal_statuses = input_count;
    std::memcpy(msg.sensor_input_signal_types, p + offset, input_count);
    offset += input_declared;
    std::memcpy(msg.sensor_input_signal_statuses, p + offset, input_count);
    offset += input_declared;

    // Time Sync. Current interface removed the old time-sync offset float.
    if (offset >= len) return -1;
    msg.sensor_time_sync = static_cast<bts_shi_sensor_time_sync_t>(p[offset++]);
    msg.sensor_time_sync_offset_value = 0.0f;

    // Sensor calibration arrays (component, status, state parallel arrays).
    if (offset >= len) return -1;
    const uint8_t cal_declared = p[offset++];
    if (offset + static_cast<size_t>(cal_declared) * 3 > len) return -1;
    const uint8_t cal_count =
        std::min(cal_declared, static_cast<uint8_t>(BTS_SHI_MAX_CALIBRATION_COMPONENTS));
    msg.num_valid_calibration_components = cal_count;
    std::memcpy(msg.sensor_calibration_components, p + offset, cal_count);
    offset += cal_declared;
    std::memcpy(msg.sensor_calibration_statuses, p + offset, cal_count);
    offset += cal_declared;
    std::memcpy(msg.sensor_calibration_states, p + offset, cal_count);

    return 0;
}

}  // namespace afi920_driver
