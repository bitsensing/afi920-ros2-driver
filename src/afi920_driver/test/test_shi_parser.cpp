// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
// R2: SHI (SHII) Parser negative/adversarial tests
#include <gtest/gtest.h>

#include "afi920_driver/interface/AFI920/bts_iso23150_shi.h"
#include "afi920_driver/parsers/shi_parser.hpp"

#include <vector>

using afi920_driver::ShiMessage;
using afi920_driver::ShiParser;

static constexpr size_t kCommonHeader = 24;
static constexpr size_t kShiMinSize = 32;

// Helper: build a current variable-length SHII payload.
static std::vector<uint8_t> make_shi_payload(uint8_t n_op = 3,
                                             uint8_t n_input = 2,
                                             uint8_t n_cal = 3)
{
    const size_t total = kCommonHeader + 1 + n_op + 4 + 1 +
        2 * static_cast<size_t>(n_input) + 1 + 1 + 3 * static_cast<size_t>(n_cal);
    std::vector<uint8_t> buf(total, 0);
    buf[3] = 0x0D;  // interface_id = SHII
    buf[4] = 1;     // num_serving_sensors

    size_t off = kCommonHeader;
    buf[off++] = n_op;
    for (uint8_t i = 0; i < n_op; i++) buf[off++] = i;
    buf[off++] = 0;  // defect recognised
    buf[off++] = 0;  // defect reason
    buf[off++] = 2;  // supply voltage within limits
    buf[off++] = 2;  // temperature in limits
    buf[off++] = n_input;
    for (uint8_t i = 0; i < n_input; i++) buf[off++] = i;
    for (uint8_t i = 0; i < n_input; i++) buf[off++] = 0;
    buf[off++] = 0;  // time sync within limits
    buf[off++] = n_cal;
    for (uint8_t i = 0; i < n_cal; i++) buf[off++] = i;
    for (uint8_t i = 0; i < n_cal; i++) buf[off++] = 0;
    for (uint8_t i = 0; i < n_cal; i++) buf[off++] = 0;
    return buf;
}

// --- Positive baseline ---

TEST(ShiParserTest, R2_01_ValidVariablePayload)
{
    auto buf = make_shi_payload(BTS_SHI_MAX_OPERATION_MODES,
                                BTS_SHI_MAX_INPUT_SIGNALS,
                                BTS_SHI_MAX_CALIBRATION_COMPONENTS);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.interface_id, 0x0D);
    EXPECT_EQ(msg.num_valid_operation_modes, BTS_SHI_MAX_OPERATION_MODES);
    EXPECT_EQ(msg.num_valid_input_signal_statuses, BTS_SHI_MAX_INPUT_SIGNALS);
    EXPECT_EQ(msg.num_valid_calibration_components, BTS_SHI_MAX_CALIBRATION_COMPONENTS);
    EXPECT_EQ(msg.sensor_calibration_components[2], 2);
    EXPECT_EQ(msg.sensor_calibration_statuses[2], 0);
    EXPECT_EQ(msg.sensor_calibration_states[2], 0);
}

TEST(ShiParserTest, R2_08_OperationModesMaxLocalStorage)
{
    auto buf = make_shi_payload(BTS_SHI_MAX_OPERATION_MODES, 1, 0);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.num_valid_operation_modes, BTS_SHI_MAX_OPERATION_MODES);
    for (int i = 0; i < BTS_SHI_MAX_OPERATION_MODES; i++) {
        EXPECT_EQ(msg.sensor_operation_modes[i], i);
    }
}

TEST(ShiParserTest, R2_09_OversizePayload)
{
    auto buf = make_shi_payload();
    buf.resize(1000, 0);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.interface_id, 0x0D);
}

// --- Negative / Adversarial ---

TEST(ShiParserTest, R2_02_TruncatedPayload)
{
    std::vector<uint8_t> buf(30, 0);  // < minimum variable SHII payload
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_LT(parser.Parse(buf.data(), buf.size(), msg), 0);
}

TEST(ShiParserTest, R2_03_ZeroLengthPayload)
{
    ShiParser parser;
    ShiMessage msg{};
    uint8_t dummy = 0;
    EXPECT_LT(parser.Parse(&dummy, 0, msg), 0);
}

TEST(ShiParserTest, R2_04_NullPointer)
{
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_LT(parser.Parse(nullptr, 62, msg), 0);
}

TEST(ShiParserTest, R2_05_AllZerosPayload)
{
    std::vector<uint8_t> buf(kShiMinSize, 0);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.sensor_id, 0);
    EXPECT_EQ(msg.message_counter, 0u);
    EXPECT_EQ(msg.sensor_defect_recognised, BTS_SHI_SDR_FULLY_FUNCTIONAL);
}

TEST(ShiParserTest, R2_06_AllOnesPayload)
{
    std::vector<uint8_t> buf(62, 0xFF);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_LT(parser.Parse(buf.data(), buf.size(), msg), 0);
}

TEST(ShiParserTest, R2_07_EnumOutOfRange)
{
    auto buf = make_shi_payload(0, 0, 0);
    buf[25] = 99;  // defect_recognised = 99 (valid enum 0-2)
    buf[26] = 99;  // defect_reason = 99 (valid enum 0-11)
    buf[27] = 99;  // supply_voltage_status = 99 (valid enum 0-4)
    buf[28] = 99;  // temperature_status = 99 (valid enum 0-4)
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(static_cast<uint8_t>(msg.sensor_defect_recognised), 99);
    EXPECT_EQ(static_cast<uint8_t>(msg.sensor_defect_reason), 99);
}

TEST(ShiParserTest, R2_10_SingleBytePayload)
{
    uint8_t one = 0x42;
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_LT(parser.Parse(&one, 1, msg), 0);
}

TEST(ShiParserTest, R2_11_OffByOnePayload)
{
    std::vector<uint8_t> buf(kShiMinSize - 1, 0);
    ShiParser parser;
    ShiMessage msg{};
    EXPECT_LT(parser.Parse(buf.data(), buf.size(), msg), 0);
}
