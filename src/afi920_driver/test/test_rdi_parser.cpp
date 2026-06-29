// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
// R1: RDI Parser negative/adversarial tests
#include <gtest/gtest.h>
#include "afi920_driver/parsers/rdi_parser.hpp"
#include "afi920_driver/interface/AFI920/bts_iso23150_rdi.h"
#include <cstring>
#include <vector>

using afi920_driver::RdiParser;
using afi920_driver::RdiFrame;

static constexpr size_t kHdr = BTS_RDI_HEADER_SIZE;  // 36
static constexpr size_t kDet = BTS_RDI_DETECTION_SIZE;  // 51

// Helper: build a minimal well-formed RDI payload with N detections
static std::vector<uint8_t> make_rdi_payload(uint16_t num_det) {
    std::vector<uint8_t> buf(kHdr + kDet * num_det, 0);
    buf[3] = 0x08;  // interface_id = RDI
    buf[4] = 1;     // num_serving_sensors
    // detection capability at offset 32, num_detections at offset 34 (uint16 LE)
    buf[32] = 0x00;
    buf[33] = 0x10;
    buf[34] = num_det & 0xFF;
    buf[35] = (num_det >> 8) & 0xFF;
    return buf;
}

// --- Positive baseline ---

TEST(RdiParserTest, R1_11_ValidSingleDetection) {
    auto buf = make_rdi_payload(1);
    // Set some recognizable values in detection[0]
    buf[kHdr + 0] = 80;  // existence_probability
    // radial_distance at det offset 19: set to 50.0f
    float dist = 50.0f;
    std::memcpy(buf.data() + kHdr + 19, &dist, 4);

    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_EQ(frame.message.num_detections, 1);
    ASSERT_EQ(frame.detections_storage.size(), 1u);
    EXPECT_EQ(frame.detections_storage[0].existence_probability, 80);
    EXPECT_FLOAT_EQ(frame.detections_storage[0].position_radial_distance, 50.0f);
}

TEST(RdiParserTest, R1_02_ExactHeaderNoDetections) {
    auto buf = make_rdi_payload(0);
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_EQ(frame.message.num_detections, 0);
    EXPECT_TRUE(frame.detections_storage.empty());
}

// --- Negative / Adversarial ---

TEST(RdiParserTest, R1_01_TruncatedHeader) {
    std::vector<uint8_t> buf(30, 0);  // < 36B header
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_LT(parser.Parse(buf.data(), buf.size(), frame), 0);
}

TEST(RdiParserTest, R1_03_HeaderClaimsMoreDetectionsThanBuffer) {
    // Header says 100 detections but buffer only has space for 2
    auto buf = make_rdi_payload(2);
    buf[34] = 100;  // claim 100
    buf[35] = 0;

    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    // Parser should clamp to what fits in buffer
    EXPECT_LE(frame.message.num_detections, 2);
    EXPECT_EQ(frame.detections_storage.size(), frame.message.num_detections);
}

TEST(RdiParserTest, R1_04_ZeroLengthPayload) {
    RdiParser parser;
    RdiFrame frame{};
    uint8_t dummy = 0;
    EXPECT_LT(parser.Parse(&dummy, 0, frame), 0);
}

TEST(RdiParserTest, R1_05_NullPointer) {
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_LT(parser.Parse(nullptr, 100, frame), 0);
}

TEST(RdiParserTest, R1_06_AllZerosPayload) {
    std::vector<uint8_t> buf(kHdr + kDet, 0);
    // num_detections = 0 at offset 34-35 (already zero)
    // but we have space for 1 detection, header says 0 → 0 detections
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_EQ(frame.message.num_detections, 0);
}

TEST(RdiParserTest, R1_07_AllOnesPayload) {
    std::vector<uint8_t> buf(kHdr + kDet, 0xFF);
    // num_detections = 0xFFFF → should be clamped to 4096
    // but buffer only has 1 detection worth → further clamped to 1
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_LE(frame.message.num_detections, 1);
    // Should not crash even with NaN/max float values
}

TEST(RdiParserTest, R1_08_MaxDetections4096) {
    std::vector<uint8_t> buf(kHdr + kDet * 4096, 0);
    buf[3] = 0x08;
    buf[4] = 1;
    // num_detections = 4096 = 0x1000 LE
    buf[34] = 0x00;
    buf[35] = 0x10;

    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_EQ(frame.message.num_detections, 4096);
    EXPECT_EQ(frame.detections_storage.size(), 4096u);
}

TEST(RdiParserTest, R1_09_SingleBytePayload) {
    uint8_t one = 0x42;
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_LT(parser.Parse(&one, 1, frame), 0);
}

TEST(RdiParserTest, R1_10_DetectionCountOverflow) {
    // num_detections = 0xFFFF (65535) but only header in buffer
    std::vector<uint8_t> buf(kHdr, 0);
    buf[34] = 0xFF;
    buf[35] = 0xFF;

    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    // Should clamp: first to 4096, then to buffer capacity = 0
    EXPECT_EQ(frame.message.num_detections, 0);
    EXPECT_TRUE(frame.detections_storage.empty());
}

TEST(RdiParserTest, R1_12_InterfaceIdPreserved) {
    auto buf = make_rdi_payload(0);
    buf[3] = 0x08;
    RdiParser parser;
    RdiFrame frame{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), frame), 0);
    EXPECT_EQ(frame.message.interface_id, 0x08);
}
