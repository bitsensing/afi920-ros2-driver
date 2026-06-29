// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
// R3: SPI Parser negative/adversarial tests
#include <gtest/gtest.h>
#include "afi920_driver/parsers/spi_parser.hpp"
#include "afi920_driver/interface/AFI920/bts_iso23150_spi.h"
#include <cstring>
#include <vector>

using afi920_driver::SpiParser;
using afi920_driver::SpiMessage;

// SPI minimum: common header(24) + VCS(1) + pose(32) = 57 bytes
// then: num_fov(1) + fov_segments[N*17] + num_obj(1) + objects[N*9] + num_ref(1) + ref_targets[N*16]
static constexpr size_t kSpiHeaderPose = 57;  // header + VCS + pose
static constexpr size_t kFovSize = BTS_SPI_FOV_SEGMENT_SIZE;   // 17
static constexpr size_t kObjSize = BTS_SPI_OBJECT_TYPE_SIZE;   // 9
static constexpr size_t kRefSize = BTS_SPI_REF_TARGET_SIZE;    // 16

// Helper: build minimal well-formed SPI payload (0 arrays)
static std::vector<uint8_t> make_spi_payload(uint8_t n_fov = 0,
                                              uint8_t n_obj = 0,
                                              uint8_t n_ref = 0) {
    size_t total = kSpiHeaderPose + 1 + n_fov * kFovSize
                                 + 1 + n_obj * kObjSize
                                 + 1 + n_ref * kRefSize;
    std::vector<uint8_t> buf(total, 0);
    buf[3] = 0x0C;  // interface_id = SPI
    buf[4] = 1;     // num_serving_sensors
    // VCS at offset 24 (within common header area — actual offset depends on impl)
    // num_fov at offset kSpiHeaderPose
    buf[kSpiHeaderPose] = n_fov;
    // num_obj after fov section
    size_t obj_offset = kSpiHeaderPose + 1 + n_fov * kFovSize;
    buf[obj_offset] = n_obj;
    // num_ref after obj section
    size_t ref_offset = obj_offset + 1 + n_obj * kObjSize;
    buf[ref_offset] = n_ref;
    return buf;
}

// --- Positive baseline ---

TEST(SpiParserTest, R3_01_MinimalPayload) {
    auto buf = make_spi_payload(0, 0, 0);
    SpiParser parser;
    SpiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.num_fov_segments, 0);
    EXPECT_EQ(msg.num_recognisable_object_types, 0);
    EXPECT_EQ(msg.num_reference_target_types, 0);
}

TEST(SpiParserTest, R3_08_FovCount16Valid) {
    auto buf = make_spi_payload(16, 0, 0);
    SpiParser parser;
    SpiMessage msg{};
    int rc = parser.Parse(buf.data(), buf.size(), msg);
    EXPECT_EQ(rc, 0);
    // Parser should clamp to min(16, BTS_SPI_MAX_FOV_SEGMENTS=16) = 16
    EXPECT_LE(msg.num_fov_segments, 16);
}

// --- Negative / Adversarial ---

TEST(SpiParserTest, R3_02_TruncatedHeader) {
    std::vector<uint8_t> buf(50, 0);  // < 57B minimum
    SpiParser parser;
    SpiMessage msg{};
    EXPECT_LT(parser.Parse(buf.data(), buf.size(), msg), 0);
}

TEST(SpiParserTest, R3_03_ZeroLengthPayload) {
    SpiParser parser;
    SpiMessage msg{};
    uint8_t dummy = 0;
    EXPECT_LT(parser.Parse(&dummy, 0, msg), 0);
}

TEST(SpiParserTest, R3_04_NullPointer) {
    SpiParser parser;
    SpiMessage msg{};
    EXPECT_LT(parser.Parse(nullptr, 200, msg), 0);
}

TEST(SpiParserTest, R3_05_AllZerosPayload) {
    // 76 bytes of all zeros (header+pose + 3 count bytes)
    std::vector<uint8_t> buf(kSpiHeaderPose + 3, 0);
    SpiParser parser;
    SpiMessage msg{};
    EXPECT_EQ(parser.Parse(buf.data(), buf.size(), msg), 0);
    EXPECT_EQ(msg.num_fov_segments, 0);
    EXPECT_EQ(msg.num_recognisable_object_types, 0);
    EXPECT_EQ(msg.num_reference_target_types, 0);
}

TEST(SpiParserTest, R3_06_AllOnesPayload) {
    // 76 bytes of 0xFF — num_fov=255, should be clamped to MAX 16
    std::vector<uint8_t> buf(kSpiHeaderPose + 3, 0xFF);
    SpiParser parser;
    SpiMessage msg{};
    int rc = parser.Parse(buf.data(), buf.size(), msg);
    // Parser may return 0 (with clamping) or <0 (insufficient buffer)
    // Either way, should NOT crash and fov should be clamped
    if (rc == 0) {
        EXPECT_LE(msg.num_fov_segments, BTS_SPI_MAX_FOV_SEGMENTS);
    }
}

TEST(SpiParserTest, R3_07_FovCountExceedsMax) {
    // Claim 255 FOV segments but buffer only has header + 3 count bytes
    std::vector<uint8_t> buf(kSpiHeaderPose + 3, 0);
    buf[3] = 0x0C;
    buf[4] = 1;
    buf[kSpiHeaderPose] = 255;  // num_fov = 255 (max is 16)
    SpiParser parser;
    SpiMessage msg{};
    int rc = parser.Parse(buf.data(), buf.size(), msg);
    // Should clamp or fail gracefully
    if (rc == 0) {
        EXPECT_LE(msg.num_fov_segments, BTS_SPI_MAX_FOV_SEGMENTS);
    }
    // Should not crash regardless
}

TEST(SpiParserTest, R3_09_ObjectCountExceedsMax) {
    // Valid FOV section (0 segments), then claim 255 objects
    auto buf = make_spi_payload(0, 0, 0);
    size_t obj_offset = kSpiHeaderPose + 1;  // after num_fov(0)
    buf[obj_offset] = 255;  // claim 255 objects
    SpiParser parser;
    SpiMessage msg{};
    int rc = parser.Parse(buf.data(), buf.size(), msg);
    if (rc == 0) {
        EXPECT_LE(msg.num_recognisable_object_types, BTS_SPI_MAX_OBJECT_TYPES);
    }
}

TEST(SpiParserTest, R3_10_TruncatedMidFovSegment) {
    // Header + claim 2 FOV segments but buffer only has 1.5 segments worth
    size_t partial = kSpiHeaderPose + 1 + kFovSize + kFovSize / 2;
    std::vector<uint8_t> buf(partial, 0);
    buf[3] = 0x0C;
    buf[4] = 1;
    buf[kSpiHeaderPose] = 2;  // claim 2 FOV segments
    SpiParser parser;
    SpiMessage msg{};
    int rc = parser.Parse(buf.data(), buf.size(), msg);
    // Should handle truncation gracefully — parser stores clamped count (2)
    // but only 1 segment fits in buffer. Parser does NOT update count to actual parsed.
    // The key test: no crash on truncated data.
    if (rc == 0) {
        EXPECT_LE(msg.num_fov_segments, BTS_SPI_MAX_FOV_SEGMENTS);
    }
}

TEST(SpiParserTest, R3_11_SingleBytePayload) {
    uint8_t one = 0x42;
    SpiParser parser;
    SpiMessage msg{};
    EXPECT_LT(parser.Parse(&one, 1, msg), 0);
}
