// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file test_someip_parser.cpp
 * @brief Unit tests for SOME/IP header deserialization and TP reassembly
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "afi920_driver/someip/e2e.hpp"
#include "afi920_driver/someip/someip_parser.hpp"
#include "afi920_driver/interface/bts_iso23150.h"

using namespace afi920_driver;  // NOLINT(build/namespaces)

namespace {

void pack_be64(uint8_t* p, uint64_t v)
{
    p[0] = static_cast<uint8_t>(v >> 56);
    p[1] = static_cast<uint8_t>(v >> 48);
    p[2] = static_cast<uint8_t>(v >> 40);
    p[3] = static_cast<uint8_t>(v >> 32);
    p[4] = static_cast<uint8_t>(v >> 24);
    p[5] = static_cast<uint8_t>(v >> 16);
    p[6] = static_cast<uint8_t>(v >> 8);
    p[7] = static_cast<uint8_t>(v);
}

void pack_be32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

}  // namespace

TEST(SomeipParserTest, ValidateAcceptsCorrectServiceId) {
    SomeipHeader hdr{};
    hdr.service_id = 0x6000;
    hdr.message_type = 0x02;  // NOTIFICATION
    EXPECT_TRUE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateRejectsWrongServiceId) {
    SomeipHeader hdr{};
    hdr.service_id = 0x1234;
    hdr.message_type = 0x02;
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateAcceptsTpNotification) {
    SomeipHeader hdr{};
    hdr.service_id = 0x6000;
    hdr.message_type = 0x22;  // TP + NOTIFICATION
    EXPECT_TRUE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateRejectsRequest) {
    SomeipHeader hdr{};
    hdr.service_id = 0x6000;
    hdr.message_type = 0x00;  // REQUEST
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateRejectsResponse) {
    SomeipHeader hdr{};
    hdr.service_id = 0x6000;
    hdr.message_type = 0x80;  // RESPONSE
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, EventIdConstants) {
    EXPECT_EQ(kEventRdi, 0x8002);
    EXPECT_EQ(kEventShii, 0x8003);
    EXPECT_EQ(kEventSpi, 0x8004);
}

TEST(SomeipParserTest, ReturnCodeConstants) {
    EXPECT_EQ(BTS_RC_SUCCESS, 0x00);
    EXPECT_EQ(BTS_RC_INVALID_PARAM, 0x01);
    EXPECT_EQ(BTS_RC_TIMEOUT, 0x0B);
}

TEST(SomeipParserTest, E2eCrc64XzKnownAnswer) {
    const char* text = "123456789";
    EXPECT_EQ(e2e_crc64_xz(reinterpret_cast<const uint8_t*>(text), 9),
              0x995DC9BBDF1939FAull);
}

TEST(SomeipParserTest, E2eValidateAcceptsProtectedFrame) {
    std::vector<uint8_t> frame(kE2eHeaderSize + 4, 0);
    pack_be32(frame.data() + kE2eOffsetLength, 4);
    pack_be32(frame.data() + kE2eOffsetCounter, 7);
    pack_be32(frame.data() + kE2eOffsetDataId, kE2eDataIdRdi);
    frame[kE2eHeaderSize + 0] = 0x01;
    frame[kE2eHeaderSize + 1] = 0x02;
    frame[kE2eHeaderSize + 2] = 0x03;
    frame[kE2eHeaderSize + 3] = 0x04;

    const uint64_t crc = e2e_crc64_xz(
        frame.data() + kE2eOffsetLength, frame.size() - kE2eOffsetLength);
    pack_be64(frame.data() + kE2eOffsetCrc, crc);

    E2eHeader e2e{};
    EXPECT_EQ(e2e_validate(frame.data(), frame.size(), e2e), E2eStatus::kOk);
    EXPECT_EQ(e2e.length, 4u);
    EXPECT_EQ(e2e.counter, 7u);
    EXPECT_EQ(e2e.data_id, kE2eDataIdRdi);
}

TEST(SomeipParserTest, E2eValidateRejectsCorruptedFrame) {
    std::vector<uint8_t> frame(kE2eHeaderSize + 1, 0);
    pack_be32(frame.data() + kE2eOffsetLength, 1);
    pack_be32(frame.data() + kE2eOffsetCounter, 1);
    pack_be32(frame.data() + kE2eOffsetDataId, kE2eDataIdSpi);
    frame[kE2eHeaderSize] = 0x42;

    const uint64_t crc = e2e_crc64_xz(
        frame.data() + kE2eOffsetLength, frame.size() - kE2eOffsetLength);
    pack_be64(frame.data() + kE2eOffsetCrc, crc);
    frame[kE2eHeaderSize] ^= 0x01;

    E2eHeader e2e{};
    EXPECT_EQ(e2e_validate(frame.data(), frame.size(), e2e), E2eStatus::kCrcMismatch);
}

TEST(SomeipParserTest, TpReassemblerSessionReset) {
    SomeipTpReassembler tp;
    // Feed segment with session 1 (offset=0, more_segments=true)
    uint8_t data1[100];
    memset(data1, 0xAA, sizeof(data1));
    tp.Feed(data1, sizeof(data1), 0, true, 1);

    // Feed segment with different session — should reset
    uint8_t data2[100];
    memset(data2, 0xBB, sizeof(data2));
    tp.Feed(data2, sizeof(data2), 0, false, 2);

    // Should not be timed out (just fed)
    EXPECT_FALSE(tp.IsTimedOut());
}

TEST(SomeipParserTest, DeserializeValidHeader) {
    // Build a minimal valid SOME/IP header (16 bytes, big-endian)
    uint8_t raw[16] = {};
    // Service ID = 0x6000, Method ID = 0x8002
    raw[0] = 0x60; raw[1] = 0x00; raw[2] = 0x80; raw[3] = 0x02;
    // Length = 8 (just header, no payload)
    raw[4] = 0x00; raw[5] = 0x00; raw[6] = 0x00; raw[7] = 0x08;
    // Client ID = 0x0001, Session ID = 0x0001
    raw[8] = 0x00; raw[9] = 0x01; raw[10] = 0x00; raw[11] = 0x01;
    // Protocol=1, Interface=1, MsgType=0x02, RetCode=0x00
    raw[12] = 0x01; raw[13] = 0x01; raw[14] = 0x02; raw[15] = 0x00;

    SomeipHeader hdr{};
    int result = someip_deserialize(raw, sizeof(raw), hdr);
    EXPECT_EQ(result, 0);
    EXPECT_EQ(hdr.service_id, 0x6000);
    EXPECT_EQ(hdr.method_id, 0x8002);
    EXPECT_EQ(hdr.length, 8u);
    EXPECT_EQ(hdr.message_type, 0x02);
}

// --- Negative / Adversarial Tests ---

TEST(SomeipParserTest, DeserializeTooShort) {
    uint8_t raw[8] = {};  // Only 8 bytes, need 16
    SomeipHeader hdr{};
    EXPECT_LT(someip_deserialize(raw, sizeof(raw), hdr), 0);
}

TEST(SomeipParserTest, DeserializeZeroLength) {
    SomeipHeader hdr{};
    EXPECT_LT(someip_deserialize(nullptr, 0, hdr), 0);
}

TEST(SomeipParserTest, ValidateZeroServiceId) {
    SomeipHeader hdr{};
    hdr.service_id = 0x0000;
    hdr.message_type = 0x02;
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateMaxServiceId) {
    SomeipHeader hdr{};
    hdr.service_id = 0xFFFF;
    hdr.message_type = 0x02;
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, ValidateErrorMessageType) {
    SomeipHeader hdr{};
    hdr.service_id = 0x6000;
    hdr.message_type = 0x81;  // ERROR response
    EXPECT_FALSE(someip_validate(hdr));
}

TEST(SomeipParserTest, TpReassemblerTimeout) {
    SomeipTpReassembler tp;
    uint8_t data[100];
    memset(data, 0xCC, sizeof(data));
    // Feed first segment (more_segments=true)
    tp.Feed(data, sizeof(data), 0, true, 1);
    // Immediately check — should NOT be timed out yet
    EXPECT_FALSE(tp.IsTimedOut(std::chrono::milliseconds(100)));
    // Note: Can't easily test actual timeout without sleeping, but verify the API works
}

TEST(SomeipParserTest, TpReassemblerResetClearsState) {
    SomeipTpReassembler tp;
    uint8_t data[100];
    memset(data, 0xDD, sizeof(data));
    tp.Feed(data, sizeof(data), 0, true, 1);
    tp.Reset();
    // After reset, feeding a new session should work cleanly
    uint8_t data2[50];
    memset(data2, 0xEE, sizeof(data2));
    tp.Feed(data2, sizeof(data2), 0, false, 2);
    // Should complete without issues
    EXPECT_FALSE(tp.IsTimedOut());
}

TEST(SomeipParserTest, DeserializeMalformedLength) {
    // Header with impossibly large length
    uint8_t raw[16] = {};
    raw[0] = 0x60; raw[1] = 0x00; raw[2] = 0x80; raw[3] = 0x02;
    raw[4] = 0xFF; raw[5] = 0xFF; raw[6] = 0xFF; raw[7] = 0xFF;  // Length = 4GB
    raw[12] = 0x01; raw[13] = 0x01; raw[14] = 0x02; raw[15] = 0x00;
    SomeipHeader hdr{};
    // Should still parse the header (length is just a field value),
    // but caller should validate length against actual data
    int result = someip_deserialize(raw, sizeof(raw), hdr);
    EXPECT_EQ(result, 0);  // Header itself is parseable
    EXPECT_EQ(hdr.length, 0xFFFFFFFFu);  // But length is suspicious
}

// ─── S2: TP Reassembly Edge Cases ───

TEST(SomeipParserTest, S2_1_TpMultiSegmentReassembly) {
    SomeipTpReassembler tp;
    // 3 segments: 1392 + 1392 + 500 = 3284 bytes total
    std::vector<uint8_t> seg1(1392, 0xAA);
    std::vector<uint8_t> seg2(1392, 0xBB);
    std::vector<uint8_t> seg3(500, 0xCC);

    EXPECT_FALSE(tp.Feed(seg1.data(), seg1.size(), 0, true, 1));
    EXPECT_FALSE(tp.Feed(seg2.data(), seg2.size(), 1392, true, 1));
    EXPECT_TRUE(tp.Feed(seg3.data(), seg3.size(), 2784, false, 1));

    EXPECT_GE(tp.GetPayloadSize(), 3284u);
    // First byte should be from seg1
    EXPECT_EQ(tp.GetPayload()[0], 0xAA);
    EXPECT_EQ(tp.GetPayload()[1392], 0xBB);
    EXPECT_EQ(tp.GetPayload()[2784], 0xCC);
}

TEST(SomeipParserTest, S2_2_TpDuplicateSegment) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg(100, 0x11);

    tp.Feed(seg.data(), seg.size(), 0, true, 1);
    // Feed same offset again — should overwrite cleanly
    std::vector<uint8_t> seg_dup(100, 0x22);
    tp.Feed(seg_dup.data(), seg_dup.size(), 0, true, 1);

    // Complete the message with a final segment
    std::vector<uint8_t> seg_last(50, 0x33);
    EXPECT_TRUE(tp.Feed(seg_last.data(), seg_last.size(), 100, false, 1));

    EXPECT_GE(tp.GetPayloadSize(), 150u);
    // Duplicate overwrote first segment, so byte[0] should be 0x22
    EXPECT_EQ(tp.GetPayload()[0], 0x22);
    EXPECT_EQ(tp.GetPayload()[100], 0x33);
}

TEST(SomeipParserTest, S2_3_TpOutOfOrderSegments) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg1(100, 0xAA);
    std::vector<uint8_t> seg2(100, 0xBB);
    std::vector<uint8_t> seg3(50, 0xCC);

    // Feed out of order: seg2, seg1, seg3(last)
    EXPECT_FALSE(tp.Feed(seg2.data(), seg2.size(), 100, true, 1));
    EXPECT_FALSE(tp.Feed(seg1.data(), seg1.size(), 0, true, 1));
    EXPECT_TRUE(tp.Feed(seg3.data(), seg3.size(), 200, false, 1));

    EXPECT_GE(tp.GetPayloadSize(), 250u);
    EXPECT_EQ(tp.GetPayload()[0], 0xAA);
    EXPECT_EQ(tp.GetPayload()[100], 0xBB);
    EXPECT_EQ(tp.GetPayload()[200], 0xCC);
}

TEST(SomeipParserTest, S2_4_TpGapInSegments) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg1(100, 0xAA);
    std::vector<uint8_t> seg3(50, 0xCC);

    // Feed seg1 and seg3, skip seg2 — gap at offset 100-199
    EXPECT_FALSE(tp.Feed(seg1.data(), seg1.size(), 0, true, 1));
    // seg3 claims to be last (more=false) at offset 200
    bool complete = tp.Feed(seg3.data(), seg3.size(), 200, false, 1);
    // May report complete (it saw more=false) but data has a gap
    // The key test: should NOT crash
    (void)complete;
    EXPECT_GE(tp.GetPayloadSize(), 200u);
}

TEST(SomeipParserTest, S2_5_TpLastSegmentFirst) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg_last(50, 0xFF);
    std::vector<uint8_t> seg_first(100, 0x11);

    // Feed last segment first (more=false, offset=100)
    bool done = tp.Feed(seg_last.data(), seg_last.size(), 100, false, 1);
    // May or may not report complete — depends on impl
    // Now feed first segment
    tp.Feed(seg_first.data(), seg_first.size(), 0, true, 1);
    // Should not crash regardless of completion status
    (void)done;
}

TEST(SomeipParserTest, S2_6_TpZeroLengthSegment) {
    SomeipTpReassembler tp;
    uint8_t dummy = 0;
    // Zero length segment should not crash
    bool result = tp.Feed(&dummy, 0, 0, true, 1);
    EXPECT_FALSE(result);  // Can't be complete with 0 bytes and more=true
}

TEST(SomeipParserTest, S2_7_TpOverlappingOffsets) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg1(1400, 0xAA);
    std::vector<uint8_t> seg2(1400, 0xBB);

    // seg1 at offset 0, len 1400
    tp.Feed(seg1.data(), seg1.size(), 0, true, 1);
    // seg2 at offset 100, len 1400 — overlaps with seg1
    tp.Feed(seg2.data(), seg2.size(), 100, false, 1);
    // Should not crash, buffer should grow to accommodate
    EXPECT_GE(tp.GetPayloadSize(), 1500u);
}

TEST(SomeipParserTest, S2_8_TpSessionChangeMidReassembly) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> data1(100, 0x11);
    std::vector<uint8_t> data2(50, 0x22);

    // Start session 1
    tp.Feed(data1.data(), data1.size(), 0, true, 1);
    // Switch to session 2 — should reset
    bool done = tp.Feed(data2.data(), data2.size(), 0, false, 2);
    EXPECT_TRUE(done);  // session 2 is complete (single segment, more=false)
    // Data should be from session 2
    EXPECT_EQ(tp.GetPayload()[0], 0x22);
}

TEST(SomeipParserTest, S2_9_TpResetBetweenMessages) {
    SomeipTpReassembler tp;
    std::vector<uint8_t> seg(200, 0xDD);

    tp.Feed(seg.data(), seg.size(), 0, false, 1);
    tp.Reset();

    // After reset, new Feed should start fresh
    std::vector<uint8_t> seg2(300, 0xEE);
    bool done = tp.Feed(seg2.data(), seg2.size(), 0, false, 2);
    EXPECT_TRUE(done);
    EXPECT_EQ(tp.GetPayload()[0], 0xEE);
    EXPECT_GE(tp.GetPayloadSize(), 300u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
