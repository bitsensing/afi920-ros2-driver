// Copyright (c) 2026 bitsensing Inc. All rights reserved.
// Interface v1.2.0 RDI scan assembly (Interface Overview §3.2.3).
// Header and chunk bodies are cut from a single-message payload.
#include <gtest/gtest.h>
#include "afi920_driver/parsers/rdi_assembler.hpp"
#include <cstring>
#include <vector>

using afi920_driver::E2eCrcFormat;
using afi920_driver::E2eStatus;
using afi920_driver::RdiMessageE2e;
using afi920_driver::RdiScan;
using afi920_driver::RdiScanAssembler;

static constexpr uint64_t kMs = 1000000ull;

static void pack_le16(uint8_t* p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}

static void pack_le32(uint8_t* p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (v >> (8 * i)) & 0xFF;
}

static void pack_f32(uint8_t* p, float f)
{
    uint32_t u;
    std::memcpy(&u, &f, 4);
    pack_le32(p, u);
}

// Single-message RDI payload (Detection Header + n detections) to split.
static std::vector<uint8_t> make_payload(uint16_t n, uint32_t message_counter)
{
    std::vector<uint8_t> buf(BTS_RDI_HEADER_SIZE + n * BTS_RDI_DETECTION_SIZE, 0);
    buf[0] = 1;
    buf[1] = 2;
    buf[3] = 0x08;
    buf[4] = 1;
    buf[5] = 1;
    for (int i = 0; i < 8; i++) buf[6 + i] = static_cast<uint8_t>(0x10 + i);
    pack_le32(&buf[14], message_counter);
    pack_f32(&buf[24], -111.11f);
    pack_f32(&buf[28], 55.56f);
    pack_le16(&buf[32], 2048);
    pack_le16(&buf[34], n);
    for (uint16_t i = 0; i < n; i++) {
        uint8_t* d = buf.data() + BTS_RDI_HEADER_SIZE + i * BTS_RDI_DETECTION_SIZE;
        d[0] = 90;
        pack_le16(d + 1, i);                       // DetectionID = index
        pack_f32(d + 19, 1.0f + i);                // radial distance
    }
    return buf;
}

struct Scan {
    std::vector<uint8_t> header;
    std::vector<std::vector<uint8_t>> chunks;
};

// Header body: Detection Header + List Layout. Chunk k: Chunk Header
// (MessageCounter, FirstEntityIndex, EntityCount) + its detections.
static Scan split(const std::vector<uint8_t>& payload)
{
    constexpr uint16_t kPerChunk = BTS_RDI_ENTITIES_PER_CHUNK;
    const uint16_t n = static_cast<uint16_t>(payload[34] | (payload[35] << 8));
    const uint16_t k_count = static_cast<uint16_t>((n + kPerChunk - 1) / kPerChunk);
    Scan s;
    s.header.assign(payload.begin(), payload.begin() + BTS_RDI_HEADER_SIZE);
    s.header.resize(BTS_RDI_HEADER_MSG_SIZE);
    pack_le16(&s.header[BTS_RDI_HEADER_SIZE], kPerChunk);
    pack_le16(&s.header[BTS_RDI_HEADER_SIZE + 2], k_count);
    for (uint16_t k = 0; k < k_count; k++) {
        const uint16_t first = static_cast<uint16_t>(k * kPerChunk);
        const uint16_t count = static_cast<uint16_t>(
            n - first < kPerChunk ? n - first : kPerChunk);
        std::vector<uint8_t> c(BTS_RDI_CHUNK_HEADER_SIZE);
        std::memcpy(c.data(), payload.data() + 14, 4);  // MessageCounter
        pack_le16(&c[4], first);
        pack_le16(&c[6], count);
        const auto* d = payload.data() + BTS_RDI_HEADER_SIZE + first * BTS_RDI_DETECTION_SIZE;
        c.insert(c.end(), d, d + count * BTS_RDI_DETECTION_SIZE);
        s.chunks.push_back(c);
    }
    return s;
}

// Scans are copied with their detections; the detections pointer is rebound.
struct Sink {
    std::vector<RdiScan> scans;
    RdiScanAssembler::Emit emit()
    {
        return [this](RdiScan& s) {
            scans.push_back(s);
            auto& f = scans.back().frame;
            f.message.detections = f.detections_storage.empty() ? nullptr : f.detections_storage.data();
        };
    }
};

static RdiMessageE2e ok_e2e()
{
    RdiMessageE2e e2e;
    e2e.format = E2eCrcFormat::kCrc32;
    return e2e;
}

static void feed_header(RdiScanAssembler& a, Sink& sink, uint32_t counter,
                        const Scan& s, uint64_t t, const RdiMessageE2e& e2e = ok_e2e())
{
    a.OnHeader(counter, e2e, s.header.data(), s.header.size(), t, sink.emit());
}

static void feed_chunk(RdiScanAssembler& a, Sink& sink, uint32_t counter,
                       const Scan& s, uint16_t k, uint64_t t,
                       const RdiMessageE2e& e2e = ok_e2e())
{
    a.OnChunk(counter, e2e, k, s.chunks[k].data(), s.chunks[k].size(), t, sink.emit());
}

static bool detections_in_order(const RdiScan& s, uint16_t first, uint16_t count)
{
    const auto& m = s.frame.message;
    if (m.num_detections != count || s.frame.detections_storage.size() != count) return false;
    for (uint16_t i = 0; i < count; i++) {
        if (m.detections[i].detection_id != first + i) return false;
        if (m.detections[i].position_radial_distance != 1.0f + first + i) return false;
    }
    return true;
}

TEST(RdiScanAssemblerTest, FullScanClosesOnLastChunkAndHeaderBIsLate)
{
    const Scan s = split(make_payload(60, 500));
    ASSERT_EQ(s.chunks.size(), 3u);
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 7, s, 0);
    for (uint16_t k = 0; k < 3; k++) feed_chunk(a, sink, 7, s, k, 1 * kMs);
    ASSERT_EQ(sink.scans.size(), 1u);
    feed_header(a, sink, 7, s, 2 * kMs);  // header B after completion
    EXPECT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(a.LateMessages(), 1u);

    const RdiScan& r = sink.scans[0];
    EXPECT_TRUE(r.has_header);
    EXPECT_EQ(r.e2e_counter, 7u);
    EXPECT_EQ(r.e2e.status, E2eStatus::kOk);
    EXPECT_EQ(r.e2e.format, E2eCrcFormat::kCrc32);
    EXPECT_EQ(r.chunk_count, 3u);
    EXPECT_EQ(r.chunks_received, 3u);
    EXPECT_EQ(r.chunks_rejected, 0u);
    EXPECT_EQ(r.announced_detections, 60u);
    EXPECT_TRUE(r.IsComplete());
    EXPECT_EQ(r.frame.message.message_counter, 500u);
    EXPECT_EQ(r.frame.message.interface_version.minor, 2u);
    EXPECT_EQ(r.frame.message.recognised_detections_capability, 2048u);
    EXPECT_FLOAT_EQ(r.frame.message.ambiguity_domain.radial_velocity_begin, -111.11f);
    EXPECT_TRUE(detections_in_order(r, 0, 60));
}

TEST(RdiScanAssemblerTest, MaximumScanUses76Chunks)
{
    const Scan s = split(make_payload(2048, 1));
    ASSERT_EQ(s.chunks.size(), 76u);
    EXPECT_EQ(s.chunks[75].size(), BTS_RDI_CHUNK_HEADER_SIZE + 23 * BTS_RDI_DETECTION_SIZE);
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 1, s, 0);
    for (uint16_t k = 0; k < 76; k++) feed_chunk(a, sink, 1, s, k, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunk_count, 76u);
    EXPECT_TRUE(sink.scans[0].IsComplete());
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 2048));
}

TEST(RdiScanAssemblerTest, ZeroDetectionsCloseOnHeaderA)
{
    const Scan s = split(make_payload(0, 9));
    ASSERT_TRUE(s.chunks.empty());
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 3, s, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunk_count, 0u);
    EXPECT_EQ(sink.scans[0].frame.message.num_detections, 0u);
    EXPECT_EQ(sink.scans[0].frame.message.detections, nullptr);
    EXPECT_TRUE(sink.scans[0].IsComplete());
    feed_header(a, sink, 3, s, 1 * kMs);
    EXPECT_EQ(sink.scans.size(), 1u);
}

TEST(RdiScanAssemblerTest, LostHeaderAIsReplacedByHeaderB)
{
    const Scan s = split(make_payload(30, 4));
    RdiScanAssembler a;
    Sink sink;
    feed_chunk(a, sink, 11, s, 0, 0);
    feed_chunk(a, sink, 11, s, 1, 0);
    EXPECT_TRUE(sink.scans.empty());
    feed_header(a, sink, 11, s, 1 * kMs);  // the only header: completes the scan
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_TRUE(sink.scans[0].IsComplete());
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 30));
}

TEST(RdiScanAssemblerTest, MissingChunkIsDeliveredOnHeaderB)
{
    const Scan s = split(make_payload(60, 4));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 20, s, 0);
    feed_chunk(a, sink, 20, s, 0, 0);
    feed_chunk(a, sink, 20, s, 2, 0);
    EXPECT_TRUE(sink.scans.empty());
    feed_header(a, sink, 20, s, 1 * kMs);  // header B
    ASSERT_EQ(sink.scans.size(), 1u);
    const RdiScan& r = sink.scans[0];
    EXPECT_FALSE(r.IsComplete());
    EXPECT_EQ(r.chunk_count, 3u);
    EXPECT_EQ(r.chunks_received, 2u);
    EXPECT_EQ(r.announced_detections, 60u);
    ASSERT_EQ(r.frame.message.num_detections, 27u + 6u);
    EXPECT_EQ(r.frame.message.detections[26].detection_id, 26u);
    EXPECT_EQ(r.frame.message.detections[27].detection_id, 54u);
}

TEST(RdiScanAssemblerTest, ScanWithoutHeaderIsEmittedWithoutDetections)
{
    const Scan s = split(make_payload(30, 4));
    const Scan next = split(make_payload(0, 5));
    RdiScanAssembler a;
    Sink sink;
    feed_chunk(a, sink, 30, s, 0, 0);
    feed_chunk(a, sink, 30, s, 1, 0);
    feed_header(a, sink, 31, next, 1 * kMs);  // another counter closes scan 30
    ASSERT_EQ(sink.scans.size(), 2u);
    EXPECT_FALSE(sink.scans[0].has_header);
    EXPECT_FALSE(sink.scans[0].IsComplete());
    EXPECT_EQ(sink.scans[0].e2e_counter, 30u);
    EXPECT_TRUE(sink.scans[0].frame.detections_storage.empty());
    EXPECT_TRUE(sink.scans[1].has_header);
    EXPECT_EQ(sink.scans[1].e2e_counter, 31u);
}

TEST(RdiScanAssemblerTest, TimeoutClosesScanWithoutHeaderB)
{
    const Scan s = split(make_payload(60, 4));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 40, s, 0);
    feed_chunk(a, sink, 40, s, 0, 0);
    a.Poll(99 * kMs, sink.emit());
    EXPECT_TRUE(sink.scans.empty());
    a.Poll(100 * kMs, sink.emit());
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    feed_chunk(a, sink, 40, s, 1, 101 * kMs);  // after closing: late
    EXPECT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(a.LateMessages(), 1u);
}

TEST(RdiScanAssemblerTest, NewCounterClosesOpenScan)
{
    const Scan s1 = split(make_payload(60, 1));
    const Scan s2 = split(make_payload(10, 2));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 50, s1, 0);
    feed_chunk(a, sink, 50, s1, 0, 0);
    feed_header(a, sink, 51, s2, 1 * kMs);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].e2e_counter, 50u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    feed_chunk(a, sink, 51, s2, 0, 1 * kMs);
    ASSERT_EQ(sink.scans.size(), 2u);
    EXPECT_EQ(sink.scans[1].e2e_counter, 51u);
    EXPECT_TRUE(sink.scans[1].IsComplete());
}

TEST(RdiScanAssemblerTest, LateMessagesOfLastTwoScansAreDropped)
{
    const Scan s = split(make_payload(0, 1));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 60, s, 0);
    feed_header(a, sink, 61, s, 0);
    feed_header(a, sink, 62, s, 0);
    EXPECT_EQ(sink.scans.size(), 3u);
    feed_header(a, sink, 61, s, 0);  // late
    feed_header(a, sink, 62, s, 0);  // late
    EXPECT_EQ(sink.scans.size(), 3u);
    EXPECT_EQ(a.LateMessages(), 2u);
    feed_header(a, sink, 60, s, 0);  // older than two scans: a new scan (restart)
    EXPECT_EQ(sink.scans.size(), 4u);
}

TEST(RdiScanAssemblerTest, ChunkWithAnotherMessageCounterIsRejected)
{
    const Scan s = split(make_payload(30, 100));
    const Scan other = split(make_payload(30, 101));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 70, s, 0);
    feed_chunk(a, sink, 70, s, 0, 0);
    feed_chunk(a, sink, 70, other, 1, 0);  // MessageCounter 101 != 100
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_EQ(sink.scans[0].frame.message.num_detections, 27u);
}

TEST(RdiScanAssemblerTest, ChunkWithWrongSizeIsRejected)
{
    Scan s = split(make_payload(30, 4));
    s.chunks[1].pop_back();  // EntityCount 3 no longer matches the body
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 75, s, 0);
    feed_chunk(a, sink, 75, s, 0, 0);
    feed_chunk(a, sink, 75, s, 1, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_FALSE(sink.scans[0].IsComplete());
}

TEST(RdiScanAssemblerTest, ChunkWithFirstEntityIndexOffGridIsRejected)
{
    Scan s = split(make_payload(30, 6));
    pack_le16(&s.chunks[1][4], 28);  // not a multiple of EntitiesPerChunk 27
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 76, s, 0);
    feed_chunk(a, sink, 76, s, 0, 0);
    feed_chunk(a, sink, 76, s, 1, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 27));
}

TEST(RdiScanAssemblerTest, ChunksArePlacedByFirstEntityIndex)
{
    const Scan s = split(make_payload(60, 7));
    ASSERT_EQ(s.chunks.size(), 3u);
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 77, s, 0);
    // Event index and FirstEntityIndex disagree: the body decides the place.
    a.OnChunk(77, ok_e2e(), 0, s.chunks[1].data(), s.chunks[1].size(), 0, sink.emit());
    a.OnChunk(77, ok_e2e(), 1, s.chunks[0].data(), s.chunks[0].size(), 0, sink.emit());
    a.OnChunk(77, ok_e2e(), 2, s.chunks[2].data(), s.chunks[2].size(), 0, sink.emit());
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 3u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 0u);
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 60));
}

TEST(RdiScanAssemblerTest, SecondChunkForTheSameFirstEntityIndexIsRejected)
{
    const Scan s = split(make_payload(60, 8));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 78, s, 0);
    feed_chunk(a, sink, 78, s, 0, 0);
    a.OnChunk(78, ok_e2e(), 1, s.chunks[0].data(), s.chunks[0].size(), 0, sink.emit());
    feed_chunk(a, sink, 78, s, 2, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 2u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_EQ(sink.scans[0].frame.message.num_detections, 33u);
}

TEST(RdiScanAssemblerTest, ChunkWhoseSlotIsNotBelowChunkCountIsRejected)
{
    Scan s = split(make_payload(30, 9));
    ASSERT_EQ(s.chunks.size(), 2u);
    pack_le16(&s.chunks[1][4], 81);  // slot 3, ChunkCount is 2
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 79, s, 0);
    feed_chunk(a, sink, 79, s, 0, 0);
    feed_chunk(a, sink, 79, s, 1, 0);
    feed_header(a, sink, 79, s, 0);  // header B closes the scan
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 1u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 27));
}

TEST(RdiScanAssemblerTest, ChunkThatPassedE2eReplacesAFailedCopyOfItsSlot)
{
    const Scan s = split(make_payload(60, 10));
    Scan corrupt = s;
    pack_le16(&corrupt.chunks[1][4], 27);  // failed copy also claims slot 1
    corrupt.chunks[1][BTS_RDI_CHUNK_HEADER_SIZE + 1] = 0xEE;  // DetectionID damaged
    RdiMessageE2e mismatch = ok_e2e();
    mismatch.status = E2eStatus::kCrcMismatch;
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 81, s, 0);
    // Event 0x8100 carries the failed copy of slot 1; Event 0x8101 the good one.
    a.OnChunk(81, mismatch, 0, corrupt.chunks[1].data(), corrupt.chunks[1].size(), 0,
              sink.emit());
    feed_chunk(a, sink, 81, s, 1, 0);
    feed_chunk(a, sink, 81, s, 2, 0);
    feed_header(a, sink, 81, s, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    const RdiScan& r = sink.scans[0];
    EXPECT_EQ(r.chunks_received, 2u);
    EXPECT_EQ(r.chunks_rejected, 1u);
    ASSERT_EQ(r.frame.message.num_detections, 33u);
    EXPECT_EQ(r.frame.message.detections[0].detection_id, 27u);
    EXPECT_EQ(r.frame.message.detections[32].detection_id, 59u);
}

TEST(RdiScanAssemblerTest, FailedChunkDoesNotReplaceAPassedCopyOfItsSlot)
{
    const Scan s = split(make_payload(60, 11));
    RdiMessageE2e mismatch = ok_e2e();
    mismatch.status = E2eStatus::kCrcMismatch;
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 82, s, 0);
    feed_chunk(a, sink, 82, s, 0, 0);
    a.OnChunk(82, mismatch, 1, s.chunks[0].data(), s.chunks[0].size(), 0, sink.emit());
    feed_chunk(a, sink, 82, s, 2, 0);
    feed_header(a, sink, 82, s, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].chunks_received, 2u);
    EXPECT_EQ(sink.scans[0].chunks_rejected, 1u);
    EXPECT_EQ(sink.scans[0].frame.message.detections[0].detection_id, 0u);
}

TEST(RdiScanAssemblerTest, HeaderBodyLongerThan40BytesIsAccepted)
{
    Scan s = split(make_payload(30, 12));
    s.header.resize(BTS_RDI_HEADER_MSG_SIZE + 4, 0xAB);  // trailing bytes are ignored
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 83, s, 0);
    feed_chunk(a, sink, 83, s, 0, 0);
    feed_chunk(a, sink, 83, s, 1, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_TRUE(sink.scans[0].has_header);
    EXPECT_TRUE(sink.scans[0].IsComplete());
    EXPECT_TRUE(detections_in_order(sink.scans[0], 0, 30));
}

TEST(RdiScanAssemblerTest, HeaderBodyShorterThan40BytesIsInvalid)
{
    Scan s = split(make_payload(30, 13));
    s.header.resize(BTS_RDI_HEADER_MSG_SIZE - 1);
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 84, s, 0);
    feed_chunk(a, sink, 84, s, 0, 0);
    feed_chunk(a, sink, 84, s, 1, 0);
    a.Poll(100 * kMs, sink.emit());
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_FALSE(sink.scans[0].has_header);
    EXPECT_EQ(sink.scans[0].frame.message.num_detections, 0u);
}

TEST(RdiScanAssemblerTest, WorstE2eResultMarksTheScan)
{
    const Scan s = split(make_payload(60, 4));
    RdiScanAssembler a;
    Sink sink;
    RdiMessageE2e zero = ok_e2e();
    zero.status = E2eStatus::kCrcZero;
    RdiMessageE2e mismatch = ok_e2e();
    mismatch.status = E2eStatus::kCrcMismatch;
    mismatch.format = E2eCrcFormat::kUnknown;
    mismatch.length_ok = false;
    feed_header(a, sink, 80, s, 0);
    feed_chunk(a, sink, 80, s, 0, 0, mismatch);
    feed_chunk(a, sink, 80, s, 1, 0, zero);
    feed_chunk(a, sink, 80, s, 2, 0);
    ASSERT_EQ(sink.scans.size(), 1u);
    const RdiScan& r = sink.scans[0];
    EXPECT_EQ(r.e2e.status, E2eStatus::kCrcMismatch);
    EXPECT_EQ(r.e2e.format, E2eCrcFormat::kCrc32);
    EXPECT_FALSE(r.e2e.length_ok);
    EXPECT_TRUE(r.e2e.data_id_ok);
    EXPECT_TRUE(r.IsComplete());
}

TEST(RdiScanAssemblerTest, HeaderCopyThatPassedE2eIsPreferred)
{
    // ChunkCount 0 would close on the first header, so the scan has a chunk.
    const Scan s = split(make_payload(10, 4));
    Scan corrupt = s;
    corrupt.header[14] = 0xEE;  // MessageCounter differs in the failed copy
    RdiMessageE2e mismatch = ok_e2e();
    mismatch.status = E2eStatus::kCrcMismatch;
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 90, corrupt, 0, mismatch);
    EXPECT_EQ(sink.scans.size(), 0u);
    feed_header(a, sink, 90, s, 0);  // header B, passed: used and closes
    ASSERT_EQ(sink.scans.size(), 1u);
    EXPECT_EQ(sink.scans[0].frame.message.message_counter, 4u);
    EXPECT_EQ(sink.scans[0].chunks_received, 0u);
    EXPECT_EQ(sink.scans[0].e2e.status, E2eStatus::kCrcMismatch);
}

TEST(RdiScanAssemblerTest, ResetForgetsOpenAndClosedScans)
{
    const Scan s = split(make_payload(30, 4));
    RdiScanAssembler a;
    Sink sink;
    feed_header(a, sink, 95, s, 0);
    feed_chunk(a, sink, 95, s, 0, 0);
    a.Reset();
    a.Poll(1000 * kMs, sink.emit());
    EXPECT_TRUE(sink.scans.empty());
    feed_header(a, sink, 95, s, 0);
    feed_chunk(a, sink, 95, s, 0, 0);
    feed_chunk(a, sink, 95, s, 1, 0);
    EXPECT_EQ(sink.scans.size(), 1u);
}
