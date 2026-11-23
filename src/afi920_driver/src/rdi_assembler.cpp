// Copyright (c) 2026 bitsensing Inc. All rights reserved.
#include "afi920_driver/parsers/rdi_assembler.hpp"

#include "afi920_driver/parsers/endian_util.hpp"

#include <array>

namespace afi920_driver {

static_assert(kRdiMaxChunks == BTS_RDI_MAX_CHUNKS, "chunk Event range and interface limit differ");

namespace {

constexpr size_t kChunkHeaderSize = BTS_RDI_CHUNK_HEADER_SIZE;  // 8
constexpr size_t kDetectionSize = BTS_RDI_DETECTION_SIZE;       // 51

int StatusRank(E2eStatus status)
{
    switch (status) {
        case E2eStatus::kCrcMismatch: return 2;
        case E2eStatus::kCrcZero: return 1;
        default: return 0;
    }
}

bool Passed(const RdiMessageE2e& e2e)
{
    return e2e.status == E2eStatus::kOk && e2e.length_ok && e2e.data_id_ok;
}

}  // namespace

bool RdiScanAssembler::Admit(uint32_t e2e_counter, const RdiMessageE2e& e2e,
                             uint64_t now_ns, const Emit& emit)
{
    for (size_t i = 0; i < closed_count_; ++i) {
        if (closed_[i] == e2e_counter) {
            ++late_messages_;
            return false;
        }
    }
    if (open_ && e2e_counter != counter_) Close(emit);
    if (!open_) {
        open_ = true;
        counter_ = e2e_counter;
        first_ns_ = now_ns;
        headers_seen_ = 0;
        header_e2e_ok_ = false;
        header_.clear();
        e2e_ = RdiMessageE2e{};
    }
    if (StatusRank(e2e.status) > StatusRank(e2e_.status)) e2e_.status = e2e.status;
    if (e2e_.format == E2eCrcFormat::kUnknown) e2e_.format = e2e.format;
    e2e_.length_ok = e2e_.length_ok && e2e.length_ok;
    e2e_.data_id_ok = e2e_.data_id_ok && e2e.data_id_ok;
    return true;
}

void RdiScanAssembler::OnHeader(uint32_t e2e_counter, const RdiMessageE2e& e2e,
                                const uint8_t* body, size_t len,
                                uint64_t now_ns, const Emit& emit)
{
    if (!Admit(e2e_counter, e2e, now_ns, emit)) return;
    ++headers_seen_;
    // Header A and header B carry the same body; one is used, preferring a
    // copy that passed its E2E check.
    const bool ok = Passed(e2e);
    if (body && (header_.empty() || (!header_e2e_ok_ && ok))) {
        header_.assign(body, body + len);
        header_e2e_ok_ = ok;
    }
    if (headers_seen_ >= 2 || IsComplete()) Close(emit);
}

void RdiScanAssembler::OnChunk(uint32_t e2e_counter, const RdiMessageE2e& e2e,
                               uint16_t chunk_index, const uint8_t* body,
                               size_t len, uint64_t now_ns, const Emit& emit)
{
    if (chunk_index >= kRdiMaxChunks || !body) return;
    if (!Admit(e2e_counter, e2e, now_ns, emit)) return;
    Chunk& c = chunks_[chunk_index];
    const bool ok = Passed(e2e);
    if (!c.present || (!c.e2e_ok && ok)) {
        c.present = true;
        c.e2e_ok = ok;
        c.body.assign(body, body + len);
    }
    if (IsComplete()) Close(emit);
}

void RdiScanAssembler::Poll(uint64_t now_ns, const Emit& emit)
{
    if (open_ && now_ns - first_ns_ >= timeout_ns_) Close(emit);
}

void RdiScanAssembler::Reset()
{
    open_ = false;
    header_.clear();
    for (Chunk& c : chunks_) c.present = false;
    closed_count_ = 0;
}

bool RdiScanAssembler::IsComplete() const
{
    if (header_.size() < BTS_RDI_HEADER_MSG_SIZE) return false;
    const uint16_t chunk_count = detail::unpack_le16(header_.data() + BTS_RDI_HEADER_SIZE + 2);
    if (chunk_count > kRdiMaxChunks) return false;
    for (uint16_t k = 0; k < chunk_count; ++k) {
        if (!chunks_[k].present) return false;
    }
    return true;
}

void RdiScanAssembler::Close(const Emit& emit)
{
    open_ = false;
    closed_[1] = closed_[0];
    closed_[0] = counter_;
    if (closed_count_ < 2) ++closed_count_;

    RdiScan scan;
    scan.e2e_counter = counter_;
    scan.e2e = e2e_;

    RdiListLayout layout;
    scan.has_header =
        !header_.empty() &&
        RdiParser::ParseHeaderMessage(header_.data(), header_.size(),
                                      scan.frame.message, layout) == 0 &&
        layout.entities_per_chunk != 0 && layout.chunk_count <= kRdiMaxChunks;
    if (!scan.has_header) {
        // Without a header the timestamp and Ambiguity Domain are unknown.
        scan.frame = RdiFrame{};
        for (Chunk& c : chunks_) c.present = false;
        emit(scan);
        return;
    }

    // Validate each chunk, then place it by FirstEntityIndex (01_RDI section 2,
    // step 5). The slot of a chunk is FirstEntityIndex / EntitiesPerChunk; a
    // chunk whose FirstEntityIndex is not a multiple of EntitiesPerChunk is
    // dropped (Interface Overview section 3.2.3, partial reception). When two
    // chunks claim one slot, a copy that passed its E2E check replaces one that
    // did not; otherwise the first one is kept.
    std::array<int, kRdiMaxChunks> slot_chunk;
    slot_chunk.fill(-1);
    for (uint16_t k = 0; k < kRdiMaxChunks; ++k) {
        Chunk& c = chunks_[k];
        if (!c.present) continue;
        c.present = false;
        const uint8_t* b = c.body.data();
        const size_t len = c.body.size();
        if (k >= layout.chunk_count || len < kChunkHeaderSize) {
            ++scan.chunks_rejected;
            continue;
        }
        const uint32_t message_counter = detail::unpack_le32(b);
        const uint16_t first_index = detail::unpack_le16(b + 4);
        const uint16_t entity_count = detail::unpack_le16(b + 6);
        const uint16_t slot = static_cast<uint16_t>(first_index / layout.entities_per_chunk);
        if (entity_count > layout.entities_per_chunk ||
            len != kChunkHeaderSize + static_cast<size_t>(entity_count) * kDetectionSize ||
            message_counter != scan.frame.message.message_counter ||
            first_index % layout.entities_per_chunk != 0 ||
            slot >= layout.chunk_count) {
            ++scan.chunks_rejected;
            continue;
        }
        if (slot_chunk[slot] >= 0) {
            ++scan.chunks_rejected;
            if (!c.e2e_ok || chunks_[slot_chunk[slot]].e2e_ok) continue;
        }
        slot_chunk[slot] = k;
    }

    auto& storage = scan.frame.detections_storage;
    for (uint16_t slot = 0; slot < layout.chunk_count; ++slot) {
        if (slot_chunk[slot] < 0) continue;
        const std::vector<uint8_t>& body = chunks_[slot_chunk[slot]].body;
        const uint8_t* b = body.data();
        const uint16_t entity_count = detail::unpack_le16(b + 6);
        const size_t base = storage.size();
        storage.resize(base + entity_count);
        for (uint16_t i = 0; i < entity_count; ++i) {
            RdiParser::ParseDetection(b + kChunkHeaderSize + i * kDetectionSize,
                                      storage[base + i]);
        }
        ++scan.chunks_received;
    }

    scan.announced_detections = scan.frame.message.num_detections;
    scan.chunk_count = layout.chunk_count;
    scan.frame.message.num_detections = static_cast<uint16_t>(storage.size());
    scan.frame.message.detections = storage.empty() ? nullptr : storage.data();
    emit(scan);
}

}  // namespace afi920_driver
