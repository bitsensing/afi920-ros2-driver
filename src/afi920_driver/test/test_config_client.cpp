// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file test_config_client.cpp
 * @brief Unit tests for Config API frame building and format detection
 */

#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <vector>

#include "afi920_driver/someip/e2e.hpp"
#include "afi920_driver/transport/config_client.hpp"

using namespace afi920_driver;  // NOLINT(build/namespaces)

namespace {

uint32_t be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

void put_be32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

std::vector<uint8_t> response_header(uint16_t method_id, size_t body_len, uint8_t rc)
{
    std::vector<uint8_t> frame(16, 0);
    frame[0] = 0x60;
    frame[2] = static_cast<uint8_t>(method_id >> 8);
    frame[3] = static_cast<uint8_t>(method_id);
    put_be32(frame.data() + 4, static_cast<uint32_t>(8 + body_len));
    frame[12] = 0x01;
    frame[13] = 0x01;
    frame[14] = 0x80;
    frame[15] = rc;
    return frame;
}

std::vector<uint8_t> legacy_response(uint16_t method_id, const std::string& json, uint8_t rc = 0)
{
    std::vector<uint8_t> frame = response_header(method_id, json.size(), rc);
    frame.insert(frame.end(), json.begin(), json.end());
    return frame;
}

std::vector<uint8_t> e2e_response(uint16_t method_id, const std::string& json, uint8_t rc = 0)
{
    std::vector<uint8_t> frame = response_header(method_id, kE2eHeaderSize + json.size(), rc);
    std::vector<uint8_t> e2e(kE2eHeaderSize);
    e2e_build_header(e2e.data(), kE2eDataIdServiceBase | method_id, 1,
                     reinterpret_cast<const uint8_t*>(json.data()), json.size());
    frame.insert(frame.end(), e2e.begin(), e2e.end());
    frame.insert(frame.end(), json.begin(), json.end());
    return frame;
}

const char* kNetworkJson =
    "{\"return_code\": 0, \"return_name\": \"SUCCESS\", \"data\": {"
    "\"ip_address\": \"192.168.10.150\", \"api_port\": 30500, "
    "\"detection_ip\": \"192.168.10.100\", \"detection_protocol\": \"tcp\", \"detection_port\": 30509, "
    "\"health_ip\": \"192.168.10.100\", \"health_protocol\": \"tcp\", \"health_port\": 30510, "
    "\"performance_ip\": \"0.0.0.0\", \"performance_protocol\": \"tcp\", \"performance_port\": 30511}}";

StreamConfigRequest default_request(const std::string& mode)
{
    StreamConfigRequest req;
    req.host_ip = "192.168.10.100";
    req.data_port = 30509;
    req.shi_port = 30510;
    req.spi_port = 30511;
    req.transport_mode = mode;
    return req;
}

}  // namespace

// ─── Request frames ───

TEST(ConfigClientTest, LegacyRequestHasJsonAfterSomeipHeader) {
    const auto frame = build_config_request(0x0021, 7, "{}", ConfigApiFormat::kLegacy);
    ASSERT_EQ(frame.size(), 18u);
    EXPECT_EQ(be32(frame.data()), 0x60000021u);
    EXPECT_EQ(be32(frame.data() + 4), 10u);
    EXPECT_EQ(frame[10], 0x00);
    EXPECT_EQ(frame[11], 0x07);
    EXPECT_EQ(frame[14], 0x00);
    EXPECT_EQ(frame[16], '{');
}

TEST(ConfigClientTest, E2eRequestMatchesSystemSpecExample) {
    // System Spec 4.4: Length = 8 + 20 (E2E) + 2 ("{}") = 30.
    const auto frame = build_config_request(0x0011, 1, "{}", ConfigApiFormat::kE2e);
    ASSERT_EQ(frame.size(), 16u + 20u + 2u);
    EXPECT_EQ(be32(frame.data()), 0x60000011u);
    EXPECT_EQ(be32(frame.data() + 4), 30u);

    const uint8_t* e2e = frame.data() + 16;
    EXPECT_EQ(be32(e2e + kE2eOffsetCrc), 0u);
    EXPECT_EQ(be32(e2e + kE2eOffsetCrc32), e2e_crc32(frame.data() + 36, 2));
    EXPECT_EQ(be32(e2e + kE2eOffsetLength), 2u);
    EXPECT_EQ(be32(e2e + kE2eOffsetDataId), 0x60000011u);
    EXPECT_EQ(frame[36], '{');
    EXPECT_EQ(frame[37], '}');
}

// ─── Response format detection ───

TEST(ConfigClientTest, ParsesLegacyResponse) {
    const auto frame = legacy_response(0x0020, kNetworkJson);
    ConfigResponse rsp;
    ASSERT_EQ(parse_config_response(frame.data(), frame.size(), 0x0020, rsp), 0);
    EXPECT_EQ(rsp.format, ConfigApiFormat::kLegacy);
    EXPECT_EQ(rsp.json, kNetworkJson);
    EXPECT_TRUE(rsp.crc_ok);
}

TEST(ConfigClientTest, ParsesLegacyErrorResponse) {
    // A v1.0.x sensor answering an E2E-prefixed request with a JSON error.
    const auto frame = legacy_response(0x0020, "{\"return_code\": 1}", 0x01);
    ConfigResponse rsp;
    ASSERT_EQ(parse_config_response(frame.data(), frame.size(), 0x0020, rsp), 0);
    EXPECT_EQ(rsp.format, ConfigApiFormat::kLegacy);
    EXPECT_EQ(rsp.return_code, 0x01);
}

TEST(ConfigClientTest, ParsesE2eResponse) {
    const auto frame = e2e_response(0x0020, kNetworkJson);
    ConfigResponse rsp;
    ASSERT_EQ(parse_config_response(frame.data(), frame.size(), 0x0020, rsp), 0);
    EXPECT_EQ(rsp.format, ConfigApiFormat::kE2e);
    EXPECT_EQ(rsp.json, kNetworkJson);
    EXPECT_TRUE(rsp.crc_ok);
}

TEST(ConfigClientTest, E2eResponseCrcMismatchIsReported) {
    auto frame = e2e_response(0x0020, kNetworkJson);
    frame.back() ^= 0x01;
    ConfigResponse rsp;
    ASSERT_EQ(parse_config_response(frame.data(), frame.size(), 0x0020, rsp), 0);
    EXPECT_EQ(rsp.format, ConfigApiFormat::kE2e);
    EXPECT_FALSE(rsp.crc_ok);
}

TEST(ConfigClientTest, E2eResponseOfOtherMethodIsRejected) {
    const auto frame = e2e_response(0x0011, kNetworkJson);
    ConfigResponse rsp;
    EXPECT_LT(parse_config_response(frame.data(), frame.size(), 0x0020, rsp), 0);
}

TEST(ConfigClientTest, TooShortResponseIsRejected) {
    const uint8_t frame[8] = {};
    ConfigResponse rsp;
    EXPECT_LT(parse_config_response(frame, sizeof(frame), 0x0020, rsp), 0);
}

// ─── GetNetworkConfig comparison ───

TEST(ConfigClientTest, ParsesNetworkConfig) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    EXPECT_EQ(cfg.detection.ip, "192.168.10.100");
    EXPECT_EQ(cfg.detection.protocol, "tcp");
    EXPECT_EQ(cfg.detection.port, 30509);
    EXPECT_EQ(cfg.health.port, 30510);
    EXPECT_EQ(cfg.performance.ip, "0.0.0.0");
    EXPECT_EQ(cfg.performance.port, 30511);
}

TEST(ConfigClientTest, NetworkConfigWithMissingFieldIsRejected) {
    NetworkConfig cfg;
    EXPECT_FALSE(parse_network_config("{\"data\": {\"detection_ip\": \"1.2.3.4\"}}", cfg));
}

TEST(ConfigClientTest, TcpRequestMatchesFactoryDefaults) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    EXPECT_TRUE(compare_stream_config(cfg, default_request("tcp")).empty());
}

TEST(ConfigClientTest, TcpRequestIgnoresDestinationIp) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    StreamConfigRequest req = default_request("tcp");
    req.host_ip = "10.0.0.5";
    EXPECT_TRUE(compare_stream_config(cfg, req).empty());
}

TEST(ConfigClientTest, UdpRequestReportsProtocolMismatch) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    const auto mismatches = compare_stream_config(cfg, default_request("udp"));
    ASSERT_EQ(mismatches.size(), 3u);
    EXPECT_EQ(mismatches[0], "detection_protocol=tcp (requested udp)");
}

TEST(ConfigClientTest, UdpRequestResolvesZeroDestination) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    cfg.detection.protocol = cfg.health.protocol = cfg.performance.protocol = "udp";
    // performance_ip 0.0.0.0 resolves to the factory destination 192.168.10.100.
    EXPECT_TRUE(compare_stream_config(cfg, default_request("udp")).empty());

    StreamConfigRequest req = default_request("udp");
    req.host_ip = "192.168.10.20";
    const auto mismatches = compare_stream_config(cfg, req);
    ASSERT_EQ(mismatches.size(), 3u);
    EXPECT_EQ(mismatches[2], "performance_ip=0.0.0.0 (requested 192.168.10.20)");
}

TEST(ConfigClientTest, PortMismatchIsReported) {
    NetworkConfig cfg;
    ASSERT_TRUE(parse_network_config(kNetworkJson, cfg));
    StreamConfigRequest req = default_request("tcp");
    req.data_port = 40000;
    const auto mismatches = compare_stream_config(cfg, req);
    ASSERT_EQ(mismatches.size(), 1u);
    EXPECT_EQ(mismatches[0], "detection_port=30509 (requested 40000)");
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
