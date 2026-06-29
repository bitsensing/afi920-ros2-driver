// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file config_client.cpp
 * @brief One-shot TCP client for AFI920 sensor stream configuration
 */

#include "afi920_driver/transport/config_client.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace afi920_driver {

// ─── SOME/IP Config API constants ────────────────────────────────────────────

static constexpr uint16_t kServiceId        = 0x6000;
static constexpr uint16_t kMethodSetNetworkConfig = 0x0021;
static constexpr uint16_t kClientId         = 0x0001;
static constexpr uint8_t  kProtocolVersion  = 0x01;
static constexpr uint8_t  kInterfaceVersion = 0x01;
static constexpr uint8_t  kMsgTypeRequest   = 0x00;
static constexpr uint8_t  kMsgTypeResponse  = 0x80;
static constexpr size_t   kHeaderSize       = 16;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static void pack_be16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

static void pack_be32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}

static uint32_t unpack_be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
           (static_cast<uint32_t>(p[3]));
}

static int send_all(int fd, const uint8_t* buf, size_t n)
{
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = ::send(fd, buf + sent, n - sent, MSG_NOSIGNAL);
        if (r <= 0) return -1;
        sent += static_cast<size_t>(r);
    }
    return 0;
}

static int recv_exact(int fd, uint8_t* buf, size_t n)
{
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::recv(fd, buf + got, n - got, 0);
        if (r <= 0) return -1;
        got += static_cast<size_t>(r);
    }
    return 0;
}

static bool parse_json_return_code(const std::vector<uint8_t>& body, int& code)
{
    code = 0;
    if (body.empty()) {
        return false;
    }

    const std::string text(reinterpret_cast<const char*>(body.data()), body.size());
    size_t pos = text.find("\"return_code\"");
    if (pos == std::string::npos) {
        return false;
    }

    pos = text.find(':', pos);
    if (pos == std::string::npos) {
        return false;
    }
    ++pos;
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }

    char* end = nullptr;
    const int64_t parsed = std::strtoll(text.c_str() + pos, &end, 10);
    if (end == text.c_str() + pos) {
        return false;
    }

    code = static_cast<int>(parsed);
    return true;
}

// ─── Public API ──────────────────────────────────────────────────────────────

std::string detect_local_ip(const std::string& sensor_ip)
{
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return {};

    struct sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(1);
    if (::inet_pton(AF_INET, sensor_ip.c_str(), &remote.sin_addr) != 1) {
        ::close(fd);
        return {};
    }

    // connect() on UDP doesn't send anything — just sets the routing
    if (::connect(fd, reinterpret_cast<struct sockaddr*>(&remote), sizeof(remote)) < 0) {
        ::close(fd);
        return {};
    }

    struct sockaddr_in local{};
    socklen_t len = sizeof(local);
    if (::getsockname(fd, reinterpret_cast<struct sockaddr*>(&local), &len) < 0) {
        ::close(fd);
        return {};
    }

    ::close(fd);

    char buf[INET_ADDRSTRLEN];
    if (!::inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf))) {
        return {};
    }
    return std::string(buf);
}

int configure_streams(
    const std::string& sensor_ip,
    uint16_t config_port,
    const std::string& host_ip,
    uint16_t data_port,
    uint16_t shi_port,
    uint16_t spi_port,
    int timeout_ms,
    const std::string& transport_mode)
{
    // --- Build JSON payload ---
    // Manual construction to avoid JSON library dependency
    std::string json =
        "{\"detection_ip\":\"" + host_ip + "\""
        ",\"detection_port\":" + std::to_string(data_port) +
        ",\"detection_protocol\":\"" + transport_mode + "\""
        ",\"health_ip\":\"" + host_ip + "\""
        ",\"health_port\":" + std::to_string(shi_port) +
        ",\"health_protocol\":\"" + transport_mode + "\""
        ",\"performance_ip\":\"" + host_ip + "\""
        ",\"performance_port\":" + std::to_string(spi_port) +
        ",\"performance_protocol\":\"" + transport_mode + "\""
        "}";

    // --- Build SOME/IP header ---
    uint32_t payload_len = static_cast<uint32_t>(json.size());
    uint32_t someip_length = 8 + payload_len;  // 8 = remaining header after length field

    uint8_t header[kHeaderSize];
    pack_be16(header + 0, kServiceId);
    pack_be16(header + 2, kMethodSetNetworkConfig);
    pack_be32(header + 4, someip_length);
    pack_be16(header + 8, kClientId);
    pack_be16(header + 10, 0x0001);  // session_id
    header[12] = kProtocolVersion;
    header[13] = kInterfaceVersion;
    header[14] = kMsgTypeRequest;
    header[15] = 0x00;  // return_code

    // --- TCP connect ---
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_port);
    if (::inet_pton(AF_INET, sensor_ip.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        return -2;
    }

    if (::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -3;
    }

    // --- Send request ---
    std::vector<uint8_t> request(kHeaderSize + payload_len);
    std::memcpy(request.data(), header, kHeaderSize);
    std::memcpy(request.data() + kHeaderSize, json.data(), payload_len);

    if (send_all(fd, request.data(), request.size()) < 0) {
        ::close(fd);
        return -4;
    }

    // --- Receive response header ---
    uint8_t resp_hdr[kHeaderSize];
    if (recv_exact(fd, resp_hdr, kHeaderSize) < 0) {
        ::close(fd);
        return -5;
    }

    uint8_t msg_type = resp_hdr[14];
    uint8_t ret_code = resp_hdr[15];

    // Drain and inspect any response body. Config API reports validation
    // failures in JSON return_code even when the SOME/IP header rc is OK.
    uint32_t resp_length = unpack_be32(resp_hdr + 4);
    std::vector<uint8_t> body;
    if (resp_length > 8) {
        uint32_t body_len = resp_length - 8;
        if (body_len > 1024 * 1024) body_len = 0;  // sanity
        body.resize(body_len);
        recv_exact(fd, body.data(), body_len);  // best-effort drain
    }

    ::close(fd);

    // Check response
    if ((msg_type & kMsgTypeResponse) == 0) return -6;
    if (ret_code != 0x00) return -7;
    int json_return_code = 0;
    if (parse_json_return_code(body, json_return_code) && json_return_code != 0) {
        return -8;
    }

    return 0;
}

}  // namespace afi920_driver
