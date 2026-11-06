// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file config_client.cpp
 * @brief One-shot TCP client for the AFI920 Config API stream destination
 */

#include "afi920_driver/transport/config_client.hpp"
#include "afi920_driver/someip/e2e.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace afi920_driver {

// ─── SOME/IP Config API constants ────────────────────────────────────────────

static constexpr uint16_t kServiceId        = 0x6000;
static constexpr uint16_t kClientId         = 0x0001;
static constexpr uint8_t  kProtocolVersion  = 0x01;
static constexpr uint8_t  kInterfaceVersion = 0x01;
static constexpr uint8_t  kMsgTypeRequest   = 0x00;
static constexpr uint8_t  kMsgTypeResponse  = 0x80;
static constexpr size_t   kHeaderSize       = 16;
static constexpr uint32_t kMaxResponseBody  = 1024 * 1024;

// v1.1.0+ sends stream data to this address when the stored destination is 0.0.0.0.
static constexpr const char* kDefaultStreamDestination = "192.168.10.100";

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

// Returns the position right after `"key":` and any whitespace, or npos.
static size_t json_value_pos(const std::string& text, const char* key)
{
    const std::string quoted = std::string("\"") + key + "\"";
    size_t pos = text.find(quoted);
    if (pos == std::string::npos) {
        return std::string::npos;
    }
    pos = text.find(':', pos + quoted.size());
    if (pos == std::string::npos) {
        return std::string::npos;
    }
    ++pos;
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }
    return pos;
}

static bool json_get_int(const std::string& text, const char* key, int& out)
{
    const size_t pos = json_value_pos(text, key);
    if (pos == std::string::npos) {
        return false;
    }
    char* end = nullptr;
    const int64_t parsed = std::strtoll(text.c_str() + pos, &end, 10);
    if (end == text.c_str() + pos) {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

static bool json_get_string(const std::string& text, const char* key, std::string& out)
{
    const size_t pos = json_value_pos(text, key);
    if (pos == std::string::npos || pos >= text.size() || text[pos] != '"') {
        return false;
    }
    const size_t end = text.find('"', pos + 1);
    if (end == std::string::npos) {
        return false;
    }
    out = text.substr(pos + 1, end - pos - 1);
    return true;
}

static uint16_t next_session_id()
{
    static std::atomic<uint16_t> session{0};
    return static_cast<uint16_t>(session.fetch_add(1) + 1);
}

static uint32_t next_e2e_counter()
{
    static std::atomic<uint32_t> counter{0};
    return counter.fetch_add(1) + 1;
}

/**
 * @brief Sends one request and reads one response on a fresh connection.
 * @return 0 on success, -1 socket, -2 address, -3 connect, -4 send,
 *         -5 no complete response, -9 unparseable response.
 */
static int config_transact(const std::string& sensor_ip, uint16_t config_port,
                           int timeout_ms, const std::vector<uint8_t>& request,
                           uint16_t method_id, ConfigResponse& out)
{
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

    if (send_all(fd, request.data(), request.size()) < 0) {
        ::close(fd);
        return -4;
    }

    std::vector<uint8_t> frame(kHeaderSize);
    if (recv_exact(fd, frame.data(), kHeaderSize) < 0) {
        ::close(fd);
        return -5;
    }

    const uint32_t length = unpack_be32(frame.data() + 4);
    if (length < 8 || length - 8 > kMaxResponseBody) {
        ::close(fd);
        return -9;
    }
    const size_t body_len = length - 8;
    frame.resize(kHeaderSize + body_len);
    if (body_len > 0 && recv_exact(fd, frame.data() + kHeaderSize, body_len) < 0) {
        ::close(fd);
        return -5;
    }
    ::close(fd);

    return (parse_config_response(frame.data(), frame.size(), method_id, out) == 0) ? 0 : -9;
}

// ─── Frame helpers ───────────────────────────────────────────────────────────

std::vector<uint8_t> build_config_request(uint16_t method_id, uint16_t session_id,
                                          const std::string& json, ConfigApiFormat format)
{
    const size_t e2e_len = (format == ConfigApiFormat::kE2e) ? kE2eHeaderSize : 0;
    std::vector<uint8_t> frame(kHeaderSize + e2e_len + json.size());

    pack_be16(frame.data() + 0, kServiceId);
    pack_be16(frame.data() + 2, method_id);
    pack_be32(frame.data() + 4, static_cast<uint32_t>(8 + e2e_len + json.size()));
    pack_be16(frame.data() + 8, kClientId);
    pack_be16(frame.data() + 10, session_id);
    frame[12] = kProtocolVersion;
    frame[13] = kInterfaceVersion;
    frame[14] = kMsgTypeRequest;
    frame[15] = 0x00;  // return_code

    uint8_t* body = frame.data() + kHeaderSize + e2e_len;
    std::memcpy(body, json.data(), json.size());
    if (e2e_len > 0) {
        e2e_build_header(frame.data() + kHeaderSize, kE2eDataIdServiceBase | method_id,
                         next_e2e_counter(), body, json.size());
    }
    return frame;
}

int parse_config_response(const uint8_t* frame, size_t len, uint16_t method_id,
                          ConfigResponse& out)
{
    if (!frame || len < kHeaderSize) {
        return -1;
    }

    out.message_type = frame[14];
    out.return_code = frame[15];
    out.crc_ok = true;

    const uint8_t* body = frame + kHeaderSize;
    const size_t body_len = len - kHeaderSize;

    // v1.1.0+: CRC upper word 0, Length equal to the JSON size, Data ID of the method.
    if (body_len >= kE2eHeaderSize &&
        unpack_be32(body + kE2eOffsetCrc) == 0u &&
        unpack_be32(body + kE2eOffsetLength) == body_len - kE2eHeaderSize &&
        unpack_be32(body + kE2eOffsetDataId) == (kE2eDataIdServiceBase | method_id)) {
        const uint8_t* json = body + kE2eHeaderSize;
        const size_t json_len = body_len - kE2eHeaderSize;
        out.format = ConfigApiFormat::kE2e;
        out.crc_ok = (e2e_crc32(json, json_len) == unpack_be32(body + kE2eOffsetCrc32));
        out.json.assign(reinterpret_cast<const char*>(json), json_len);
        return 0;
    }

    // v1.0.x: the body is JSON text (or empty).
    size_t i = 0;
    while (i < body_len && std::isspace(body[i])) {
        ++i;
    }
    if (i == body_len || body[i] == '{') {
        out.format = ConfigApiFormat::kLegacy;
        out.json.assign(reinterpret_cast<const char*>(body), body_len);
        return 0;
    }

    out.format = ConfigApiFormat::kUnknown;
    return -2;
}

bool parse_network_config(const std::string& json, NetworkConfig& out)
{
    struct Entry {
        const char* ip_key;
        const char* protocol_key;
        const char* port_key;
        StreamDestination* dest;
    };
    const Entry entries[] = {
        {"detection_ip", "detection_protocol", "detection_port", &out.detection},
        {"health_ip", "health_protocol", "health_port", &out.health},
        {"performance_ip", "performance_protocol", "performance_port", &out.performance},
    };

    for (const auto& e : entries) {
        if (!json_get_string(json, e.ip_key, e.dest->ip) ||
            !json_get_string(json, e.protocol_key, e.dest->protocol) ||
            !json_get_int(json, e.port_key, e.dest->port)) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> compare_stream_config(const NetworkConfig& actual,
                                               const StreamConfigRequest& request)
{
    struct Entry {
        const char* name;
        const StreamDestination* dest;
        uint16_t port;
    };
    const Entry entries[] = {
        {"detection", &actual.detection, request.data_port},
        {"health", &actual.health, request.shi_port},
        {"performance", &actual.performance, request.spi_port},
    };

    std::vector<std::string> mismatches;
    for (const auto& e : entries) {
        const std::string prefix = e.name;
        if (e.dest->protocol != request.transport_mode) {
            mismatches.push_back(prefix + "_protocol=" + e.dest->protocol +
                                 " (requested " + request.transport_mode + ")");
        }
        if (e.dest->port != static_cast<int>(e.port)) {
            mismatches.push_back(prefix + "_port=" + std::to_string(e.dest->port) +
                                 " (requested " + std::to_string(e.port) + ")");
        }
        // TCP streams go to the connected host, so the IP matters only for UDP.
        if (request.transport_mode == "udp" && !request.host_ip.empty()) {
            const std::string effective_ip =
                (e.dest->ip == "0.0.0.0") ? kDefaultStreamDestination : e.dest->ip;
            if (effective_ip != request.host_ip) {
                mismatches.push_back(prefix + "_ip=" + e.dest->ip +
                                     " (requested " + request.host_ip + ")");
            }
        }
    }
    return mismatches;
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

int sync_stream_config(
    const std::string& sensor_ip,
    uint16_t config_port,
    const StreamConfigRequest& request,
    int timeout_ms,
    StreamConfigResult& result)
{
    result = StreamConfigResult{};

    // --- Detect the frame format with GetNetworkConfig ---
    ConfigResponse rsp;
    int rc = config_transact(
        sensor_ip, config_port, timeout_ms,
        build_config_request(kConfigMethodGetNetworkConfig, next_session_id(), "{}",
                             ConfigApiFormat::kE2e),
        kConfigMethodGetNetworkConfig, rsp);
    if (rc == -5) {
        // A v1.0.x sensor may drop a request whose body does not start with JSON.
        rc = config_transact(
            sensor_ip, config_port, timeout_ms,
            build_config_request(kConfigMethodGetNetworkConfig, next_session_id(), "{}",
                                 ConfigApiFormat::kLegacy),
            kConfigMethodGetNetworkConfig, rsp);
    }
    if (rc < 0) {
        return rc;
    }
    result.format = rsp.format;

    if (rsp.format == ConfigApiFormat::kE2e) {
        result.response_crc_ok = rsp.crc_ok;
        if ((rsp.message_type & kMsgTypeResponse) == 0) return -6;
        if (rsp.return_code != 0x00) return -7;

        NetworkConfig actual;
        if (!parse_network_config(rsp.json, actual)) {
            return -9;
        }
        result.mismatches = compare_stream_config(actual, request);
        return 0;
    }

    // --- v1.0.x: apply the destination with SetNetworkConfig ---
    // Manual construction to avoid JSON library dependency
    const std::string json =
        "{\"detection_ip\":\"" + request.host_ip + "\""
        ",\"detection_port\":" + std::to_string(request.data_port) +
        ",\"detection_protocol\":\"" + request.transport_mode + "\""
        ",\"health_ip\":\"" + request.host_ip + "\""
        ",\"health_port\":" + std::to_string(request.shi_port) +
        ",\"health_protocol\":\"" + request.transport_mode + "\""
        ",\"performance_ip\":\"" + request.host_ip + "\""
        ",\"performance_port\":" + std::to_string(request.spi_port) +
        ",\"performance_protocol\":\"" + request.transport_mode + "\""
        "}";

    rc = config_transact(
        sensor_ip, config_port, timeout_ms,
        build_config_request(kConfigMethodSetNetworkConfig, next_session_id(), json,
                             ConfigApiFormat::kLegacy),
        kConfigMethodSetNetworkConfig, rsp);
    if (rc < 0) {
        return rc;
    }

    if ((rsp.message_type & kMsgTypeResponse) == 0) return -6;
    if (rsp.return_code != 0x00) return -7;
    // Config API reports validation failures in JSON return_code even when the
    // SOME/IP header rc is OK.
    int json_return_code = 0;
    if (json_get_int(rsp.json, "return_code", json_return_code) && json_return_code != 0) {
        return -8;
    }

    result.applied = true;
    return 0;
}

}  // namespace afi920_driver
