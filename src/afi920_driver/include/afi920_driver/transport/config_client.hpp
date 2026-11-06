// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file config_client.hpp
 * @brief One-shot TCP client for the AFI920 Config API stream destination
 *
 * Supports both Config API frame formats:
 *   Interface v1.0.x  : SOME/IP 16B + JSON
 *   Interface v1.1.0+ : SOME/IP 16B + E2E 20B + JSON (request and response)
 *
 * The format is detected from the GetNetworkConfig (0x0020) response. On
 * v1.0.x the stream destination is applied with SetNetworkConfig (0x0021),
 * which takes effect immediately. On v1.1.0+ SetNetworkConfig requires the
 * diagnostic access mode, writes NVM and takes effect only after Restart, so
 * the driver only reads the configuration and reports differences.
 */

#ifndef AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_
#define AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace afi920_driver {

static constexpr uint16_t kConfigMethodGetNetworkConfig = 0x0020;
static constexpr uint16_t kConfigMethodSetNetworkConfig = 0x0021;

/// Config API frame format of the connected sensor.
enum class ConfigApiFormat : uint8_t {
    kUnknown = 0,
    kLegacy,   ///< Interface v1.0.x: no E2E header
    kE2e,      ///< Interface v1.1.0+: E2E header before JSON
};

/// One stream destination as reported by GetNetworkConfig.
struct StreamDestination {
    std::string ip;
    std::string protocol;
    int port = -1;
};

/// Stream destinations of RDI (detection), SHII (health) and SPI (performance).
struct NetworkConfig {
    StreamDestination detection;
    StreamDestination health;
    StreamDestination performance;
};

/// Stream destination the driver expects the sensor to use.
struct StreamConfigRequest {
    std::string host_ip;
    uint16_t data_port = 0;
    uint16_t shi_port = 0;
    uint16_t spi_port = 0;
    std::string transport_mode = "tcp";
};

/// Outcome of sync_stream_config().
struct StreamConfigResult {
    ConfigApiFormat format = ConfigApiFormat::kUnknown;
    bool applied = false;               ///< v1.0.x: SetNetworkConfig succeeded
    bool response_crc_ok = true;        ///< v1.1.0+: GetNetworkConfig response CRC-32 matched
    std::vector<std::string> mismatches;  ///< v1.1.0+: differences to the request
};

/// Parsed Config API response.
struct ConfigResponse {
    ConfigApiFormat format = ConfigApiFormat::kUnknown;
    uint8_t message_type = 0;
    uint8_t return_code = 0;
    bool crc_ok = true;   ///< E2E CRC-32 of the JSON body (always true for kLegacy)
    std::string json;
};

/**
 * @brief Detect the local IP address that routes to the given sensor IP.
 * @param sensor_ip  Sensor IP address (e.g. "192.168.10.150")
 * @return Local interface IP, or empty string on failure.
 */
std::string detect_local_ip(const std::string& sensor_ip);

/**
 * @brief Bring the sensor stream destination in line with the driver request.
 *
 * Sends GetNetworkConfig with an E2E header, falling back to the v1.0.x frame
 * when the sensor does not answer. v1.0.x sensors then receive a
 * SetNetworkConfig partial update. v1.1.0+ sensors are only compared against
 * the request and result.mismatches lists every difference.
 *
 * @return 0 on success (including v1.1.0+ with mismatches), negative on error.
 */
int sync_stream_config(
    const std::string& sensor_ip,
    uint16_t config_port,
    const StreamConfigRequest& request,
    int timeout_ms,
    StreamConfigResult& result);

// --- Frame helpers (exposed for unit tests) ---

/// Builds a Config API request frame in the given format.
std::vector<uint8_t> build_config_request(uint16_t method_id, uint16_t session_id,
                                          const std::string& json, ConfigApiFormat format);

/**
 * @brief Parses a Config API response frame (SOME/IP header + body).
 * @return 0 on success, negative when the frame is malformed or the body
 *         matches neither format.
 */
int parse_config_response(const uint8_t* frame, size_t len, uint16_t method_id,
                          ConfigResponse& out);

/// Extracts the stream destinations from a GetNetworkConfig JSON response.
bool parse_network_config(const std::string& json, NetworkConfig& out);

/// Lists differences between the sensor configuration and the request.
std::vector<std::string> compare_stream_config(const NetworkConfig& actual,
                                               const StreamConfigRequest& request);

}  // namespace afi920_driver

#endif  // AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_
