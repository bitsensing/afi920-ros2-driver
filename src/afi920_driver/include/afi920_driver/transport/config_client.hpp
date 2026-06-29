// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file config_client.hpp
 * @brief One-shot TCP client to configure AFI920 sensor data streams
 *
 * Sends a SOME/IP SetNetworkConfig (method 0x0021) partial update to the
 * sensor's Config API, telling it where to send RDI/SHII/SPI streams.
 */

#ifndef AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_
#define AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_

#include <cstdint>
#include <string>

namespace afi920_driver {

/**
 * @brief Detect the local IP address that routes to the given sensor IP.
 * @param sensor_ip  Sensor IP address (e.g. "192.168.10.150")
 * @return Local interface IP, or empty string on failure.
 */
std::string detect_local_ip(const std::string& sensor_ip);

/**
 * @brief Configure sensor to stream data to this host (one-shot TCP call).
 *
 * Connects to sensor_ip:config_port via TCP, sends a SOME/IP
 * SetNetworkConfig partial update with the given host IP and ports,
 * waits for the response, and disconnects.
 *
 * @param sensor_ip      Sensor IP address
 * @param config_port    Sensor Config API TCP port (default 30500)
 * @param host_ip        Host IP to receive streams
 * @param data_port      RDI detection port
 * @param shi_port       Health info port
 * @param spi_port       Sensor performance port
 * @param timeout_ms     TCP connect/recv timeout in milliseconds
 * @param transport_mode Transport protocol: "tcp" or "udp"
 * @return 0 on success, negative on error
 */
int configure_streams(
    const std::string& sensor_ip,
    uint16_t config_port,
    const std::string& host_ip,
    uint16_t data_port,
    uint16_t shi_port,
    uint16_t spi_port,
    int timeout_ms = 5000,
    const std::string& transport_mode = "tcp");

}  // namespace afi920_driver

#endif  // AFI920_DRIVER__TRANSPORT__CONFIG_CLIENT_HPP_
