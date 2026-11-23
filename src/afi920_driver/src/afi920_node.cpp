// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file afi920_node.cpp
 * @brief ROS2 node for AFI920 4D Imaging Radar
 *
 * Directly handles:
 *   - TCP/UDP stream reception (threaded; TCP by default)
 *   - SOME/IP header parsing and TP reassembly
 *   - RDI / SHII / SPI payload deserialization
 *   - PointCloud2 / DetectionArray / HealthInfo / SensorPerformance publishing
 */

#include "afi920_driver/afi920_node.hpp"
#include "afi920_driver/transport/config_client.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
// Lyrical ships only the .hpp header; Foxy ships only the .h header.
// cppcheck 1.90 (Foxy) cannot evaluate __has_include, hence the suppression.
#ifdef __has_include
// cppcheck-suppress preprocessorErrorDirective
#  if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#    define AFI920_HAS_TF2_QUATERNION_HPP 1
#  endif
#endif
#ifdef AFI920_HAS_TF2_QUATERNION_HPP
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace afi920_driver {

namespace {

// RDI receive timeout (UDP receive timeout, TCP idle timeout). The v1.2.0 scan
// timeout (100 ms) is checked after each receive, so the receive must return
// well within it without traffic.
constexpr int kRdiReceiveTimeoutMs = 20;

uint64_t steady_now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

Afi920Node::Afi920Node(const rclcpp::NodeOptions& options)
    : rclcpp::Node("afi920_driver", options)
{
    declare_parameters();
    read_parameters();

    RCLCPP_INFO(get_logger(), "Sensor: %s, RDI:%d SHI:%d SPI:%d, frame_id: %s",
                sensor_ip_.c_str(), data_port_, shi_port_, spi_port_, frame_id_.c_str());

    // --- Publishers ---
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "PointCloud2", rclcpp::SensorDataQoS());
    if (publish_detections_) {
        pub_rdi_ = this->create_publisher<afi920_msgs::msg::DetectionArray>(
            "RDI", rclcpp::SensorDataQoS());
    }
    if (publish_health_) {
        pub_shi_ = this->create_publisher<afi920_msgs::msg::HealthInfo>(
            "SHI", rclcpp::SensorDataQoS());
    }
    if (publish_performance_) {
        pub_spi_ = this->create_publisher<afi920_msgs::msg::SensorPerformance>(
            "SPI", rclcpp::SensorDataQoS());
    }
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "detection_count", rclcpp::SensorDataQoS());
    pub_e2e_ = this->create_publisher<afi920_msgs::msg::E2eStatus>(
        "E2E", rclcpp::SensorDataQoS());

    // --- TF broadcaster (mounting pose from SPI) ---
    if (publish_tf_) {
        if (publish_performance_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            RCLCPP_INFO(get_logger(),
                "TF broadcast enabled: %s -> %s (from SPI sensor pose)",
                parent_frame_id_.c_str(), frame_id_.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "publish_tf is true but publish_performance is false -- "
                "no SPI sensor pose available, TF will not be broadcast");
        }
    }

    // --- PointCloud builder ---
    PointCloudConfig pc_config;
    pc_config.frame_id = frame_id_;
    pc_config.min_range = min_range_;
    pc_config.max_range = max_range_;
    pc_config.min_snr = min_snr_;
    pc_config.min_existence_probability = min_existence_probability_;
    pc_builder_ = std::make_unique<PointCloudBuilder>(pc_config);
    rdi_scan_emit_ = [this](RdiScan& scan) { on_rdi_scan(scan); };

    // --- Configure sensor stream destination ---
    configure_sensor_streams();

    // --- Create receivers ---
    if (transport_mode_ == "tcp") {
        rdi_receiver_ = std::make_unique<TcpStreamClient>(
            sensor_ip_, static_cast<uint16_t>(data_port_), 512 * 1024, 5000,
            kRdiReceiveTimeoutMs);
        shi_receiver_ = std::make_unique<TcpStreamClient>(
            sensor_ip_, static_cast<uint16_t>(shi_port_), 4 * 1024);
        spi_receiver_ = std::make_unique<TcpStreamClient>(
            sensor_ip_, static_cast<uint16_t>(spi_port_), 4 * 1024);
    } else {
        rdi_receiver_ = std::make_unique<UdpReceiver>(
            static_cast<uint16_t>(data_port_), 16 * 1024 * 1024, kRdiReceiveTimeoutMs);
        shi_receiver_ = std::make_unique<UdpReceiver>(static_cast<uint16_t>(shi_port_));
        spi_receiver_ = std::make_unique<UdpReceiver>(static_cast<uint16_t>(spi_port_));
    }
    // --- Connect receivers ---
    // TCP: a receiver that cannot connect now keeps retrying in its thread with
    // exponential backoff (1 s doubling to 30 s) for as long as the node runs.
    // UDP: a port that cannot be bound is a local error and is not retried.
    const bool tcp = (transport_mode_ == "tcp");
    const bool rdi_ok = connect_receiver(*rdi_receiver_, "RDI", data_port_);
    const bool shi_ok = publish_health_ && connect_receiver(*shi_receiver_, "SHII", shi_port_);
    const bool spi_ok =
        publish_performance_ && connect_receiver(*spi_receiver_, "SPI", spi_port_);
    if (!tcp && !rdi_ok && !shi_ok && !spi_ok) {
        RCLCPP_FATAL(get_logger(), "No UDP receive port could be bound -- shutting down");
        rclcpp::shutdown();
        return;
    }

    // --- Start threads ---
    running_.store(true);
    if (tcp || rdi_ok) {
        rdi_thread_ = std::make_unique<std::thread>(&Afi920Node::rdi_thread_func, this);
    }
    if (publish_health_ && (tcp || shi_ok)) {
        shi_thread_ = std::make_unique<std::thread>(&Afi920Node::shi_thread_func, this);
    }
    if (publish_performance_ && (tcp || spi_ok)) {
        spi_thread_ = std::make_unique<std::thread>(&Afi920Node::spi_thread_func, this);
    }
    // --- Parameter change callback ---
    params_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Afi920Node::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Active -- listening on %s (RDI:%d)",
                sensor_ip_.c_str(), data_port_);
}

Afi920Node::~Afi920Node()
{
    running_.store(false);
    if (rdi_receiver_) rdi_receiver_->Close();
    if (shi_receiver_) shi_receiver_->Close();
    if (spi_receiver_) spi_receiver_->Close();
    if (rdi_thread_ && rdi_thread_->joinable()) rdi_thread_->join();
    if (shi_thread_ && shi_thread_->joinable()) shi_thread_->join();
    if (spi_thread_ && spi_thread_->joinable()) spi_thread_->join();
}

// Parameters

void Afi920Node::declare_parameters()
{
    this->declare_parameter<std::string>("sensor_ip", "192.168.10.150");
    this->declare_parameter<int>("config_port", 30500);
    this->declare_parameter<std::string>("host_ip", "auto");
    this->declare_parameter<int>("data_port", 30509);
    this->declare_parameter<int>("shi_port", 30510);
    this->declare_parameter<int>("spi_port", 30511);
    this->declare_parameter<std::string>("frame_id", "afi920");
    this->declare_parameter<std::string>("parent_frame_id", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("min_range", 0.0);
    this->declare_parameter<double>("max_range", 300.0);
    this->declare_parameter<double>("min_snr", 0.0);
    this->declare_parameter<double>("min_existence_probability", 0.0);
    this->declare_parameter<bool>("publish_detections", true);
    this->declare_parameter<bool>("publish_health", true);
    this->declare_parameter<bool>("publish_performance", true);
    this->declare_parameter<std::string>("transport_mode", "tcp");
    this->declare_parameter<bool>("validate_e2e", true);
    this->declare_parameter<bool>("e2e_strict", false);
}

void Afi920Node::read_parameters()
{
    sensor_ip_                  = this->get_parameter("sensor_ip").as_string();
    config_port_                = this->get_parameter("config_port").as_int();
    host_ip_                    = this->get_parameter("host_ip").as_string();
    data_port_                  = this->get_parameter("data_port").as_int();
    shi_port_                   = this->get_parameter("shi_port").as_int();
    spi_port_                   = this->get_parameter("spi_port").as_int();
    frame_id_                   = this->get_parameter("frame_id").as_string();
    parent_frame_id_            = this->get_parameter("parent_frame_id").as_string();
    publish_tf_                 = this->get_parameter("publish_tf").as_bool();
    min_range_                  = this->get_parameter("min_range").as_double();
    max_range_                  = this->get_parameter("max_range").as_double();
    min_snr_                    = this->get_parameter("min_snr").as_double();
    min_existence_probability_  = this->get_parameter("min_existence_probability").as_double();
    publish_detections_         = this->get_parameter("publish_detections").as_bool();
    publish_health_             = this->get_parameter("publish_health").as_bool();
    publish_performance_        = this->get_parameter("publish_performance").as_bool();
    transport_mode_             = this->get_parameter("transport_mode").as_string();
    validate_e2e_.store(this->get_parameter("validate_e2e").as_bool());
    e2e_strict_.store(this->get_parameter("e2e_strict").as_bool());
}

// Sensor configuration

void Afi920Node::configure_sensor_streams()
{
    std::string host = host_ip_;
    if (host == "auto") {
        host = detect_local_ip(sensor_ip_);
        if (host.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to detect local IP for sensor %s",
                        sensor_ip_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Detected local IP: %s", host.c_str());
        }
    }
    if (host.empty()) {
        return;
    }

    StreamConfigRequest request;
    request.host_ip = host;
    request.data_port = static_cast<uint16_t>(data_port_);
    request.shi_port = static_cast<uint16_t>(shi_port_);
    request.spi_port = static_cast<uint16_t>(spi_port_);
    request.transport_mode = transport_mode_;

    StreamConfigResult result;
    const int rc = sync_stream_config(
        sensor_ip_, static_cast<uint16_t>(config_port_), request, 5000, result);
    if (rc != 0) {
        RCLCPP_WARN(get_logger(),
            "Failed to configure sensor streams (rc=%d) -- "
            "sensor may already be configured or unreachable via TCP:%d",
            rc, config_port_);
        return;
    }

    if (result.format == ConfigApiFormat::kLegacy) {
        RCLCPP_INFO(get_logger(),
            "Sensor configured (Config API v1.0.x): streams -> %s (RDI:%d SHI:%d SPI:%d, transport=%s)",
            host.c_str(), data_port_, shi_port_, spi_port_, transport_mode_.c_str());
        return;
    }

    if (!result.response_crc_ok) {
        RCLCPP_WARN(get_logger(), "GetNetworkConfig response E2E CRC mismatch");
    }
    if (result.mismatches.empty()) {
        RCLCPP_INFO(get_logger(),
            "Sensor stream config matches (Config API v1.1.0+): RDI:%d SHI:%d SPI:%d, transport=%s",
            data_port_, shi_port_, spi_port_, transport_mode_.c_str());
        return;
    }

    // v1.1.0+ applies SetNetworkConfig only in diagnostic mode and after Restart,
    // so the driver reports the difference instead of writing the sensor NVM.
    std::string detail;
    for (const auto& m : result.mismatches) {
        detail += (detail.empty() ? "" : ", ") + m;
    }
    RCLCPP_WARN(get_logger(),
        "Sensor stream config differs from driver parameters (Config API v1.1.0+): %s. "
        "The driver does not change it: SetNetworkConfig(0x0021) requires diagnostic mode "
        "(SetMode 0x00E1) and takes effect after Restart(0x0014).",
        detail.c_str());
}

// Thread functions

void Afi920Node::rdi_thread_func()
{
    RCLCPP_INFO(get_logger(), "RDI thread started (port %d)", data_port_);

    static constexpr size_t kUdpBufSize = 400 * 1024;
    static constexpr size_t kTcpBufSize = 512 * 1024;
    const size_t buf_size = (transport_mode_ == "tcp") ? kTcpBufSize : kUdpBufSize;
    std::vector<uint8_t> buf(buf_size);
    uint32_t recv_count = 0;
    bool was_open = rdi_receiver_->IsOpen();

    while (running_.load()) {
        int n = rdi_receiver_->Receive(buf.data(), buf_size);
        track_connection(*rdi_receiver_, "RDI", data_port_, n, was_open);
        if (n > 0) {
            recv_count++;
            if (recv_count <= 3 || recv_count % 1000 == 0) {
                RCLCPP_INFO(get_logger(), "RDI recv #%u: %d bytes", recv_count, n);
            }
            process_rdi_datagram(buf.data(), static_cast<size_t>(n));
        }
        // Close a v1.2.0 scan whose header B never came (Interface Overview §3.2.3).
        rdi_assembler_.Poll(steady_now_ns(), rdi_scan_emit_);
    }

    RCLCPP_INFO(get_logger(), "RDI thread stopped");
}

void Afi920Node::shi_thread_func()
{
    RCLCPP_INFO(get_logger(), "SHII thread started (port %d)", shi_port_);

    static constexpr size_t kBufSize = 4 * 1024;
    std::vector<uint8_t> buf(kBufSize);
    bool was_open = shi_receiver_->IsOpen();

    while (running_.load()) {
        int n = shi_receiver_->Receive(buf.data(), kBufSize);
        track_connection(*shi_receiver_, "SHII", shi_port_, n, was_open);
        if (n <= 0) continue;
        process_shi_datagram(buf.data(), static_cast<size_t>(n));
    }

    RCLCPP_INFO(get_logger(), "SHII thread stopped");
}

void Afi920Node::spi_thread_func()
{
    RCLCPP_INFO(get_logger(), "SPI thread started (port %d)", spi_port_);

    static constexpr size_t kBufSize = 4 * 1024;
    std::vector<uint8_t> buf(kBufSize);
    bool was_open = spi_receiver_->IsOpen();

    while (running_.load()) {
        int n = spi_receiver_->Receive(buf.data(), kBufSize);
        track_connection(*spi_receiver_, "SPI", spi_port_, n, was_open);
        if (n <= 0) continue;
        process_spi_datagram(buf.data(), static_cast<size_t>(n));
    }

    RCLCPP_INFO(get_logger(), "SPI thread stopped");
}

// Datagram processing

bool Afi920Node::consume_e2e_header(const uint8_t*& payload, size_t& payload_len,
                                    uint32_t expected_data_id, const char* stream_name,
                                    E2eVerdict& verdict)
{
    verdict = E2eVerdict{};
    E2eHeader& e2e = verdict.header;
    if (payload_len < kE2eHeaderSize) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s frame shorter than E2E header: %zu", stream_name, payload_len);
        return false;
    }

    E2eStatus status = E2eStatus::kOk;
    if (validate_e2e_.load(std::memory_order_relaxed)) {
        status = e2e_validate(payload, payload_len, e2e);
        verdict.checked = true;
    } else if (e2e_parse(payload, payload_len, e2e) < 0) {
        status = E2eStatus::kTooShort;
    }

    if (status == E2eStatus::kTooShort) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E parse failed: payload_len=%zu", stream_name, payload_len);
        return false;
    }
    verdict.parsed = true;
    verdict.status = status;

    const size_t iso_len = payload_len - kE2eHeaderSize;

    if (e2e.length != iso_len) {
        verdict.length_ok = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E length mismatch: e2e=%u actual=%zu",
            stream_name, static_cast<unsigned int>(e2e.length), iso_len);
    }

    if (e2e.data_id != expected_data_id) {
        verdict.data_id_ok = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E data_id mismatch: e2e=0x%08X expected=0x%08X",
            stream_name, static_cast<unsigned int>(e2e.data_id),
            static_cast<unsigned int>(expected_data_id));
    }
    const bool metadata_ok = verdict.length_ok && verdict.data_id_ok;

    // CRC results are reported on the E2E topic and in the log; frames are never dropped on CRC.
    if (status == E2eStatus::kCrcMismatch) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E CRC mismatch (counter=%u, data_id=0x%08X)",
            stream_name, static_cast<unsigned int>(e2e.counter),
            static_cast<unsigned int>(e2e.data_id));
    } else if (status == E2eStatus::kCrcZero) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E CRC field is 0, not verified (counter=%u)",
            stream_name, static_cast<unsigned int>(e2e.counter));
    } else if (e2e.format != E2eCrcFormat::kUnknown) {
        const auto format = static_cast<uint8_t>(e2e.format);
        if (e2e_format_.exchange(format, std::memory_order_relaxed) != format) {
            RCLCPP_INFO(get_logger(), "%s E2E format detected: %s",
                        stream_name, e2e_crc_format_name(e2e.format));
        }
    }

    if (!metadata_ok && e2e_strict_.load(std::memory_order_relaxed)) {
        return false;
    }

    payload += kE2eHeaderSize;
    payload_len = iso_len;
    return true;
}

void Afi920Node::track_e2e_counter(E2eCounterTracker& tracker, const char* stream_name,
                                   E2eVerdict& verdict)
{
    // MessageCounter carries the scan index on v1.1.0+ (repeated on resend,
    // skipped on late seals), so message loss is judged by the E2E Counter.
    const uint32_t counter = verdict.header.counter;
    verdict.counter = tracker.Update(counter);
    if (verdict.counter.restarted) {
        RCLCPP_WARN(get_logger(), "%s E2E Counter decreased to %u -- sensor restarted",
                    stream_name, static_cast<unsigned int>(counter));
    } else if (verdict.counter.missing > 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "%s E2E Counter gap: %u message(s) missing before counter=%u",
            stream_name, static_cast<unsigned int>(verdict.counter.missing),
            static_cast<unsigned int>(counter));
    }
}

void Afi920Node::publish_e2e_status(uint8_t stream, const E2eVerdict& verdict,
                                    const rclcpp::Time& stamp, bool data_published,
                                    uint32_t message_counter, uint8_t data_qualifier)
{
    using afi920_msgs::msg::E2eStatus;
    if (!pub_e2e_ || !verdict.parsed) return;

    E2eStatus st;
    st.header.stamp = stamp;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        st.header.frame_id = frame_id_;
    }
    st.stream = stream;
    if (!verdict.checked) {
        st.result = E2eStatus::RESULT_NOT_CHECKED;
    } else if (verdict.status == afi920_driver::E2eStatus::kCrcMismatch) {
        st.result = E2eStatus::RESULT_CRC_MISMATCH;
    } else if (verdict.status == afi920_driver::E2eStatus::kCrcZero) {
        st.result = E2eStatus::RESULT_CRC_ZERO;
    } else {
        st.result = E2eStatus::RESULT_OK;
    }
    switch (verdict.header.format) {
        case E2eCrcFormat::kCrc32:
            st.crc_format = E2eStatus::CRC_FORMAT_CRC32;
            break;
        case E2eCrcFormat::kCrc64Xz:
            st.crc_format = E2eStatus::CRC_FORMAT_CRC64_XZ;
            break;
        default:
            st.crc_format = E2eStatus::CRC_FORMAT_UNKNOWN;
            break;
    }
    st.length_ok = verdict.length_ok;
    st.data_id_ok = verdict.data_id_ok;
    st.e2e_counter = verdict.header.counter;
    st.missing_before = verdict.counter.missing;
    st.sensor_restarted = verdict.counter.restarted;
    st.data_published = data_published;
    st.message_counter = message_counter;
    st.data_qualifier = data_qualifier;
    pub_e2e_->publish(std::move(st));
}

void Afi920Node::count_rdi_loss(const E2eVerdict& verdict)
{
    const uint32_t total = total_packets_.fetch_add(1) + 1;
    const uint32_t missing = verdict.counter.missing;
    if (missing > 0) {
        const uint32_t lost = lost_packets_.fetch_add(missing) + missing;
        const float loss_pct = 100.0f * static_cast<float>(lost) /
                               static_cast<float>(static_cast<uint64_t>(total) + lost);
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "Packet loss: %u lost, %u received (%.1f%%)", lost, total, loss_pct);
    }
}

void Afi920Node::deliver_rdi_frame(RdiFrame& frame, const E2eVerdict& verdict)
{
    count_rdi_loss(verdict);
    on_rdi_frame(frame, verdict);
}

void Afi920Node::process_rdi_chunked(const uint8_t* data, size_t len, const SomeipHeader& hdr)
{
    using afi920_msgs::msg::E2eStatus;
    if (!rdi_chunked_seen_) {
        rdi_chunked_seen_ = true;
        RCLCPP_INFO(get_logger(),
            "RDI framing detected: interface v1.2.0 header/chunk messages");
    }

    // Each header and chunk message carries its own E2E header, whose Data ID
    // is 0x6000 << 16 | Event ID (0x60008002 or 0x60008100 + k).
    const uint8_t* payload = data + kSomeipHeaderSize;
    size_t payload_len = len - kSomeipHeaderSize;
    E2eVerdict verdict;
    if (!consume_e2e_header(payload, payload_len, e2e_data_id(hdr.method_id), "RDI", verdict)) {
        // A message dropped by e2e_strict is reported on its own.
        publish_e2e_status(E2eStatus::STREAM_RDI, verdict, this->now(), false, 0, 0);
        return;
    }

    RdiMessageE2e e2e;
    e2e.status = verdict.status;
    e2e.format = verdict.header.format;
    e2e.length_ok = verdict.length_ok;
    e2e.data_id_ok = verdict.data_id_ok;
    const uint32_t counter = verdict.header.counter;
    const uint64_t now_ns = steady_now_ns();
    if (hdr.method_id == kEventRdi) {
        rdi_assembler_.OnHeader(counter, e2e, payload, payload_len, hdr.session_id,
                                now_ns, rdi_scan_emit_);
    } else {
        rdi_assembler_.OnChunk(counter, e2e,
                               static_cast<uint16_t>(hdr.method_id - kEventRdiChunkBase),
                               payload, payload_len, now_ns, rdi_scan_emit_);
    }
}

void Afi920Node::on_rdi_scan(RdiScan& scan)
{
    using afi920_msgs::msg::E2eStatus;
    E2eVerdict verdict;
    verdict.header.counter = scan.e2e_counter;
    verdict.header.format = scan.e2e.format;
    verdict.status = scan.e2e.status;
    verdict.parsed = true;
    verdict.checked = validate_e2e_.load(std::memory_order_relaxed);
    verdict.length_ok = scan.e2e.length_ok;
    verdict.data_id_ok = scan.e2e.data_id_ok;
    // All messages of a scan share one E2E Counter, so loss is counted per scan.
    track_e2e_counter(rdi_counter_, "RDI", verdict);

    if (scan.chunks_rejected > 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "RDI scan (E2E counter %u): %u chunk(s) rejected (size, MessageCounter, "
            "chunk index or slot not below ChunkCount, FirstEntityIndex not a multiple "
            "of EntitiesPerChunk, EntityCount above EntitiesPerChunk, or duplicate slot)",
            static_cast<unsigned int>(scan.e2e_counter),
            static_cast<unsigned int>(scan.chunks_rejected));
    }
    if (!scan.has_header) {
        count_rdi_loss(verdict);
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "RDI scan (E2E counter %u) dropped: no header message received",
            static_cast<unsigned int>(scan.e2e_counter));
        publish_e2e_status(E2eStatus::STREAM_RDI, verdict, this->now(), false, 0, 0);
        return;
    }
    if (!scan.IsComplete()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "RDI scan (E2E counter %u) partial: %u of %u chunks, %u of %u detections",
            static_cast<unsigned int>(scan.e2e_counter),
            static_cast<unsigned int>(scan.chunks_received),
            static_cast<unsigned int>(scan.chunk_count),
            static_cast<unsigned int>(scan.frame.message.num_detections),
            static_cast<unsigned int>(scan.announced_detections));
    }

    RdiFrame& frame = scan.frame;
    frame.recv_timestamp_ns = steady_now_ns();
    frame.frame_seq = rdi_frame_seq_.fetch_add(1);
    deliver_rdi_frame(frame, verdict);
}

void Afi920Node::process_rdi_datagram(const uint8_t* data, size_t len)
{
    rdi_diag_count_++;
    bool diag = (rdi_diag_count_ <= 5 || rdi_diag_count_ % 200 == 0);

    if (len < kSomeipHeaderSize) {
        if (diag) RCLCPP_WARN(get_logger(), "[DIAG #%u] Packet too small: %zu < %zu",
                               rdi_diag_count_, len, kSomeipHeaderSize);
        return;
    }

    SomeipHeader hdr;
    if (someip_deserialize(data, len, hdr) < 0) {
        if (diag) RCLCPP_WARN(get_logger(), "[DIAG #%u] SOME/IP deserialize failed (len=%zu)",
                               rdi_diag_count_, len);
        return;
    }

    if (!someip_validate(hdr)) { return; }

    // Interface v1.2.0: header A, chunks and header B, each a single datagram.
    if (hdr.interface_version == kSomeipInterfaceVersionRdiChunked) {
        if (someip_is_tp(hdr.message_type) ||
            (hdr.method_id != kEventRdi && !someip_is_rdi_chunk_event(hdr.method_id))) {
            if (rdi_diag_count_ <= 5) {
                RCLCPP_WARN(get_logger(),
                    "[DIAG #%u] Ignoring RDI message: interface_version=0x02 method=0x%04X type=0x%02X",
                    rdi_diag_count_, hdr.method_id, hdr.message_type);
            }
            return;
        }
        process_rdi_chunked(data, len, hdr);
        return;
    }

    // Interface v1.0.x and v1.1.0: one Event 0x8002 message per scan, split
    // by SOME/IP-TP over UDP.
    if (hdr.method_id != kEventRdi) {
        if (diag) {
            RCLCPP_WARN(get_logger(),
                "[DIAG #%u] Ignoring non-RDI event on RDI receiver: method=0x%04X",
                rdi_diag_count_, hdr.method_id);
        }
        return;
    }

    if (diag) {
        RCLCPP_INFO(get_logger(),
            "[DIAG #%u] SOMEIP: svc=0x%04X method=0x%04X len=%u type=0x%02X rc=%u",
            rdi_diag_count_, hdr.service_id, hdr.method_id, hdr.length,
            hdr.message_type, hdr.return_code);
    }

    bool is_tp = someip_is_tp(hdr.message_type);
    size_t offset = kSomeipHeaderSize;

    const uint8_t* payload;
    size_t payload_len;

    if (is_tp) {
        if (len < kSomeipHeaderSize + kSomeipTpHeaderSize) return;

        SomeipTpHeader tp;
        someip_tp_deserialize(data + offset, tp);
        offset += kSomeipTpHeaderSize;

        size_t seg_len = len - offset;
        bool complete = rdi_tp_.Feed(data + offset, seg_len, tp.offset_bytes, tp.more_segments, hdr.session_id);

        if (diag) {
            RCLCPP_INFO(get_logger(),
                "[DIAG #%u] TP seg: offset=%u more=%d seg_len=%zu complete=%d",
                rdi_diag_count_, tp.offset_bytes, tp.more_segments, seg_len, complete);
        }

        if (!complete) return;

        payload = rdi_tp_.GetPayload();
        payload_len = rdi_tp_.GetPayloadSize();

        if (diag) {
            RCLCPP_INFO(get_logger(),
                "[DIAG #%u] TP reassembled: payload_len=%zu", rdi_diag_count_, payload_len);
        }
    } else {
        payload = data + offset;
        payload_len = len - offset;
        if (diag) {
            RCLCPP_INFO(get_logger(),
                "[DIAG #%u] Non-TP packet: payload_len=%zu", rdi_diag_count_, payload_len);
        }
    }

    using afi920_msgs::msg::E2eStatus;
    E2eVerdict verdict;
    const bool keep = consume_e2e_header(payload, payload_len, kE2eDataIdRdi, "RDI", verdict);
    if (verdict.parsed) {
        track_e2e_counter(rdi_counter_, "RDI", verdict);
    }
    if (!keep) {
        publish_e2e_status(E2eStatus::STREAM_RDI, verdict, this->now(), false, 0, 0);
        if (is_tp) rdi_tp_.Reset();
        return;
    }
    const E2eHeader& e2e = verdict.header;
    if (diag) {
        RCLCPP_INFO(get_logger(),
            "[DIAG #%u] E2E stripped: crc=0x%016" PRIX64 " len=%u counter=%u data_id=0x%08X iso_len=%zu",
            rdi_diag_count_, static_cast<uint64_t>(e2e.crc),
            static_cast<unsigned int>(e2e.length),
            static_cast<unsigned int>(e2e.counter),
            static_cast<unsigned int>(e2e.data_id), payload_len);
    }

    RdiFrame frame{};
    frame.recv_timestamp_ns = steady_now_ns();
    frame.frame_seq = rdi_frame_seq_.fetch_add(1);

    int parse_rc = rdi_parser_.Parse(payload, payload_len, frame);
    if (parse_rc == 0) {
        if (diag) {
            RCLCPP_INFO(get_logger(),
                "[DIAG #%u] RDI parsed OK: num_det=%u counter=%u sensor_id=%u",
                rdi_diag_count_, frame.message.num_detections,
                frame.message.message_counter, frame.message.sensor_id);
        }
        deliver_rdi_frame(frame, verdict);
    } else {
        if (diag) {
            RCLCPP_WARN(get_logger(),
                "[DIAG #%u] RDI parse FAILED (rc=%d, payload_len=%zu)",
                rdi_diag_count_, parse_rc, payload_len);
        }
        publish_e2e_status(E2eStatus::STREAM_RDI, verdict, this->now(), false, 0, 0);
    }

    if (is_tp) rdi_tp_.Reset();
}

void Afi920Node::process_shi_datagram(const uint8_t* data, size_t len)
{
    if (len < kSomeipHeaderSize) return;

    SomeipHeader hdr;
    if (someip_deserialize(data, len, hdr) < 0) return;
    if (!someip_validate(hdr)) { return; }
    if (hdr.method_id != kEventShii) return;

    using afi920_msgs::msg::E2eStatus;
    const uint8_t* payload = data + kSomeipHeaderSize;
    size_t payload_len = len - kSomeipHeaderSize;
    E2eVerdict verdict;
    const bool keep = consume_e2e_header(payload, payload_len, kE2eDataIdShii, "SHII", verdict);
    if (verdict.parsed) {
        track_e2e_counter(shi_counter_, "SHII", verdict);
    }

    ShiMessage msg{};
    if (keep && shi_parser_.Parse(payload, payload_len, msg) == 0) {
        on_shi_message(msg, verdict);
    } else {
        publish_e2e_status(E2eStatus::STREAM_SHI, verdict, this->now(), false, 0, 0);
    }
}

void Afi920Node::process_spi_datagram(const uint8_t* data, size_t len)
{
    if (len < kSomeipHeaderSize) return;

    SomeipHeader hdr;
    if (someip_deserialize(data, len, hdr) < 0) return;
    if (!someip_validate(hdr)) { return; }

    if (hdr.method_id != kEventSpi) return;

    using afi920_msgs::msg::E2eStatus;
    const uint8_t* payload = data + kSomeipHeaderSize;
    size_t payload_len = len - kSomeipHeaderSize;
    E2eVerdict verdict;
    const bool keep = consume_e2e_header(payload, payload_len, kE2eDataIdSpi, "SPI", verdict);
    if (verdict.parsed) {
        track_e2e_counter(spi_counter_, "SPI", verdict);
    }

    SpiMessage msg{};
    if (keep && spi_parser_.Parse(payload, payload_len, msg) == 0) {
        on_spi_message(msg, verdict);
    } else {
        publish_e2e_status(E2eStatus::STREAM_SPI, verdict, this->now(), false, 0, 0);
    }
}

// Data callbacks

void Afi920Node::on_rdi_frame(const RdiFrame& frame, const E2eVerdict& verdict)
{
    uint64_t fc = rdi_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    if (!pub_pointcloud_) return;

    // DQ 1 (not this scan's data) and DQ 6 (resend of the previous scan) are not
    // new measurements. They reach DetectionArray only, with data_qualifier set.
    const auto dq = frame.message.data_qualifier;
    const bool publish_cloud = (dq != BTS_DQ_NOT_AVAILABLE && dq != BTS_DQ_INVALID);
    if (!publish_cloud) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
            "RDI DataQualifier=%u frame excluded from PointCloud2 (message_counter=%u)",
            static_cast<unsigned int>(dq), frame.message.message_counter);
    }

    auto stamp = this->now();

    // Build messages under lock (protects against concurrent config updates)
    sensor_msgs::msg::PointCloud2 cloud;
    afi920_msgs::msg::DetectionArray arr;
    bool build_arr = publish_detections_.load(std::memory_order_relaxed) && pub_rdi_;

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        if (publish_cloud) {
            cloud = pc_builder_->build(frame, stamp);
        }
        if (build_arr) {
            arr = pc_builder_->build_detection_array(frame, stamp);
        }
    }

    if (build_arr) {
        arr.timestamp_measurement_ns = frame.message.timestamp;
    }

    if (publish_cloud) {
        pub_pointcloud_->publish(std::move(cloud));
    }
    if (build_arr) {
        pub_rdi_->publish(std::move(arr));
    }
    publish_e2e_status(afi920_msgs::msg::E2eStatus::STREAM_RDI, verdict, stamp,
                       publish_cloud || build_arr, frame.message.message_counter,
                       static_cast<uint8_t>(dq));

    // Publish detection count as text marker for rviz overlay
    if (publish_cloud && pub_marker_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = "detection_count";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 3.0;
        marker.scale.z = 1.5;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.text = "Pts: " + std::to_string(frame.message.num_detections);
        marker.lifetime = rclcpp::Duration(0, 200'000'000);  // 200ms
        pub_marker_->publish(std::move(marker));
    }

    if (fc <= 3 || fc % 100 == 0) {
        RCLCPP_INFO(get_logger(),
            "[DIAG] Published RDI frame #%lu: %u detections, arr=%d",
            fc, frame.message.num_detections, build_arr);
    }
}

void Afi920Node::on_shi_message(const ShiMessage& msg, const E2eVerdict& verdict)
{
    if (!pub_shi_) return;

    const rclcpp::Time stamp = this->now();
    afi920_msgs::msg::HealthInfo hi;
    hi.header.stamp = stamp;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        hi.header.frame_id = frame_id_;
    }

    hi.sensor_id = msg.sensor_id;
    hi.message_counter = msg.message_counter;
    hi.data_qualifier = static_cast<uint8_t>(msg.data_qualifier);

    // Operation modes
    hi.num_operation_modes = msg.num_valid_operation_modes;
    for (uint8_t i = 0; i < BTS_SHI_MAX_OPERATION_MODES; ++i) {
        hi.operation_modes[i] = msg.sensor_operation_modes[i];
    }

    // Defect status
    hi.defect_recognised = static_cast<uint8_t>(msg.sensor_defect_recognised);
    hi.defect_reason = static_cast<uint8_t>(msg.sensor_defect_reason);

    // Environmental status
    hi.supply_voltage_status = static_cast<uint8_t>(msg.supply_voltage_status);
    hi.temperature_status = static_cast<uint8_t>(msg.sensor_temperature_status);

    // Input signal status
    hi.num_input_signal_statuses = msg.num_valid_input_signal_statuses;
    for (uint8_t i = 0; i < BTS_SHI_MAX_INPUT_SIGNALS; ++i) {
        hi.input_signal_types[i] = msg.sensor_input_signal_types[i];
        hi.input_signal_statuses[i] = msg.sensor_input_signal_statuses[i];
    }

    // Time synchronization
    hi.time_sync_status = static_cast<uint8_t>(msg.sensor_time_sync);
    hi.time_sync_offset = msg.sensor_time_sync_offset_value;

    // Calibration
    hi.num_calibration_components = msg.num_valid_calibration_components;
    for (uint8_t i = 0; i < BTS_SHI_MAX_CALIBRATION_COMPONENTS; ++i) {
        hi.calibration_components[i] = msg.sensor_calibration_components[i];
        hi.calibration_statuses[i] = msg.sensor_calibration_statuses[i];
        hi.calibration_states[i] = msg.sensor_calibration_states[i];
    }

    hi.timestamp_measurement_ns = msg.timestamp;

    pub_shi_->publish(std::move(hi));
    publish_e2e_status(afi920_msgs::msg::E2eStatus::STREAM_SHI, verdict, stamp, true,
                       msg.message_counter, static_cast<uint8_t>(msg.data_qualifier));
}

void Afi920Node::on_spi_message(const SpiMessage& msg, const E2eVerdict& verdict)
{
    if (!pub_spi_) return;

    const rclcpp::Time stamp = this->now();
    afi920_msgs::msg::SensorPerformance sp;
    sp.header.stamp = stamp;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        sp.header.frame_id = frame_id_;
    }

    sp.sensor_id = msg.sensor_id;
    sp.message_counter = msg.message_counter;
    sp.data_qualifier = static_cast<uint8_t>(msg.data_qualifier);

    sp.timestamp_measurement_ns = msg.timestamp;

    // Vehicle coordinate system
    sp.vehicle_coordinate_system = static_cast<uint8_t>(msg.vehicle_coordinate_system);

    // Sensor pose
    const auto& pose = msg.sensor_pose;
    sp.sensor_pose.origin_x = pose.origin_point_x;
    sp.sensor_pose.origin_y = pose.origin_point_y;
    sp.sensor_pose.origin_z = pose.origin_point_z;
    sp.sensor_pose.origin_error_x = pose.origin_point_error_x;
    sp.sensor_pose.origin_error_y = pose.origin_point_error_y;
    sp.sensor_pose.origin_error_z = pose.origin_point_error_z;
    sp.sensor_pose.yaw = pose.orientation_yaw;
    sp.sensor_pose.pitch = pose.orientation_pitch;
    sp.sensor_pose.roll = pose.orientation_roll;
    sp.sensor_pose.yaw_error = pose.orientation_error_yaw;
    sp.sensor_pose.pitch_error = pose.orientation_error_pitch;
    sp.sensor_pose.roll_error = pose.orientation_error_roll;

    // Clamp array counts to prevent out-of-bounds access
    const uint8_t n_fov = std::min(msg.num_fov_segments, static_cast<uint8_t>(BTS_SPI_MAX_FOV_SEGMENTS));
    const uint8_t n_obj = std::min(msg.num_recognisable_object_types, static_cast<uint8_t>(BTS_SPI_MAX_OBJECT_TYPES));
    const uint8_t n_ref = std::min(msg.num_reference_target_types, static_cast<uint8_t>(BTS_SPI_MAX_REF_TARGETS));

    // FOV segments
    sp.fov_segments.resize(n_fov);
    for (uint8_t i = 0; i < n_fov; ++i) {
        auto& dst = sp.fov_segments[i];
        const auto& src = msg.fov_segments[i];
        dst.azimuth_begin = src.azimuth_begin;
        dst.azimuth_end = src.azimuth_end;
        dst.elevation_begin = src.elevation_begin;
        dst.elevation_end = src.elevation_end;
        dst.blockage_status = static_cast<uint8_t>(src.blockage_status);
    }

    // Recognisable objects
    sp.recognisable_objects.resize(n_obj);
    for (uint8_t i = 0; i < n_obj; ++i) {
        auto& dst = sp.recognisable_objects[i];
        const auto& src = msg.recognisable_object_types[i];
        dst.object_type = static_cast<uint8_t>(src.object_type);
        dst.detection_range_begin = src.detection_range_begin;
        dst.detection_range_end = src.detection_range_end;
    }

    // Reference targets
    sp.reference_targets.resize(n_ref);
    for (uint8_t i = 0; i < n_ref; ++i) {
        auto& dst = sp.reference_targets[i];
        const auto& src = msg.reference_target_types[i];
        dst.radar_cross_section = src.radar_cross_section;
        dst.detection_range_begin = src.detection_range_begin;
        dst.detection_range_end = src.detection_range_end;
        dst.signal_to_noise_ratio = src.signal_to_noise_ratio;
    }

    // Broadcast mounting pose as TF: parent_frame -> frame_id.
    // Lets RViz/consumers place each sensor's cloud at its calibrated position
    // when the fixed frame is set to parent_frame_id (e.g. base_link).
    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent_frame_id_;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            tf.child_frame_id = frame_id_;
        }
        tf.transform.translation.x = pose.origin_point_x;
        tf.transform.translation.y = pose.origin_point_y;
        tf.transform.translation.z = pose.origin_point_z;

        tf2::Quaternion q;
        q.setRPY(pose.orientation_roll, pose.orientation_pitch, pose.orientation_yaw);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    pub_spi_->publish(std::move(sp));
    publish_e2e_status(afi920_msgs::msg::E2eStatus::STREAM_SPI, verdict, stamp, true,
                       msg.message_counter, static_cast<uint8_t>(msg.data_qualifier));
}

// Connection state

bool Afi920Node::connect_receiver(StreamReceiver& rx, const char* name, int port)
{
    if (rx.Connect() >= 0) {
        if (transport_mode_ == "tcp") {
            RCLCPP_INFO(get_logger(), "%s connected to %s:%d", name, sensor_ip_.c_str(), port);
        }
        return true;
    }
    if (transport_mode_ == "tcp") {
        RCLCPP_WARN(get_logger(),
            "%s connect to %s:%d failed -- retrying every 1 s to 30 s until it connects",
            name, sensor_ip_.c_str(), port);
    } else {
        RCLCPP_ERROR(get_logger(), "%s: cannot bind UDP port %d -- stream disabled", name, port);
    }
    return false;
}

void Afi920Node::track_connection(StreamReceiver& rx, const char* name, int port,
                                  int n, bool& was_open)
{
    const bool open = rx.IsOpen();
    if (open && !was_open) {
        RCLCPP_INFO(get_logger(), "%s connected to %s:%d", name, sensor_ip_.c_str(), port);
    } else if (!open && was_open && running_.load()) {
        // The destructor clears running_ before Close(), so shutdown is not logged as a loss.
        RCLCPP_WARN(get_logger(), "%s connection to %s:%d lost -- reconnecting",
                    name, sensor_ip_.c_str(), port);
    } else if (!open && n < 0 && running_.load()) {
        // One line per failed attempt; attempts are spaced by the backoff.
        const auto* tcp = dynamic_cast<const TcpStreamClient*>(&rx);
        if (tcp) {
            RCLCPP_WARN(get_logger(), "%s reconnect to %s:%d failed -- next attempt in %d s",
                        name, sensor_ip_.c_str(), port, tcp->NextReconnectDelayMs() / 1000);
        }
    }
    was_open = open;
}

// Dynamic parameters

rcl_interfaces::msg::SetParametersResult Afi920Node::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Only these parameters take effect at runtime. Any other parameter in the
    // request rejects the whole request, so nothing is applied half-way.
    static const char* const kDynamic[] = {
        "frame_id", "min_range", "max_range", "min_snr",
        "min_existence_probability", "validate_e2e", "e2e_strict"};
    for (const auto& param : params) {
        bool dynamic = false;
        for (const char* name : kDynamic) {
            if (param.get_name() == name) {
                dynamic = true;
                break;
            }
        }
        if (!dynamic) {
            result.successful = false;
            result.reason = param.get_name() + " cannot be changed at runtime; restart the node";
            return result;
        }
    }

    bool config_changed = false;

    // Collect new values first, then apply under lock
    std::string new_frame_id = frame_id_;
    double new_min_range = min_range_;
    double new_max_range = max_range_;
    double new_min_snr = min_snr_;
    double new_min_ep = min_existence_probability_;

    for (const auto& param : params) {
        if (param.get_name() == "frame_id") {
            new_frame_id = param.as_string();
            config_changed = true;
        } else if (param.get_name() == "min_range") {
            new_min_range = param.as_double();
            config_changed = true;
        } else if (param.get_name() == "max_range") {
            new_max_range = param.as_double();
            config_changed = true;
        } else if (param.get_name() == "min_snr") {
            new_min_snr = param.as_double();
            config_changed = true;
        } else if (param.get_name() == "min_existence_probability") {
            new_min_ep = param.as_double();
            config_changed = true;
        } else if (param.get_name() == "validate_e2e") {
            validate_e2e_.store(param.as_bool(), std::memory_order_relaxed);
        } else if (param.get_name() == "e2e_strict") {
            e2e_strict_.store(param.as_bool(), std::memory_order_relaxed);
        }
    }

    if (config_changed && pc_builder_) {
        PointCloudConfig cfg;
        cfg.frame_id = new_frame_id;
        cfg.min_range = new_min_range;
        cfg.max_range = new_max_range;
        cfg.min_snr = new_min_snr;
        cfg.min_existence_probability = new_min_ep;

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            frame_id_ = new_frame_id;
            min_range_ = new_min_range;
            max_range_ = new_max_range;
            min_snr_ = new_min_snr;
            min_existence_probability_ = new_min_ep;
            pc_builder_->update_config(cfg);
        }

        RCLCPP_INFO(get_logger(),
            "Config updated: frame=%s, range=[%.1f, %.1f], snr>=%.1f, existence>=%.1f",
            new_frame_id.c_str(), new_min_range, new_max_range, new_min_snr, new_min_ep);
    }

    return result;
}

}  // namespace afi920_driver

RCLCPP_COMPONENTS_REGISTER_NODE(afi920_driver::Afi920Node)
