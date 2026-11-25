// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file afi920_node.hpp
 * @brief ROS2 node for AFI920 4D Imaging Radar
 *
 * Handles TCP/UDP stream reception, SOME/IP parsing, and payload
 * deserialization directly - no external SDK dependency. Interface v1.0.x, v1.1.0 and
 * v1.2.0 sensors are supported; the E2E CRC, the RDI framing (single message
 * or v1.2.0 header/chunk messages) and the Config API format are detected
 * from the received frames.
 *
 * Published topics (all relative, namespace-prefixed):
 *   <ns>/PointCloud2  sensor_msgs/PointCloud2       (18 FLOAT32 fields)
 *   <ns>/PointCloud2XYZI sensor_msgs/PointCloud2    (x/y/z/intensity, 16 B/point;
 *                                                    publish_pointcloud_xyzi only)
 *   <ns>/RDI          afi920_msgs/DetectionArray     (full detection data)
 *   <ns>/SHI          afi920_msgs/HealthInfo         (sensor health)
 *   <ns>/SPI          afi920_msgs/SensorPerformance  (sensor performance)
 *   <ns>/E2E          afi920_msgs/E2eStatus          (E2E verdict per RDI/SHI/SPI message)
 */

#ifndef AFI920_DRIVER__AFI920_NODE_HPP_
#define AFI920_DRIVER__AFI920_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
// Lyrical ships only the .hpp header; Foxy ships only the .h header.
// cppcheck 1.90 (Foxy) cannot evaluate __has_include, hence the suppression.
#ifdef __has_include
// cppcheck-suppress preprocessorErrorDirective
#  if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#    define AFI920_HAS_TF2_ROS_BROADCASTER_HPP 1
#  endif
#endif
#ifdef AFI920_HAS_TF2_ROS_BROADCASTER_HPP
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

#include <afi920_msgs/msg/detection_array.hpp>
#include <afi920_msgs/msg/e2e_status.hpp>
#include <afi920_msgs/msg/health_info.hpp>
#include <afi920_msgs/msg/sensor_performance.hpp>

#include "afi920_driver/radar_types.hpp"
#include "afi920_driver/pointcloud_builder.hpp"
#include "afi920_driver/transport/stream_receiver.hpp"
#include "afi920_driver/transport/tcp_stream_client.hpp"
#include "afi920_driver/transport/udp_receiver.hpp"
#include "afi920_driver/someip/e2e.hpp"
#include "afi920_driver/someip/someip_parser.hpp"
#include "afi920_driver/parsers/rdi_assembler.hpp"
#include "afi920_driver/parsers/rdi_parser.hpp"
#include "afi920_driver/parsers/shi_parser.hpp"
#include "afi920_driver/parsers/spi_parser.hpp"

namespace afi920_driver {

/// E2E verdict of one received message, published on the E2E topic.
struct E2eVerdict {
    E2eHeader header;
    E2eStatus status = E2eStatus::kOk;
    bool parsed = false;     ///< E2E header present (payload >= 20 bytes)
    bool checked = false;    ///< CRC verified (validate_e2e)
    bool length_ok = true;
    bool data_id_ok = true;
    E2eCounterUpdate counter;
};

class Afi920Node : public rclcpp::Node {
public:
    explicit Afi920Node(const rclcpp::NodeOptions& options);
    ~Afi920Node() override;

private:
    // --- Parameters (set in constructor, read-only after) ---
    std::string sensor_ip_;
    int config_port_;
    std::string host_ip_;
    int data_port_;
    int shi_port_;
    int spi_port_;
    std::string frame_id_;
    std::string parent_frame_id_;
    bool publish_tf_;
    double min_range_;
    double max_range_;
    double min_snr_;
    double min_existence_probability_;
    std::atomic<bool> publish_detections_{true};
    bool publish_health_;
    bool publish_performance_;
    bool publish_pointcloud_xyzi_;
    uint32_t xyzi_pad_multiple_;
    std::string transport_mode_;
    std::atomic<bool> validate_e2e_{true};
    std::atomic<bool> e2e_strict_{false};

    // --- Transport ---
    std::unique_ptr<StreamReceiver> rdi_receiver_;
    std::unique_ptr<StreamReceiver> shi_receiver_;
    std::unique_ptr<StreamReceiver> spi_receiver_;

    // --- Protocol ---
    SomeipTpReassembler rdi_tp_;
    RdiScanAssembler rdi_assembler_;   // v1.2.0 header/chunk messages, RDI thread only
    RdiScanAssembler::Emit rdi_scan_emit_;
    RdiParser rdi_parser_;
    ShiParser shi_parser_;
    SpiParser spi_parser_;

    // --- Threads ---
    std::unique_ptr<std::thread> rdi_thread_;
    std::unique_ptr<std::thread> shi_thread_;
    std::unique_ptr<std::thread> spi_thread_;
    std::atomic<bool> running_{false};

    // --- ROS2 Components ---
    std::unique_ptr<PointCloudBuilder> pc_builder_;
    mutable std::mutex config_mutex_;  // protects pc_builder_ config updates vs SDK reads

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_xyzi_;
    rclcpp::Publisher<afi920_msgs::msg::DetectionArray>::SharedPtr pub_rdi_;
    rclcpp::Publisher<afi920_msgs::msg::HealthInfo>::SharedPtr pub_shi_;
    rclcpp::Publisher<afi920_msgs::msg::SensorPerformance>::SharedPtr pub_spi_;
    rclcpp::Publisher<afi920_msgs::msg::E2eStatus>::SharedPtr pub_e2e_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    // --- TF: broadcasts parent_frame -> frame_id using SPI sensor mounting pose ---
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Statistics ---
    std::atomic<uint64_t> rdi_frame_count_{0};
    std::atomic<uint32_t> rdi_frame_seq_{0};
    std::atomic<uint32_t> total_packets_{0};
    std::atomic<uint32_t> lost_packets_{0};
    // E2E Counter trackers, each touched only by its receiver thread.
    E2eCounterTracker rdi_counter_;
    E2eCounterTracker shi_counter_;
    E2eCounterTracker spi_counter_;

    // --- Diagnostics ---
    uint32_t rdi_diag_count_{0};
    bool rdi_chunked_seen_{false};     // RDI thread only
    std::atomic<uint8_t> e2e_format_{static_cast<uint8_t>(E2eCrcFormat::kUnknown)};

    // --- Thread Functions ---
    void rdi_thread_func();
    void shi_thread_func();
    void spi_thread_func();

    // --- Datagram Processing ---
    void process_rdi_datagram(const uint8_t* data, size_t len);
    void process_shi_datagram(const uint8_t* data, size_t len);
    void process_spi_datagram(const uint8_t* data, size_t len);
    void process_rdi_chunked(const uint8_t* data, size_t len, const SomeipHeader& hdr);
    void on_rdi_scan(RdiScan& scan);
    void count_rdi_loss(const E2eVerdict& verdict);
    void deliver_rdi_frame(RdiFrame& frame, const E2eVerdict& verdict);
    bool consume_e2e_header(const uint8_t*& payload, size_t& payload_len,
                            uint32_t expected_data_id, const char* stream_name,
                            E2eVerdict& verdict);
    void track_e2e_counter(E2eCounterTracker& tracker, const char* stream_name,
                           E2eVerdict& verdict);
    void publish_e2e_status(uint8_t stream, const E2eVerdict& verdict,
                            const rclcpp::Time& stamp, bool data_published,
                            uint32_t message_counter, uint8_t data_qualifier);
    void configure_sensor_streams();

    // --- Data Callbacks (from receiver threads) ---
    void on_rdi_frame(const RdiFrame& frame, const E2eVerdict& verdict);
    void on_shi_message(const ShiMessage& msg, const E2eVerdict& verdict);
    void on_spi_message(const SpiMessage& msg, const E2eVerdict& verdict);

    // --- Parameter Management ---
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
    void declare_parameters();

    /// Connect or bind one receiver at startup and log the result.
    bool connect_receiver(StreamReceiver& rx, const char* name, int port);
    /// Log connect, loss and failed reconnects of a TCP receiver.
    void track_connection(StreamReceiver& rx, const char* name, int port,
                          int n, bool& was_open);
    void read_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter>& params);
};

}  // namespace afi920_driver

#endif  // AFI920_DRIVER__AFI920_NODE_HPP_
