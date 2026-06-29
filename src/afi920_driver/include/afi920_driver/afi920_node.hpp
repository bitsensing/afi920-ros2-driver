// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file afi920_node.hpp
 * @brief ROS2 node for AFI920 4D Imaging Radar
 *
 * Handles UDP reception, SOME/IP parsing, and payload deserialization
 * directly — no external SDK dependency.
 *
 * Published topics (all relative, namespace-prefixed):
 *   ~/PointCloud2  sensor_msgs/PointCloud2       (18 FLOAT32 fields)
 *   ~/RDI          afi920_msgs/DetectionArray     (full detection data)
 *   ~/SHI          afi920_msgs/HealthInfo         (sensor health)
 *   ~/SPI          afi920_msgs/SensorPerformance  (sensor performance)
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
#include <tf2_ros/transform_broadcaster.h>

#include <afi920_msgs/msg/detection_array.hpp>
#include <afi920_msgs/msg/health_info.hpp>
#include <afi920_msgs/msg/sensor_performance.hpp>

#include "afi920_driver/radar_types.hpp"
#include "afi920_driver/pointcloud_builder.hpp"
#include "afi920_driver/transport/stream_receiver.hpp"
#include "afi920_driver/transport/tcp_stream_client.hpp"
#include "afi920_driver/transport/udp_receiver.hpp"
#include "afi920_driver/someip/e2e.hpp"
#include "afi920_driver/someip/someip_parser.hpp"
#include "afi920_driver/parsers/rdi_parser.hpp"
#include "afi920_driver/parsers/shi_parser.hpp"
#include "afi920_driver/parsers/spi_parser.hpp"

namespace afi920_driver {

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
    std::string transport_mode_;
    std::atomic<bool> validate_e2e_{true};
    std::atomic<bool> e2e_strict_{false};

    // --- Transport ---
    std::unique_ptr<StreamReceiver> rdi_receiver_;
    std::unique_ptr<StreamReceiver> shi_receiver_;
    std::unique_ptr<StreamReceiver> spi_receiver_;

    // --- Protocol ---
    SomeipTpReassembler rdi_tp_;
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
    rclcpp::Publisher<afi920_msgs::msg::DetectionArray>::SharedPtr pub_rdi_;
    rclcpp::Publisher<afi920_msgs::msg::HealthInfo>::SharedPtr pub_shi_;
    rclcpp::Publisher<afi920_msgs::msg::SensorPerformance>::SharedPtr pub_spi_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    // --- TF: broadcasts parent_frame -> frame_id using SPI sensor mounting pose ---
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Statistics ---
    std::atomic<uint64_t> rdi_frame_count_{0};
    std::atomic<uint32_t> rdi_frame_seq_{0};
    std::atomic<uint32_t> total_packets_{0};
    std::atomic<uint32_t> lost_packets_{0};
    uint32_t last_rdi_counter_ = 0;
    bool first_rdi_ = true;
    uint32_t last_shi_counter_{0};
    bool first_shi_{true};
    uint32_t last_spi_counter_{0};
    bool first_spi_{true};

    // --- Diagnostics ---
    uint32_t rdi_diag_count_{0};

    // --- Thread Functions ---
    void rdi_thread_func();
    void shi_thread_func();
    void spi_thread_func();

    // --- Datagram Processing ---
    void process_rdi_datagram(const uint8_t* data, size_t len);
    void process_shi_datagram(const uint8_t* data, size_t len);
    void process_spi_datagram(const uint8_t* data, size_t len);
    void deliver_rdi_frame(RdiFrame& frame);
    bool consume_e2e_header(const uint8_t*& payload, size_t& payload_len,
                            uint32_t expected_data_id, const char* stream_name,
                            E2eHeader* out = nullptr);

    // --- Data Callbacks (from receiver threads) ---
    void on_rdi_frame(const RdiFrame& frame);
    void on_shi_message(const ShiMessage& msg);
    void on_spi_message(const SpiMessage& msg);

    // --- Parameter Management ---
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
    void declare_parameters();
    void read_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter>& params);
};

}  // namespace afi920_driver

#endif  // AFI920_DRIVER__AFI920_NODE_HPP_
