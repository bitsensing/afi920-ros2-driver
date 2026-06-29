// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file pointcloud_builder.hpp
 * @brief Build sensor_msgs/PointCloud2 and afi920_msgs/DetectionArray from RDI frames
 *
 * PointCloud2 fields (18, all FLOAT32):
 *   x, y, z, radial_distance, azimuth_angle, elevation_angle,
 *   radial_velocity, radar_cross_section, signal_noise_ratio,
 *   radial_distance_error, azimuth_angle_error, elevation_angle_error,
 *   radial_velocity_error, existence_probability, multi_target_probability,
 *   detection_id, sensor_id, intensity
 */

#ifndef AFI920_DRIVER__POINTCLOUD_BUILDER_HPP_
#define AFI920_DRIVER__POINTCLOUD_BUILDER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "afi920_driver/radar_types.hpp"

#include <afi920_msgs/msg/detection_array.hpp>

namespace afi920_driver {

struct PointCloudConfig {
    std::string frame_id = "afi920";
    double min_range = 0.0;
    double max_range = 300.0;
    double min_snr = 0.0;                     // dB
    double min_existence_probability = 0.0;    // 0-100
};

class PointCloudBuilder {
public:
    explicit PointCloudBuilder(const PointCloudConfig& config);

    void update_config(const PointCloudConfig& config);

    /**
     * @brief Build PointCloud2 from RDI frame (18 FLOAT32 fields).
     * Performs spherical-to-Cartesian conversion and range/SNR/existence filtering.
     */
    sensor_msgs::msg::PointCloud2 build(
        const RdiFrame& frame, const rclcpp::Time& stamp) const;

    /**
     * @brief Build DetectionArray from RDI frame.
     * Full 1:1 mapping of bts_rdi_detection_t fields including classifications.
     */
    afi920_msgs::msg::DetectionArray build_detection_array(
        const RdiFrame& frame, const rclcpp::Time& stamp) const;

private:
    PointCloudConfig config_;

    /** @brief Check if detection passes all filters */
    bool passes_filter(const bts_rdi_detection_t& det) const;
};

}  // namespace afi920_driver

#endif  // AFI920_DRIVER__POINTCLOUD_BUILDER_HPP_
