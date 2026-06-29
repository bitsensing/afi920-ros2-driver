// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file pointcloud_builder.cpp
 * @brief PointCloud2 and DetectionArray message construction from RDI frames
 *
 * Converts bts_rdi_detection_t (spherical, ISO 23150) to:
 *   - sensor_msgs/PointCloud2 (18 FLOAT32 fields, Bosch-level richness)
 *   - afi920_msgs/DetectionArray (full detection data)
 *
 * PointCloud2 field layout (72 bytes/point, all FLOAT32):
 *   x, y, z                          — Cartesian position (m)
 *   radial_distance, azimuth_angle, elevation_angle — spherical (m, rad)
 *   radial_velocity                   — m/s
 *   radar_cross_section               — dBm²
 *   signal_noise_ratio                — dB
 *   radial_distance_error, azimuth_angle_error,
 *   elevation_angle_error, radial_velocity_error — variances
 *   existence_probability             — 0-100 (as float)
 *   multi_target_probability          — 0-100 (as float)
 *   detection_id                      — ID (as float)
 *   sensor_id                         — ID (as float)
 *   intensity                         — = radar_cross_section (RViz compat)
 */

#include "afi920_driver/pointcloud_builder.hpp"

#include <cmath>
#include <algorithm>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace afi920_driver {

// Number of FLOAT32 fields in PointCloud2
static constexpr int PC2_NUM_FIELDS = 18;

static void setup_pointcloud2_fields(sensor_msgs::msg::PointCloud2& cloud)
{
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(PC2_NUM_FIELDS,
        "x",                        1, sensor_msgs::msg::PointField::FLOAT32,
        "y",                        1, sensor_msgs::msg::PointField::FLOAT32,
        "z",                        1, sensor_msgs::msg::PointField::FLOAT32,
        "radial_distance",          1, sensor_msgs::msg::PointField::FLOAT32,
        "azimuth_angle",            1, sensor_msgs::msg::PointField::FLOAT32,
        "elevation_angle",          1, sensor_msgs::msg::PointField::FLOAT32,
        "radial_velocity",          1, sensor_msgs::msg::PointField::FLOAT32,
        "radar_cross_section",      1, sensor_msgs::msg::PointField::FLOAT32,
        "signal_noise_ratio",       1, sensor_msgs::msg::PointField::FLOAT32,
        "radial_distance_error",    1, sensor_msgs::msg::PointField::FLOAT32,
        "azimuth_angle_error",      1, sensor_msgs::msg::PointField::FLOAT32,
        "elevation_angle_error",    1, sensor_msgs::msg::PointField::FLOAT32,
        "radial_velocity_error",    1, sensor_msgs::msg::PointField::FLOAT32,
        "existence_probability",    1, sensor_msgs::msg::PointField::FLOAT32,
        "multi_target_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
        "detection_id",             1, sensor_msgs::msg::PointField::FLOAT32,
        "sensor_id",                1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity",                1, sensor_msgs::msg::PointField::FLOAT32);
}

PointCloudBuilder::PointCloudBuilder(const PointCloudConfig& config)
    : config_(config)
{
}

void PointCloudBuilder::update_config(const PointCloudConfig& config)
{
    config_ = config;
}

bool PointCloudBuilder::passes_filter(const bts_rdi_detection_t& det) const
{
    if (det.position_radial_distance < config_.min_range ||
        det.position_radial_distance > config_.max_range) return false;
    if (det.signal_to_noise_ratio < config_.min_snr) return false;
    if (det.existence_probability < config_.min_existence_probability) return false;
    return true;
}

// ─── PointCloud2 (18 FLOAT32) ───────────────────────────────────────────────

sensor_msgs::msg::PointCloud2 PointCloudBuilder::build(
    const RdiFrame& frame, const rclcpp::Time& stamp) const
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = config_.frame_id;
    cloud.height = 1;
    cloud.is_dense = true;

    const auto& detections = frame.detections_storage;
    const uint16_t num_det = frame.message.num_detections;

    // Always set up field definitions (even for empty clouds) so downstream
    // subscribers see a consistent schema and don't crash on missing fields.
    setup_pointcloud2_fields(cloud);

    if (num_det == 0 || detections.empty()) {
        sensor_msgs::PointCloud2Modifier(cloud).resize(0);
        return cloud;
    }

    // Count valid points after filtering
    size_t valid_count = 0;
    for (uint16_t i = 0; i < num_det && i < detections.size(); ++i) {
        if (passes_filter(detections[i])) valid_count++;
    }

    if (valid_count == 0) {
        sensor_msgs::PointCloud2Modifier(cloud).resize(0);
        return cloud;
    }

    sensor_msgs::PointCloud2Modifier(cloud).resize(valid_count);

    // Create iterators for all 18 fields
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_rd(cloud, "radial_distance");
    sensor_msgs::PointCloud2Iterator<float> it_az(cloud, "azimuth_angle");
    sensor_msgs::PointCloud2Iterator<float> it_el(cloud, "elevation_angle");
    sensor_msgs::PointCloud2Iterator<float> it_rv(cloud, "radial_velocity");
    sensor_msgs::PointCloud2Iterator<float> it_rcs(cloud, "radar_cross_section");
    sensor_msgs::PointCloud2Iterator<float> it_snr(cloud, "signal_noise_ratio");
    sensor_msgs::PointCloud2Iterator<float> it_rde(cloud, "radial_distance_error");
    sensor_msgs::PointCloud2Iterator<float> it_aze(cloud, "azimuth_angle_error");
    sensor_msgs::PointCloud2Iterator<float> it_ele(cloud, "elevation_angle_error");
    sensor_msgs::PointCloud2Iterator<float> it_rve(cloud, "radial_velocity_error");
    sensor_msgs::PointCloud2Iterator<float> it_ep(cloud, "existence_probability");
    sensor_msgs::PointCloud2Iterator<float> it_mtp(cloud, "multi_target_probability");
    sensor_msgs::PointCloud2Iterator<float> it_did(cloud, "detection_id");
    sensor_msgs::PointCloud2Iterator<float> it_sid(cloud, "sensor_id");
    sensor_msgs::PointCloud2Iterator<float> it_int(cloud, "intensity");

    const float sensor_id_f = static_cast<float>(frame.message.sensor_id);

    for (uint16_t i = 0; i < num_det && i < detections.size(); ++i) {
        const auto& det = detections[i];
        if (!passes_filter(det)) continue;

        float r  = det.position_radial_distance;
        float az = det.position_azimuth;
        float el = det.position_elevation;

        // Spherical to Cartesian (ISO 23150 convention)
        float cos_el = std::cos(el);
        *it_x   = r * cos_el * std::cos(az);
        *it_y   = r * cos_el * std::sin(az);
        *it_z   = r * std::sin(el);

        // Spherical raw
        *it_rd  = r;
        *it_az  = az;
        *it_el  = el;

        // Dynamics
        *it_rv  = det.relative_velocity_radial;

        // Signal quality
        *it_rcs = det.radar_cross_section;
        *it_snr = det.signal_to_noise_ratio;

        // Variances
        *it_rde = det.position_radial_distance_error;
        *it_aze = det.position_azimuth_error;
        *it_ele = det.position_elevation_error;
        *it_rve = det.relative_velocity_radial_error;

        // Probabilities & IDs (uint→float for uniform FLOAT32 layout)
        *it_ep  = static_cast<float>(det.existence_probability);
        *it_mtp = static_cast<float>(det.multi_target_probability);
        *it_did = static_cast<float>(det.detection_id);
        *it_sid = sensor_id_f;

        // intensity = radar_cross_section (RViz visualization)
        *it_int = det.radar_cross_section;

        ++it_x; ++it_y; ++it_z;
        ++it_rd; ++it_az; ++it_el;
        ++it_rv; ++it_rcs; ++it_snr;
        ++it_rde; ++it_aze; ++it_ele; ++it_rve;
        ++it_ep; ++it_mtp; ++it_did; ++it_sid; ++it_int;
    }

    return cloud;
}

// ─── DetectionArray ─────────────────────────────────────────────────────────

afi920_msgs::msg::DetectionArray PointCloudBuilder::build_detection_array(
    const RdiFrame& frame, const rclcpp::Time& stamp) const
{
    afi920_msgs::msg::DetectionArray arr;
    arr.header.stamp = stamp;
    arr.header.frame_id = config_.frame_id;

    const auto& msg = frame.message;
    arr.sensor_id = msg.sensor_id;
    arr.message_counter = msg.message_counter;
    arr.data_qualifier = static_cast<uint8_t>(msg.data_qualifier);

    // Ambiguity domains
    arr.velocity_ambiguity_begin  = msg.ambiguity_domain.radial_velocity_begin;
    arr.velocity_ambiguity_end    = msg.ambiguity_domain.radial_velocity_end;
    arr.range_ambiguity_begin     = msg.ambiguity_domain.range_begin;
    arr.range_ambiguity_end       = msg.ambiguity_domain.range_end;
    arr.azimuth_ambiguity_begin   = msg.ambiguity_domain.azimuth_begin;
    arr.azimuth_ambiguity_end     = msg.ambiguity_domain.azimuth_end;
    arr.elevation_ambiguity_begin = msg.ambiguity_domain.elevation_begin;
    arr.elevation_ambiguity_end   = msg.ambiguity_domain.elevation_end;

    // Detection info
    arr.max_detections = msg.recognised_detections_capability;
    arr.recognised_detections_status = msg.recognised_detections_status;

    const auto& detections = frame.detections_storage;
    const uint16_t num_det = msg.num_detections;

    arr.detections.reserve(num_det);

    for (uint16_t i = 0; i < num_det && i < detections.size(); ++i) {
        const auto& det = detections[i];
        if (!passes_filter(det)) continue;

        afi920_msgs::msg::Detection d;

        // Identification
        d.detection_id = det.detection_id;
        d.object_id_reference = det.object_id_reference;
        d.existence_probability = det.existence_probability;
        d.timestamp_difference = det.timestamp_difference;

        // Spherical position
        d.radial_distance = det.position_radial_distance;
        d.azimuth = det.position_azimuth;
        d.elevation = det.position_elevation;
        d.radial_distance_error = det.position_radial_distance_error;
        d.azimuth_error = det.position_azimuth_error;
        d.elevation_error = det.position_elevation_error;

        // Cartesian (computed)
        float r  = det.position_radial_distance;
        float az = det.position_azimuth;
        float el = det.position_elevation;
        float cos_el = std::cos(el);
        d.x = r * cos_el * std::cos(az);
        d.y = r * cos_el * std::sin(az);
        d.z = r * std::sin(el);

        // Dynamics
        d.radial_velocity = det.relative_velocity_radial;
        d.radial_velocity_error = det.relative_velocity_radial_error;

        // Signal quality
        d.rcs = det.radar_cross_section;
        d.rcs_error = det.radar_cross_section_error;
        d.snr = det.signal_to_noise_ratio;
        d.snr_error = det.signal_to_noise_ratio_error;

        // Probabilities
        d.multi_target_probability = det.multi_target_probability;
        d.ambiguity_grouping_id = det.ambiguity_grouping_id;
        d.detection_ambiguity_probability = det.detection_ambiguity_probability;
        d.free_space_probability = det.free_space_probability;

        // Classification
        d.num_classifications = det.num_classifications;
        for (int c = 0; c < 8; ++c) {
            d.classification_type[c] = det.classification_type[c];
            d.classification_confidence[c] = det.classification_confidence[c];
        }

        // Sensor ID
        d.sensor_id = msg.sensor_id;

        // Debug fields
        d.debug_power = det.debug_power;
        d.debug_azimuth_method = det.debug_azimuth_method;
        d.debug_elevation_method = det.debug_elevation_method;
        d.debug_quality_distance = det.debug_quality_distance;
        d.debug_quality_azimuth = det.debug_quality_azimuth;
        d.debug_quality_elevation = det.debug_quality_elevation;
        d.debug_ambiguity_azimuth = det.debug_ambiguity_azimuth;
        d.debug_ambiguity_elevation = det.debug_ambiguity_elevation;
        d.debug_quality_velocity = det.debug_quality_velocity;
        d.debug_ambiguity_model_velocity = det.debug_ambiguity_model_velocity;
        d.debug_ambiguity_index_velocity = det.debug_ambiguity_index_velocity;

        arr.detections.push_back(std::move(d));
    }

    return arr;
}

}  // namespace afi920_driver
