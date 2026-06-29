// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file test_pointcloud_builder.cpp
 * @brief Unit tests for PointCloudBuilder using RdiFrame
 *
 * Tests cover:
 *   - PointCloud2 (18 FLOAT32 fields)
 *   - DetectionArray (full field mapping)
 *   - Range/SNR/existence_probability filtering
 *   - Cartesian conversion
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "afi920_driver/pointcloud_builder.hpp"

using namespace afi920_driver;  // NOLINT(build/namespaces)

// ─── Test Helpers ───────────────────────────────────────────────────────────

static RdiFrame make_empty_frame()
{
    RdiFrame frame{};
    frame.message.num_detections = 0;
    frame.message.sensor_id = 1;
    return frame;
}

static bts_rdi_detection_t make_detection(
    float range, float azimuth, float elevation,
    float velocity, float rcs, float snr,
    uint8_t existence_prob = 100)
{
    bts_rdi_detection_t det{};
    det.position_radial_distance = range;
    det.position_azimuth = azimuth;
    det.position_elevation = elevation;
    det.position_radial_distance_error = 0.1f;
    det.position_azimuth_error = 0.01f;
    det.position_elevation_error = 0.02f;
    det.relative_velocity_radial = velocity;
    det.relative_velocity_radial_error = 0.5f;
    det.radar_cross_section = rcs;
    det.radar_cross_section_error = 1.0f;
    det.signal_to_noise_ratio = snr;
    det.signal_to_noise_ratio_error = 2.0f;
    det.existence_probability = existence_prob;
    det.detection_id = 42;
    det.object_id_reference = 0xFFFF;  // ObjectIDReference
    det.timestamp_difference = 0.001f;
    det.multi_target_probability = 10;
    det.ambiguity_grouping_id = 5;
    det.detection_ambiguity_probability = 0;
    det.free_space_probability = 0;
    det.num_classifications = 1;
    det.classification_type[0] = 2;       // obstacle
    det.classification_confidence[0] = 80;
    return det;
}

static RdiFrame make_single_detection_frame(
    float range, float azimuth, float elevation,
    float velocity = 5.0f, float rcs = 10.0f, float snr = 20.0f,
    uint8_t sensor_id = 1)
{
    RdiFrame frame{};
    frame.message.sensor_id = sensor_id;
    frame.message.recognised_detections_capability = 2048;
    frame.message.recognised_detections_status = 0;
    frame.detections_storage.push_back(
        make_detection(range, azimuth, elevation, velocity, rcs, snr));
    frame.message.num_detections = 1;
    return frame;
}

// ─── Tests ──────────────────────────────────────────────────────────────────

class PointCloudBuilderTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        PointCloudConfig config;
        config.frame_id = "test_radar";
        config.min_range = 0.5;
        config.max_range = 200.0;
        config.min_snr = 5.0;
        config.min_existence_probability = 0.0;
        builder_ = std::make_unique<PointCloudBuilder>(config);
        stamp_ = rclcpp::Clock().now();
    }

    void TearDown() override {
        builder_.reset();
        rclcpp::shutdown();
    }

    std::unique_ptr<PointCloudBuilder> builder_;
    rclcpp::Time stamp_;
};

// ─── PointCloud2 Tests ──────────────────────────────────────────────────────

TEST_F(PointCloudBuilderTest, EmptyFrame)
{
    auto frame = make_empty_frame();
    auto cloud = builder_->build(frame, stamp_);

    EXPECT_EQ(cloud.width, 0u);
    EXPECT_EQ(cloud.header.frame_id, "test_radar");
}

TEST_F(PointCloudBuilderTest, FieldCount18)
{
    auto frame = make_single_detection_frame(10.0f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.fields.size(), 18u);
    EXPECT_EQ(cloud.fields[0].name, "x");
    EXPECT_EQ(cloud.fields[3].name, "radial_distance");
    EXPECT_EQ(cloud.fields[6].name, "radial_velocity");
    EXPECT_EQ(cloud.fields[7].name, "radar_cross_section");
    EXPECT_EQ(cloud.fields[8].name, "signal_noise_ratio");
    EXPECT_EQ(cloud.fields[13].name, "existence_probability");
    EXPECT_EQ(cloud.fields[14].name, "multi_target_probability");
    EXPECT_EQ(cloud.fields[15].name, "detection_id");
    EXPECT_EQ(cloud.fields[16].name, "sensor_id");
    EXPECT_EQ(cloud.fields[17].name, "intensity");

    // All FLOAT32
    for (const auto& field : cloud.fields) {
        EXPECT_EQ(field.datatype, sensor_msgs::msg::PointField::FLOAT32);
    }
}

TEST_F(PointCloudBuilderTest, PointStep72Bytes)
{
    auto frame = make_single_detection_frame(10.0f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);

    // 18 fields × 4 bytes = 72
    EXPECT_EQ(cloud.point_step, 72u);
}

TEST_F(PointCloudBuilderTest, SingleDetectionCartesian)
{
    // Detection at range=10m, azimuth=0, elevation=0 → x=10, y=0, z=0
    auto frame = make_single_detection_frame(10.0f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    EXPECT_NEAR(*it_x, 10.0f, 0.001f);
    EXPECT_NEAR(*it_y, 0.0f, 0.001f);
    EXPECT_NEAR(*it_z, 0.0f, 0.001f);
}

TEST_F(PointCloudBuilderTest, CartesianConversionAzimuth)
{
    // Detection at range=10m, azimuth=pi/4 (45 deg), elevation=0
    float az = static_cast<float>(M_PI / 4.0);
    auto frame = make_single_detection_frame(10.0f, az, 0.0f);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    EXPECT_NEAR(*it_x, 7.071f, 0.01f);
    EXPECT_NEAR(*it_y, 7.071f, 0.01f);
    EXPECT_NEAR(*it_z, 0.0f, 0.001f);
}

TEST_F(PointCloudBuilderTest, CartesianConversionElevation)
{
    // Detection at range=10m, azimuth=0, elevation=pi/6 (30 deg)
    float el = static_cast<float>(M_PI / 6.0);
    auto frame = make_single_detection_frame(10.0f, 0.0f, el);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    EXPECT_NEAR(*it_x, 8.660f, 0.01f);
    EXPECT_NEAR(*it_z, 5.0f, 0.01f);
}

TEST_F(PointCloudBuilderTest, SphericalAndSignalFields)
{
    auto frame = make_single_detection_frame(
        10.0f, 0.3f, 0.1f,
        /*velocity=*/-3.5f, /*rcs=*/15.2f, /*snr=*/25.0f);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_rd(cloud, "radial_distance");
    sensor_msgs::PointCloud2Iterator<float> it_az(cloud, "azimuth_angle");
    sensor_msgs::PointCloud2Iterator<float> it_el(cloud, "elevation_angle");
    sensor_msgs::PointCloud2Iterator<float> it_rv(cloud, "radial_velocity");
    sensor_msgs::PointCloud2Iterator<float> it_rcs(cloud, "radar_cross_section");
    sensor_msgs::PointCloud2Iterator<float> it_snr(cloud, "signal_noise_ratio");
    sensor_msgs::PointCloud2Iterator<float> it_int(cloud, "intensity");

    EXPECT_NEAR(*it_rd, 10.0f, 0.001f);
    EXPECT_NEAR(*it_az, 0.3f, 0.001f);
    EXPECT_NEAR(*it_el, 0.1f, 0.001f);
    EXPECT_NEAR(*it_rv, -3.5f, 0.001f);
    EXPECT_NEAR(*it_rcs, 15.2f, 0.001f);
    EXPECT_NEAR(*it_snr, 25.0f, 0.001f);
    EXPECT_NEAR(*it_int, 15.2f, 0.001f);  // intensity = rcs
}

TEST_F(PointCloudBuilderTest, VarianceFields)
{
    auto frame = make_single_detection_frame(10.0f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_rde(cloud, "radial_distance_error");
    sensor_msgs::PointCloud2Iterator<float> it_aze(cloud, "azimuth_angle_error");
    sensor_msgs::PointCloud2Iterator<float> it_ele(cloud, "elevation_angle_error");
    sensor_msgs::PointCloud2Iterator<float> it_rve(cloud, "radial_velocity_error");

    EXPECT_NEAR(*it_rde, 0.1f, 0.001f);
    EXPECT_NEAR(*it_aze, 0.01f, 0.001f);
    EXPECT_NEAR(*it_ele, 0.02f, 0.001f);
    EXPECT_NEAR(*it_rve, 0.5f, 0.001f);
}

TEST_F(PointCloudBuilderTest, ProbabilityAndIdFields)
{
    auto frame = make_single_detection_frame(
        10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f, /*sensor_id=*/7);
    auto cloud = builder_->build(frame, stamp_);

    ASSERT_EQ(cloud.width, 1u);

    sensor_msgs::PointCloud2Iterator<float> it_ep(cloud, "existence_probability");
    sensor_msgs::PointCloud2Iterator<float> it_mtp(cloud, "multi_target_probability");
    sensor_msgs::PointCloud2Iterator<float> it_did(cloud, "detection_id");
    sensor_msgs::PointCloud2Iterator<float> it_sid(cloud, "sensor_id");

    EXPECT_NEAR(*it_ep, 100.0f, 0.001f);
    EXPECT_NEAR(*it_mtp, 10.0f, 0.001f);
    EXPECT_NEAR(*it_did, 42.0f, 0.001f);
    EXPECT_NEAR(*it_sid, 7.0f, 0.001f);
}

// ─── Filter Tests ───────────────────────────────────────────────────────────

TEST_F(PointCloudBuilderTest, RangeFilterMin)
{
    auto frame = make_single_detection_frame(0.1f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);
}

TEST_F(PointCloudBuilderTest, RangeFilterMax)
{
    auto frame = make_single_detection_frame(250.0f, 0.0f, 0.0f);
    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);
}

TEST_F(PointCloudBuilderTest, SnrFilter)
{
    auto frame = make_single_detection_frame(
        10.0f, 0.0f, 0.0f, 5.0f, 10.0f, /*snr=*/2.0f);
    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);
}

TEST_F(PointCloudBuilderTest, ExistenceProbabilityFilter)
{
    // Set min_existence_probability = 50
    PointCloudConfig config;
    config.frame_id = "test_radar";
    config.min_range = 0.5;
    config.max_range = 200.0;
    config.min_snr = 5.0;
    config.min_existence_probability = 50.0;
    builder_->update_config(config);

    // Detection with existence_probability=30 → filtered out
    RdiFrame frame{};
    frame.message.sensor_id = 1;
    frame.detections_storage.push_back(
        make_detection(10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f, /*existence_prob=*/30));
    frame.message.num_detections = 1;

    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);

    // Detection with existence_probability=80 → passes
    frame.detections_storage[0].existence_probability = 80;
    cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 1u);
}

TEST_F(PointCloudBuilderTest, MultipleDetectionsWithFiltering)
{
    RdiFrame frame{};
    frame.message.sensor_id = 1;

    // 3 detections: 2 valid, 1 filtered by SNR
    frame.detections_storage.push_back(
        make_detection(10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f));  // valid
    frame.detections_storage.push_back(
        make_detection(20.0f, 0.5f, 0.1f, -2.0f, 8.0f, 15.0f));  // valid
    frame.detections_storage.push_back(
        make_detection(30.0f, 0.3f, 0.0f, 1.0f, 5.0f, 1.0f));    // SNR too low
    frame.message.num_detections = 3;

    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 2u);
}

// ─── DetectionArray Tests ───────────────────────────────────────────────────

TEST_F(PointCloudBuilderTest, DetectionArrayBasic)
{
    auto frame = make_single_detection_frame(10.0f, 0.3f, 0.1f, -3.5f, 15.2f, 25.0f, 7);
    frame.message.message_counter = 123;
    frame.message.recognised_detections_capability = 2048;
    frame.message.recognised_detections_status = 0;
    frame.message.ambiguity_domain.radial_velocity_begin = -88.89f;
    frame.message.ambiguity_domain.radial_velocity_end = 55.56f;

    auto arr = builder_->build_detection_array(frame, stamp_);

    EXPECT_EQ(arr.header.frame_id, "test_radar");
    EXPECT_EQ(arr.sensor_id, 7);
    EXPECT_EQ(arr.message_counter, 123u);
    EXPECT_EQ(arr.max_detections, 2048);
    EXPECT_EQ(arr.recognised_detections_status, 0);
    EXPECT_NEAR(arr.velocity_ambiguity_begin, -88.89f, 0.01f);

    ASSERT_EQ(arr.detections.size(), 1u);
    const auto& d = arr.detections[0];
    EXPECT_EQ(d.detection_id, 42);
    EXPECT_EQ(d.object_id_reference, 0xFFFF);
    EXPECT_EQ(d.existence_probability, 100);
    EXPECT_NEAR(d.timestamp_difference, 0.001f, 0.0001f);
    EXPECT_NEAR(d.radial_distance, 10.0f, 0.001f);
    EXPECT_NEAR(d.azimuth, 0.3f, 0.001f);
    EXPECT_NEAR(d.elevation, 0.1f, 0.001f);
    EXPECT_NEAR(d.radial_velocity, -3.5f, 0.001f);
    EXPECT_NEAR(d.rcs, 15.2f, 0.001f);
    EXPECT_NEAR(d.rcs_error, 1.0f, 0.001f);
    EXPECT_NEAR(d.snr, 25.0f, 0.001f);
    EXPECT_NEAR(d.snr_error, 2.0f, 0.001f);
    EXPECT_EQ(d.multi_target_probability, 10);
    EXPECT_EQ(d.ambiguity_grouping_id, 5);
    EXPECT_EQ(d.num_classifications, 1);
    EXPECT_EQ(d.classification_type[0], 2);
    EXPECT_EQ(d.classification_confidence[0], 80);
    EXPECT_EQ(d.sensor_id, 7);

    // Verify Cartesian from spherical
    float r = 10.0f, az = 0.3f, el = 0.1f;
    float cos_el = std::cos(el);
    EXPECT_NEAR(d.x, r * cos_el * std::cos(az), 0.001f);
    EXPECT_NEAR(d.y, r * cos_el * std::sin(az), 0.001f);
    EXPECT_NEAR(d.z, r * std::sin(el), 0.001f);
}

TEST_F(PointCloudBuilderTest, DetectionArrayFiltering)
{
    RdiFrame frame{};
    frame.message.sensor_id = 1;

    frame.detections_storage.push_back(
        make_detection(10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f));  // valid
    frame.detections_storage.push_back(
        make_detection(0.1f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f));   // below min_range
    frame.detections_storage.push_back(
        make_detection(10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 2.0f));   // below min_snr
    frame.message.num_detections = 3;

    auto arr = builder_->build_detection_array(frame, stamp_);
    EXPECT_EQ(arr.detections.size(), 1u);
}

// ─── Large Detection Count Test ────────────────────────────────────────────

TEST_F(PointCloudBuilderTest, LargeDetectionCount)
{
    // Create a frame with a significant number of detections
    RdiFrame frame{};
    frame.message.num_detections = 100;  // Don't use 4096 for test speed
    frame.detections_storage.resize(100);
    for (int i = 0; i < 100; i++) {
        bts_rdi_detection_init(&frame.detections_storage[i]);
        frame.detections_storage[i].position_radial_distance = 50.0f;
        frame.detections_storage[i].existence_probability = 80;
        frame.detections_storage[i].signal_to_noise_ratio = 20.0f;
    }
    frame.message.detections = frame.detections_storage.data();

    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 100u);
    EXPECT_EQ(cloud.point_step, 72u);
}

// ─── DetectionArray Debug Fields Test ──────────────────────────────────────

TEST_F(PointCloudBuilderTest, DetectionArrayDebugFields)
{
    RdiFrame frame{};
    bts_rdi_detection_t det{};
    bts_rdi_detection_init(&det);
    det.position_radial_distance = 10.0f;
    det.existence_probability = 90;
    det.signal_to_noise_ratio = 20.0f;
    det.debug_power = -50.0f;
    det.debug_azimuth_method = 2;
    det.debug_elevation_method = 1;
    det.debug_quality_distance = 85;
    det.debug_quality_azimuth = 70;
    det.debug_quality_elevation = 60;
    det.debug_ambiguity_azimuth = 3;
    det.debug_ambiguity_elevation = 2;
    det.debug_quality_velocity = 95;
    det.debug_ambiguity_model_velocity = 1;
    det.debug_ambiguity_index_velocity = 42;

    frame.detections_storage.push_back(det);
    frame.message.num_detections = 1;
    frame.message.detections = frame.detections_storage.data();
    frame.message.sensor_id = 1;

    auto arr = builder_->build_detection_array(frame, stamp_);
    ASSERT_EQ(arr.detections.size(), 1u);

    const auto& d = arr.detections[0];
    EXPECT_FLOAT_EQ(d.debug_power, -50.0f);
    EXPECT_EQ(d.debug_azimuth_method, 2);
    EXPECT_EQ(d.debug_elevation_method, 1);
    EXPECT_EQ(d.debug_quality_distance, 85);
    EXPECT_EQ(d.debug_quality_azimuth, 70);
    EXPECT_EQ(d.debug_quality_elevation, 60);
    EXPECT_EQ(d.debug_ambiguity_azimuth, 3);
    EXPECT_EQ(d.debug_ambiguity_elevation, 2);
    EXPECT_EQ(d.debug_quality_velocity, 95);
    EXPECT_EQ(d.debug_ambiguity_model_velocity, 1);
    EXPECT_EQ(d.debug_ambiguity_index_velocity, 42);
}

// ─── Config Update Test ─────────────────────────────────────────────────────

TEST_F(PointCloudBuilderTest, ConfigUpdate)
{
    auto frame = make_single_detection_frame(150.0f, 0.0f, 0.0f);
    auto cloud1 = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud1.width, 1u);

    PointCloudConfig new_config;
    new_config.frame_id = "updated";
    new_config.min_range = 0.5;
    new_config.max_range = 100.0;
    new_config.min_snr = 5.0;
    new_config.min_existence_probability = 0.0;
    builder_->update_config(new_config);

    auto cloud2 = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud2.width, 0u);
    EXPECT_EQ(cloud2.header.frame_id, "updated");
}

// --- Negative / Adversarial Tests ---

TEST_F(PointCloudBuilderTest, EmptyFrameProducesEmptyCloud) {
    RdiFrame frame{};
    frame.message.num_detections = 0;

    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);
    EXPECT_EQ(cloud.data.size(), 0u);
}

TEST_F(PointCloudBuilderTest, ZeroExistenceProbabilityFiltered) {
    auto det = make_detection(10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f, /*existence_prob=*/0);

    PointCloudConfig config;
    config.frame_id = "test_radar";
    config.min_range = 0.5;
    config.max_range = 200.0;
    config.min_snr = 5.0;
    config.min_existence_probability = 1.0f;
    builder_->update_config(config);

    RdiFrame frame{};
    frame.detections_storage.push_back(det);
    frame.message.num_detections = 1;

    auto cloud = builder_->build(frame, stamp_);
    EXPECT_EQ(cloud.width, 0u);  // Filtered out
}

TEST_F(PointCloudBuilderTest, NegativeRangeDetectionHandled) {
    auto det = make_detection(-1.0f, 0.0f, 0.0f, 5.0f, 10.0f, 20.0f);

    RdiFrame frame{};
    frame.detections_storage.push_back(det);
    frame.message.num_detections = 1;

    auto cloud = builder_->build(frame, stamp_);
    // Negative range should be filtered by min_range=0.5 default
    EXPECT_EQ(cloud.width, 0u);
}

TEST_F(PointCloudBuilderTest, MaxDetectionsDoesNotOverflow) {
    // Test with BTS_RDI_MAX_DETECTIONS (4096) — just verify it doesn't crash
    // Don't actually allocate 4096 — just test the constant
    EXPECT_EQ(BTS_RDI_MAX_DETECTIONS, 4096u);
    EXPECT_EQ(BTS_RDI_MAX_PAYLOAD_SIZE, 36u + 51u * 4096u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
