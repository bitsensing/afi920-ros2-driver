# Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
r"""
AFI920 single sensor launch file.

Publishes:
    <ns>/PointCloud2  (sensor_msgs/PointCloud2)
    <ns>/RDI          (afi920_msgs/DetectionArray)
    <ns>/SHI          (afi920_msgs/HealthInfo)
    <ns>/SPI          (afi920_msgs/SensorPerformance)

Usage:
    # Default (single sensor):
    ros2 launch afi920_driver afi920.launch.py

    # With parameters:
    ros2 launch afi920_driver afi920.launch.py sensor_ip:=192.168.10.150 namespace:=radar_front

    # With config file:
    ros2 launch afi920_driver afi920.launch.py config_file:=/path/to/config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')
    has_config = PythonExpression(["'", config_file, "' != ''"])
    dds_profile = os.path.join(
        get_package_share_directory('afi920_driver'), 'config', 'dds_profile.xml'
    )

    # --- Node (with config file) ---
    node_with_config = Node(
        condition=IfCondition(has_config),
        package='afi920_driver',
        executable='afi920_node',
        name='afi920_driver',
        namespace=ns,
        parameters=[config_file],
        output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': dds_profile},
    )

    # --- Node (with individual parameters) ---
    node_with_params = Node(
        condition=UnlessCondition(has_config),
        package='afi920_driver',
        executable='afi920_node',
        name='afi920_driver',
        namespace=ns,
        parameters=[{
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'data_port': LaunchConfiguration('data_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'parent_frame_id': LaunchConfiguration('parent_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'min_snr': LaunchConfiguration('min_snr'),
            'min_existence_probability': LaunchConfiguration('min_existence_probability'),
            'publish_detections': LaunchConfiguration('publish_detections'),
            'publish_health': LaunchConfiguration('publish_health'),
            'publish_performance': LaunchConfiguration('publish_performance'),
            'transport_mode': LaunchConfiguration('transport_mode'),
            'validate_e2e': LaunchConfiguration('validate_e2e'),
            'e2e_strict': LaunchConfiguration('e2e_strict'),
            'config_port': LaunchConfiguration('config_port'),
            'host_ip': LaunchConfiguration('host_ip'),
            'shi_port': LaunchConfiguration('shi_port'),
            'spi_port': LaunchConfiguration('spi_port'),
        }],
        output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': dds_profile},
    )

    return LaunchDescription([
        # --- Launch Arguments (Basic) ---
        DeclareLaunchArgument('sensor_ip', default_value='192.168.10.150',
                              description='AFI920 sensor IP address'),
        DeclareLaunchArgument('data_port', default_value='30509',
                              description='RDI data port (TCP or UDP per transport_mode)'),
        DeclareLaunchArgument('frame_id', default_value='afi920',
                              description='TF frame ID for messages'),
        DeclareLaunchArgument('parent_frame_id', default_value='base_link',
                              description='Parent TF frame; sensor pose is broadcast as '
                                          'parent_frame_id -> frame_id'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Broadcast sensor mounting pose (from SPI) as TF'),
        DeclareLaunchArgument('namespace', default_value='afi920',
                              description='Node namespace (changes topic prefix)'),
        DeclareLaunchArgument('min_range', default_value='0.0',
                              description='Minimum range filter (meters)'),
        DeclareLaunchArgument('max_range', default_value='300.0',
                              description='Maximum range filter (meters)'),
        DeclareLaunchArgument('min_snr', default_value='0.0',
                              description='Minimum SNR filter (dB)'),
        DeclareLaunchArgument('min_existence_probability', default_value='0.0',
                              description='Minimum existence probability filter (0-100)'),
        DeclareLaunchArgument('publish_detections', default_value='true',
                              description='Publish RDI topic'),
        DeclareLaunchArgument('publish_health', default_value='true',
                              description='Publish SHI topic'),
        DeclareLaunchArgument('publish_performance', default_value='true',
                              description='Publish SPI topic'),
        DeclareLaunchArgument('transport_mode', default_value='tcp',
                              description='Data stream transport: tcp or udp'),
        DeclareLaunchArgument('validate_e2e', default_value='true',
                              description='Validate E2E CRC (results on E2E topic, no drop)'),
        DeclareLaunchArgument('e2e_strict', default_value='false',
                              description='Drop frames on E2E Length/Data ID mismatch'),
        # --- Advanced ---
        DeclareLaunchArgument('config_port', default_value='30500',
                              description='Config API TCP port'),
        DeclareLaunchArgument('host_ip', default_value='auto',
                              description='Host IP for stream destination (auto-detect)'),
        DeclareLaunchArgument('shi_port', default_value='30510',
                              description='SHII data port (TCP or UDP per transport_mode)'),
        DeclareLaunchArgument('spi_port', default_value='30511',
                              description='SPI data port (TCP or UDP per transport_mode)'),
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='Path to YAML config file (other arguments are then ignored)'),

        # --- Nodes ---
        node_with_config,
        node_with_params,
    ])
