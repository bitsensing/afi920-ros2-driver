# Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
r"""
AFI920 driver with an Isaac ROS pointcloud_utils pipeline.

Requires Isaac ROS (isaac_ros_pointcloud_utils) at run time only; the driver
package does not depend on it.

Pipeline:
    afi920_driver  -> <ns>/PointCloud2XYZI  (x/y/z/intensity, 16 B/point)
    PointCloudToFlatScanNode  -> <ns>/flatscan
    FlatScantoLaserScanNode   -> <ns>/scan

Usage:
    ros2 launch afi920_driver afi920_isaac.launch.py sensor_ip:=192.168.10.150
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('afi920_driver'), 'launch', 'afi920.launch.py')),
        launch_arguments={
            'namespace': ns,
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'host_ip': LaunchConfiguration('host_ip'),
            'data_port': LaunchConfiguration('data_port'),
            'shi_port': LaunchConfiguration('shi_port'),
            'spi_port': LaunchConfiguration('spi_port'),
            'transport_mode': LaunchConfiguration('transport_mode'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_pointcloud_xyzi': 'true',
            'xyzi_pad_multiple': '16',
        }.items())

    isaac = ComposableNodeContainer(
        name='afi920_isaac_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode',
                name='pointcloud_to_flatscan',
                namespace=ns,
                parameters=[{
                    # The driver publishes with SensorDataQoS (best effort).
                    'input_qos': 'SENSOR_DATA',
                    # Finite points pass; z=+inf padding points are dropped.
                    'min_z': LaunchConfiguration('min_z'),
                    'max_z': LaunchConfiguration('max_z'),
                }],
                remappings=[('pointcloud', 'PointCloud2XYZI')]),
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode',
                name='flatscan_to_laserscan',
                namespace=ns,
                parameters=[{
                    'max_range_fallback': LaunchConfiguration('max_range_fallback'),
                }]),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='afi920',
                              description='Namespace of the driver and Isaac ROS nodes'),
        DeclareLaunchArgument('sensor_ip', default_value='192.168.10.150',
                              description='AFI920 sensor IP address'),
        DeclareLaunchArgument('host_ip', default_value='auto',
                              description='Host IP for stream destination (auto-detect)'),
        DeclareLaunchArgument('data_port', default_value='30509',
                              description='RDI data port'),
        DeclareLaunchArgument('shi_port', default_value='30510',
                              description='SHII data port'),
        DeclareLaunchArgument('spi_port', default_value='30511',
                              description='SPI data port'),
        DeclareLaunchArgument('transport_mode', default_value='tcp',
                              description='Data stream transport: tcp or udp'),
        DeclareLaunchArgument('frame_id', default_value='afi920',
                              description='TF frame ID for messages'),
        DeclareLaunchArgument('min_z', default_value='-1000.0',
                              description='FlatScan lower z bound in frame_id (meters)'),
        DeclareLaunchArgument('max_z', default_value='1000.0',
                              description='FlatScan upper z bound in frame_id (meters)'),
        DeclareLaunchArgument('max_range_fallback', default_value='300.0',
                              description='LaserScan range_max (meters)'),
        driver,
        isaac,
    ])
