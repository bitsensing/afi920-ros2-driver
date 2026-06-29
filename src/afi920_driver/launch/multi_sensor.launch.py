# Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
r"""
AFI920 multi-sensor launch file.

Each sensor runs in a separate namespace with independent topics.

Result topics per sensor:
    /<namespace>/PointCloud2
    /<namespace>/RDI
    /<namespace>/SHI
    /<namespace>/SPI

Usage:
    # Default 4 sensors:
    ros2 launch afi920_driver multi_sensor.launch.py

    # Custom IPs:
    ros2 launch afi920_driver multi_sensor.launch.py front_ip:=10.0.0.1 rear_ip:=10.0.0.2

    # With config file:
    ros2 launch afi920_driver multi_sensor.launch.py \
        config_file:=$(ros2 pkg prefix afi920_driver)/share/afi920_driver/config/multi_sensor.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    transport_mode = LaunchConfiguration('transport_mode')
    validate_e2e = LaunchConfiguration('validate_e2e')
    e2e_strict = LaunchConfiguration('e2e_strict')
    parent_frame = LaunchConfiguration('parent_frame')
    publish_tf = LaunchConfiguration('publish_tf')
    config_file = LaunchConfiguration('config_file')
    has_config = PythonExpression(["'", config_file, "' != ''"])
    dds_profile = os.path.join(
        get_package_share_directory('afi920_driver'), 'config', 'dds_profile.xml'
    )

    # Sensor position definitions
    positions = ['front', 'rear', 'left', 'right']
    default_ips = {
        'front': '192.168.10.150',
        'rear':  '192.168.10.151',
        'left':  '192.168.10.152',
        'right': '192.168.10.153',
    }

    entities = [
        DeclareLaunchArgument('transport_mode', default_value='tcp',
                              description='Data stream transport: tcp or udp'),
        DeclareLaunchArgument('validate_e2e', default_value='true',
                              description='Validate AUTOSAR E2E CRC on data streams'),
        DeclareLaunchArgument('e2e_strict', default_value='false',
                              description='Drop data stream frames on E2E mismatch'),
        DeclareLaunchArgument('parent_frame', default_value='base_link',
                              description='Common parent TF frame; each sensor pose is '
                                          'broadcast as parent_frame -> <pos>_frame'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Broadcast each sensor mounting pose (from SPI) as TF'),
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='Path to YAML config file (overrides individual params)'),
    ]

    # Declare per-sensor launch arguments
    for pos in positions:
        entities.append(DeclareLaunchArgument(
            f'{pos}_ip', default_value=default_ips[pos],
            description=f'IP address for {pos} sensor'))
        entities.append(DeclareLaunchArgument(
            f'{pos}_frame', default_value=f'radar_{pos}',
            description=f'TF frame ID for {pos} sensor'))

    # Create nodes
    for pos in positions:
        ns = f'radar_{pos}'

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
                'sensor_ip': LaunchConfiguration(f'{pos}_ip'),
                'frame_id': LaunchConfiguration(f'{pos}_frame'),
                'parent_frame_id': parent_frame,
                'publish_tf': publish_tf,
                'publish_detections': True,
                'publish_health': True,
                'publish_performance': True,
                'transport_mode': transport_mode,
                'validate_e2e': validate_e2e,
                'e2e_strict': e2e_strict,
            }],
            output='screen',
            additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': dds_profile},
        )

        entities.append(node_with_config)
        entities.append(node_with_params)

    return LaunchDescription(entities)
