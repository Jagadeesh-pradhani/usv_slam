#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
            ),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'base_scan'
            }.items()
        )
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        arguments=[
            'image_size', "[640,480]",
            'camera_frrame_id:', "base_link"
        ],
        output='screen',
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('usv_slam'), 'launch', 'cartographer.launch.py')
        )
    )

    worldToMap = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        output="screen"
    )

    mapToBase = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_footprint"],
        output="screen"
    )

    return LaunchDescription([
        lidar,
        camera,
        cartographer,
        # worldToMap,
        # mapToBase
    ])
