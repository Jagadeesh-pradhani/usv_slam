#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('usv_slam'), 'rviz', 'usv.rviz')

   

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('usv_slam'), 'launch','rsp.launch.py')
        ),
        # launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
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

    localize = Node(
        package="usv_slam",
        executable="localization_node",
        output="screen"
    )




    return LaunchDescription([
        rsp,
        rviz2,
        worldToMap,
        mapToBase,
        # localize
    ])
