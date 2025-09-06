#!/usr/bin/env python3
"""
Mapless Mission Controller Launch
Runs the custom mission node + Nav2 without SLAM
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('robot_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Config files
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(os.path.expanduser('~/robot_rviz'), 'robot_navigation.rviz')

    return LaunchDescription([

        # === Mapless Mission Controller ===
        Node(
            package='robot_bringup',
            executable='mapless_mission_controller',
            name='mapless_mission_controller',
            output='screen',
        ),

        # === Nav2 Bringup (no map_server, no SLAM) ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),

        # === RViz2 (optional visualization) ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
