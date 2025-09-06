#!/usr/bin/env python3
"""
Unified Autonomous SLAM + Nav2 Launch
URDF disabled (no robot_state_publisher / joint_state_publisher)
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
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(os.path.expanduser('~/robot_rviz'), 'robot_navigation.rviz')

    return LaunchDescription([

        # === Static TFs (manual transforms only) ===
        # CRITICAL: Match the TF frames exactly with your Raspberry Pi
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0.06", "0", "0", "0", "base_footprint", "base_link"],
            name='static_tf_base_footprint_to_base_link'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.10", "0", "0.15", "0", "0", "0", "base_link", "laser_frame"],  # MATCH RPi: (0.10, 0, 0.15)
            name='static_tf_base_to_laser'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],  # MATCH RPi: (0, 0, 0.05)
            name='static_tf_base_to_imu'
        ),

        # === SLAM Toolbox ===
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            remappings=[
                ('/odom', '/slam_odom')  # Remap to avoid conflicts with potential EKF odom
            ]
        ),

        # === Nav2 Bringup ===
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

        # === RViz2 ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
