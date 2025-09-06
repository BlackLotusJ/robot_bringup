#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the URDF file
    # Assuming you place the URDF file in your robot_bringup package
    urdf_file_name = 'rocker_bogie_rover_with_lidar.urdf.xacro'

    # You can either use package path or direct path
    # Option 1: If you have a ROS2 package
    # urdf_file = os.path.join(
    #     get_package_share_directory('robot_bringup'),
    #     'urdf',
    #     urdf_file_name
    # )

    # Option 2: Direct path (modify as needed)
    urdf_file = os.path.join(
        os.path.expanduser('~'),
        'robot_rviz',  # or wherever you saved the URDF
        urdf_file_name
    )

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Joint State Publisher (for movable joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Static Transform Publishers for sensor frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0.36', '0', '0.17', '0', '0', '0', 'base_link', 'laser_frame'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_broadcaster',
            arguments=['0', '0', '0.11', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
