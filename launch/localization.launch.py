#!/usr/bin/env python3
"""
Localization Launch File - For when you have a pre-built map
This is separate from SLAM mode
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rocker_bogie_dir = get_package_share_directory('rocker_bogie_description')

    # Configuration files
    nav2_params = os.path.join(robot_bringup_dir, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')
    rviz_config = os.path.join(robot_bringup_dir, 'rviz', 'navigation.rviz')

    # URDF file
    urdf_file = os.path.join(rocker_bogie_dir, 'urdf', 'rocker_bogie_rover_with_lidar.urdf.xacro')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_bringup_dir, 'maps', 'my_map.yaml'),
        description='Path to the map file'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0'
    )

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baudrate', default_value='115200'
    )

    return LaunchDescription([
        # Launch arguments
        map_arg,
        serial_port_arg,
        serial_baud_arg,

        # === Hardware Nodes (same as SLAM launch) ===
        Node(
            package='motor_control_pkg',
            executable='dual_motor_driver_node',
            name='dual_motor_driver',
            output='screen'
        ),

        Node(
            package='motor_control_pkg',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen'
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'auto_standby': True
            }]
        ),

        Node(
            package='imu_mpu6050',
            executable='imu_mpu6050_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x68,
                'frame_id': 'imu_link',
                'publish_rate': 50.0
            }]
        ),

        # === Static Transforms ===
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),

        # === Robot State Publisher ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': False
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # === EKF (robot_localization) ===
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('odometry/filtered', 'odom'),
                ('/imu/data', 'imu/data')
            ]
        ),

        # === Map Server ===
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': False
            }]
        ),

        # === AMCL Localization ===
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/scan', 'scan'),
                ('/initialpose', 'initialpose'),
                ('/amcl_pose', 'amcl_pose')
            ]
        ),

        # === Nav2 Navigation Stack ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),

        # === Lifecycle Manager ===
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
