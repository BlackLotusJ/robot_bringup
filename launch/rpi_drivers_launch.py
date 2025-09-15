#!/usr/bin/env python3
"""
Raspberry Pi Launch File for Autonomous Mission
Runs only the hardware drivers: LIDAR, IMU, Motors, EKF
This is the RPi-side launch file for distributed autonomous missions
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')

    # Config files
    ekf_params = os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LIDAR serial port'
    )

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='LIDAR serial baudrate'
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        serial_baud_arg,

        # === Motor Control Nodes ===
        Node(
            package='motor_control_pkg',
            executable='dual_motor_driver_node',
            name='dual_motor_driver',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

        Node(
            package='motor_control_pkg',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0,
                'wheel_base': 0.3,
                'wheel_radius': 0.065
            }]
        ),

        # === RPLIDAR Node ===
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
                'auto_standby': True,
                'scan_frequency': 10.0
            }]
        ),

        # === IMU Node ===
        Node(
            package='imu_mpu6050',
            executable='imu_mpu6050_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x68,
                'frame_id': 'imu_link',
                'publish_rate': 50.0,
                'use_sim_time': False
            }]
        ),

        # === Static Transforms ===
        # Base link to laser frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),

        # Base link to IMU frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # === EKF Node (robot_localization) ===
        #Node(
            #package='robot_localization',
            #executable='ekf_node',
            #name='ekf_filter_node',
            #output='screen',
            #parameters=[ekf_params],
            #remappings=[
                #('odometry/filtered', 'odom'),
                #('/imu/data', 'imu/data'),
                #('/cmd_vel', 'cmd_vel')
            #]
        #),
    ])
