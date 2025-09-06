#!/usr/bin/env python3
pkg_share = FindPackageShare('robot_bringup')


rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'mapping.rviz'])
slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_params.yaml'])
scan_matcher_params = PathJoinSubstitution([pkg_share, 'config', 'laser_scan_matcher_params.yaml'])


# RPLiDAR driver
rplidar_node = Node(
package='rplidar_ros',
executable='rplidar_composition',
name='rplidar_node',
parameters=[{
'serial_port': LaunchConfiguration('serial_port'),
'serial_baudrate': LaunchConfiguration('serial_baudrate'),
'frame_id': 'laser'
}],
output='screen'
)


# static transform base_link -> laser
static_tf = Node(
package='tf2_ros',
executable='static_transform_publisher',
name='static_tf_laser',
arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
output='screen'
)


# Laser scan matcher (provides /odom and publishes odom->base_link tf)
scan_matcher = Node(
package='ros2_laser_scan_matcher',
executable='laser_scan_matcher',
name='laser_scan_matcher',
parameters=[scan_matcher_params],
output='screen'
)


# Include slam_toolbox's online_async launch, point it to our params
slam_include = IncludeLaunchDescription(
PythonLaunchDescriptionSource(PathJoinSubstitution([
FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
])),
launch_arguments={'params_file': slam_params}.items()
)


# RViz
rviz_node = Node(
package='rviz2',
executable='rviz2',
name='rviz2',
arguments=['-d', rviz_config],
output='screen'
)


return LaunchDescription([
serial_port_arg,
serial_baud_arg,
rplidar_node,
static_tf,
scan_matcher,
slam_include,
rviz_node,
])
