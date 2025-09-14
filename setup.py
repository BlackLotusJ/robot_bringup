from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install maps
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # Install RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

        # Install scripts (if you later add any executable scripts)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='jerrykiche61@gmail.com',
    description='Bringup package for robot with LiDAR + IMU + SLAM',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mapless_mission_controller = robot_bringup.mapless_mission_controller:main',
            'mission_controller = robot_bringup.mission_controller:main',
            'balloon_dijkstra_mission_node = robot_bringup.balloon_dijkstra_mission_node:main',
            'map_path_visualizer = robot_bringup.map_path_visualizer:main',
            'safety_monitor = robot_bringup.safety_monitor:main',
        ],
    },
)
