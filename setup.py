from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_bringup'

# Automatically find all Python scripts in robot_bringup/ and register them
def generate_entry_points():
    scripts = glob(os.path.join(package_name, "*.py"))
    entry_points = []
    for script in scripts:
        name = os.path.splitext(os.path.basename(script))[0]
        if name != "__init__":  # skip __init__.py
            entry_points.append(f"{name} = {package_name}.{name}:main")
    return entry_points

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(),
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
        'console_scripts': generate_entry_points(),
    },
)
