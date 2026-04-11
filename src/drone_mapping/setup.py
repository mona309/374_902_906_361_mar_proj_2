from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'opencv-python'],
    zip_safe=True,
    maintainer='mona',
    maintainer_email='monishasharma134@gmail.com',
    description='Drone-based 3D Mapping using ICP and ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_mapping_node = drone_mapping.drone_mapping_node:main',
            'drone_controller = drone_mapping.drone_controller:main',
            'teleop_keyboard = drone_mapping.teleop_keyboard:main',
            'waypoint_navigator = drone_mapping.waypoint_navigator:main',
            'safety_node = drone_mapping.safety_node:main',
        ],
    },
)
