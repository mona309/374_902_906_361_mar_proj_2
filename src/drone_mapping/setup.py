from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_mapping'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))
data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')))
data_files.append((os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')))
data_files.append((os.path.join('share', package_name, 'config'), glob('config/*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mona',
    maintainer_email='user@todo.todo',
    description='Drone-Based 3D Mapping using ROS2 Humble + Webots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = drone_mapping.drone_controller:main',
            'teleop_keyboard = drone_mapping.teleop_keyboard:main',
            'waypoint_navigator = drone_mapping.waypoint_navigator:main',
            'safety_node = drone_mapping.safety_node:main',
        ],
    },
)
