from setuptools import find_packages, setup

package_name = 'drone_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_mapping_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/drone_mapping_world.sdf']),
        ('share/' + package_name + '/rviz', ['rviz/drone_mapping.rviz']),
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
        ],
    },
)
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
>>>>>>> 8b71ccf53fff8e21c7ff19e681fc2e14b9c58cc8
        ],
    },
)
