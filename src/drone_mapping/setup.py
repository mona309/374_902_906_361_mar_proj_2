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
