import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # We use slam_toolbox for 2D SLAM
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/drone/lidar/scan',
            'mode': 'mapping'
        }]
    )

    return LaunchDescription([
        slam_node
    ])
