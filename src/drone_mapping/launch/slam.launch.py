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
            # No wheel odometry available, keep slam in the base frame so TF is complete.
            'odom_frame': 'base_link',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/drone/lidar/scan',
            'mode': 'mapping'
        }]
    )

    return LaunchDescription([
        slam_node
    ])
