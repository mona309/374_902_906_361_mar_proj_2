import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    # SLAM Toolbox node
    slam_toolbox = Node(
        parameters=[
            os.path.join(package_dir, 'config', 'slam_toolbox_config.yaml'),
            {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/scan', '/drone/lidar/scan'),
            ('/map_metadata', '/map_metadata'),
        ]
    )
    
    # Static transform from odom to base_link
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        name='odom_to_base_link'
    )

    return LaunchDescription([
        slam_toolbox,
        odom_to_base_link
    ])
