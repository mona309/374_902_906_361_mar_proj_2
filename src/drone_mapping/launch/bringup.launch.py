import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'map_world.wbt')
    )
    
    # Robot description specifically for mapping Python Plugin
    robot_description = """<?xml version="1.0" ?>
<robot name="Mavic2Pro">
  <webots>
    <plugin type="drone_mapping.drone_controller.DroneController" />
  </webots>
</robot>
"""
    
    drone_driver = WebotsController(
        robot_name='Mavic2Pro',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera']
    )
    
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'lidar']
    )
    
    # Odometry is required for mapping, providing a fake static one 
    # since we don't have visual odometry implemented.
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    safety_node = Node(
        package='drone_mapping',
        executable='safety_node'
    )

    return LaunchDescription([
        webots,
        drone_driver,
        static_tf_cam,
        static_tf_lidar,
        fake_odom,
        safety_node
    ])
