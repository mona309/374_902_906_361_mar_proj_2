import os
from launch import LaunchDescription
import launch
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    webots = ExecuteProcess(
        cmd=['webots', '--batch', '--mode=fast', os.path.join(package_dir, 'worlds', 'map_world.wbt')],
        output='screen'
    )
    
    # Robot description specifically for mapping Python Plugin
    robot_description = """<?xml version="1.0" ?>
<robot name="Mavic2Pro">
  <webots>
    <plugin type="drone_mapping.drone_controller.DroneController" />
    <device reference="camera" type="Camera">
      <ros>
        <topicName>/drone/camera/image_raw</topicName>
      </ros>
    </device>
    <device reference="lidar" type="Lidar">
      <ros>
        <topicName>/drone/lidar/scan</topicName>
      </ros>
    </device>
    <device reference="lidar_3d" type="Lidar">
      <ros>
        <topicName>/drone/lidar_3d/pointcloud</topicName>
      </ros>
    </device>
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

    # Publish the transform for the 3D lidar so the point cloud can be visualized.
    static_tf_lidar_3d = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'lidar_3d']
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
        static_tf_lidar_3d,
        fake_odom,
        safety_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
