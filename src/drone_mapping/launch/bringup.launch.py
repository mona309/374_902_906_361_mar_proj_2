import os
from launch import LaunchDescription
import launch
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    webots = ExecuteProcess(
        cmd=['webots', os.path.join(package_dir, 'worlds', 'mavic_world.wbt')],
        output='screen'
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

    static_tf_lidar_3d = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'lidar_3d']
    )
    
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    odom_publisher = Node(
        package='drone_mapping',
        executable='odom_publisher',
        name='odom_publisher'
    )

    return LaunchDescription([
        webots,
        static_tf_cam,
        static_tf_lidar,
        static_tf_lidar_3d,
        fake_odom,
        odom_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
