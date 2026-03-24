import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_mapping')
    
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'bringup.launch.py'))
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'slam.launch.py'))
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    waypoint_navigator_node = Node(
        package='drone_mapping',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        bringup_launch,
        slam_launch,
        rviz_node,
        waypoint_navigator_node
    ])
