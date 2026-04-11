import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(package_dir, 'worlds', 'drone_world.world'),
        description='Full path to the world model file to load'
    )
    
    # Include bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'gazebo_bringup.launch.py')),
        launch_arguments={'world_file': LaunchConfiguration('world_file')}.items()
    )
    
    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'gazebo_slam.launch.py'))
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    # Waypoint navigator (optional)
    waypoint_navigator_node = Node(
        package='drone_mapping',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_file_arg,
        bringup_launch,
        slam_launch,
        rviz_node,
        waypoint_navigator_node
    ])
