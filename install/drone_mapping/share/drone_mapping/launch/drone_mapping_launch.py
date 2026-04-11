import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='drone_mapping_world.sdf')

    # Get package directories
    pkg_project = FindPackageShare('drone_mapping')

    # World path
    world_path = PathJoinSubstitution([
        pkg_project,
        'worlds',
        world_file
    ])

    # Start Gazebo with our world
    gz_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Bridge for Gazebo-ROS communication
    bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/drone/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/drone/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ]
    )

    # Our mapping node
    mapping_node = launch_ros.actions.Node(
        package='drone_mapping',
        executable='drone_mapping_node',
        name='drone_mapping_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='drone_mapping_world.sdf'),
        gz_launch,
        bridge,
        mapping_node
    ])