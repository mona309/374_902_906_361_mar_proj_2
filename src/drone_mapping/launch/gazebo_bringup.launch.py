import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('drone_mapping')
    
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(package_dir, 'worlds', 'drone_world.world'),
        description='Full path to the world model file to load'
    )
    
    # Set GAZEBO_MODEL_PATH environment variable
    gazebo_models_path = os.path.join(package_dir, 'models')
    
    # Start Gazebo server (WSL compatible - no GUI)
    world_file_path = os.path.join(package_dir, 'worlds', 'drone_world.world')
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', world_file_path],
        output='screen',
        env={'GAZEBO_MODEL_PATH': gazebo_models_path, 'HOME': '/home/mona'}
    )
    
    # Static transform publishers
    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        name='camera_tf_broadcaster'
    )
    
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        name='lidar_tf_broadcaster'
    )
    
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        name='imu_tf_broadcaster'
    )
    
    # Static transform from odom to base_link
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        name='odom_to_base_link'
    )
    
        
    # Gazebo drone controller
    drone_controller = Node(
        package='drone_mapping',
        executable='gazebo_drone_controller',
        name='gazebo_drone_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Safety node
    safety_node = Node(
        package='drone_mapping',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_server,
        static_tf_cam,
        static_tf_lidar,
        static_tf_imu,
        odom_to_base_link,
        drone_controller,
        safety_node
    ])
