import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/drone/lidar_3d/pointcloud',
            'subscribe_depth': 'false',
            'subscribe_rgb': 'false',
            'subscribe_stereo': 'false',
            'subscribe_rgbd': 'false',
            'subscribe_sensor_data': 'false',
            'subscribe_odom_info': 'false',
            'subscribe_user_data': 'false',
            'subscribe_scan': 'false',
            'subscribe_scan_descriptor': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'true',
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'queue_size': '50',
            'sync_queue_size': '50',
            'scan_normal_k': '0',
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'approx_sync': 'false',
            'qos_image': '0',
            'qos_camera_info': '0',
            'qos_scan': '0',
            'qos_odom': '0',
            'qos_user_data': '0',
            'odom_topic': '/odom'
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])
