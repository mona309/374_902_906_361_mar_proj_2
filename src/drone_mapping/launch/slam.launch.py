import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    # We use slam_toolbox for 2D SLAM
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Keep a simple fixed odom frame and let slam_toolbox publish map->odom.
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/drone/lidar/scan',
            'mode': 'mapping',
            'transform_publish_period': 0.05
        }]
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription([
        slam_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=slam_node,
                on_start=[configure_slam],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_node,
                goal_state='inactive',
                entities=[activate_slam],
            )
        )
    ])
