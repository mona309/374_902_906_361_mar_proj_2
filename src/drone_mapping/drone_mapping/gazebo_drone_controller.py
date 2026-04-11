#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.msg import ModelStates
import math
import time

class GazeboDroneController(Node):
    def __init__(self):
        super().__init__('gazebo_drone_controller')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/drone/cmd_vel_safe', self.cmd_vel_callback, 10
        )
        
        # Service clients
        self.apply_wrench_client = self.create_client(
            ApplyBodyWrench, '/gazebo/default/apply_body_wrench'
        )
        
        # Link states subscriber for position feedback
        self.link_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.pose_callback, 10
        )
        
        # Variables
        self.current_twist = Twist()
        self.drone_position = None
        self.drone_link_name = 'quadrotor::base_link'
        
        # Timer for applying forces
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        
        # Wait for service
        while not self.apply_wrench_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for apply_body_wrench service...')
            
        self.get_logger().info('Gazebo Drone Controller initialized')

    def cmd_vel_callback(self, msg):
        self.current_twist = msg

    def pose_callback(self, msg):
        try:
            if 'quadrotor' in msg.name:
                index = msg.name.index('quadrotor')
                self.drone_position = msg.pose[index]
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        if self.drone_position is None:
            return
            
        # Get current commands
        forward = self.current_twist.linear.x
        strafe = self.current_twist.linear.y
        altitude = self.current_twist.linear.z
        yaw = self.current_twist.angular.z
        
        # Calculate forces based on commands
        # These are simplified force calculations for demonstration
        force_x = forward * 2.0  # Force factor for forward/backward
        force_y = strafe * 2.0   # Force factor for left/right
        force_z = altitude * 3.0  # Force factor for up/down (compensates for gravity)
        
        # Add hover thrust to counteract gravity (simplified)
        hover_force = 14.7  # Approximate force to counteract gravity for 1.5kg drone
        force_z += hover_force
        
        # Create wrench request
        req = ApplyBodyWrench.Request()
        req.body_name = 'quadrotor::base_link'
        req.reference_frame = 'world'
        req.wrench.force.x = force_x
        req.wrench.force.y = force_y
        req.wrench.force.z = force_z
        req.wrench.torque.z = yaw * 0.5  # Torque for yaw
        req.duration.sec = 0
        req.duration.nanosec = 20000000  # 20ms
        
        # Apply the wrench
        future = self.apply_wrench_client.call_async(req)
        
        # Log current state periodically
        if self.drone_position and self.get_clock().now().seconds_nanoseconds()[0] % 5 == 0:
            self.get_logger().info(
                f'Position: x={self.drone_position.pose.position.x:.2f}, '
                f'y={self.drone_position.pose.position.y:.2f}, '
                f'z={self.drone_position.pose.position.z:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboDroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
