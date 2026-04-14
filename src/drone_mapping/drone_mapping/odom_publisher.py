#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        self.odom_pub = self.create_publisher(Odometry, '/rtabmap/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        
        self.start_time = time.time()
        
        self.get_logger().info('Odometry publisher started')

    def publish_odom(self):
        current_time = self.get_clock().now()
        
        # Create a simple circular motion for testing
        elapsed = time.time() - self.start_time
        radius = 2.0
        angular_vel = 0.1  # rad/s
        
        x = radius * math.cos(angular_vel * elapsed)
        y = radius * math.sin(angular_vel * elapsed)
        theta = angular_vel * elapsed
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 1.8  # Keep at drone height
        
        # Convert theta to quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set covariance (diagonal matrix with some uncertainty)
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        # Set twist (velocity)
        odom.twist.twist.linear.x = -radius * angular_vel * math.sin(angular_vel * elapsed)
        odom.twist.twist.linear.y = radius * angular_vel * math.cos(angular_vel * elapsed)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        # Set twist covariance
        odom.twist.covariance = odom.pose.covariance
        
        self.odom_pub.publish(odom)
        
        # Also publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 1.8
        transform.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()