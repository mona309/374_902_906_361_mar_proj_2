#!/usr/bin/env python3
"""Simple Webots Mavic 2 Pro controller - runs directly in Webots as extern controller."""

from controller import Robot, Keyboard
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header
import struct
import math
import time

class MavicController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get motors
        self.motors = [
            self.robot.getDevice('front left propeller'),
            self.robot.getDevice('front right propeller'),
            self.robot.getDevice('rear left propeller'),
            self.robot.getDevice('rear right propeller'),
        ]
        
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        # Get sensors
        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.timestep)
        
        self.lidar_2d = self.robot.getDevice('lidar')
        if self.lidar_2d:
            self.lidar_2d.enable(self.timestep)
        
        self.lidar_3d = self.robot.getDevice('lidar_3d')
        if self.lidar_3d:
            self.lidar_3d.enable(self.timestep)
        
        # Get keyboard
        self.keyboard = self.robot.getDevice('keyboard')
        self.keyboard.enable(self.timestep)
        
        # ROS2 setup
        rclpy.init(args=None)
        self.node = rclpy.create_node('mavic_controller')
        self.cmd_sub = self.node.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.lidar_pub = self.node.create_publisher(PointCloud2, '/drone/lidar_3d/pointcloud', 10)
        self.target_twist = Twist()
        self.last_ros_cmd_time = time.time()
        
        print("Mavic Controller initialized")
        print("Controls: W/S forward/back, A/D left/right, Q/E altitude, Arrow keys rotate")
        print("Lidar publishing to /drone/lidar_3d/pointcloud")
        
    def cmd_callback(self, msg):
        """Receive commands from ROS2 /cmd_vel topic."""
        self.target_twist = msg
        self.last_ros_cmd_time = time.time()
    
    def get_keyboard_velocity(self):
        """Get velocity from keyboard input."""
        key = self.keyboard.getKey()
        
        forward = 0.0
        lateral = 0.0
        vertical = 0.0
        yaw = 0.0
        
        if key == ord('W'):
            forward = 0.2
        elif key == ord('S'):
            forward = -0.2
        
        if key == ord('A'):
            lateral = -0.2
        elif key == ord('D'):
            lateral = 0.2
        
        if key == ord('Q'):
            vertical = 0.2
        elif key == ord('E'):
            vertical = -0.2
        
        if key == Keyboard.RIGHT:
            yaw = -0.5
        elif key == Keyboard.LEFT:
            yaw = 0.5
        
        return forward, lateral, vertical, yaw
    
    def set_motor_velocities(self, forward, lateral, vertical, yaw):
        """Set motor velocities based on control inputs."""
        base_vel = 68.5 + vertical * 10.0
        
        fl = base_vel - forward - lateral + yaw
        fr = base_vel - forward + lateral - yaw
        rl = base_vel + forward - lateral - yaw
        rr = base_vel + forward + lateral + yaw
        
        MAX_VEL = 100.0
        
        velocities = [
            max(-MAX_VEL, min(fl, MAX_VEL)),
            -max(-MAX_VEL, min(fr, MAX_VEL)),
            -max(-MAX_VEL, min(rl, MAX_VEL)),
            max(-MAX_VEL, min(rr, MAX_VEL)),
        ]
        
        for i, motor in enumerate(self.motors):
            motor.setVelocity(velocities[i])
    
    def publish_lidar(self):
        """Publish lidar 3D point cloud."""
        if not self.lidar_3d:
            return
        
        try:
            points = self.lidar_3d.getPointCloud()
            if not points:
                return
            
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "lidar_3d"
            
            msg.height = 1
            msg.width = len(points)
            
            # Define fields
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = msg.point_step * msg.width
            msg.is_dense = True
            
            # Pack point data
            data = b''
            for pt in points:
                data += struct.pack('<fff', pt.x, pt.y, pt.z)
            
            msg.data = data
            self.lidar_pub.publish(msg)
        except:
            pass
    
    def step(self):
        """Main control loop step."""
        rclpy.spin_once(self.node, timeout_sec=0)
        
        time_since_ros = time.time() - self.last_ros_cmd_time
        
        if time_since_ros < 0.5:
            forward = self.target_twist.linear.x
            lateral = self.target_twist.linear.y
            vertical = self.target_twist.linear.z
            yaw = self.target_twist.angular.z
        else:
            forward, lateral, vertical, yaw = self.get_keyboard_velocity()
        
        self.set_motor_velocities(forward, lateral, vertical, yaw)
        self.publish_lidar()
    
    def run(self):
        """Run the controller."""
        try:
            while self.robot.step(self.timestep) != -1:
                self.step()
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    controller = MavicController()
    controller.run()

def main(args=None):
    controller = MavicController()
    controller.run()
