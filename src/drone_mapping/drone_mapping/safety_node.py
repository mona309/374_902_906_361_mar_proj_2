import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribe to raw commands and lidar
        self.cmd_sub = self.create_subscription(Twist, '/drone/cmd_vel', self.cmd_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/drone/lidar/scan', self.lidar_callback, 10)
        
        # Publish safe commands
        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel_safe', 10)
        
        self.min_distance = 0.55 # meters
        self.min_valid_range = 0.20 # ignore near-zero returns from drone body/noise
        self.obstacle_detected = False
        self.latest_cmd = Twist()
        
    def lidar_callback(self, msg):
        # Check ranges for obstacles right in front
        # Assuming 0 is straight ahead, depending on Lidar mounting
        center = len(msg.ranges) // 2
        window = 20
        front_ranges = msg.ranges[center - window : center + window]
        valid_ranges = [
            r for r in front_ranges
            if not math.isnan(r) and not math.isinf(r) and r > self.min_valid_range
        ]
        
        if valid_ranges and min(valid_ranges) < self.min_distance:
            if not self.obstacle_detected:
                self.get_logger().warn('Obstacle too close! SAFETY STOP ENGAGED')
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def cmd_callback(self, msg):
        safe_msg = Twist()
        
        if self.obstacle_detected:
            # Stop forward motion and lateral movement if an obstacle is close.
            safe_msg.linear.x = min(0.0, msg.linear.x)
            safe_msg.linear.y = 0.0
            safe_msg.linear.z = msg.linear.z
            safe_msg.angular.z = msg.angular.z
            safe_msg.angular.x = msg.angular.x
            safe_msg.angular.y = msg.angular.y
        else:
            safe_msg = msg
            
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
