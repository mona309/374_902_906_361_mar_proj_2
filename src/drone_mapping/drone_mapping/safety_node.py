import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribe to raw commands and lidar
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/drone/lidar/scan', self.lidar_callback, 10)
        
        # Publish safe commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.min_distance = 1.0 # meters
        self.obstacle_detected = False
        self.latest_cmd = Twist()
        
    def lidar_callback(self, msg):
        # Check ranges for obstacles right in front
        # Assuming 0 is straight ahead, depending on Lidar mounting
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        valid_ranges = [r for r in front_ranges if not tuple(str(r)) == ('n', 'a', 'n') and r > 0]
        
        if valid_ranges and min(valid_ranges) < self.min_distance:
            if not self.obstacle_detected:
                self.get_logger().warn('Obstacle too close! SAFETY STOP ENGAGED')
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def cmd_callback(self, msg):
        safe_msg = Twist()
        
        if self.obstacle_detected:
            # Only allow turning or moving backwards (angular.y > 0 for backward, wait)
            # Just panic stop forward motion
            safe_msg.linear.z = msg.linear.z
            safe_msg.angular.z = msg.angular.z
            if msg.angular.y > 0: # Check if trying to move backward
                safe_msg.angular.y = msg.angular.y
            else:
                safe_msg.angular.y = 0.0 # Force no forward pitch
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
