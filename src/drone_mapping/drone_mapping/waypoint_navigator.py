import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/drone/lidar/scan', self.scan_cb, 10)
        
        self.state = 'INIT'
        self.timer = self.create_timer(0.1, self.execute_navigation)
        self.start_time = None
        self.front_dist = 10.0
        
    def scan_cb(self, msg):
        center = len(msg.ranges) // 2
        window = 40
        front_ranges = msg.ranges[center-window : center+window]
        valid_ranges = [r for r in front_ranges if not math.isnan(r) and not math.isinf(r) and r > 0.1]
        if valid_ranges:
            self.front_dist = min(valid_ranges)
        else:
            self.front_dist = 10.0

    def execute_navigation(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0 or self.publisher_.get_subscription_count() == 0:
            return

        if self.start_time is None:
            self.start_time = now
            self.state = 'TAKEOFF'
            self.get_logger().info('Drone connected. Autonomous mapping taking off!')
            
        t = Twist()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        if self.state == 'TAKEOFF':
            t.linear.z = 2.5 # Altitude up quickly
            if elapsed > 2.0: # Reach high altitude
                self.state = 'EXPLORE'
                self.get_logger().info('Switching to EXPLORE room.')
        elif self.state == 'EXPLORE':
            t.linear.z = 0.8  # Positive bias to maintain high altitude against pitch
            t.angular.y = 5.0 # Pitch forward FAST = Move forward FAST
            if self.front_dist < 2.0: # Detect walls generously to brake in time
                self.state = 'AVOID'
                self.get_logger().info(f'Wall detected at {self.front_dist:.2f}m. Turning to avoid.')
        elif self.state == 'AVOID':
            t.linear.z = 0.6
            t.angular.y = 0.0 # Stop moving forward
            t.angular.z = -5.0 # Yaw left FAST to spin
            if self.front_dist > 3.0: # Wait until we see a giant clear path
                self.state = 'EXPLORE'
                self.get_logger().info('Path clear. Resuming fast mapping.')

        self.publisher_.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
