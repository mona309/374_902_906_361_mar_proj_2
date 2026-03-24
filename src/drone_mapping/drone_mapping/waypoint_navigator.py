import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Simple time-based hardcoded navigation commands
        # [duration_sec, linear_x, linear_y, linear_z, angular_z]
        self.waypoints = [
            [2.0, 0.0, 0.0, 1.0, 0.0],   # Takeoff
            [3.0, 0.0, -1.0, 0.0, 0.0],  # Move Forward
            [2.0, 0.0, 0.0, 0.0, 1.0],   # Turn
            [3.0, 0.0, -1.0, 0.0, 0.0],  # Move Forward
            [2.0, 0.0, 0.0, -1.0, 0.0]   # Land
        ]
        
        self.current_wp = 0
        self.timer = self.create_timer(0.1, self.execute_navigation)
        self.start_time = time.time()
        
    def execute_navigation(self):
        if self.current_wp >= len(self.waypoints):
            t = Twist()
            self.publisher_.publish(t)
            return

        wp = self.waypoints[self.current_wp]
        duration = wp[0]
        
        elapsed = time.time() - self.start_time
        if elapsed > duration:
            self.get_logger().info(f'Finished waypoint {self.current_wp}')
            self.current_wp += 1
            self.start_time = time.time()
            return
            
        t = Twist()
        # Mapping to the teleop convention: pitch = angular.y
        t.linear.z = wp[3]
        t.angular.y = wp[2]  # Pitch for forward/backward
        t.angular.z = wp[4]
        
        self.get_logger().info(f'Navigating WP {self.current_wp} ...', once=True)
        self.publisher_.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
