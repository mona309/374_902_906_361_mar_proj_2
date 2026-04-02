import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        # Publish raw commands so safety_node can enforce collision prevention.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/drone/lidar/scan', self.scan_cb, 10)
        
        self.state = 'INIT'
        self.timer = self.create_timer(0.1, self.execute_navigation)
        self.start_time = None
        self.phase_start_time = None
        self.front_dist = 10.0
        self.turn_dir = -1.0
        self.explore_step = 0
        
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
            self.phase_start_time = now
            self.get_logger().info('Drone connected. Autonomous mapping taking off!')
            
        t = Twist()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        phase_elapsed = (now - self.phase_start_time).nanoseconds / 1e9
        
        if self.state == 'TAKEOFF':
            t.linear.z = 1.8
            if elapsed > 2.5:
                self.state = 'EXPLORE'
                self.phase_start_time = now
                self.get_logger().info('Switching to EXPLORE mode.')
        elif self.state == 'EXPLORE':
            # Alternate between long and short forward pushes plus periodic turns
            # so the drone does not stay trapped in one local loop.
            t.linear.z = 0.35

            if self.explore_step % 2 == 0:
                t.angular.y = 3.8
                if phase_elapsed > 6.0:
                    self.state = 'TURN'
                    self.phase_start_time = now
            else:
                t.angular.y = 2.7
                if phase_elapsed > 3.5:
                    self.state = 'TURN'
                    self.phase_start_time = now

            if self.front_dist < 1.8:
                self.state = 'AVOID'
                self.phase_start_time = now
                self.get_logger().info(f'Obstacle at {self.front_dist:.2f} m, avoiding.')
        elif self.state == 'TURN':
            t.linear.z = 0.3
            t.angular.z = self.turn_dir * 3.8
            if phase_elapsed > 1.6:
                self.explore_step += 1
                # Alternate turn side to increase global coverage.
                self.turn_dir *= -1.0
                self.state = 'EXPLORE'
                self.phase_start_time = now
        elif self.state == 'AVOID':
            t.linear.z = 0.3
            t.angular.y = -2.5
            t.angular.z = self.turn_dir * 4.2
            if phase_elapsed > 1.2 and self.front_dist > 2.5:
                self.explore_step += 1
                self.turn_dir *= -1.0
                self.state = 'EXPLORE'
                self.phase_start_time = now
                self.get_logger().info('Path clear, resuming exploration.')

        self.publisher_.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
