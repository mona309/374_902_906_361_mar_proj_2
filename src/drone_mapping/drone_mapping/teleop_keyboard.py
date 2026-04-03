#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Drone Teleop Controller
---------------------------
Moving around:
        w
   a    s    d

Altitude:
   q (up), e (down)

k : Kill switch (Zero Velocity)
r : Reset Base Velocity
CTRL-C to quit
"""

moveBindings = {
    'w': (0, 11, 0, 0),   # Forward: Pitch Positive
    's': (0, -11, 0, 0),  # Backward: Pitch Negative
    'a': (0, 0, 0, -8),   # Yaw Left
    'd': (0, 0, 0, 8),    # Yaw Right
    'q': (0, 0, 8, 0),    # Up
    'e': (0, 0, -8, 0),   # Down
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    # Wait a short time in raw mode so normal key presses are captured reliably.
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # Publish raw commands so safety_node can filter before forwarding to /cmd_vel.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.target_linear_z = 0.0
        self.target_angular_x = 0.0
        self.target_angular_y = 0.0
        self.target_angular_z = 0.0
        self.get_logger().info(msg)

    def timer_callback(self):
        key = getKey()
        if key in moveBindings.keys():
            x, y, z, th = moveBindings[key]
            self.target_angular_x = float(x)
            self.target_angular_y = float(y)
            self.target_linear_z = float(z)
            self.target_angular_z = float(th)
        elif key == 'k':
            self.target_linear_z = -10.0 # Force down or kill
            self.target_angular_x = 0.0
            self.target_angular_y = 0.0
            self.target_angular_z = 0.0
        elif key == 'r':
            self.target_linear_z = 0.0
            self.target_angular_x = 0.0
            self.target_angular_y = 0.0
            self.target_angular_z = 0.0
        else:
            if (key == '\x03'):
                sys.exit(0)

        t = Twist()
        t.linear.z = self.target_linear_z
        t.angular.x = self.target_angular_x
        t.angular.y = self.target_angular_y
        t.angular.z = self.target_angular_z
        self.publisher_.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
