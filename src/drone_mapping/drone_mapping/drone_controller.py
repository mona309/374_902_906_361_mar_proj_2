import rclpy
from geometry_msgs.msg import Twist

class DroneController:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        
        self.__front_left_motor = self.__robot.getDevice('front left propeller')
        self.__front_right_motor = self.__robot.getDevice('front right propeller')
        self.__rear_left_motor = self.__robot.getDevice('rear left propeller')
        self.__rear_right_motor = self.__robot.getDevice('rear right propeller')
        
        for motor in [self.__front_left_motor, self.__front_right_motor, self.__rear_left_motor, self.__rear_right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            
        self.__camera = self.__robot.getDevice('camera')
        if self.__camera:
            self.__camera.enable(self.__timestep)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic2pro_driver')
        self.__cmd_vel_sub = self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_cb, 10)
        self.__target_twist = Twist()

    def __cmd_vel_cb(self, msg):
        self.__target_twist = msg

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        forward = self.__target_twist.linear.x
        strafe = self.__target_twist.linear.y
        yaw = self.__target_twist.angular.z
        altitude = self.__target_twist.linear.z

        pitch = forward
        roll = strafe
        
        base_vel = 68.5 + altitude * 10.0
        
        fl = base_vel - pitch - roll + yaw
        fr = base_vel - pitch + roll - yaw
        rl = base_vel + pitch - roll - yaw
        rr = base_vel + pitch + roll + yaw
        
        MAX_VEL = 100.0
        self.__front_left_motor.setVelocity(max(-MAX_VEL, min(fl, MAX_VEL)))
        self.__front_right_motor.setVelocity(-max(-MAX_VEL, min(fr, MAX_VEL)))
        self.__rear_left_motor.setVelocity(-max(-MAX_VEL, min(rl, MAX_VEL)))
        self.__rear_right_motor.setVelocity(max(-MAX_VEL, min(rr, MAX_VEL)))
