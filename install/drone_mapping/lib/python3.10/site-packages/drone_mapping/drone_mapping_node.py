#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import String
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import tf2_ros
from cv_bridge import CvBridge

class DroneMappingNode(Node):
    def __init__(self):
        super().__init__('drone_mapping_node')

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/drone/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/drone/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        self.map_pub = self.create_publisher(
            PointCloud2,
            '/drone_map',
            10
        )

        # State variables
        self.current_pose = None
        self.global_map_points = []  # List of 3D points
        self.last_pose = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.is_mapping = False

        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.1  # rad/s
        self.altitude = 2.0       # meters
        self.map_resolution = 0.05  # meters for downsampling

        # Create timer for mapping control
        self.timer = self.create_timer(0.1, self.control_loop)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Drone Mapping Node initialized with basic 3D mapping')

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/drone/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/drone/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        self.map_pub = self.create_publisher(
            PointCloud2,
            '/drone_map',
            10
        )

        # State variables
        self.current_pose = None
        if self.icp_available:
            import open3d as o3d
            self.global_map = o3d.geometry.PointCloud()
        else:
            self.global_map = []
        self.last_pose = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.is_mapping = False

        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.1  # rad/s
        self.altitude = 2.0       # meters
        self.map_resolution = 0.02  # meters

        # Create timer for mapping control
        self.timer = self.create_timer(0.1, self.control_loop)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Drone Mapping Node initialized with ICP-based 3D mapping')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def depth_callback(self, msg):
        if not self.is_mapping or self.current_pose is None or self.camera_info is None:
            return

        try:
            # Convert ROS image to OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert depth image to point cloud
            points_3d = self.depth_to_point_cloud(depth_image, self.camera_info, self.current_pose)

            if len(points_3d) == 0:
                return

            # Simple point cloud accumulation (basic version without ICP)
            self.global_map_points.extend(points_3d)

            # Basic downsampling by keeping only points within resolution grid
            if len(self.global_map_points) > 5000:
                self.downsample_map()

            self.last_pose = self.current_pose

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def downsample_map(self):
        """Simple downsampling by grid-based filtering"""
        if len(self.global_map_points) < 1000:
            return

        # Convert to numpy array
        points = np.array(self.global_map_points)

        # Create grid indices
        grid_size = self.map_resolution
        grid_indices = np.floor(points / grid_size).astype(int)

        # Keep only one point per grid cell
        _, unique_indices = np.unique(grid_indices, axis=0, return_index=True)
        self.global_map_points = points[unique_indices].tolist()

        self.get_logger().info(f'Downsampled map to {len(self.global_map_points)} points')

    def depth_to_point_cloud(self, depth_image, camera_info, pose):
        """Convert depth image to 3D point cloud in world frame"""
        height, width = depth_image.shape

        # Camera intrinsics
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Get pose transformation
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        rot = R.from_quat([pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w])

        points = []

        # Sample points (not all pixels for efficiency)
        step = 10
        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u]
                if np.isnan(depth) or depth <= 0 or depth > 10.0:
                    continue

                # Convert to camera coordinates
                x_cam = (u - cx) * depth / fx
                y_cam = (v - cy) * depth / fy
                z_cam = depth

                # Transform to world coordinates
                point_cam = np.array([x_cam, y_cam, z_cam])
                point_world = rot.apply(point_cam) + np.array([x, y, z])

                points.append(point_world)

        return points

    def publish_map(self):
        if not self.global_map_points:
            return

        points = np.array(self.global_map_points)

        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'map'

        # Point cloud fields
        cloud_msg.fields = [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1}
        ]

        cloud_msg.point_step = 12
        cloud_msg.row_step = len(points) * cloud_msg.point_step
        cloud_msg.data = points.astype(np.float32).tobytes()
        cloud_msg.width = len(points)
        cloud_msg.height = 1
        cloud_msg.is_dense = True

        self.map_pub.publish(cloud_msg)
        self.get_logger().info(f'Published 3D map with {len(points)} points')

    def control_loop(self):
        if self.current_pose is None:
            return

        if not self.is_mapping:
            self.start_mapping()
            return

        # Autonomous mapping pattern: spiral search
        cmd_vel = Twist()

        # Maintain altitude
        if abs(self.current_pose.position.z - self.altitude) > 0.2:
            cmd_vel.linear.z = (self.altitude - self.current_pose.position.z) * 0.5
        else:
            cmd_vel.linear.z = 0.0

        # Spiral motion
        current_time = self.get_clock().now().nanoseconds / 1e9
        radius = 1.0 + (current_time * 0.05) % 4.0  # Expanding spiral
        angular_vel = 0.2

        cmd_vel.linear.x = radius * angular_vel * np.cos(current_time * angular_vel)
        cmd_vel.linear.y = radius * angular_vel * np.sin(current_time * angular_vel)
        cmd_vel.angular.z = angular_vel

        self.cmd_vel_pub.publish(cmd_vel)

    def start_mapping(self):
        self.is_mapping = True
        self.global_map_points = []
        self.get_logger().info('Started 3D mapping mission')

def main(args=None):
    rclpy.init(args=args)
    node = DroneMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()