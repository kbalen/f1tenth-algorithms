#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

class PursuitController(Node):
    def __init__(self):
        super().__init__('pursuit_controller')
        
        # Subscribe to laser scan and odometry
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Publisher for drive commands
        self.drive_pub = self.create_publisher(
            Twist,
            '/drive',
            10)
        
        # Current state
        self.current_pose = None
        self.lookahead_distance = 1.0  # meters
        self.max_speed = 2.0  # m/s
        
        self.get_logger().info('Pure Pursuit controller initialized')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        if self.current_pose is None:
            return
        
        # Simple wall following behavior
        # Find the longest distance in front
        front_angles = np.arange(-30, 31)
        front_indices = [(i + 540) % 1080 for i in front_angles]
        
        distances = [msg.ranges[i] for i in front_indices]
        max_dist_idx = front_indices[np.argmax(distances)]
        
        # Calculate steering angle
        angle_rad = (max_dist_idx - 540) * msg.angle_increment
        
        # Create drive command
        drive_cmd = Twist()
        drive_cmd.linear.x = min(self.max_speed, np.max(distances) * 0.5)  # Speed proportional to distance
        drive_cmd.angular.z = angle_rad * 0.5  # Simple P controller
        
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info(f'Publishing: Speed={drive_cmd.linear.x:.2f}, Angle={drive_cmd.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = PursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()