import numpy as np
from math import radians
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import geometry_msgs.msg

class Racer(Node):
    def __init__(self):
        super().__init__('f1_racer')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)  # Marker publisher

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges = np.clip(ranges, 0, 10)  # Clean up max range values

        # Step 1: Create "bubble" around closest obstacle
        closest_idx = np.argmin(ranges)
        bubble_radius = 30  # number of points to zero out
        ranges[max(0, closest_idx - bubble_radius):closest_idx + bubble_radius] = 0.0

        # Step 2: Find widest gap (longest sequence of non-zero)
        gaps = np.ma.masked_equal(ranges, 0)
        if gaps.count() == 0:
            self.get_logger().warn("No gap found! Stopping.")
            return

        best_idx = gaps.argmax()
        steer_angle = self.index_to_steering(best_idx, scan.angle_min, scan.angle_increment, len(ranges))

        # Step 3: Detect curvature in front 1.5m
        curvature_type = self.detect_curvature(ranges)
        speed = self.choose_speed(curvature_type)

        # Publish command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        # Debug print
        self.get_logger().info(f"Steer: {steer_angle:.2f}, Speed: {speed:.2f}, Path: {curvature_type}")

        # Step 4: Publish markers for visualization
        fake_traj = [(i * 0.5, 0.0) for i in range(10)]  # Mock trajectory points
        self.publish_markers(fake_traj, curvature_type)

    def index_to_steering(self, index, angle_min, angle_inc, num_points):
        angle = angle_min + index * angle_inc
        return angle

    def detect_curvature(self, ranges):
        # Look at ±15° window (approx indices 540 ± 90)
        center = len(ranges) // 2
        window = ranges[center - 90:center + 90]
        diffs = np.abs(np.diff(window))
        mean_diff = np.mean(diffs)

        if mean_diff < 0.05:
            return "STRAIGHT"
        elif mean_diff < 0.2:
            return "CURVE"
        else:
            return "CORNER"

    def choose_speed(self, curvature):
        if curvature == "STRAIGHT":
            return 2.0
        elif curvature == "CURVE":
            return 1.2
        else:  # CORNER
            return 0.8

    def publish_markers(self, points, curvature_type):
        """Publish markers for visualization in RViz."""
        marker_array = MarkerArray()

        # Visualize trajectory points
        line_marker = Marker()
        line_marker.header.frame_id = "ego_racecar/base_link"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        line_marker.pose.orientation.w = 1.0

        for pt in points:
            p = geometry_msgs.msg.Point()
            p.x, p.y = pt[0], pt[1]
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # Curvature label marker
        text_marker = Marker()
        text_marker.header.frame_id = "ego_racecar/base_link"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "curvature"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = 1.0
        text_marker.scale.z = 0.5
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f"Curvature: {curvature_type}"
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    racer = Racer()
    rclpy.spin(racer)
    racer.destroy_node()
    rclpy.shutdown()