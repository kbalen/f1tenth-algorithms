import rclpy
from rclpy.node import Node
from f1tenth_cdt.scripts.lidar_data import LidarData
from f1tenth_cdt.scripts.environment_planner import EnvironmentPlanner
from f1tenth_cdt.scripts.triangulation import Triangulation
from f1tenth_cdt.scripts.path_planner import PathPlanner
from f1tenth_cdt.scripts.driving import Driving
from shapely.geometry import Polygon, Point as ShapelyPoint, LineString
from shapely.ops import unary_union
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import math

class CDTPlanner(Node):
    def __init__(self):
        super().__init__('cdt_planner')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 2.5)
        self.declare_parameter('max_speed', 5) 
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('map_resolution', 0.3)
        self.declare_parameter('use_dynamic_boundaries', True)
        self.declare_parameter('wall_follow_distance', 2.0)
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.use_dynamic_boundaries = self.get_parameter('use_dynamic_boundaries').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value

        # Initialize with a reasonable default boundary for F1TENTH
        initial_boundary = Polygon([[-5, -5], [15, -5], [15, 5], [-5, 5]])
        
        # Initialize components
        self.lidar_data = LidarData(
            map_boundary=initial_boundary, 
            safety_margin=self.safety_margin
        )
        self.environment_planner = EnvironmentPlanner(
            inner_boundary=initial_boundary.buffer(-self.safety_margin), 
            map_resolution=self.map_resolution
        )
        self.triangulation = Triangulation(
            mesh_publisher=self.create_publisher(MarkerArray, '/triangulation_mesh', 10),
            clock=self.get_clock()
        )
        self.path_planner = PathPlanner(
            lookahead_distance=self.lookahead_distance,
            path_publisher=self.create_publisher(Path, '/planned_path', 10),
            clock=self.get_clock()
        )
        self.driving = Driving(
            drive_publisher=self.create_publisher(AckermannDriveStamped, '/drive', 10)
        )

        # Publishers for visualization
        self.boundary_publisher = self.create_publisher(Marker, '/track_boundary', 10)
        self.free_points_publisher = self.create_publisher(Marker, '/free_space_points', 10)
        
        # State variables
        self.current_position = None
        self.current_orientation = None
        self.current_velocity = 0.0
        self.planned_path = []
        self.track_boundaries = None
        self.accumulated_scan_points = []
        self.boundary_update_counter = 0
        self.last_scan_msg = None

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        # Timer for planning loop
        self.create_timer(0.1, self.planning_loop)  # 10 Hz planning

        self.get_logger().info('CDTPlanner initialized with combined quick fix and dynamic boundaries')

    def scan_callback(self, msg):
        """Process LIDAR scan data."""
        self.last_scan_msg = msg
        
        if self.current_position is None:
            return
        
        # Process LIDAR to get obstacle points
        obstacle_points = self.lidar_data.process_scan(
            msg, self.current_position, self.current_orientation
        )
        
        # Accumulate scan points for boundary detection
        if self.use_dynamic_boundaries and len(obstacle_points) > 0:
            self.accumulated_scan_points.extend(obstacle_points.tolist())
            
            # Limit accumulated points to prevent memory issues
            if len(self.accumulated_scan_points) > 10000:
                self.accumulated_scan_points = self.accumulated_scan_points[-10000:]
            
            # Update boundaries periodically
            self.boundary_update_counter += 1
            if self.boundary_update_counter > 50 and len(self.accumulated_scan_points) > 1000:
                self.update_track_boundaries()
                self.boundary_update_counter = 0

    def update_track_boundaries(self):
        """Update track boundaries from accumulated LIDAR data."""
        try:
            points = np.array(self.accumulated_scan_points)
            
            # Simple approach: find convex hull and buffer inward
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points)
            hull_points = points[hull.vertices]
            
            # Create polygon from hull
            outer_boundary = Polygon(hull_points)
            
            # Create inner boundary by buffering inward
            inner_boundary = outer_boundary.buffer(-self.safety_margin * 3)
            
            if inner_boundary.is_valid and not inner_boundary.is_empty:
                # Update boundaries
                self.track_boundaries = inner_boundary
                self.environment_planner.inner_boundary = inner_boundary
                
                # Visualize boundary
                self.publish_boundary_visualization(inner_boundary)
                
                self.get_logger().info("Updated track boundaries from LIDAR data")
            
        except Exception as e:
            self.get_logger().debug(f"Boundary update failed (normal during startup): {e}")

    def publish_boundary_visualization(self, boundary):
        """Publish track boundary for visualization."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "track_boundary"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Extract boundary coordinates
        if hasattr(boundary, 'exterior'):
            coords = list(boundary.exterior.coords)
            for coord in coords:
                point = Point()
                point.x = float(coord[0])
                point.y = float(coord[1])
                point.z = 0.1
                marker.points.append(point)
        
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.boundary_publisher.publish(marker)

    def publish_free_points_visualization(self, free_points):
        """Visualize the free space points."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "free_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        for pt in free_points:
            point = Point()
            point.x = float(pt[0])
            point.y = float(pt[1])
            point.z = 0.05
            marker.points.append(point)
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        
        self.free_points_publisher.publish(marker)

    def odom_callback(self, msg):
        """Process odometry information."""
        self.current_position = np.array([
            msg.pose.pose.position.x, 
            msg.pose.pose.position.y
        ])
        
        # Extract orientation (yaw) from quaternion
        self.current_orientation = math.atan2(
            2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + 
                 msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1 - 2 * (msg.pose.pose.orientation.y ** 2 + 
                     msg.pose.pose.orientation.z ** 2)
        )
        
        # Get velocity
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x ** 2 + 
            msg.twist.twist.linear.y ** 2
        )

    def planning_loop(self):
        """Main planning loop."""
        if self.current_position is None:
            return
        
        try:
            # Generate local grid points for triangulation
            local_points = self.environment_planner.generate_local_grid(
                self.current_position
            )
            
            # Filter points based on both approaches
            filtered_points = []
            
            for point in local_points:
                # Quick fix: Check if point is in free space (away from LIDAR obstacles)
                in_free_space = self.is_point_in_free_space(point)
                
                # Dynamic boundaries: Check if within detected track (if available)
                in_track = True
                if self.track_boundaries is not None:
                    in_track = self.track_boundaries.contains(ShapelyPoint(point))
                
                # Point must satisfy both conditions
                if in_free_space and in_track:
                    filtered_points.append(point)
            
            # Visualize free points
            self.publish_free_points_visualization(filtered_points)
            
            self.get_logger().debug(f"Generated {len(local_points)} points, {len(filtered_points)} in free space")
            
            if len(filtered_points) < 3:
                self.get_logger().debug("Not enough points for triangulation - using wall following")
                # Use wall following as fallback
                self.follow_wall()
                return
            
            # Perform triangulation only on free space points
            triangles = self.triangulation.perform_triangulation(filtered_points)
            
            if len(triangles) == 0:
                self.get_logger().debug("No triangles generated - using wall following")
                self.follow_wall()
                return
            
            # Visualize mesh
            self.triangulation.publish_mesh(filtered_points, triangles)
            
            # Plan path using CDT
            self.plan_path_with_cdt(filtered_points, triangles)
            
            # Execute path following
            self.follow_path()
            
        except Exception as e:
            self.get_logger().error(f"Planning loop error: {str(e)}")
            # Safe fallback
            self.driving.send_drive_command(0.0, 0.0)

    def is_point_in_free_space(self, point, max_obstacle_distance=0.8):
        """Check if a point is in free space (not too close to walls)."""
        if not hasattr(self.lidar_data, 'obstacle_points') or self.lidar_data.obstacle_points.size == 0:
            return True
        
        # Calculate distance to nearest obstacle
        point_array = np.array(point)
        distances = np.linalg.norm(self.lidar_data.obstacle_points - point_array, axis=1)
        min_distance = np.min(distances)
        
        # Point is in free space if far enough from obstacles
        return min_distance > max_obstacle_distance

    def follow_wall(self):
        """Simple wall following behavior as fallback."""
        if self.last_scan_msg is None:
            self.driving.send_drive_command(0.5, 0.0)
            return
        
        # Get scan ranges
        ranges = np.array(self.last_scan_msg.ranges)
        angle_min = self.last_scan_msg.angle_min
        angle_increment = self.last_scan_msg.angle_increment
        
        # Find valid ranges
        valid_mask = (ranges > self.last_scan_msg.range_min) & (ranges < self.last_scan_msg.range_max)
        
        if not np.any(valid_mask):
            self.driving.send_drive_command(0.5, 0.0)
            return
        
        # Get ranges at specific angles for wall following
        # Right side: -90 degrees (-pi/2)
        right_idx = int((-np.pi/2 - angle_min) / angle_increment)
        # Right-front: -45 degrees (-pi/4)
        right_front_idx = int((-np.pi/4 - angle_min) / angle_increment)
        # Front: 0 degrees
        front_idx = int((0 - angle_min) / angle_increment)
        
        # Ensure indices are valid
        right_idx = np.clip(right_idx, 0, len(ranges)-1)
        right_front_idx = np.clip(right_front_idx, 0, len(ranges)-1)
        front_idx = np.clip(front_idx, 0, len(ranges)-1)
        
        right_dist = ranges[right_idx] if valid_mask[right_idx] else 10.0
        right_front_dist = ranges[right_front_idx] if valid_mask[right_front_idx] else 10.0
        front_dist = ranges[front_idx] if valid_mask[front_idx] else 10.0
        
        # Simple right-hand wall following logic
        steering = 0.0
        speed = 0.8
        
        # If too close to front wall, turn left
        if front_dist < 1.0:
            steering = 0.4  # Turn left
            speed = 0.5
        # If too close to right wall, turn left slightly
        elif right_dist < self.wall_follow_distance * 0.8:
            steering = 0.2
        # If too far from right wall, turn right slightly
        elif right_dist > self.wall_follow_distance * 1.2:
            steering = -0.2
        # If right-front is close, prepare to turn left
        elif right_front_dist < self.wall_follow_distance:
            steering = 0.3
            speed = 0.6
        
        self.driving.send_drive_command(speed, steering)
        self.get_logger().debug(f"Wall following: front={front_dist:.2f}, right={right_dist:.2f}, steer={steering:.2f}")

    def plan_path_with_cdt(self, points, triangles):
        """Plan a path using the CDT mesh."""
        if len(triangles) == 0:
            return
        
        # Compute goal point
        goal = self.path_planner.compute_goal_ahead(
            self.current_position, self.current_orientation
        )
        
        # Ensure goal is in free space
        if not self.is_point_in_free_space(goal):
            # Adjust goal to be in free space
            for dist in np.linspace(self.lookahead_distance, 0.5, 5):
                test_goal = self.path_planner.compute_goal_ahead(
                    self.current_position, self.current_orientation, dist
                )
                if self.is_point_in_free_space(test_goal):
                    goal = test_goal
                    break
        
        # Use the path planner's A* method if available
        if hasattr(self.path_planner, 'find_path_through_triangulation'):
            path = self.path_planner.find_path_through_triangulation(
                self.current_position, goal, points, triangles
            )
        else:
            # Fallback: Simple path planning
            path = [tuple(self.current_position)]
            
            # Find path through triangles
            current = np.array(self.current_position)
            while np.linalg.norm(current - goal) > 0.5:
                direction = (goal - current) / np.linalg.norm(goal - current)
                next_point = current + direction * 0.5
                
                # Ensure next point is in free space
                if self.is_point_in_free_space(next_point) and self.point_in_triangulation(next_point, points, triangles):
                    path.append(tuple(next_point))
                    current = next_point
                else:
                    break
            
            path.append(tuple(goal))
        
        self.planned_path = path
        
        # Publish path for visualization
        self.path_planner.publish_path(path)

    def point_in_triangulation(self, point, points, triangles):
        """Check if a point is within any triangle."""
        for tri in triangles:
            p1, p2, p3 = [points[i] for i in tri]
            if self.point_in_triangle(point, p1, p2, p3):
                return True
        return False

    def point_in_triangle(self, p, a, b, c):
        """Check if point p is inside triangle abc."""
        def sign(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        
        d1 = sign(p, a, b)
        d2 = sign(p, b, c)
        d3 = sign(p, c, a)
        
        has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
        has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
        
        return not (has_neg and has_pos)

    def follow_path(self):
        """Follow the planned path using pure pursuit."""
        if not self.planned_path or len(self.planned_path) < 2:
            self.driving.send_drive_command(0.0, 0.0)
            return
        
        # Find lookahead point
        lookahead_point = None
        current_pos = np.array(self.current_position)
        
        for i in range(1, len(self.planned_path)):
            point = np.array(self.planned_path[i])
            dist = np.linalg.norm(point - current_pos)
            if dist >= self.lookahead_distance:
                lookahead_point = point
                break
        
        if lookahead_point is None and len(self.planned_path) > 0:
            lookahead_point = np.array(self.planned_path[-1])
        
        if lookahead_point is None:
            self.driving.send_drive_command(0.0, 0.0)
            return
        
        # Pure pursuit control
        steering_angle = self.pure_pursuit_control(lookahead_point)
        
        # Speed control
        speed = min(self.max_speed, 1.0 + 0.5 * abs(1.0 / (abs(steering_angle) + 0.1)))
        
        # Send drive command
        self.driving.send_drive_command(speed, steering_angle)

    def pure_pursuit_control(self, lookahead_point):
        """Calculate steering angle using pure pursuit."""
        lookahead = np.array(lookahead_point)
        current_pos = np.array(self.current_position)
        
        # Transform lookahead point to vehicle frame
        dx = lookahead[0] - current_pos[0]
        dy = lookahead[1] - current_pos[1]
        
        # Rotate to vehicle frame
        cos_yaw = math.cos(self.current_orientation)
        sin_yaw = math.sin(self.current_orientation)
        
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        # Calculate steering angle
        L = np.sqrt(local_x**2 + local_y**2)
        if L < 0.01:
            return 0.0
            
        steering_angle = math.atan2(2 * local_y, L)
        
        # Limit steering angle
        max_steering = 0.4189  # ~24 degrees
        steering_angle = np.clip(steering_angle, -max_steering, max_steering)
        
        return float(steering_angle)


def main(args=None):
    rclpy.init(args=args)
    node = CDTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()