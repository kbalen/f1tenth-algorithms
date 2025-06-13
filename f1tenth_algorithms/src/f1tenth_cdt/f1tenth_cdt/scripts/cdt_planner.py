#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
from scipy.spatial import Delaunay
import time
from ackermann_msgs.msg import AckermannDriveStamped
import yaml
from shapely.geometry import Polygon, Point as ShapelyPoint, LineString
from shapely.ops import nearest_points
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point as ROSPoint
import traceback
import networkx as nx

class CDTPlanner(Node):
    def __init__(self):
        super().__init__('cdt_planner')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('goal_reached_threshold', 0.5)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/cmd_vel')  # or '/ego_racecar/drive'
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('obstacle_filter_distance', 0.5)
        
        # Get parameter values
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.obstacle_filter_distance = self.get_parameter('obstacle_filter_distance').value
        
        # Get topic names
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10)
        
        # Publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10)
        
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10)
        
        self.mesh_pub = self.create_publisher(
            MarkerArray,
            '/triangulation_mesh',
            10)
        
        self.boundary_pub = self.create_publisher(
            MarkerArray,
            '/track_boundary',
            10
        )
        
        # State variables
        self.current_pose = None
        self.current_position = None
        self.current_orientation = None
        self.goal_point = np.array([10.0, 0.0])  # Default goal
        self.obstacle_points = []
        self.path_points = []
        self.path_computed = False
        self.last_path_time = 0.0
        self.path_recompute_interval = 2.0  # seconds
        
        # Add debug flags
        self.odom_received = False
        self.first_odom_logged = False
        self.path_following_active = False

        # Add status timer
        self.status_timer = self.create_timer(2.0, self.print_status)

        # Add auto-goal timer for testing (optional)
        self.auto_goal_timer = self.create_timer(5.0, self.set_auto_goal)
        
        # Load map boundary
        try:
            package_share_directory = get_package_share_directory('f1tenth_cdt')
            map_file = f"{package_share_directory}/maps/Spielberg_map.yaml"

            with open(map_file, 'r') as file:
                map_data = yaml.safe_load(file)

            # Create polygon from boundary
            self.map_boundary = Polygon(map_data['boundary'])
            
            # Validate polygon
            if not self.map_boundary.is_valid:
                self.get_logger().error("Map boundary polygon is invalid!")
                self.map_boundary = self.map_boundary.buffer(0)  # Try to fix
            
            # Create inner boundary for safety
            self.inner_boundary = self.map_boundary.buffer(-self.safety_margin)
            
            self.get_logger().info(f"Map boundary loaded with {len(self.map_boundary.exterior.coords)} points")
            self.get_logger().info(f"Boundary bounds: {self.map_boundary.bounds}")
            
            # Publish boundary for visualization
            self.publish_boundary()
            
        except Exception as e:
            self.get_logger().error(f"Error loading map boundary: {str(e)}")
            traceback.print_exc()
            # Create default boundary if loading fails
            self.map_boundary = Polygon([
                [-10, -10], [10, -10], [10, 10], [-10, 10]
            ])
            self.inner_boundary = self.map_boundary.buffer(-self.safety_margin)
        
        self.get_logger().info('CDT Planner initialized - Waiting for odometry and goal...')
    
    def print_status(self):
        """Print system status every 2 seconds"""
        status = "=== CDT Planner Status ===\n"
        status += f"Odometry: {'RECEIVED' if self.odom_received else 'WAITING'}\n"

        if self.current_position is not None:
            status += f"Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})\n"
        else:
            status += "Position: NOT SET\n"

        status += f"Goal: ({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})\n"
        status += f"Path Computed: {self.path_computed}\n"
        status += f"Path Points: {len(self.path_points) if self.path_points else 0}\n"
        status += f"Path Following: {'ACTIVE' if self.path_following_active else 'INACTIVE'}"

        self.get_logger().info(status)

    def set_auto_goal(self):
        """Automatically set a goal for testing"""
        if self.current_position is not None and np.linalg.norm(self.goal_point - np.array([10.0, 0.0])) < 0.1:
            # Only set if still using default goal
            new_goal = self.current_position + np.array([10.0, 0.0])
            self.goal_point = new_goal
            self.get_logger().warning(f"AUTO GOAL SET: ({new_goal[0]:.2f}, {new_goal[1]:.2f}) - Use RViz for custom goals")
            self.path_computed = False
            self.auto_goal_timer.cancel()

    def publish_boundary(self):
        """Publish the track boundaries for visualization"""
        marker_array = MarkerArray()
        # Populate it with Marker objects
        for i, coord in enumerate(self.map_boundary.exterior.coords):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boundary_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.boundary_pub.publish(marker_array)
    
    def odom_callback(self, msg):
        """Process odometry information with debug logging"""
        # First time receiving odom
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("First odometry message received!")

        self.current_pose = msg.pose.pose
        self.current_position = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])

        # Extract orientation (yaw) from quaternion
        q = self.current_pose.orientation
        self.current_orientation = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        # Log position once
        if not self.first_odom_logged:
            self.first_odom_logged = True
            self.get_logger().info(f"Robot starting position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})")

        # Check if robot is inside track
        if not self.inner_boundary.contains(ShapelyPoint(self.current_position)):
            self.get_logger().warning("Robot is outside track boundaries!")

        # Follow path if computed
        if self.path_computed and self.path_points and len(self.path_points) > 0:
            if not self.path_following_active:
                self.path_following_active = True
                self.get_logger().info("PATH FOLLOWING ACTIVATED!")
            self.follow_path()
        else:
            if self.path_following_active:
                self.path_following_active = False
                self.get_logger().info("Path following deactivated - no valid path")
            # Stop if no path
            self.send_drive_command(0.0, 0.0)

    def goal_callback(self, msg):
        """Process a new goal point from RViz"""
        goal = np.array([msg.pose.position.x, msg.pose.position.y])

        # Cancel auto goal if user sets manual goal
        if hasattr(self, 'auto_goal_timer'):
            self.auto_goal_timer.cancel()

        # Ensure goal is inside track
        goal_point = ShapelyPoint(goal)
        if not self.inner_boundary.contains(goal_point):
            # Project goal to nearest point inside track
            self.get_logger().warning("Goal is outside track! Projecting to nearest valid point.")
            nearest = nearest_points(self.inner_boundary, goal_point)[0]
            self.goal_point = np.array([nearest.x, nearest.y])
        else:
            self.goal_point = goal

        self.get_logger().info(f'NEW GOAL SET: ({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})')
        self.path_computed = False  # Force path recomputation
        self.path_following_active = False  # Reset following state

    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        if self.current_position is None:
            return

        # Convert scan to obstacle points in world frame
        obstacle_points = []

        # Get obstacle points from laser scan
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if msg.range_min < range_val < msg.range_max:
                # Convert to cartesian coordinates (robot frame)
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                # Transform to world frame
                world_x = x * math.cos(self.current_orientation) - y * math.sin(self.current_orientation) + self.current_position[0]
                world_y = x * math.sin(self.current_orientation) + y * math.cos(self.current_orientation) + self.current_position[1]
                
                world_point = np.array([world_x, world_y])
                
                # Only keep points that are inside the track and not too close to walls
                if self.map_boundary.contains(ShapelyPoint(world_point)):
                    # Check distance to boundary
                    dist_to_boundary = self.map_boundary.exterior.distance(ShapelyPoint(world_point))
                    if dist_to_boundary > self.obstacle_filter_distance:
                        obstacle_points.append(world_point)
            
            angle += msg.angle_increment

        self.obstacle_points = np.array(obstacle_points) if obstacle_points else np.array([])
        
        # Recompute path periodically
        current_time = time.time()
        if (not self.path_computed or 
            current_time - self.last_path_time > self.path_recompute_interval):
            self.compute_path()
            self.last_path_time = current_time
    
    def compute_path(self):
        """Compute a path using Constrained Delaunay Triangulation"""
        if self.current_position is None:
            return

        self.get_logger().info("Computing CDT path...")

        # 1. Local bounding box around the robot
        radius = 5.0  # Reduced radius for testing
        x0, y0 = self.current_position
        x_vals = np.arange(x0 - radius, x0 + radius, self.map_resolution * 2)  # Increased resolution
        y_vals = np.arange(y0 - radius, y0 + radius, self.map_resolution * 2)

        local_points = []
        for x in x_vals:
            for y in y_vals:
                pt = ShapelyPoint(x, y)
                if self.inner_boundary.contains(pt):
                    local_points.append((x, y))

        # Validate points
        local_points = list(set(local_points))  # Remove duplicates
        if len(local_points) < 3:
            self.get_logger().warn("Not enough local points for triangulation.")
            return

        self.get_logger().info(f"Added {len(local_points)} unique interior grid points")

        # Use SciPy's Delaunay for triangulation
        try:
            tri = Delaunay(np.array(local_points))
            triangles = tri.simplices
        except Exception as e:
            self.get_logger().error(f"Triangulation failed with exception: {str(e)}")
            return

        self.get_logger().info(f"Filtered to {len(triangles)} valid triangles inside track")

        # 3. Compute shortest path to lookahead goal
        self.goal_point = self.compute_goal_ahead()
        self.path = self.extract_path(local_points, triangles, self.current_position, self.goal_point)

        self.get_logger().info(f"Path computed with {len(self.path)} points")

        # 4. Trigger path following
        self.follow_path()

    def compute_goal_ahead(self):
        dx = self.lookahead_distance * math.cos(self.current_orientation)
        dy = self.lookahead_distance * math.sin(self.current_orientation)
        goal = (self.current_position[0] + dx, self.current_position[1] + dy)
        return goal

    def follow_path(self):
        """Follow the computed path using pure pursuit with debug"""
        if not self.path or len(self.path) < 2:
            self.get_logger().warn("Path is empty or too short")
            return

        if self.check_collision_ahead():
            self.send_drive_command(0.0, 0.0)
            self.get_logger().warn("Obstacle ahead! Stopping.")
            return

        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            self.get_logger().warn("No lookahead point found")
            return

        # Compute steering angle using Pure Pursuit logic
        dx = lookahead_point[0] - self.current_position[0]
        dy = lookahead_point[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - self.current_orientation
        steering_angle = math.atan2(2.0 * self.wheelbase_length * math.sin(heading_error), distance)

        # Speed decision based on curvature
        speed = self.max_speed if abs(steering_angle) < 0.2 else self.min_speed
        self.send_drive_command(speed, steering_angle)

    def check_collision_ahead(self):
        if not self.lidar_ranges:
            return False

        for angle, r in enumerate(self.lidar_ranges):
            if r < self.robot_radius + self.safety_margin:
                if -15 <= angle <= 15:  # +/- 15 deg forward
                    return True
        return False

    def find_lookahead_point(self):
        if not self.path or len(self.path) < 2:
            return None

        x0, y0 = self.current_position

        for point in self.path:
            dx = point[0] - x0
            dy = point[1] - y0
            dist = math.hypot(dx, dy)

            if dist >= self.lookahead_distance:
                return point

        return self.path[-1]  # fallback: last point if none ahead

    def send_drive_command(self, speed, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # neutral rotation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def debug_triangulation(self):
        """Debug why triangulation extends outside track"""

        # Test 1: Check if boundary is valid
        self.get_logger().info(f"Track boundary valid: {self.map_boundary.is_valid}")
        self.get_logger().info(f"Track boundary is closed: {self.map_boundary.exterior.is_closed}")

        # Test 2: Check boundary type
        self.get_logger().info(f"Boundary type: {self.map_boundary.geom_type}")
        self.get_logger().info(f"Is convex: {self.map_boundary.convex_hull.equals(self.map_boundary)}")

        # Test 3: Show what Triangle sees
        test_points = np.array(list(self.map_boundary.exterior.coords))
        from scipy.spatial import ConvexHull
        hull = ConvexHull(test_points)
        hull_area = hull.volume  # In 2D, volume is area
        track_area = self.map_boundary.area

        self.get_logger().info(f"Convex hull area: {hull_area:.2f}")
        self.get_logger().info(f"Track area: {track_area:.2f}")
        self.get_logger().info(f"Ratio: {track_area/hull_area:.2f} (should be < 1 for non-convex)")

    def extract_path(self, points, triangles, start, goal):
        # Step 1: Build triangle centroids and adjacency graph
        centroids = []
        triangle_map = {}  # map: centroid → triangle index
        for i, tri in enumerate(triangles):
            pts = [points[idx] for idx in tri]
            cx = sum([p[0] for p in pts]) / 3
            cy = sum([p[1] for p in pts]) / 3
            centroids.append((cx, cy))
            triangle_map[(cx, cy)] = i

        # Step 2: Create graph with centroids as nodes
        G = nx.Graph()
        for i, c1 in enumerate(centroids):
            for j, c2 in enumerate(centroids):
                if i >= j:
                    continue
                if self.share_edge(triangles[i], triangles[j]):
                    dist = math.hypot(c1[0]-c2[0], c1[1]-c2[1])
                    G.add_edge(c1, c2, weight=dist)

        # Step 3: Add virtual start/goal nodes connected to nearest centroids
        start_node = self.nearest_centroid(start, centroids)
        goal_node = self.nearest_centroid(goal, centroids)
        G.add_node("start")
        G.add_node("goal")
        G.add_edge("start", start_node, weight=math.hypot(start[0]-start_node[0], start[1]-start_node[1]))
        G.add_edge("goal", goal_node, weight=math.hypot(goal[0]-goal_node[0], goal[1]-goal_node[1]))

        # Step 4: Run Dijkstra
        try:
            path_nodes = nx.shortest_path(G, source="start", target="goal", weight="weight")
            path_coords = [start] + [p for p in path_nodes if isinstance(p, tuple)] + [goal]
            return path_coords
        except nx.NetworkXNoPath:
            self.get_logger().warn("No path found in triangle graph")
            return [start, goal]

    # Helper: Check Triangle Adjacency
    def share_edge(self, tri1, tri2):
        return len(set(tri1) & set(tri2)) >= 2

    # Helper: Nearest Centroid
    def nearest_centroid(self, pt, centroids):
        return min(centroids, key=lambda c: math.hypot(pt[0] - c[0], pt[1] - c[1]))


def main(args=None):
    rclpy.init(args=args)
    node = CDTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()