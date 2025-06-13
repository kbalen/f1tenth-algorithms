import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from collections import deque
import heapq

class PathPlanner:
    def __init__(self, lookahead_distance, path_publisher, clock):
        self.lookahead_distance = lookahead_distance
        self.path_publisher = path_publisher
        self.clock = clock
        self.global_path = []
        self.local_path = []

    def compute_goal_ahead(self, current_position, current_orientation, distance=None):
        """Compute the goal point ahead of the robot."""
        if distance is None:
            distance = self.lookahead_distance
            
        dx = distance * math.cos(current_orientation)
        dy = distance * math.sin(current_orientation)
        return np.array([current_position[0] + dx, current_position[1] + dy])

    def find_path_through_triangulation(self, start, goal, points, triangles):
        """Find a path through the triangulation using A* on the dual graph."""
        if len(triangles) == 0:
            return [start, goal]
        
        # Build adjacency graph of triangles
        adjacency = self.build_triangle_adjacency(triangles)
        
        # Find start and goal triangles
        start_tri = self.find_containing_triangle(start, points, triangles)
        goal_tri = self.find_containing_triangle(goal, points, triangles)
        
        if start_tri is None or goal_tri is None:
            return [start, goal]
        
        # A* search on dual graph
        path_triangles = self.astar_on_triangles(
            start_tri, goal_tri, triangles, points, adjacency
        )
        
        # Convert triangle path to point path
        return self.triangles_to_point_path(
            path_triangles, triangles, points, start, goal
        )

    def build_triangle_adjacency(self, triangles):
        """Build adjacency list for triangles (dual graph)."""
        adjacency = {i: [] for i in range(len(triangles))}
        
        for i in range(len(triangles)):
            for j in range(i + 1, len(triangles)):
                # Check if triangles share an edge
                shared_verts = len(set(triangles[i]) & set(triangles[j]))
                if shared_verts == 2:  # Share an edge
                    adjacency[i].append(j)
                    adjacency[j].append(i)
        
        return adjacency

    def find_containing_triangle(self, point, points, triangles):
        """Find which triangle contains the given point."""
        for i, tri in enumerate(triangles):
            p1, p2, p3 = [points[idx] for idx in tri]
            if self.point_in_triangle(point, p1, p2, p3):
                return i
        return None

    def point_in_triangle(self, p, a, b, c):
        """Check if point p is inside triangle abc using barycentric coordinates."""
        v0 = np.array(c) - np.array(a)
        v1 = np.array(b) - np.array(a)
        v2 = np.array(p) - np.array(a)
        
        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1)
        dot02 = np.dot(v0, v2)
        dot11 = np.dot(v1, v1)
        dot12 = np.dot(v1, v2)
        
        inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * inv_denom
        v = (dot00 * dot12 - dot01 * dot02) * inv_denom
        
        return (u >= 0) and (v >= 0) and (u + v <= 1)

    def astar_on_triangles(self, start, goal, triangles, points, adjacency):
        """A* search on the dual graph of triangles."""
        def heuristic(tri_idx):
            # Distance from triangle centroid to goal triangle centroid
            centroid = np.mean([points[i] for i in triangles[tri_idx]], axis=0)
            goal_centroid = np.mean([points[i] for i in triangles[goal]], axis=0)
            return np.linalg.norm(centroid - goal_centroid)
        
        # Priority queue: (f_score, triangle_index)
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return list(reversed(path))
            
            for neighbor in adjacency[current]:
                # Cost is distance between triangle centroids
                current_centroid = np.mean([points[i] for i in triangles[current]], axis=0)
                neighbor_centroid = np.mean([points[i] for i in triangles[neighbor]], axis=0)
                tentative_g = g_score[current] + np.linalg.norm(
                    current_centroid - neighbor_centroid
                )
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found

    def triangles_to_point_path(self, triangle_path, triangles, points, start, goal):
        """Convert a path through triangles to a path of points."""
        if not triangle_path:
            return [start, goal]
        
        path = [start]
        
        # For each pair of consecutive triangles, find the midpoint of shared edge
        for i in range(len(triangle_path) - 1):
            tri1 = triangles[triangle_path[i]]
            tri2 = triangles[triangle_path[i + 1]]
            
            # Find shared edge
            shared = list(set(tri1) & set(tri2))
            if len(shared) == 2:
                # Midpoint of shared edge
                p1 = points[shared[0]]
                p2 = points[shared[1]]
                midpoint = (np.array(p1) + np.array(p2)) / 2
                path.append(midpoint)
        
        path.append(goal)
        
        # Smooth the path
        return self.smooth_path(path, points, triangles)

    def smooth_path(self, path, points, triangles, iterations=3):
        """Smooth the path using a simple averaging filter."""
        if len(path) <= 2:
            return path
        
        smoothed = list(path)
        
        for _ in range(iterations):
            new_path = [smoothed[0]]  # Keep start
            
            for i in range(1, len(smoothed) - 1):
                # Average with neighbors
                prev_point = smoothed[i - 1]
                curr_point = smoothed[i]
                next_point = smoothed[i + 1]
                
                new_point = (np.array(prev_point) + 
                           2 * np.array(curr_point) + 
                           np.array(next_point)) / 4
                
                # Ensure new point is still in a valid triangle
                if self.find_containing_triangle(new_point, points, triangles) is not None:
                    new_path.append(new_point)
                else:
                    new_path.append(curr_point)
            
            new_path.append(smoothed[-1])  # Keep goal
            smoothed = new_path
        
        return smoothed

    def publish_path(self, path_points):
        """Publish the planned path to RViz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.clock.now().to_msg()

        for i, point in enumerate(path_points):  # Using enumerate instead of index()
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            
            # Calculate orientation for each segment
            if i < len(path_points) - 1:
                next_point = path_points[i + 1]
                yaw = math.atan2(
                    next_point[1] - point[1], 
                    next_point[0] - point[0]
                )
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.local_path = list(path_points)  # Store as list for reference