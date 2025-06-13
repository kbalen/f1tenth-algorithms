import numpy as np
from scipy.spatial import Delaunay
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Triangulation:
    def __init__(self, mesh_publisher, clock):
        self.mesh_publisher = mesh_publisher
        self.clock = clock

    def perform_triangulation(self, local_points):
        """Perform Delaunay triangulation with robust error handling."""
        if len(local_points) < 3:
            return []
        
        try:
            points_array = np.array(local_points)
            
            # Check for degenerate configurations
            if self._is_degenerate(points_array):
                # Add small random perturbations to avoid colinearity
                points_array = self._perturb_points(points_array)
            
            # Perform triangulation with qhull options for robustness
            tri = Delaunay(points_array, qhull_options='QJ')  # QJ = joggle input
            triangles = tri.simplices
            return triangles
            
        except Exception as e:
            # Log but don't crash - return empty triangulation
            print(f"Triangulation warning: {str(e)}")
            return []

    def _is_degenerate(self, points):
        """Check if points are degenerate (collinear or coplanar)."""
        if len(points) < 3:
            return True
        
        # Check if all points are on a line (collinear)
        # Calculate the range in x and y
        x_range = np.ptp(points[:, 0])  # peak to peak (max - min)
        y_range = np.ptp(points[:, 1])
        
        # If either dimension has very small range, points are nearly collinear
        min_range = 0.1  # threshold for considering points collinear
        if x_range < min_range or y_range < min_range:
            return True
        
        return False

    def _perturb_points(self, points):
        """Add small random perturbations to avoid degeneracy."""
        perturbation = np.random.normal(0, 0.01, points.shape)
        return points + perturbation

    def publish_mesh(self, local_points, triangles):
        """Publish triangular mesh to RViz."""
        marker_array = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.clock.now().to_msg()
        clear_marker.ns = "triangulation_mesh"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Only publish if we have triangles
        if len(triangles) == 0:
            self.mesh_publisher.publish(marker_array)
            return

        for i, triangle in enumerate(triangles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.clock.now().to_msg()
            marker.ns = "triangulation_mesh"
            marker.id = i + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Add triangle vertices
            for idx in triangle:
                if idx < len(local_points):
                    point = Point()
                    point.x = float(local_points[idx][0])
                    point.y = float(local_points[idx][1])
                    point.z = 0.0
                    marker.points.append(point)

            # Close the triangle
            if len(marker.points) > 0:
                marker.points.append(marker.points[0])

            marker.scale.x = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.mesh_publisher.publish(marker_array)