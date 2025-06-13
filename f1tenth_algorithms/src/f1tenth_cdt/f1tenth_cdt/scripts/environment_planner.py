import numpy as np
from shapely.geometry import Point as ShapelyPoint

class EnvironmentPlanner:
    def __init__(self, inner_boundary, map_resolution):
        self.inner_boundary = inner_boundary
        self.map_resolution = map_resolution

    def generate_local_grid(self, current_position, radius=5.0):
        """Generate a local grid around the robot with better distribution."""
        # Ensure we generate a 2D grid, not just along one axis
        x_min = current_position[0] - radius
        x_max = current_position[0] + radius
        y_min = current_position[1] - radius
        y_max = current_position[1] + radius

        # Generate grid points
        x_steps = max(3, int((x_max - x_min) / self.map_resolution))
        y_steps = max(3, int((y_max - y_min) / self.map_resolution))
        
        x_vals = np.linspace(x_min, x_max, x_steps)
        y_vals = np.linspace(y_min, y_max, y_steps)

        local_points = []
        for x in x_vals:
            for y in y_vals:
                pt = ShapelyPoint(x, y)
                # Check if point is within the track boundaries
                if self.inner_boundary.contains(pt):
                    local_points.append((x, y))

        # Always include the current position
        local_points.append(tuple(current_position))
        
        # Remove duplicates while preserving order
        seen = set()
        unique_points = []
        for point in local_points:
            point_tuple = (round(point[0], 3), round(point[1], 3))  # Round to avoid floating point issues
            if point_tuple not in seen:
                seen.add(point_tuple)
                unique_points.append(point)

        return unique_points

    def generate_forward_biased_grid(self, current_position, current_orientation, forward_bias=2.0):
        """Generate a grid biased in the forward direction of the robot."""
        # Calculate forward direction
        forward_x = np.cos(current_orientation)
        forward_y = np.sin(current_orientation)
        
        # Generate an elliptical grid biased forward
        local_points = []
        
        # Angular samples around the robot
        angles = np.linspace(-np.pi/2, np.pi/2, 7)  # Focus on forward 180 degrees
        distances = np.linspace(0.5, 5.0, 5)
        
        for dist in distances:
            for angle in angles:
                # Rotate angle relative to robot orientation
                global_angle = current_orientation + angle
                
                # Bias distance forward
                biased_dist = dist * (1.0 + forward_bias * np.cos(angle))
                
                x = current_position[0] + biased_dist * np.cos(global_angle)
                y = current_position[1] + biased_dist * np.sin(global_angle)
                
                pt = ShapelyPoint(x, y)
                if self.inner_boundary.contains(pt):
                    local_points.append((x, y))
        
        # Add current position
        local_points.append(tuple(current_position))
        
        # Remove duplicates
        return list(set(local_points))