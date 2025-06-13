import numpy as np
import math
from shapely.geometry import Point as ShapelyPoint

class LidarData:
    def __init__(self, map_boundary, safety_margin):
        self.map_boundary = map_boundary
        self.safety_margin = safety_margin
        self.obstacle_points = []

    def process_scan(self, scan_msg, current_position, current_orientation):
        """Process LIDAR scan data."""
        obstacle_points = []
        angle = scan_msg.angle_min

        for range_val in scan_msg.ranges:
            if scan_msg.range_min < range_val < scan_msg.range_max:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                world_x = x * math.cos(current_orientation) - y * math.sin(current_orientation) + current_position[0]
                world_y = x * math.sin(current_orientation) + y * math.cos(current_orientation) + current_position[1]

                world_point = np.array([world_x, world_y])

                if self.map_boundary.contains(ShapelyPoint(world_point)):
                    dist_to_boundary = self.map_boundary.exterior.distance(ShapelyPoint(world_point))
                    if dist_to_boundary > self.safety_margin:
                        obstacle_points.append(world_point)

            angle += scan_msg.angle_increment

        self.obstacle_points = np.array(obstacle_points) if obstacle_points else np.array([])
        return self.obstacle_points
