#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import time
from collections import deque

class DisparityExtenderRacer(Node):
    def __init__(self):
        super().__init__('disparity_extender_racer')
        
        # 🏆 CHAMPIONSHIP ALGORITHM PARAMETERS
        self.declare_parameter('car_width', 0.31)           # Car width in meters  
        self.declare_parameter('wheelbase', 0.33)           # Wheelbase length
        self.declare_parameter('safety_margin', 0.05)      # Extra safety buffer
        self.declare_parameter('disparity_threshold', 0.3) # Minimum disparity to detect
        self.declare_parameter('max_speed', 7.0)           # Maximum speed (m/s)
        self.declare_parameter('min_speed', 1.0)           # Minimum speed (m/s)
        self.declare_parameter('brake_distance', 0.5)      # Distance to start braking
        self.declare_parameter('full_speed_distance', 3.0) # Distance for full speed
        
        # Get parameters
        self.car_width = self.get_parameter('car_width').value
        self.wheelbase = self.get_parameter('wheelbase').value  
        self.safety_margin = self.get_parameter('safety_margin').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.brake_distance = self.get_parameter('brake_distance').value
        self.full_speed_distance = self.get_parameter('full_speed_distance').value
        
        # 📡 ROS COMMUNICATIONS
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.target_pub = self.create_publisher(Marker, '/disparity_target', 10)
        self.filtered_scan_pub = self.create_publisher(MarkerArray, '/filtered_scan', 10)
        self.telemetry_pub = self.create_publisher(String, '/disparity_telemetry', 10)
        
        # 🚗 VEHICLE STATE
        self.current_position = None
        self.current_orientation = None
        self.current_velocity = 0.0
        self.scan_data = None
        
        # 🎯 ALGORITHM STATE
        self.filtered_ranges = None
        self.target_angle = 0.0
        self.target_distance = 0.0
        self.forward_distance = 0.0
        
        # 🕐 LAP TIMING  
        self.race_start_time = time.time()
        self.lap_start_time = None
        self.lap_times = []
        self.current_lap = 0
        self.best_lap_time = float('inf')
        
        # 📍 POSITION TRACKING
        self.position_history = deque(maxlen=100)
        self.start_line_position = None
        self.total_distance = 0.0
        self.last_position = None
        
        # 📊 PERFORMANCE METRICS
        self.average_speed = 0.0
        self.top_speed_achieved = 0.0
        self.disparities_detected = 0
        
        # Control timer - Championship algorithm runs at 40Hz (like the original)
        self.control_timer = self.create_timer(0.025, self.disparity_extender_loop)  # 40Hz
        self.telemetry_timer = self.create_timer(1.0, self.publish_telemetry)        # 1Hz
        
        self.get_logger().info('🏆 Disparity Extender Championship Algorithm Ready!')
        self.get_logger().info(f'🎯 This algorithm won F1TENTH with 11.5s lap times!')
        self.get_logger().info(f'⚡ Max Speed: {self.max_speed} m/s, Car Width: {self.car_width}m')
        self.get_logger().info('🚗 Algorithm: Find longest safe distance -> Go there!')
    
    def odom_callback(self, msg):
        """📡 Update vehicle state and track position"""
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        # Extract heading
        q = msg.pose.pose.orientation
        self.current_orientation = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )
        
        # Calculate speed
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        
        # Track performance
        self.top_speed_achieved = max(self.top_speed_achieved, self.current_velocity)
        
        # Position tracking for lap detection
        self.update_position_tracking()
        self.check_lap_completion()
    
    def scan_callback(self, msg):
        """👁️ Store LIDAR data for processing"""
        self.scan_data = msg
    
    def disparity_extender_loop(self):
        """🏆 MAIN CHAMPIONSHIP ALGORITHM (40Hz like the original)"""
        if self.scan_data is None or self.current_position is None:
            return
        
        # Step 1: Process LIDAR data
        ranges = np.array(self.scan_data.ranges)
        angles = np.arange(len(ranges)) * self.scan_data.angle_increment + self.scan_data.angle_min
        
        # Step 2: Find and extend disparities (THE CORE ALGORITHM)
        self.filtered_ranges = self.apply_disparity_extender(ranges, angles)
        
        # Step 3: Find the longest safe distance (not behind the car)
        target_angle, target_distance = self.find_longest_safe_distance(self.filtered_ranges, angles)
        
        # Step 4: Calculate forward distance for speed control
        forward_distance = self.get_forward_distance(ranges, angles)
        
        # Step 5: Check rear corners before turning (safety feature)
        safe_to_turn = self.check_rear_clearance(ranges, angles, target_angle)
        
        # Step 6: Calculate steering angle
        if safe_to_turn:
            steering_angle = self.calculate_steering_angle(target_angle)
        else:
            steering_angle = 0.0  # Go straight if turning is unsafe
        
        # Step 7: Calculate speed based on forward distance (simple but effective)
        speed = self.calculate_speed(forward_distance)
        
        # Step 8: Send drive command
        self.send_drive_command(speed, steering_angle)
        
        # Step 9: Publish visualization
        self.publish_visualization(target_angle, target_distance)
        
        # Store for telemetry
        self.target_angle = target_angle
        self.target_distance = target_distance
        self.forward_distance = forward_distance
    
    def apply_disparity_extender(self, ranges, angles):
        """🔍 THE CORE ALGORITHM: Disparity Extender"""
        # Create a copy of ranges for filtering
        filtered_ranges = ranges.copy()
        
        # Step 1: Find disparities in LIDAR data
        disparities = []
        for i in range(len(ranges) - 1):
            if (ranges[i] > 0.1 and ranges[i+1] > 0.1 and  # Valid readings
                abs(ranges[i] - ranges[i+1]) > self.disparity_threshold):
                disparities.append((i, i+1))
        
        self.disparities_detected = len(disparities)
        
        # Step 2: Extend each disparity
        for disp_start, disp_end in disparities:
            # Find the closer of the two points
            if ranges[disp_start] < ranges[disp_end]:
                closer_idx = disp_start
                farther_idx = disp_end
            else:
                closer_idx = disp_end
                farther_idx = disp_start
            
            closer_distance = ranges[closer_idx]
            
            # Step 3: Calculate car width in LIDAR samples at this distance
            if closer_distance > 0.1:  # Valid distance
                # Angular width of car at this distance
                car_angular_width = (self.car_width/2 + self.safety_margin) / closer_distance
                
                # Convert to number of LIDAR samples
                samples_to_mask = int(car_angular_width / abs(self.scan_data.angle_increment))
                samples_to_mask = max(1, samples_to_mask)  # At least 1 sample
                
                # Step 4: Extend the disparity (mask out unsafe area)
                if farther_idx > closer_idx:  # Extending to the right
                    start_idx = farther_idx
                    end_idx = min(len(filtered_ranges), farther_idx + samples_to_mask)
                    direction = 1
                else:  # Extending to the left
                    start_idx = farther_idx
                    end_idx = max(0, farther_idx - samples_to_mask)
                    direction = -1
                
                # Mask out the unsafe region
                if direction == 1:
                    for idx in range(start_idx, end_idx):
                        # Only overwrite if current distance is farther (safer)
                        if filtered_ranges[idx] > closer_distance:
                            filtered_ranges[idx] = closer_distance
                else:
                    for idx in range(start_idx, end_idx, -1):
                        if idx >= 0 and filtered_ranges[idx] > closer_distance:
                            filtered_ranges[idx] = closer_distance
        
        return filtered_ranges
    
    def find_longest_safe_distance(self, ranges, angles):
        """🎯 Find the farthest safely reachable point (not behind the car)"""
        max_distance = 0.0
        best_angle = 0.0
        
        # Only consider angles in front of the car (-90° to +90°)
        for i, angle in enumerate(angles):
            if -math.pi/2 <= angle <= math.pi/2:  # Forward hemisphere
                if ranges[i] > max_distance and ranges[i] > 0.1:  # Valid reading
                    max_distance = ranges[i]
                    best_angle = angle
        
        return best_angle, max_distance
    
    def get_forward_distance(self, ranges, angles):
        """📏 Get distance directly in front of car for speed control"""
        # Find the index closest to straight ahead (0 degrees)
        center_idx = len(ranges) // 2
        
        # Average a few samples around center for stability
        forward_samples = []
        sample_range = 5  # +/- 5 samples around center
        
        for i in range(max(0, center_idx - sample_range), 
                      min(len(ranges), center_idx + sample_range + 1)):
            if ranges[i] > 0.1:  # Valid reading
                forward_samples.append(ranges[i])
        
        if forward_samples:
            return np.mean(forward_samples)
        else:
            return 0.5  # Conservative fallback
    
    def check_rear_clearance(self, ranges, angles, target_angle):
        """🚧 Check if it's safe to turn (avoid hitting rear corners)"""
        # Determine turn direction
        if abs(target_angle) < 0.1:  # Going straight
            return True
        
        turn_direction = 1 if target_angle > 0 else -1  # 1 = left, -1 = right
        
        # Check the side we're turning towards
        min_safe_distance = self.car_width/2 + self.safety_margin
        
        for i, angle in enumerate(angles):
            # Check angles on the side of the turn (behind the car)
            if turn_direction > 0:  # Turning left, check left rear
                if math.pi/2 < angle <= math.pi:
                    if 0.1 < ranges[i] < min_safe_distance:
                        return False
            else:  # Turning right, check right rear  
                if -math.pi <= angle < -math.pi/2:
                    if 0.1 < ranges[i] < min_safe_distance:
                        return False
        
        return True
    
    def calculate_steering_angle(self, target_angle):
        """🎛️ Calculate steering angle towards target"""
        # The target angle IS the steering angle (simple!)
        steering_angle = target_angle
        
        # Limit to realistic steering range
        max_steering_angle = 0.35  # ~20 degrees
        steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)
        
        return steering_angle
    
    def calculate_speed(self, forward_distance):
        """⚡ Championship speed control: Simple distance-based scaling"""
        # Emergency brake
        if forward_distance < self.brake_distance:
            return 0.0
        
        # Full speed zone
        elif forward_distance >= self.full_speed_distance:
            return self.max_speed
        
        # Linear scaling between brake and full speed distances
        else:
            speed_range = self.max_speed - self.min_speed
            distance_range = self.full_speed_distance - self.brake_distance
            distance_ratio = (forward_distance - self.brake_distance) / distance_range
            
            speed = self.min_speed + (speed_range * distance_ratio)
            return np.clip(speed, self.min_speed, self.max_speed)
    
    def update_position_tracking(self):
        """📍 Track position for lap detection"""
        if self.last_position is not None:
            distance = np.linalg.norm(self.current_position - self.last_position)
            self.total_distance += distance
            
            # Calculate average speed
            if self.total_distance > 0:
                elapsed_time = time.time() - self.race_start_time
                self.average_speed = self.total_distance / elapsed_time
        
        self.position_history.append(self.current_position.copy())
        self.last_position = self.current_position.copy()
        
        # Set start line
        if self.start_line_position is None and len(self.position_history) > 20:
            self.start_line_position = self.current_position.copy()
            self.lap_start_time = time.time()
            self.get_logger().info('🏁 Disparity Extender timing started!')
    
    def check_lap_completion(self):
        """🏁 Check for lap completion"""
        if self.start_line_position is None or len(self.position_history) < 50:
            return
        
        distance_to_start = np.linalg.norm(self.current_position - self.start_line_position)
        
        if (distance_to_start < 2.0 and 
            self.total_distance > 15.0 and 
            self.lap_start_time is not None):
            
            self.complete_lap()
    
    def complete_lap(self):
        """🏆 Handle lap completion with championship celebration"""
        if self.lap_start_time is None:
            return
        
        lap_time = time.time() - self.lap_start_time
        self.current_lap += 1
        self.lap_times.append(lap_time)
        
        # Check for best lap
        if lap_time < self.best_lap_time:
            self.best_lap_time = lap_time
            self.get_logger().info(f'🏆 NEW BEST LAP! Lap {self.current_lap}: {lap_time:.2f}s')
            self.get_logger().info(f'🚀 Championship target: 11.5s (Original winner time)')
        else:
            self.get_logger().info(f'🏁 Lap {self.current_lap}: {lap_time:.2f}s (Best: {self.best_lap_time:.2f}s)')
        
        # Performance analysis
        avg_lap_time = np.mean(self.lap_times) if self.lap_times else 0.0
        self.get_logger().info(f'📊 Avg: {avg_lap_time:.2f}s, Top Speed: {self.top_speed_achieved:.1f}m/s')
        
        # Championship comparison
        if self.best_lap_time < 12.0:
            self.get_logger().info('🥇 CHAMPIONSHIP PACE! Faster than runner-up teams!')
        elif self.best_lap_time < 15.0:
            self.get_logger().info('🥈 Competitive pace - keep pushing!')
        
        # Reset for next lap
        self.lap_start_time = time.time()
        self.total_distance = 0.0
        self.position_history.clear()
    
    def send_drive_command(self, speed, steering_angle):
        """🚗 Send drive command to vehicle"""
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = float(max(0.0, speed))
        drive_cmd.drive.steering_angle = float(steering_angle)
        
        self.drive_pub.publish(drive_cmd)
    
    def publish_visualization(self, target_angle, target_distance):
        """🎨 Visualize the disparity extender algorithm"""
        # Publish target point marker
        if target_distance > 0:
            target_marker = Marker()
            target_marker.header.frame_id = "base_link"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = "disparity_target"
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            # Calculate target position in robot frame
            target_marker.pose.position.x = target_distance * math.cos(target_angle)
            target_marker.pose.position.y = target_distance * math.sin(target_angle)
            target_marker.pose.position.z = 0.0
            target_marker.pose.orientation.w = 1.0
            
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            
            # Color based on distance (green = far, red = close)
            target_marker.color.r = max(0.0, 1.0 - target_distance/10.0)
            target_marker.color.g = min(1.0, target_distance/5.0)
            target_marker.color.b = 0.0
            target_marker.color.a = 1.0
            
            self.target_pub.publish(target_marker)
        
        # Publish filtered scan visualization
        if self.filtered_ranges is not None and self.scan_data is not None:
            marker_array = MarkerArray()
            
            # Create marker for filtered scan points
            filtered_marker = Marker()
            filtered_marker.header.frame_id = "base_link"
            filtered_marker.header.stamp = self.get_clock().now().to_msg()
            filtered_marker.ns = "filtered_scan"
            filtered_marker.id = 0
            filtered_marker.type = Marker.POINTS
            filtered_marker.action = Marker.ADD
            filtered_marker.pose.orientation.w = 1.0
            
            filtered_marker.scale.x = 0.05
            filtered_marker.scale.y = 0.05
            
            # Add filtered points
            angles = np.arange(len(self.filtered_ranges)) * self.scan_data.angle_increment + self.scan_data.angle_min
            
            for i, (distance, angle) in enumerate(zip(self.filtered_ranges, angles)):
                if 0.1 < distance < 10.0:  # Valid range
                    point = Point()
                    point.x = distance * math.cos(angle)
                    point.y = distance * math.sin(angle)
                    point.z = 0.0
                    filtered_marker.points.append(point)
                    
                    # Color: red for modified points, blue for original
                    color = ColorRGBA()
                    if abs(distance - self.scan_data.ranges[i]) > 0.01:  # Modified by disparity extender
                        color.r = 1.0
                        color.g = 0.0
                        color.b = 0.0
                    else:  # Original point
                        color.r = 0.0
                        color.g = 0.0
                        color.b = 1.0
                    color.a = 0.8
                    filtered_marker.colors.append(color)
            
            marker_array.markers.append(filtered_marker)
            self.filtered_scan_pub.publish(marker_array)
    
    def publish_telemetry(self):
        """📊 Publish championship telemetry"""
        race_time = time.time() - self.race_start_time
        
        telemetry = {
            'algorithm': 'Disparity Extender (Championship Winner)',
            'race_time': f'{race_time:.1f}s',
            'lap': self.current_lap,
            'best_lap': f'{self.best_lap_time:.2f}s' if self.best_lap_time != float('inf') else 'N/A',
            'current_speed': f'{self.current_velocity:.1f} m/s',
            'target_distance': f'{self.target_distance:.1f}m',
            'target_angle': f'{math.degrees(self.target_angle):.1f}°',
            'forward_distance': f'{self.forward_distance:.1f}m',
            'disparities_found': self.disparities_detected,
            'avg_speed': f'{self.average_speed:.1f} m/s',
            'top_speed': f'{self.top_speed_achieved:.1f} m/s'
        }
        
        # Check for 3-minute session end
        if race_time > 180:
            self.end_race_session()
        
        telemetry_msg = String()
        telemetry_msg.data = str(telemetry)
        self.telemetry_pub.publish(telemetry_msg)
    
    def end_race_session(self):
        """🏁 End race session with championship results"""
        self.send_drive_command(0.0, 0.0)
        
        total_time = time.time() - self.race_start_time
        
        self.get_logger().info('🏆 DISPARITY EXTENDER RACE SESSION COMPLETE!')
        self.get_logger().info('📊 CHAMPIONSHIP RESULTS:')
        self.get_logger().info(f'   Algorithm: Disparity Extender (Won F1TENTH 2019)')
        self.get_logger().info(f'   Laps: {self.current_lap}')
        self.get_logger().info(f'   Best Lap: {self.best_lap_time:.2f}s')
        self.get_logger().info(f'   Championship Target: 11.5s')
        self.get_logger().info(f'   Top Speed: {self.top_speed_achieved:.1f} m/s')
        self.get_logger().info(f'   Avg Speed: {self.average_speed:.1f} m/s')
        
        if self.best_lap_time < 12.0:
            self.get_logger().info('🥇 CHAMPIONSHIP LEVEL PERFORMANCE!')
        elif self.best_lap_time < 15.0:
            self.get_logger().info('🥈 COMPETITIVE PERFORMANCE!')
        else:
            self.get_logger().info('🥉 GOOD LEARNING PACE!')
        
        # Disable racing
        self.control_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    racer = DisparityExtenderRacer()
    rclpy.spin(racer)
    racer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()