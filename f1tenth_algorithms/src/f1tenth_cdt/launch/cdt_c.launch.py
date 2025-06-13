from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        DeclareLaunchArgument(
            'use_dynamic_boundaries',
            default_value='true',
            description='Enable dynamic boundary detection from LIDAR'
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='1.5',
            description='Lookahead distance for pure pursuit'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.5',
            description='Maximum speed'
        ),
        
        # CDT Planner Node with combined approach
        Node(
            package='f1tenth_cdt',
            executable='main',
            name='cdt_planner',
            output='screen',
            parameters=[{
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'max_speed': LaunchConfiguration('max_speed'),
                'safety_margin': 0.3,
                'map_resolution': 0.3,
                'use_dynamic_boundaries': LaunchConfiguration('use_dynamic_boundaries'),
                'wall_follow_distance': 1.0,
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            remappings=[
                ('/scan', '/scan'),
                ('/ego_racecar/odom', '/ego_racecar/odom'),
                ('/drive', '/drive'),
                ('/triangulation_mesh', '/triangulation_mesh'),
                ('/planned_path', '/planned_path'),
                ('/track_boundary', '/track_boundary'),
                ('/free_space_points', '/free_space_points'),
            ]
        ),
    ])