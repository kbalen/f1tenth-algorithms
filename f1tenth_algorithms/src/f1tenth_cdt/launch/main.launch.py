from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_cdt',
            executable='main',
            name='main',
            output='screen',
        ),
    ])