import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('disparity_extender')
    
    # Path to parameter file
    param_file = os.path.join(pkg_dir, 'config', 'disparity_params.yaml')
    
    return LaunchDescription([
        Node(
            package='disparity_extender',
            executable='disparity_extender',
            name='disparity_extender_racer',
            output='screen',
            emulate_tty=True,
            parameters=[param_file],
            remappings=[
                ('/scan', '/scan'),
                ('/ego_racecar/odom', '/ego_racecar/odom'),
                ('/drive', '/drive')
            ]
        )
    ])
