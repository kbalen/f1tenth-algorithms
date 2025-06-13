import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('f1tenth_cdt')
    
    # Path to the parameter file
    param_file = os.path.join(pkg_dir, 'config', 'cdt_params.yaml')
    
    return LaunchDescription([
        Node(
            package='f1tenth_cdt',
            executable='cdt_planner',
            name='cdt_planner',
            output='screen',
            emulate_tty=True,
            parameters=[param_file]
        )
    ])