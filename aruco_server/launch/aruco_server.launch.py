import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    pkg_path = get_package_share_directory('aruco_server')
    param_file = os.path.join(pkg_path, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        Node(
            name = 'aruco_server',
            package = 'aruco_server',
            executable = 'aruco_node',
            parameters = [param_file, {'use_sim_time': use_sim_time}]
        )
    ])
