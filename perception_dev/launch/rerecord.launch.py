import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('perception_dev')

    # launch robot_state_publisher from lance_sim
    replay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'replay.launch.py')
        ),
        launch_arguments = {
            'rviz': LaunchConfiguration('rviz', default='false'),
            'foxglove': LaunchConfiguration('foxglove', default='true'),
            'processing': LaunchConfiguration('processing', default='false'),
            'bag': LaunchConfiguration('bag', defualt='')
        }.items()
    )

    bag_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            '--compression-mode', 'file',
            '--compression-format', 'zstd'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('processing', default_value='false'),
        DeclareLaunchArgument('bag', default_value=''),
        replay,
        bag_recorder
    ])
