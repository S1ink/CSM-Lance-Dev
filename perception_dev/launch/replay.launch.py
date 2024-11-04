import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('perception_dev')
    sim_pkg_path = get_package_share_directory('csm_gz_sim')

    # launch robot_state_publisher from lance_sim
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )
    # bag2 play
    bag_player = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag', default=''),
            '--loop',
            # '--topics', '/multiscan/lidar_scan', '/multiscan/imu'
            # '--start-offset', '248'
        ],
        output='screen'
    )
    # perception stack
    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'perception.launch.py')
        ),
        launch_arguments = {
            'is_sim': 'false',
            'live_scan_topic': '/multiscan/lidar_scan',
            'live_imu_topic': '/multiscan/imu'
            # 'live_scan_topic': '/filtered_cloud'
        }.items(),
        condition = IfCondition(LaunchConfiguration('processing', default='false'))
    )
    # foxglove server if enabled
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )
    # rviz
    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', os.path.join(pkg_path, 'config', 'sim.rviz')],
        condition = IfCondition( LaunchConfiguration('rviz', default='false') )
    )


    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('processing', default_value='false'),
        DeclareLaunchArgument('bag', default_value=''),
        robot_state_publisher,
        bag_player,
        launch_perception,
        foxglove_node,
        rviz
    ])
