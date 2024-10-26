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
    sim_pkg_path = get_package_share_directory('csm_gz_sim')

    # launch robot_state_publisher from lance_sim
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )
    # lidar driver
    multiscan_driver = Node(
        name = 'multiscan_driver',
        package = 'multiscan_driver',
        executable = 'multiscan_driver',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'multiscan_driver.yaml')
        ],
        remappings = [
            ('lidar_scan', '/multiscan/lidar_scan'),
            ('lidar_imu', '/multiscan/imu')
        ]
    )
    # cameras
    launch_tag_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'tag_detection_live.launch.py')
        ),
        condition = IfCondition( LaunchConfiguration('processing', default='true') )
    )
    # perception stack
    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'perception.launch.py')
        ),
        launch_arguments = {
            'is_sim': 'false',
            'live_scan_topic': '/multiscan/lidar_scan',
            'live_imu_topic': '/multiscan/imu',
            'use_live_tags': 'false'
        }.items(),
        condition = IfCondition( LaunchConfiguration('processing', default='true') )
    )
    # bag2 record
    bag_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            '/multiscan/lidar_scan',
            '/multiscan/imu',
            '/tf',
            '/tf_static',
            # '--compression-mode', 'file',
            # '--compression-format', 'zstd'
        ],
        output='screen',
        condition = IfCondition( LaunchConfiguration('record', default='false') )
    )
    # foxglove server if enabled
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items(),
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
        DeclareLaunchArgument('processing', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        robot_state_publisher,
        multiscan_driver,
        launch_tag_detection,
        launch_perception,
        bag_recorder,
        foxglove_node,
        rviz
    ])
