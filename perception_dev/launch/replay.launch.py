import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def make_scan_transformer(output_frame : str, override_frame : str, sub_topic : str, pub_topic : str):
    return Node(
        package = 'debug_tools',
        executable = 'scan_transformer',
        output = 'screen',
        parameters = [{
            'target_frame': output_frame,
            'override_frame': override_frame
        }],
        remappings = [
            ('input_scan', sub_topic),
            ('transformed_scan', pub_topic)
        ]
    )

def make_imu_transformer(output_frame : str, override_frame : str, sub_topic : str, pub_topic : str):
    return Node(
        package = 'debug_tools',
        executable = 'imu_transformer',
        output = 'screen',
        parameters = [{
            'target_frame': output_frame,
            'override_frame': override_frame
        }],
        remappings = [
            ('input_imu', sub_topic),
            ('transformed_imu', pub_topic)
        ]
    )

def make_imu_visualizer(topic : str):
    return Node(
        package = 'debug_tools',
        executable = 'imu_visualizer',
        output = 'screen',
        parameters = [{ 'imu_topic': topic }]
    )

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

    # launch cardinal_perception using parameters in this project
    cardinal_perception = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'cardinal_perception_live.yaml'),
            {
                'use_sim_time': False,
                'scan_topic': '/cloud_all_fields_fullframe/transformed',
                # 'imu_topic': '/multiscan/imu',
            }
        ],
        remappings = [
            ('debug_img', 'cardinal_perception/debug_img'),
            ('filtered_scan', 'cardinal_perception/filtered_scan')
        ],
        condition = IfCondition( LaunchConfiguration('processing', default='true') )
    )

    # launch foxglove_bridge
    foxglove_bridge = Node(
        name = 'foxglove_server',
        package = 'foxglove_bridge',
        executable = 'foxglove_bridge',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'foxglove_bridge.yaml'),
            {'use_sim_time': False}
        ],
        condition = IfCondition( LaunchConfiguration('foxglove', default='true') )
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
        make_imu_transformer('frame_link', '', '/multiscan/imu', '/multiscan/transformed_imu'),
        make_scan_transformer('world', 'lidar_link', '/cloud_all_fields_fullframe', '/cloud_all_fields_fullframe/transformed'),
        make_imu_visualizer('/multiscan/transformed_imu'),
        cardinal_perception,
        foxglove_bridge,
        rviz
    ])
