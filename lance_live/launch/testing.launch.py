import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('lance_live')
    sim_pkg_path = get_package_share_directory('lance_sim')

    # launch sick_scan_xd using parameters from this project
    sick_scan_xd = Node(
        name = 'sick_scan_xd',
        package = 'sick_scan_xd',
        executable = 'sick_generic_caller',
        output = 'screen',
        arguments = [
            os.path.join(pkg_path, 'config', 'sick_multiscan.xml')
        ]
    )

    # launch image servers for each camera
    # https://github.com/ros-drivers/usb_cam
    # TODO

    # launch robot_state_publisher from lance_sim
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )

    # launch cardinal_perception using parameters in this project
    cardinal_perception = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'cardinal_perception.yaml'),   # TODO: use sick_scan_xd and camera driver output topics!
            {'use_sim_time': False}
        ],
        remappings = [
            ('debug_img', 'cardinal_perception/debug_img'),
            ('filtered_scan', 'cardinal_perception/filtered_scan')
        ]
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
        ]
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
        sick_scan_xd,
        # image servers
        cardinal_perception,
        robot_state_publisher,
        foxglove_bridge,
        rviz
    ])

