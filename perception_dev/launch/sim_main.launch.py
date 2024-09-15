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

    pkg_path = get_package_share_directory('perception_dev')
    sim_pkg_path = get_package_share_directory('csm_gz_sim')

    # launch gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments = {
            'gz_gui' : LaunchConfiguration('gz_gui', default='false'),
            'gz_map' : LaunchConfiguration('gz_map', default='arena')
        }.items()
    )
    # launch xbox control
    launch_xbox_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'xbox_ctrl.launch.py')
        )
    )
    # launch rviz if enabled
    launch_rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', os.path.join(pkg_path, 'config', 'sim.rviz')],
        condition = IfCondition( LaunchConfiguration('rviz', default='false') ),
        parameters = [{'use_sim_time' : True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='false'),
        DeclareLaunchArgument('gz_map', default_value='arena'),
        DeclareLaunchArgument('rviz', default_value='false'),
        launch_gazebo,
        launch_xbox_ctrl,
        launch_rviz
    ])
