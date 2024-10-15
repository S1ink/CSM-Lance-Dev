import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('perception_dev')

    # robot state publisher
    launch_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('csm_gz_sim'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    # cardinal_perception
    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cardinal_perception'), 'launch', 'cardinal_perception.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    # sick_perception
    launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sick_perception'), 'launch', 'sick_perception.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    # foxglove server if enabled
    foxglove_node = Node(
        name = 'foxglove',
        package = 'foxglove_bridge',
        executable = 'foxglove_bridge',
        output = 'screen',
        condition = IfCondition( LaunchConfiguration('foxglove', default='true') ),
        parameters = [
            os.path.join(pkg_path, 'config', 'foxglove_bridge.yaml'),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='true'),
        launch_state_pub,
        launch_perception,
        launch_mapping,
        foxglove_node
    ])
