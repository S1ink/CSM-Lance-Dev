import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def make_accuracy_analyzer(active_frame : str = 'base_link', origin_frame : str = 'map', validation_frame : str = 'gz_base_link', sample_window : float = 0.25):
    return Node(
        package = 'debug_tools',
        executable = 'accuracy_analyzer',
        output = 'screen',
        parameters = [{
            'origin_frame_id': origin_frame,
            'active_frame_id': active_frame,
            'validation_frame_id': validation_frame,
            'std_sample_window_s': sample_window,
            'use_sim_time': True
        }],
        # remappings = [
        #     ('input_scan', sub_topic),
        #     ('transformed_scan', pub_topic)
        # ]
    )

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
        foxglove_node,
        make_accuracy_analyzer('base_link', 'map', 'gz_base_link', 0.25)
    ])
