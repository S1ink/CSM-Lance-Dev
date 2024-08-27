import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('lance_sim')
    # nav2_bringup = get_package_share_directory('nav2_bringup')

    launch_file_dir = os.path.join( pkg_path, 'launch' )


    # publish robot state to /tf
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    # slam
    slam_impl_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'slam.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    # nav2
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
    #     ),
    #     launch_arguments = {'params_file' : os.path.join(pkg_path, 'config', 'nav2.yaml')}.items()
    # )
    # foxglove
    foxglove_node = Node(
        name = 'foxglove_server',
        package = 'foxglove_bridge',
        executable = 'foxglove_bridge',
        output = 'screen',
        parameters = [os.path.join(pkg_path, 'config', 'foxglove_bridge.yaml'), {'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_cmd,
        slam_impl_cmd,
        # nav2_launch,
        foxglove_node
    ])