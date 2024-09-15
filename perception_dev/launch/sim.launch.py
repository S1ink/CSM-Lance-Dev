import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = get_package_share_directory('perception_dev')

    # launch main componenets
    launch_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_main.launch.py')
        ),
        launch_arguments = {
            'gz_gui' : LaunchConfiguration('gz_gui', default='false'),
            'gz_map' : LaunchConfiguration('gz_map', default='arena'),
            'rviz' : LaunchConfiguration('rviz', default='false')
        }.items()
    )
    # launch coprocessor components
    launch_coprocessor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_coprocessor.launch.py')
        ),
        launch_arguments = {
            'foxglove' : LaunchConfiguration('foxglove', default='true')
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='false'),
        DeclareLaunchArgument('gz_map', default_value='arena'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        launch_main,
        launch_coprocessor
    ])
