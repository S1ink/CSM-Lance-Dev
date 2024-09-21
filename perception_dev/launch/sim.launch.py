import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
    # bag2 record
    bag_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            '/lance/lidar_scan',
            '/lance/imu',
            '/tf',
            '/tf_static',
            '/model/lance/left_cam/image/compressed',
            '/model/lance/fwd_cam/image/compressed',
            '/model/lance/rght_cam/image/compressed',
            '/model/lance/left_cam/camera_info',
            '/model/lance/fwd_cam/camera_info',
            '/model/lance/rght_cam/camera_info',
            '--use-sim-time',
            '--compression-mode', 'file',
            '--compression-format', 'zstd'
        ],
        output='screen',
        condition = IfCondition( LaunchConfiguration('record', default='false') )
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='false'),
        DeclareLaunchArgument('gz_map', default_value='arena'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        launch_main,
        launch_coprocessor,
        bag_recorder
    ])
