import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

	launch_file_dir = os.path.join( get_package_share_directory('lance_sim'), 'launch' )

	sim_server = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_server.launch.py')
		),
		launch_arguments = {
			'gz_gui' : LaunchConfiguration('gz_gui', default='false'),
			'gz_map' : LaunchConfiguration('gz_map', default='arena'),
			'rviz' : LaunchConfiguration('rviz', default='false'),
			'foxglove' : LaunchConfiguration('foxglove', default='true')
		}.items()
	)
	sim_remote = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_remote.launch.py')
		),
		launch_arguments = {
			'rviz' : 'false'
		}.items()
	)

	return LaunchDescription([
		DeclareLaunchArgument('gz_gui', default_value='false'),
		DeclareLaunchArgument('gz_map', default_value='arena'),
		DeclareLaunchArgument('rviz', default_value='false'),
		DeclareLaunchArgument('foxglove', default_value='true'),
		sim_server,
		sim_remote
	])
