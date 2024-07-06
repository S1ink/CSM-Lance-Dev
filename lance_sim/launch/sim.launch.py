import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	launch_file_dir = os.path.join( get_package_share_directory('lance_sim'), 'launch' )

	sim_server = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_server.launch.py')
		)
	)
	sim_remote = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_remote.launch.py')
		)
	)

	return LaunchDescription([
		sim_server,
		sim_remote
	])
