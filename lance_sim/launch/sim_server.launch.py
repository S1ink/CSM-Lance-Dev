import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

	launch_file_dir = os.path.join( get_package_share_directory('lance_sim'), 'launch' )

	sim_gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_gazebo.launch.py')
		),
		launch_arguments = {
			'gz_gui' : LaunchConfiguration('gz_gui', default='true'),
			'gz_map' : LaunchConfiguration('gz_map', default='arena'),
			'rviz' : LaunchConfiguration('rviz', defualt='false')
		}.items()
	)
	sim_coprocessor = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'sim_coprocessor.launch.py')
		)
	)

	return LaunchDescription([
		DeclareLaunchArgument('gz_gui', default_value='true'),
		DeclareLaunchArgument('gz_map', default_value='arena'),
		DeclareLaunchArgument('rviz', default_value='false'),
		sim_gazebo,
		sim_coprocessor
	])
