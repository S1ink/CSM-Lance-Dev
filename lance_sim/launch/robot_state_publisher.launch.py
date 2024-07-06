import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

	pkg_path = get_package_share_directory('lance_sim')
	urdf_path = os.path.join( pkg_path, 'description', 'lance.urdf' )

	use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')

	return LaunchDescription(
		[
			DeclareLaunchArgument(
				'use_sim_time',
				default_value = 'false',
				description = 'Use simulation (Gazebo) clock if true'
			),
			Node(
				package = 'robot_state_publisher',
				executable = 'robot_state_publisher',
				name = 'robot_state_publisher',
				output = 'screen',
				parameters = [
					{
						'use_sim_time': use_sim_time,
						'robot_description': xacro.process_file(urdf_path).toxml()
					}
				],
			),
		]
	)
