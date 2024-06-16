import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	pkg_path = get_package_share_directory('lance_sim')
	model_path = os.path.join(
		pkg_path,
		'description',
		'lance',
		'model.sdf'
	)
	bridge_params = os.path.join(
		pkg_path,
		'config',
		'lance_gz_bridge.yaml'
	)

	start_gazebo_ros_spawner_cmd = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
			'-name', 'lance',
			'-file', model_path,
			'-x', '1.0',
			'-y', '1.0',
			'-z', '0.0'
		],
		output='screen',
	)
	start_gazebo_ros_bridge_cmd = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
			'--ros-args',
			'-p',
			f'config_file:={bridge_params}',
		],
		output='screen',
	)

	# start_gazebo_ros_image_bridge_cmd = Node(
	# 	package='ros_gz_image',
	# 	executable='image_bridge',
	# 	arguments=['/camera/image_raw'],
	# 	output='screen',
	# )

	ld = LaunchDescription()

	# Add any conditioned actions
	ld.add_action(start_gazebo_ros_spawner_cmd)
	ld.add_action(start_gazebo_ros_bridge_cmd)
	# ld.add_action(start_gazebo_ros_image_bridge_cmd)

	return ld
