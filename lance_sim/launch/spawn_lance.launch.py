import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
		# parameters=[ {
		# 	'use_sim_time' : True,
		# 	# 'override_timestamps_with_wall_time' : True
		# }],
		arguments=[
			'--ros-args',
			'-p',
			f'config_file:={bridge_params}',
		],
		output='screen',
	)

	start_gazebo_ros_image_bridge_cmd = Node(
		package='ros_gz_image',
		executable='image_bridge',
		# parameters=[{'use_sim_time' : False}],
		arguments=[
			'/model/lance/fwd_cam/image',
			'/model/lance/rght_cam/image',
			'/model/lance/left_cam/image',
			# '--ros-args',
			# '-p',
			# 'qos:=sensor_data'
		],
		output='screen',
	)


	return LaunchDescription([
		start_gazebo_ros_spawner_cmd,
		start_gazebo_ros_bridge_cmd,
		start_gazebo_ros_image_bridge_cmd
	])
