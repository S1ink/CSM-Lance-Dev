import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

	pkg_path = get_package_share_directory('lance_sim')
	ros_gz_sim = get_package_share_directory('ros_gz_sim')
	# fast_lio = get_package_share_directory('fast_lio')
	dlio = get_package_share_directory('direct_lidar_inertial_odometry')

	launch_file_dir = os.path.join( pkg_path, 'launch' )
	world_path = os.path.join( pkg_path, 'worlds', 'artemis-arena.world' )

	use_sim_time = LaunchConfiguration('use_sim_time', default='true')

	# set env vars
	set_env_vars_resources = AppendEnvironmentVariable(
		'GZ_SIM_RESOURCE_PATH',
		os.path.join(pkg_path, 'description')
	)
	# launch gazebo server with the arena SDF
	gzserver_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
		),
		launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true', 'pause': 'true'}.items()
	)
	# start gazebo client
	gzclient_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
		),
		launch_arguments={'gz_args': '-g -v4 '}.items()
	)
	# call our other launch file
	robot_state_publisher_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)
	# spawn the robot
	spawn_lance_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'spawn_lance.launch.py')
		)
		# launch_arguments={
		# 	'x_pose': 1.0,
		# 	'y_pose': 1.0
		# }.items()
	)
	# rviz
	rviz_cmd = Node(
		package = 'rviz2',
		executable = 'rviz2',
		arguments = ['-d', os.path.join(pkg_path, 'config', 'sim.rviz')]
		# condition
	)

	# fast_lio_cmd = IncludeLaunchDescription(
	# 	PythonLaunchDescriptionSource(
	# 		os.path.join(fast_lio, 'launch', 'mapping.launch.py')
	# 	),
	# 	launch_arguments = {
	# 		'use_sim_time': use_sim_time,
	# 		'config_path': os.path.join(pkg_path, 'config'),
	# 		'config_file': 'fast_lio.yaml',
	# 		'rviz': 'false'
	# 	}.items()
	# )
	dlio_cmd = Node(
		package = 'direct_lidar_inertial_odometry',
		executable = 'dlio_odom_node',
		output = 'screen',
		parameters = [os.path.join(pkg_path, 'config', 'dlio.yaml')],
		remappings = [
			('pointcloud', '/lance/lidar_points'),
            ('imu', '/lance/imu'),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed')
		]
	)

	ld = LaunchDescription()

	# Add the commands to the launch description
	ld.add_action(set_env_vars_resources)
	ld.add_action(gzserver_cmd)
	ld.add_action(gzclient_cmd)
	ld.add_action(robot_state_publisher_cmd)
	ld.add_action(spawn_lance_cmd)
	ld.add_action(rviz_cmd)
	# ld.add_action(fast_lio_cmd)
	ld.add_action(dlio_cmd)

	return ld
