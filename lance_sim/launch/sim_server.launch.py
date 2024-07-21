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
	aruco_pkg = get_package_share_directory('aruco_server')

	launch_file_dir = os.path.join( pkg_path, 'launch' )
	worlds_path = os.path.join( pkg_path, 'worlds' )

	artemis_arena_world = os.path.join( worlds_path, 'artemis-arena.world' )
	maze_world = os.path.join( worlds_path, 'maze.world' )
	moon_world = os.path.join( worlds_path, 'moon.world' )
	
	# config arg for choosing which map to use

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
		launch_arguments={'gz_args': ['-r -s -v4 ', artemis_arena_world], 'on_exit_shutdown': 'true', 'pause': 'true'}.items()
	)
	# start gazebo client
	gzclient_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
		),
		launch_arguments={'gz_args': '-g -v4 '}.items()
	)
	# spawn the robot
	spawn_lance_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'spawn_lance.launch.py')
		)
	)
	# publish robot state to /tf
	robot_state_publisher_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
		),
		launch_arguments={'use_sim_time': 'true'}.items()
	)
	# slam
	slam_impl_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(launch_file_dir, 'slam.launch.py')
		),
		launch_arguments = {'use_sim_time': 'true'}.items()
	)
	# aruco server
	aruco_server_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(aruco_pkg, 'launch', 'aruco_server.launch.py')
		),
		launch_arguments = {'use_sim_time': 'true'}.items()
	)
	# robot_localization
	robot_localization_cmd = Node(
		package = 'robot_localization',
		executable = 'ekf_node',
		name = 'ekf_filter_node',
		output = 'screen',
		parameters = [os.path.join(pkg_path, 'config', 'robot_localization.yaml'), {'use_sim_time': True}]
	)
	# velocity publisher
	teleop_node = Node(
		name = 'teleop_node',
		package='teleop_twist_joy',
		executable='teleop_node',
		parameters=[os.path.join(pkg_path, 'config', 'xbox_controller.yaml')],
		remappings=[('/cmd_vel', '/joystick_cmd_vel')]
	)


	ld = LaunchDescription()

	# Add the commands to the launch description
	ld.add_action(set_env_vars_resources)
	ld.add_action(gzserver_cmd)
	ld.add_action(gzclient_cmd)	# config for enable/disable?
	ld.add_action(robot_state_publisher_cmd)
	ld.add_action(spawn_lance_cmd)
	ld.add_action(slam_impl_cmd)
	ld.add_action(aruco_server_cmd)
	# ld.add_action(robot_localization_cmd)
	ld.add_action(teleop_node)

	return ld
