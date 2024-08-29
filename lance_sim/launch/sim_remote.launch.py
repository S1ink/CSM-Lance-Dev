import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

	pkg_path = get_package_share_directory('lance_sim')

	# joystick publisher
	joy_pub = Node(
		package='joy',
		executable='joy_node',
		parameters=[ os.path.join(pkg_path,'config','xbox_controller.yaml'), {'use_sim_time' : True} ],
	)
	# rviz
	rviz = Node(
		package = 'rviz2',
		executable = 'rviz2',
		arguments = ['-d', os.path.join(pkg_path, 'config', 'sim.rviz')],
		condition = IfCondition( LaunchConfiguration('rviz', default='true') ),
		parameters = [{'use_sim_time' : True}]
	)

	return LaunchDescription([
		DeclareLaunchArgument('rviz', default_value='true'),
		joy_pub,
		rviz
	])
