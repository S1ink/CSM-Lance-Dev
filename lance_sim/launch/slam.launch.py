import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def fast_lio_launch_description(this_pkg, use_sim_time):

    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ),
        launch_arguments = {
            'use_sim_time': use_sim_time,
            'config_path': os.path.join(this_pkg, 'config'),
            'config_file': 'fast_lio.yaml',
            'rviz': 'false'
        }.items()
    )

    return LaunchDescription([fast_lio_node])

def dlio_launch_description(this_pkg, use_sim_time):

    dlio_node = Node(
        package = 'direct_lidar_inertial_odometry',
        executable = 'dlio_odom_node',
        output = 'screen',
        parameters = [os.path.join(this_pkg, 'config', 'dlio.yaml')],
        remappings = [
            ('pointcloud', '/lance/lidar_points'),
            ('imu', '/lance/imu'),
            ('odom', 'dlio/odom'),
            ('pose', 'dlio/pose'),
            ('path', 'dlio/path'),
            ('kf_pose', 'dlio/keyframes'),
            ('kf_cloud', 'dlio/pointcloud/keyframe'),
            ('deskewed', 'dlio/pointcloud/deskewed')
        ]
    )

    return LaunchDescription([dlio_node])

def lidarslam_launch_description(this_pkg, use_sim_time):

    params_file = os.path.join(this_pkg, 'config', 'lidarslam.yaml')

    mapping_node = Node(
        package = 'scanmatcher',
        executable = 'scanmatcher_node',
        parameters = [params_file],
        remappings = [
            ('input_cloud', '/lance/lidar_points'),
            ('imu', '/lance/imu')
        ],
        output = 'screen'
    )

    graph_based_slam_node = Node(
        package = 'graph_based_slam',
        executable = 'graph_based_slam_node',
        parameters = [params_file],
        output = 'screen'
    )

    return LaunchDescription([mapping_node, graph_based_slam_node])

def dlo_launch_description(this_pkg, use_sim_time):

    # rectifier_node = Node(
    #     name = 'tf_rectifier',
    #     package = 'lance_sim',
    #     executable = 'rectifier_node',
    #     output = 'screen',
    #     remappings = [
    #         ('imu', '/lance/imu'),
    #         ('pointcloud', '/lance/lidar_points'),
    #         ('rectified_imu', '/lance/rectified/imu'),
    #         ('rectified_pc', '/lance/rectified/lidar_points')
    #     ]
    # )

    dlo_node = Node(
        name = 'dlo_odom',
        package = 'direct_lidar_odometry',
        executable = 'dlo_odom_node',
        output = 'screen',
        parameters = [os.path.join(this_pkg, 'config', 'dlo.yaml'), {'use_sim_time', use_sim_time}],
        remappings = [
            ('pointcloud', '/lance/lidar_points'),
            ('imu', '/lance/imu'),
            ('odom', 'dlo/odom'),
            ('pose', 'dlo/pose'),
            ('kfs', 'dlo/odom/keyframe'),
            ('keyframe', 'dlo/pointcloud/keyframe')
        ]
    )

    return LaunchDescription([dlo_node])


def generate_launch_description():
    this_pkg = get_package_share_directory('lance_sim')
    aruco_pkg = get_package_share_directory('aruco_server')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # aruco server
    aruco_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_pkg, 'launch', 'aruco_server.launch.py')
        ),
        launch_arguments = {'use_sim_time': use_sim_time}.items()
    )
    # robot_localization
    # robot_localization_cmd = Node(
    # 	name = 'ekf_filter_node',
    # 	package = 'robot_localization',
    # 	executable = 'ekf_node',
    # 	output = 'screen',
    # 	parameters = [os.path.join(pkg_path, 'config', 'robot_localization.yaml'), {'use_sim_time': True}]
    # )

    ld = dlo_launch_description(this_pkg, use_sim_time)
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(aruco_server_cmd)

    return ld
