import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def make_scan_transformer(output_frame : str, override_frame : str, sub_topic, pub_topic):
    return Node(
        package = 'debug_tools',
        executable = 'scan_transformer',
        output = 'screen',
        parameters = [{
            'target_frame': output_frame,
            'override_frame': override_frame
        }],
        remappings = [
            ('input_scan', sub_topic),
            ('transformed_scan', pub_topic)
        ]
    )

def make_imu_transformer(output_frame : str, override_frame : str, sub_topic, pub_topic):
    return Node(
        package = 'debug_tools',
        executable = 'imu_transformer',
        output = 'screen',
        parameters = [{
            'target_frame': output_frame,
            'override_frame': override_frame
        }],
        remappings = [
            ('input_imu', sub_topic),
            ('transformed_imu', pub_topic)
        ]
    )

def make_imu_visualizer(topic):
    return Node(
        package = 'debug_tools',
        executable = 'imu_visualizer',
        output = 'screen',
        parameters = [{ 'imu_topic': topic }]
    )

def make_accuracy_analyzer(active_frame : str = 'base_link', origin_frame : str = 'map', validation_frame : str = 'gz_base_link', sample_window : float = 0.25, is_sim : bool = True, remap = '/accuracy_analysis'):
    return Node(
        package = 'debug_tools',
        executable = 'accuracy_analyzer',
        output = 'screen',
        parameters = [{
            'origin_frame_id': origin_frame,
            'active_frame_id': active_frame,
            'validation_frame_id': validation_frame,
            'std_sample_window_s': sample_window,
            'use_sim_time': is_sim
        }],
        remappings = [
            ('accuracy_analysis', remap)
        ]
    )

def make_state_publisher(sim_time : bool = True):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('csm_gz_sim'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': str(sim_time)}.items()
    )

def make_localization_node(params, cond = None):
    return Node(
        name = 'cardinal_perception_localization',
        package = 'cardinal_perception',
        executable = 'localization_node',
        output = 'screen',
        parameters = params,
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('tags_detections', '/cardinal_perception/tags_detections')
        ],
        condition = cond
    )

def make_tag_detection_node(params, cond = None):
    return Node(
        name = 'cardinal_perception_tag_detection',
        package = 'cardinal_perception',
        executable = 'tag_detection_node',
        output = 'screen',
        parameters = params,
        remappings = [ ('tags_detections', '/cardinal_perception/tags_detections') ],
        condition = cond
    )

def make_legacy_mapper_node(params, cond = None):
    return Node(
        name = 'sick_perception',
        package = 'sick_perception',
        executable = 'sick_perception',
        output = 'screen',
        parameters = params,
        remappings = [
            ('obstacle_grid', '/sick_perception/obstacle_grid'),
            ('voxel_map', '/sick_perception/voxel_map')
        ],
        condition = cond
    )

def make_nvblox_node(params, cloud_in : str = '/cardinal_perception/filtered_scan', cond = None):
    return Node(
        name = 'nvblox',
        package = 'nvblox_ros',
        executable = 'nvblox_node',
        parameters = params,
        remappings = [ ('pointcloud', cloud_in) ],
        condition = cond
    )

def make_octomap_node(params, cloud_in : str = '/cardinal_perception/filtered_scan', cond = None):
    return Node(
        name = 'octomap_server',
        package = 'octomap_server',
        executable = 'octomap_server_node',
        output = 'screen',
        parameters = params,
        remappings = [ ('cloud_in', cloud_in) ],
        condition = cond
    )

def make_mola_cli_node(config_path, cond = None):
    return Node(
        package = 'mola_launcher',
        executable = 'mola-cli',
        output = 'screen',
        arguments = [config_path],
        condition = cond
    )

def generate_launch_description():

    pkg_path = get_package_share_directory('perception_dev')
    cardinal_perception = get_package_share_directory('cardinal_perception')
    sick_perception = get_package_share_directory('sick_perception')

    is_sim = LaunchConfiguration('is_sim', default = 'true')
    use_live_tags = LaunchConfiguration('use_live_tags', default = 'false')
    live_scan_topic = LaunchConfiguration('live_scan_topic', default = '/multiscan/lidar_scan')
    live_imu_topic = LaunchConfiguration('live_imu_topic', default = '/multiscan/imu')


    return LaunchDescription([
        DeclareLaunchArgument('is_sim', default_value = 'true'),
        DeclareLaunchArgument('use_live_tags', default_value = 'false'),
        DeclareLaunchArgument('live_scan_topic', default_value = '/multiscan/lidar_scan'),
        DeclareLaunchArgument('live_imu_topic', default_value = '/multiscan/imu'),
    # Simulator >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        GroupAction(
            actions = [
                make_localization_node([
                    os.path.join(cardinal_perception, 'config', 'localization.yaml'),
                    {'use_sim_time': True}
                ]),
                make_tag_detection_node([
                    os.path.join(cardinal_perception, 'config', 'tag_detection.yaml'),
                    {'use_sim_time': True}
                ]),
                # make_legacy_mapper_node([
                #     os.path.join(sick_perception, 'config', 'sick_perception.yaml'),
                #     {'use_sim_time': True}
                # ]),
                # make_nvblox_node([
                #     os.path.join(pkg_path, 'config', 'nvblox.yaml'),
                #     {'use_sim_time': True}
                # ], '/cardinal_perception/filtered_scan'),
                # make_octomap_node([
                #     os.path.join(pkg_path, 'config', 'octomap.yaml'),
                #     {'use_sim_time': True}
                # ], '/cardinal_perception/filtered_scan'),
                # make_mola_cli_node([
                #     os.path.join(pkg_path, 'config', 'mola-lo.yaml'),
                #     {'use_sim_time': True}
                # ]),
                make_accuracy_analyzer('base_link', 'map', 'gz_base_link', 0.25, True, 'localization_acc_analysis'),
                make_accuracy_analyzer('base_link_e0', 'map', 'gz_base_link', 0.25, True, 'tags_detection_acc_analysis')
            ],
            condition = IfCondition(is_sim)
        ),
    # Live/Replay >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        GroupAction(
            actions = [
                make_localization_node([
                    os.path.join(pkg_path, 'config', 'localization_live.yaml'),
                    {
                        'use_sim_time': False,
                        'scan_topic': live_scan_topic,
                        'imu_topic': live_imu_topic
                    }
                ]),
                make_tag_detection_node([
                    os.path.join(pkg_path, 'config', 'tag_detection_live.yaml'),
                    {'use_sim_time': False}
                ], IfCondition(use_live_tags)),
                # make_legacy_mapper_node([
                #     os.path.join(sick_perception, 'config', 'sick_perception.yaml'),
                #     {'use_sim_time': False}
                # ]),
                # make_imu_transformer('frame_link', '', live_imu_topic, '/multiscan/transformed_imu'),
                # make_scan_transformer('world', 'lidar_link', '/cloud_all_fields_fullframe', '/cloud_all_fields_fullframe/transformed'),
                # make_imu_visualizer('/multiscan/transformed_imu'),
                # make_accuracy_analyzer('base_link', 'map', '', 0.25)
            ],
            condition = UnlessCondition(is_sim)
        )
    ])
